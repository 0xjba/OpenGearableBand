/*
 * Host tests for button_gesture. Build + run:
 *   g++ -std=c++11 -Isrc tests/test_button_gesture.cpp src/button_gesture.cpp \
 *       -o /tmp/bg && /tmp/bg
 */
#include "button_gesture.h"
#include <cstdio>
#include <vector>
#include <string>

static int failures;

static void check(bool ok, const char *what)
{
	printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
	if (!ok) failures++;
}

/* Drive the machine at a 20ms sample period (what the app uses) and collect
 * every event emitted. `script` is a list of (pressed, duration_ms) segments. */
static std::vector<std::string>
run(const std::vector<std::pair<bool, uint32_t>> &script)
{
	button_gesture g;
	button_gesture_init(&g);
	std::vector<std::string> out;
	uint32_t t = 0;
	for (auto &seg : script) {
		for (uint32_t e = 0; e < seg.second; e += 20) {
			enum button_event ev = button_gesture_update(&g, seg.first, t);
			if (ev != BG_EVT_NONE) out.push_back(button_event_str(ev));
			t += 20;
		}
	}
	return out;
}
static bool eq(const std::vector<std::string> &a,
	       const std::vector<std::string> &b) { return a == b; }

int main()
{
	printf("button_gesture\n");

	/* A brief tap must not start a voice session -- the bug that motivated
	 * the confirm window in the first place. */
	check(eq(run({{true,100},{false,600}}), {"SINGLE_TAP"}),
	      "short tap -> SINGLE_TAP only, no HOLD_START");

	/* Hold to talk. */
	check(eq(run({{true,3000},{false,100}}), {"HOLD_START","HOLD_END"}),
	      "3s press -> HOLD_START then HOLD_END");

	/* THE IMPORTANT ONE: a long question must never escalate. */
	check(eq(run({{true,15000},{false,100}}), {"HOLD_START","HOLD_END"}),
	      "15s question -> still only HOLD_START/END, no MAINT_*");

	/* Double tap, no voice session either side of it. */
	check(eq(run({{true,100},{false,150},{true,100},{false,600}}),
		 {"DOUBLE_TAP"}),
	      "tap,gap,tap -> DOUBLE_TAP with no HOLD events");

	/* Too slow to be a double tap -> two singles. */
	check(eq(run({{true,100},{false,600},{true,100},{false,600}}),
		 {"SINGLE_TAP","SINGLE_TAP"}),
	      "taps 600ms apart -> two SINGLE_TAPs");

	/* Maintenance entry: tap, then press and hold. */
	check(eq(run({{true,100},{false,150},{true,12000},{false,100}}),
		 {"MAINT_5S","MAINT_10S"}),
	      "double-tap-hold 12s -> MAINT_5S then MAINT_10S");

	/* Each threshold fires once, not repeatedly. */
	{
		auto ev = run({{true,100},{false,150},{true,20000},{false,100}});
		int n5 = 0, n10 = 0;
		for (auto &e : ev) { if (e=="MAINT_5S") n5++; if (e=="MAINT_10S") n10++; }
		check(n5 == 1 && n10 == 1, "thresholds fire exactly once each");
	}

	/* Released before 5s -> nothing happens. */
	check(eq(run({{true,100},{false,150},{true,3000},{false,100}}), {}),
	      "double-tap-hold released at 3s -> no events");

	printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
	return failures != 0;
}
