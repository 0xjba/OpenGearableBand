# Absolute-Y Air-Mouse Cursor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the air-mouse cursor's vertical axis with an absolute,
gravity-anchored map (a given wrist inclination → a fixed screen height), while
the horizontal axis stays relative.

**Architecture:** `cursor_track` becomes a slam-then-servo state machine on Y:
on entry it emits a saturating delta burst to clamp the cursor at the top edge
(a manufactured known position), then servos the cursor toward
`target_counts = GAIN_Y × (vert − vert_top)`, re-pinning to the top edge with
hysteresis when fully raised. X keeps its existing relative roll path. All Y
logic is a pure function of inputs + internal state, so it is host-unit-tested
with g++; only telemetry + wiring touch hardware.

**Tech Stack:** Zephyr RTOS / nRF52840 (XIAO Sense), C++. Pure module
(`cursor_track`) host-tested with g++; firmware verified by `west build` +
hardware log observation (no on-target unit harness). Grounding spec:
`docs/superpowers/specs/2026-06-11-absolute-y-cursor-design.md`.

**Host test command (used in every test step):**
```
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
```
**Firmware build command:**
```
./build.sh
```

---

## File Structure

- `src/gesture_thresholds.h` — MODIFY. Repurpose `CURSOR_GAIN_Y` to counts/deg;
  add the absolute-Y constants (span, top default/max, bottom max, slam
  margin/floor/cap, pin hysteresis).
- `src/cursor_track.{h,cpp}` — MODIFY. Y axis: replace the relative `Δvert→dy`
  with the slam-then-servo state machine + entry calibration + top re-pin. X
  axis: unchanged relative roll. New introspection getters for telemetry.
- `tests/test_cursor_track.cpp` — REWRITE. The relative-Y assertions no longer
  hold; replace with absolute-Y/slam/servo/calibration/re-pin tests.
- `src/gesture_mode.cpp` — MODIFY. Already passes `vert`/`roll` to
  `cursor_track`; extend the `[CURSOR]` telemetry to log the new internal state.

Out of scope (per spec §10): absolute-X, true absolute HID, the §7 bottom
self-calibration upgrade, NVS persistence of `vert_top` (RAM last-good only),
clicks, SURFACE.

---

## Task 1: Constants

**Files:**
- Modify: `src/gesture_thresholds.h` (the cursor section, currently
  `CURSOR_GAIN_Y` etc. ~lines 283–304)

- [ ] **Step 1: Replace the relative-Y gain comment + value and add the absolute-Y constants**

In `src/gesture_thresholds.h`, replace the existing `CURSOR_GAIN_Y` /
`CURSOR_GAIN_X` block (the lines defining `CURSOR_GAIN_Y` and `CURSOR_GAIN_X`)
with:

```c
/* --- Absolute-Y cursor (gravity-anchored vertical) --------------------------
 * Y is no longer a relative px/deg gain.  CURSOR_GAIN_Y is now COUNTS/DEG: the
 * servo target is GAIN_Y * (vert - vert_top), tuned on '}' / '{' so a full
 * comfortable down-sweep fills the screen (this folds in the unknown host
 * count->pixel scale).  X stays relative px/deg.  See
 * docs/superpowers/specs/2026-06-11-absolute-y-cursor-design.md. */
#define CURSOR_GAIN_Y                   30.0f   /* counts/deg (servo); tune on HW */
#define CURSOR_GAIN_X                   8.0f    /* px/deg (relative roll->X)       */

/* Vertical map calibration (deg). vert = acos(|gx|/|g|), 0=vertical..90=flat. */
#define CURSOR_VERT_SPAN_DEG            40.0f   /* nominal comfortable down-range  */
#define CURSOR_VERT_TOP_DEFAULT         15.0f   /* seed/fallback top anchor        */
#define CURSOR_VERT_TOP_MAX             25.0f   /* reject entry above this (lazy raise) -> fallback */
#define CURSOR_VERT_BOTTOM_MAX          60.0f   /* comfort clamp on implied bottom */

/* Slam (manufactured edge clamp).  Over-travel is free (OS clamps); undershoot
 * poisons registration -> a GAIN_Y-INDEPENDENT floor guarantees the edge even
 * untuned. */
#define CURSOR_SLAM_MARGIN              2.0f    /* x max_counts                    */
#define CURSOR_SLAM_FLOOR_COUNTS        6000.0f /* absolute min over-travel        */
#define CURSOR_SLAM_MAX_REPORTS         80      /* generous burst cap (mis-tune)   */

/* Top re-pin hysteresis band (deg around vert_top). LEAVE >= ENTER. */
#define CURSOR_PIN_ENTER_DEG            3.0f
#define CURSOR_PIN_LEAVE_DEG            6.0f
```

Keep the existing `CURSOR_ROLL_SHADOW_INVALIDATE`, `CURSOR_ROLL_SHADOW_REVALIDATE`,
`CURSOR_FREEZE_RELEASE_DELTA`, `CURSOR_MAX_DELTA_DEG` definitions unchanged
(they now govern X only).

- [ ] **Step 2: Verify it still compiles (host)**

Run: `g++ -std=c++11 -Isrc -fsyntax-only -x c++ src/gesture_thresholds.h`
Expected: no output (clean). The header is self-contained.

- [ ] **Step 3: Commit**

```bash
git add src/gesture_thresholds.h
git commit -m "feat(cursor): absolute-Y constants (GAIN_Y now counts/deg, slam floor, pin hysteresis)"
```

---

## Task 2: vert→target map + servo (no slam yet)

**Files:**
- Modify: `src/cursor_track.{h,cpp}`
- Test: `tests/test_cursor_track.cpp` (rewrite begins here)

- [ ] **Step 1: Replace the test file with the map+servo tests**

Overwrite `tests/test_cursor_track.cpp` with:

```cpp
#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

/* Drive past any entry slam: call update until the slam burst is exhausted,
 * holding vert/roll fixed so only the slam contributes.  Returns ticks used. */
static int drain_slam(float vert, float roll)
{
    float dx, dy;
    int n = 0;
    for (; n < CURSOR_SLAM_MAX_REPORTS + 2; n++) {
        if (!cursor_track_is_slamming()) break;
        cursor_track_update(vert, roll, false, 0.0f, &dx, &dy);
    }
    return n;
}

int main(void)
{
    float dx, dy;
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f; /* shadow: X valid */

    /* --- Map + servo (Task 2) --- */

    /* M1: after the entry slam drains, the cursor estimate is pinned to top
     * (cur_y == 0) and holding vert==vert_top yields no Y motion. */
    cursor_track_start(15.0f, 50.0f);   /* vert_top = 15 */
    drain_slam(15.0f, 50.0f);
    CHECK(fabsf(cursor_track_cur_y() - 0.0f) < 1e-3f);
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                       /* target 0, cur_y 0 -> err 0 */

    /* M2: servo drives toward target = GAIN_Y*(vert-vert_top), int8-clamped. */
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    /* vert 25 -> target = 30*(25-15)=300 counts; first tick clamps to +127. */
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    /* keep holding vert=25: converges (300 = 127+127+46). */
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 46.0f) < 1e-3f);
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                       /* converged */
    CHECK(fabsf(cursor_track_cur_y() - 300.0f) < 1e-3f);

    /* M3: target clamps to max_counts at the comfort bottom.
     * vert_top=15, span=40 -> vert_bottom=min(55,60)=55; max=30*40=1200. */
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - 1200.0f) < 1e-3f); /* clamped, not 30*(90-15) */

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 2: Run the test to verify it fails to COMPILE**

Run the host test command.
Expected: COMPILE ERROR — `cursor_track_is_slamming`, `cursor_track_cur_y`
undeclared (they don't exist yet). That is the failing state.

- [ ] **Step 3: Add the introspection getters to the header**

In `src/cursor_track.h`, after the `cursor_track_get_gain` declaration, add:

```c
/* Introspection for telemetry + host tests. */
bool  cursor_track_is_slamming(void);
float cursor_track_cur_y(void);       /* Y position estimate, counts from top  */
float cursor_track_vert_top(void);    /* captured top anchor (deg)             */
```

- [ ] **Step 4: Rewrite the Y axis in cursor_track.cpp (map + servo + slam stub)**

In `src/cursor_track.cpp`, replace the static-state block (the
`s_prev_vert … s_started` group) with:

```c
static float s_prev_roll      = 0.0f;
static bool  s_roll_valid     = false;
static bool  s_started        = false;

/* Absolute-Y state. */
static float s_vert_top       = CURSOR_VERT_TOP_DEFAULT; /* top anchor (deg)        */
static float s_vert_top_good  = CURSOR_VERT_TOP_DEFAULT; /* RAM last-good fallback  */
static float s_cur_y          = 0.0f;                    /* counts from top         */
static int   s_slam_remaining = 0;                       /* >0 => slamming          */
static int   s_slam_sign      = -1;                      /* -1 up(top), +1 down     */
static bool  s_at_top         = false;                   /* re-pin hysteresis latch */
```

Then replace the entire `cursor_track_start` and `cursor_track_update`
functions with:

```c
/* vert_bottom = top + comfort span, clamped to the comfort maximum. */
static float vert_bottom(void)
{
    float vb = s_vert_top + CURSOR_VERT_SPAN_DEG;
    if (vb > CURSOR_VERT_BOTTOM_MAX) vb = CURSOR_VERT_BOTTOM_MAX;
    return vb;
}

static float max_counts(void)
{
    float mc = s_gain_y * (vert_bottom() - s_vert_top);
    return (mc > 0.0f) ? mc : 0.0f;
}

static float target_counts(float vert_deg)
{
    float t  = s_gain_y * (vert_deg - s_vert_top);
    float mc = max_counts();
    if (t < 0.0f) t = 0.0f;
    if (t > mc)   t = mc;
    return t;
}

/* Arm a slam burst.  sign -1 => up (to top), +1 => down (to bottom).  Size is a
 * GAIN_Y-independent floor max'd with margin*max_counts so it reaches the edge
 * even untuned; over-travel is harmless (OS clamps). */
static void start_slam(int sign)
{
    float counts = CURSOR_SLAM_MARGIN * max_counts();
    if (counts < CURSOR_SLAM_FLOOR_COUNTS) counts = CURSOR_SLAM_FLOOR_COUNTS;
    int n = (int)ceilf(counts / 127.0f);
    if (n > CURSOR_SLAM_MAX_REPORTS) n = CURSOR_SLAM_MAX_REPORTS;
    s_slam_remaining = n;
    s_slam_sign      = sign;
}

void cursor_track_start(float vert_deg, float roll_deg)
{
    /* Capture top anchor with a lazy-raise sanity clamp -> RAM last-good. */
    if (vert_deg <= CURSOR_VERT_TOP_MAX) {
        s_vert_top      = vert_deg;
        s_vert_top_good = vert_deg;
    } else {
        s_vert_top = s_vert_top_good;
    }
    s_prev_roll  = roll_deg;
    s_roll_valid = false;
    s_cur_y      = 0.0f;
    s_at_top     = true;     /* we enter at the top */
    start_slam(-1);          /* entry slam to the top edge */
    s_started    = true;
}

void cursor_track_update(float vert_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
    if (!s_started) { *out_dx = 0.0f; *out_dy = 0.0f; return; }

    bool slamming = (s_slam_remaining > 0);   /* snapshot before the Y branch decrements */

    /* ---- Y axis: slam-then-servo (servo owns Y stillness; no freeze gate) ---- */
    float dy = 0.0f;
    if (slamming) {
        dy = (float)(s_slam_sign * 127);   /* emit ONLY the slam burst */
        s_slam_remaining--;
        if (s_slam_remaining == 0) {
            s_cur_y = (s_slam_sign < 0) ? 0.0f : max_counts();  /* pinned at edge */
        }
    } else {
        float err = target_counts(vert_deg) - s_cur_y;
        if (err >  127.0f) err =  127.0f;
        if (err < -127.0f) err = -127.0f;
        dy = err;
        s_cur_y += dy;
        /* (Task 5 inserts the top re-pin call here.) */
    }

    /* ---- X axis: relative roll (UNCHANGED), but SUPPRESSED during a slam so
     * the slam emits only its Y burst (spec §5).  prev stays synced so X has no
     * jump when tracking resumes. ---- */
    float droll = wrap180(roll_deg - s_prev_roll);
    if (fabsf(droll) > CURSOR_MAX_DELTA_DEG) droll = 0.0f;
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_roll_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_roll_valid = false;
    bool x_frozen = at_rest && (fabsf(droll) < CURSOR_FREEZE_RELEASE_DELTA);
    float dx = 0.0f;
    if (!slamming && !x_frozen && s_roll_valid) dx = s_gain_x * droll;

    *out_dx = dx;
    *out_dy = dy;
    s_prev_roll = roll_deg;
}

bool  cursor_track_is_slamming(void) { return s_slam_remaining > 0; }
float cursor_track_cur_y(void)       { return s_cur_y; }
float cursor_track_vert_top(void)    { return s_vert_top; }
```

Leave `cursor_track_stop`, `wrap180`, the gain getters/setters, and the
`static_assert` as they are. The `s_prev_vert`-based logic is fully replaced.

- [ ] **Step 5: Run the test to verify it passes**

Run the host test command.
Expected: `ALL PASS`.

- [ ] **Step 6: Commit**

```bash
git add src/cursor_track.h src/cursor_track.cpp tests/test_cursor_track.cpp
git commit -m "feat(cursor): absolute-Y map + servo + slam-to-top state machine"
```

---

## Task 3: Slam sizing — floor guarantees the edge even untuned

**Files:**
- Test: `tests/test_cursor_track.cpp`

- [ ] **Step 1: Add the slam-sizing tests before the final printf**

In `tests/test_cursor_track.cpp`, insert before the `printf(failures …)` line:

```cpp
    /* --- Slam sizing (Task 3) --- */

    /* S1: entry arms a slam, and it lasts >= the floor's worth of reports
     * regardless of a tiny GAIN_Y (undershoot would poison registration). */
    cursor_track_set_gain(8.0f, 1.0f);          /* gain_y tiny -> max_counts small */
    cursor_track_start(15.0f, 50.0f);
    CHECK(cursor_track_is_slamming());
    int floor_reports = (int)ceilf(CURSOR_SLAM_FLOOR_COUNTS / 127.0f);
    int used = drain_slam(15.0f, 50.0f);
    CHECK(used >= floor_reports);               /* not shorter than the floor */
    CHECK(used <= CURSOR_SLAM_MAX_REPORTS);      /* and capped */

    /* S2: a large tuned range scales the slam up (margin*max_counts > floor),
     * still capped. */
    cursor_track_set_gain(8.0f, 60.0f);          /* max=60*40=2400; *2=4800<6000 floor */
    cursor_track_start(15.0f, 50.0f);
    int used2 = drain_slam(15.0f, 50.0f);
    CHECK(used2 >= floor_reports);
    cursor_track_set_gain(8.0f, 200.0f);         /* max=8000; *2=16000 -> capped */
    cursor_track_start(15.0f, 50.0f);
    int used3 = drain_slam(15.0f, 50.0f);
    CHECK(used3 == CURSOR_SLAM_MAX_REPORTS);

    /* restore default gain for any later tests */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
```

- [ ] **Step 2: Run the test**

Run the host test command.
Expected: `ALL PASS` (the Task 2 implementation already satisfies these — this
task locks the floor/cap behaviour with regression tests).

- [ ] **Step 3: Commit**

```bash
git add tests/test_cursor_track.cpp
git commit -m "test(cursor): lock slam floor (untuned-safe) and report cap"
```

---

## Task 4: Entry calibration — lazy-raise sanity clamp + fallback

**Files:**
- Test: `tests/test_cursor_track.cpp`

- [ ] **Step 1: Add the calibration tests before the final printf**

Insert before `printf(failures …)`:

```cpp
    /* --- Entry calibration (Task 4) --- */

    /* C1: a normal raised entry captures vert_top live. */
    cursor_track_start(14.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 14.0f) < 1e-3f);

    /* C2: a lazy half-raise (vert above the max) is rejected -> falls back to
     * the last good capture (14 from C1), NOT the bad 35. */
    cursor_track_start(35.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 14.0f) < 1e-3f);

    /* C3: a fresh good capture updates the last-good. */
    cursor_track_start(17.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 17.0f) < 1e-3f);
    cursor_track_start(40.0f, 50.0f);            /* lazy again */
    CHECK(fabsf(cursor_track_vert_top() - 17.0f) < 1e-3f);  /* falls back to 17 */
```

- [ ] **Step 2: Run the test**

Run the host test command.
Expected: `ALL PASS` (Task 2's `cursor_track_start` already implements the
clamp+fallback; these tests lock it).

- [ ] **Step 3: Commit**

```bash
git add tests/test_cursor_track.cpp
git commit -m "test(cursor): lock lazy-raise sanity clamp + last-good fallback"
```

---

## Task 5: Top re-pin with hysteresis

**Files:**
- Modify: `src/cursor_track.cpp` (add the re-pin call into the servo branch)
- Test: `tests/test_cursor_track.cpp`

- [ ] **Step 1: Add the re-pin tests before the final printf**

Insert before `printf(failures …)`:

```cpp
    /* --- Top re-pin hysteresis (Task 5) --- */

    /* R1: drive down so cur_y > 0, then raise back to the top: crossing
     * vert_top+ENTER re-arms a slam (cur_y will re-pin to 0 after it drains). */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(40.0f, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_cur_y() > 100.0f);        /* parked mid-screen */
    /* still inside the leave band? we are at 40 (>> top+LEAVE) so latch is off;
     * now raise to the top: */
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);  /* <= top+ENTER -> re-pin */
    CHECK(cursor_track_is_slamming());
    drain_slam(15.0f, 50.0f);
    CHECK(fabsf(cursor_track_cur_y() - 0.0f) < 1e-3f);

    /* R2: hysteresis — once latched at top, a small dip below LEAVE must NOT
     * re-fire the slam every tick (no chatter). */
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);   /* latched at top */
    /* nudge to within [ENTER, LEAVE] of top -> still latched, no new slam */
    cursor_track_update(15.0f + (CURSOR_PIN_ENTER_DEG + CURSOR_PIN_LEAVE_DEG) * 0.5f,
                        50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());

    /* R3: leaving past vert_top+LEAVE then returning re-arms exactly once. */
    cursor_track_update(15.0f + CURSOR_PIN_LEAVE_DEG + 5.0f, 50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());           /* left the band, no slam on leave */
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);   /* re-enter -> re-pin */
    CHECK(cursor_track_is_slamming());
```

- [ ] **Step 2: Run the test to verify it fails**

Run the host test command.
Expected: FAIL — R1 (`cursor_track_is_slamming()` after raising to top) fails
because the servo branch does not yet trigger a re-pin; the latch logic is absent.

- [ ] **Step 3: Add the re-pin logic to the servo branch**

In `src/cursor_track.cpp`, add this helper just above `cursor_track_update`:

```c
/* Top re-pin: latched hysteresis around the top anchor.  Entering the
 * [vert_top, vert_top+ENTER] band re-arms a to-top slam ONCE; the latch clears
 * only after leaving past vert_top+LEAVE, so a hover near the edge can't
 * chatter.  LEAVE >= ENTER (see gesture_thresholds.h). */
static void update_top_repin(float vert_deg)
{
    if (!s_at_top && vert_deg <= s_vert_top + CURSOR_PIN_ENTER_DEG) {
        s_at_top = true;
        start_slam(-1);
    } else if (s_at_top && vert_deg > s_vert_top + CURSOR_PIN_LEAVE_DEG) {
        s_at_top = false;
    }
}
```

Then, inside `cursor_track_update`, in the `else` (servo) branch, **replace the
placeholder line** `/* (Task 5 inserts the top re-pin call here.) */` with the
actual call:

```c
        s_cur_y += dy;
        update_top_repin(vert_deg);   /* may re-arm a to-top slam */
```

(The re-pin is only evaluated in the servo branch — never while a slam is in
progress — so a slam is never interrupted.)

- [ ] **Step 4: Run the test to verify it passes**

Run the host test command.
Expected: `ALL PASS`.

- [ ] **Step 5: Add a static_assert that LEAVE >= ENTER**

In `src/cursor_track.cpp`, just below the existing `static_assert` near the top,
add:

```c
static_assert(CURSOR_PIN_LEAVE_DEG >= CURSOR_PIN_ENTER_DEG,
              "CURSOR_PIN_LEAVE_DEG must be >= ENTER (re-pin hysteresis)");
```

- [ ] **Step 6: Run the test again (confirm still green)**

Run the host test command.
Expected: `ALL PASS`.

- [ ] **Step 7: Commit**

```bash
git add src/cursor_track.cpp tests/test_cursor_track.cpp
git commit -m "feat(cursor): top re-pin with latched hysteresis (no edge chatter)"
```

---

## Task 6: X axis preserved + Y stillness owned by servo

**Files:**
- Test: `tests/test_cursor_track.cpp`

- [ ] **Step 1: Add the X-preservation and Y-stillness tests before the final printf**

Insert before `printf(failures …)`:

```cpp
    /* --- X relative preserved; Y stillness via servo (Task 6) --- */

    /* X1: roll still drives dx (relative px/deg) with the cone gate. */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);        /* seed roll, valid */
    cursor_track_update(15.0f, 53.0f, false, V, &dx, &dy);        /* droll=+3 */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 3.0f) < 1e-3f);

    /* X2: cone gate still gates X (shadow below INVALIDATE -> dx 0). */
    cursor_track_update(15.0f, 56.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy);
    CHECK(dx == 0.0f);

    /* Y1: Y has NO at_rest freeze — when the wrist is still the servo already
     * yields dy=0 (target static), and at_rest must not change Y behaviour.
     * After converging to a target, dy is 0 with at_rest both false and true. */
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                     /* converged, still */
    cursor_track_update(25.0f, 50.0f, true,  V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                     /* at_rest=true changes nothing on Y */

    /* Y2: a fresh vert step still moves Y even with at_rest=true (servo, not
     * freeze, owns Y -> at_rest never blocks a real target change). */
    cursor_track_update(30.0f, 50.0f, true, V, &dx, &dy);
    CHECK(dy > 0.0f);
```

- [ ] **Step 2: Run the test**

Run the host test command.
Expected: `ALL PASS` (Task 2's update already routes Y through the servo with no
at_rest gate, and keeps the X freeze/cone path — these tests lock that the
stillness authority split is correct).

- [ ] **Step 3: Commit**

```bash
git add tests/test_cursor_track.cpp
git commit -m "test(cursor): X relative preserved; servo (not at_rest) owns Y stillness"
```

---

## Task 7: Wire telemetry + firmware build + hardware verify

**Files:**
- Modify: `src/gesture_mode.cpp` (the `[CURSOR]` telemetry block, ~lines
  994–1018, where `cursor_track_update` is called)

- [ ] **Step 1: Extend the [CURSOR] telemetry with the new internal state**

In `src/gesture_mode.cpp`, find the throttled `[CURSOR]` `LOG_INF` in
`gesture_mode_update_gyro` (it currently logs `pitch vert roll shadow at_rest
dx dy`). Replace that `LOG_INF(...)` call with:

```c
            LOG_INF("[CURSOR] vert=%d vtop=%d cur_y=%d slam=%d roll=%d "
                    "shadow=%d dx=%d dy=%d",
                    (int)vert, (int)cursor_track_vert_top(),
                    (int)cursor_track_cur_y(), (int)cursor_track_is_slamming(),
                    (int)ori.roll_deg, (int)shadow, (int)dx, (int)dy);
```

Also change the telemetry trigger so slam ticks are visible: replace the
condition `if ((dx != 0.0f || dy != 0.0f) && (++cursor_tel_ctr % 10 == 0))` with:

```c
        if ((dx != 0.0f || dy != 0.0f || cursor_track_is_slamming()) &&
            (++cursor_tel_ctr % 10 == 0)) {
```

- [ ] **Step 2: Build the firmware**

Run: `./build.sh`
Expected: clean build, `==> Flashable artifact: build/zephyr/zephyr.uf2`.

- [ ] **Step 3: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "feat(cursor): absolute-Y telemetry (vert_top, cur_y, slam state)"
```

- [ ] **Step 4: Hardware verification (manual, accel OFF)**

Prerequisite: macOS System Settings → Mouse → Advanced → Pointer acceleration
**off** (spec §2).

Flash: `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`, reconnect, confirm
`HID: link encrypted`. Enter air-mouse (`t` while raised). Verify against the
spec §9 acceptance tests:

1. **Slam-undershoot test FIRST:** fully raise → the cursor must **visibly
   clamp at the top edge with margin** (sticks there). If it stops short,
   increase `GAIN_Y` with `}` until it clamps hard. Until this passes, all
   other Y observations are suspect.
2. **Descent:** lower the wrist → the cursor descends and tracks; tune `}`/`{`
   so a comfortable down-sweep fills the screen.
3. **Determinism:** raise fully (re-pin to top), lower to the same inclination
   from two different entries → cursor lands at ~the same height.
4. **Suppression:** move the wrist during the entry slam → the slam still
   reaches the top (user motion doesn't derail it).
5. **X unaffected:** twisting still moves horizontally, cone-gated near vertical.

Report the chosen `GAIN_Y` (`Y=` in the gain log) once tuned; it becomes the
new default. **Do not claim success until observed on hardware.**

---

## Self-Review

**1. Spec coverage:**
- §2 accel-off prerequisite → Task 7 Step 4 prerequisite + acceptance tests. ✓
- §3 architecture (slam-then-servo, X relative) → Task 2. ✓
- §4 map + GAIN_Y-only tunable → Task 2 (`target_counts`, `max_counts`). ✓
- §4.1 cross-session vert_top check → telemetry logs `vtop` every entry (Task 7
  Step 1); the multi-day eyeball is a manual user activity, not code. ✓
- §4.2 lazy-raise sanity clamp + last-good fallback → Task 4. ✓ (NVS persistence
  explicitly out of scope; RAM last-good per spec-scope note.)
- §4.3 far-end comfort clamp → Task 2 (`vert_bottom` clamps to
  `CURSOR_VERT_BOTTOM_MAX`); Task 2 test M3. ✓
- §5 slam mechanics: int8 burst, floor, suppression → Task 2 + Task 3. ✓
- §6 re-pin hysteresis → Task 5. ✓
- §6b out-of-span clamp + servo owns Y stillness → Task 2 (clamp) + Task 6
  (stillness authority). ✓
- §7 A→B upgrade → out of scope (designated future), correctly excluded. ✓
- §8 constants → Task 1. ✓
- §9 telemetry + acceptance tests → Task 7. ✓

**2. Placeholder scan:** No TBD/“handle errors”/“similar to”/uncoded steps —
every code step shows full code. ✓

**3. Type consistency:** `cursor_track_start(float,float)`,
`cursor_track_update(float,float,bool,float,float*,float*)`,
`cursor_track_is_slamming()→bool`, `cursor_track_cur_y()→float`,
`cursor_track_vert_top()→float`, `start_slam(int)`, `target_counts(float)`,
`max_counts()`, `vert_bottom()`, `update_top_repin(float)` — names/signatures
match across Tasks 2/5/7 and tests. Constants `CURSOR_GAIN_Y`,
`CURSOR_VERT_SPAN_DEG`, `CURSOR_VERT_TOP_DEFAULT/MAX`, `CURSOR_VERT_BOTTOM_MAX`,
`CURSOR_SLAM_MARGIN/FLOOR_COUNTS/MAX_REPORTS`, `CURSOR_PIN_ENTER_DEG/LEAVE_DEG`
referenced consistently with Task 1. ✓
