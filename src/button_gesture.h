/*
 * button_gesture -- single-button gesture recogniser.
 *
 * PURE STATE MACHINE: no Zephyr, no I/O, no clock of its own. Feed it a
 * debounced pressed/released sample plus a millisecond timestamp and it returns
 * events. That makes it host-testable (tests/test_button_gesture.cpp), which is
 * how the timing rules below are verified rather than asserted.
 *
 * GESTURE MAP (see docs/firmware-resilience-playbook.md for the rationale):
 *
 *   Everyday, works unplugged:
 *     press & hold      -> HOLD_START on press, HOLD_END on release.
 *                          ANY duration. No timeouts, no escalation. A long
 *                          question must never trip a maintenance action.
 *     double tap        -> DOUBLE_TAP (caller toggles recording)
 *     single tap        -> SINGLE_TAP (reserved; no action assigned yet)
 *
 *   Maintenance, caller gates on USB being connected:
 *     double-tap-then-hold -> MAINT_5S at 5s, MAINT_10S at 10s.
 *                          Entry begins with a TAP, so a normal talk press --
 *                          which begins with a HOLD -- can never reach it.
 *
 * WHY HOLD IS NOT REPORTED ON PRESS-DOWN: at the instant of contact a press and
 * a tap are indistinguishable. Reporting a hold immediately would make every tap
 * open and close a voice session, so a double tap would fire voice twice. We
 * wait HOLD_CONFIRM_MS of continuous contact first. That is the whole latency
 * budget for starting to talk, and it is well under the double-tap window.
 */
#ifndef BUTTON_GESTURE_H_
#define BUTTON_GESTURE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* POLICY, not verified hardware facts -- chosen for feel, tune with real users.
 * HOLD_CONFIRM_MS is the talk-start latency; keep it well below DOUBLE_GAP_MS
 * or a deliberate double tap starts registering as a hold. */
#define BG_HOLD_CONFIRM_MS   200
#define BG_DOUBLE_GAP_MS     300
#define BG_MAINT_5S_MS      5000
#define BG_MAINT_10S_MS    10000

enum button_event {
	BG_EVT_NONE = 0,
	BG_EVT_HOLD_START,   /* voice capture begins */
	BG_EVT_HOLD_END,     /* voice capture ends   */
	BG_EVT_SINGLE_TAP,   /* reserved             */
	BG_EVT_DOUBLE_TAP,   /* recording toggle     */
	BG_EVT_MAINT_5S,     /* pairing (USB only)   */
	BG_EVT_MAINT_10S,    /* restart (USB only)   */
};

enum bg_phase {
	BG_IDLE = 0,
	BG_PRESS_PENDING,    /* down, not yet long enough to be a hold      */
	BG_HOLDING,          /* confirmed hold -- voice session active      */
	BG_TAP_WAIT,         /* one tap seen, waiting for a possible second */
	BG_SECOND_PENDING,   /* second press down, tap-or-hold undecided    */
	BG_MAINT_HOLDING,    /* double-tap-then-hold in progress            */
};

struct button_gesture {
	enum bg_phase phase;
	uint32_t      t_edge;      /* when the current phase began   */
	bool          last;        /* previous debounced sample      */
	bool          fired_5s;    /* MAINT_5S already emitted       */
	bool          fired_10s;   /* MAINT_10S already emitted      */
};

void button_gesture_init(struct button_gesture *g);

/* Feed one debounced sample. Returns at most one event per call; call at a
 * period comfortably shorter than BG_HOLD_CONFIRM_MS. */
enum button_event button_gesture_update(struct button_gesture *g,
					bool pressed, uint32_t now_ms);

const char *button_event_str(enum button_event e);

#ifdef __cplusplus
}
#endif
#endif /* BUTTON_GESTURE_H_ */
