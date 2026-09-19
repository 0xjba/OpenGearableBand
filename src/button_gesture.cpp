/*
 * button_gesture -- see button_gesture.h. Pure; no Zephyr dependency.
 */
#include "button_gesture.h"

void button_gesture_init(struct button_gesture *g)
{
	*g = (struct button_gesture){};
	g->phase = BG_IDLE;
}

enum button_event button_gesture_update(struct button_gesture *g,
					bool pressed, uint32_t now_ms)
{
	const bool edge_down = pressed && !g->last;
	const bool edge_up   = !pressed && g->last;
	const uint32_t held  = now_ms - g->t_edge;
	g->last = pressed;

	switch (g->phase) {
	case BG_IDLE:
		if (edge_down) {
			g->phase  = BG_PRESS_PENDING;
			g->t_edge = now_ms;
		}
		break;

	case BG_PRESS_PENDING:
		/* Down, still ambiguous. Released early = a tap; held past the
		 * confirm window = a hold, and voice starts. */
		if (edge_up) {
			g->phase  = BG_TAP_WAIT;
			g->t_edge = now_ms;
		} else if (held >= BG_HOLD_CONFIRM_MS) {
			g->phase  = BG_HOLDING;
			g->t_edge = now_ms;
			return BG_EVT_HOLD_START;
		}
		break;

	case BG_HOLDING:
		/* Deliberately no duration checks here: a long question must never
		 * escalate into a maintenance action. */
		if (edge_up) {
			g->phase = BG_IDLE;
			return BG_EVT_HOLD_END;
		}
		break;

	case BG_TAP_WAIT:
		if (edge_down) {
			g->phase  = BG_SECOND_PENDING;
			g->t_edge = now_ms;
		} else if (held >= BG_DOUBLE_GAP_MS) {
			g->phase = BG_IDLE;
			return BG_EVT_SINGLE_TAP;
		}
		break;

	case BG_SECOND_PENDING:
		/* Second press: a quick release is the double tap; holding it on
		 * is the maintenance entry. */
		if (edge_up) {
			g->phase = BG_IDLE;
			return BG_EVT_DOUBLE_TAP;
		} else if (held >= BG_HOLD_CONFIRM_MS) {
			g->phase     = BG_MAINT_HOLDING;
			g->t_edge    = now_ms;
			g->fired_5s  = false;
			g->fired_10s = false;
		}
		break;

	case BG_MAINT_HOLDING:
		if (edge_up) {
			g->phase = BG_IDLE;
			break;
		}
		/* Thresholds fire once each, in order, so the LED can step through
		 * them and the user can see where they are before releasing. */
		if (!g->fired_10s && held >= BG_MAINT_10S_MS) {
			g->fired_10s = true;
			return BG_EVT_MAINT_10S;
		}
		if (!g->fired_5s && held >= BG_MAINT_5S_MS) {
			g->fired_5s = true;
			return BG_EVT_MAINT_5S;
		}
		break;
	}
	return BG_EVT_NONE;
}

const char *button_event_str(enum button_event e)
{
	switch (e) {
	case BG_EVT_HOLD_START:  return "HOLD_START";
	case BG_EVT_HOLD_END:    return "HOLD_END";
	case BG_EVT_SINGLE_TAP:  return "SINGLE_TAP";
	case BG_EVT_DOUBLE_TAP:  return "DOUBLE_TAP";
	case BG_EVT_MAINT_5S:    return "MAINT_5S";
	case BG_EVT_MAINT_10S:   return "MAINT_10S";
	default:                 return "NONE";
	}
}
