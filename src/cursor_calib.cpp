#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <math.h>

static float cal_blend(float old_v, float new_v)
{
    return old_v + CAL_BLEND_ALPHA * (new_v - old_v);
}

cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          float bottom_candidate, bool bottom_valid)
{
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.bottom_candidate = bottom_valid ? bottom_candidate : 0.0f;
    r.sweep_deg        = bottom_valid ? (bottom_candidate - top_now) : 0.0f;
    r.shadow_bottom    = 0.0f;

    bool top_plausible = (top_now <= CAL_TOP_MAX);
    bool sweep_ok      = bottom_valid && ((bottom_candidate - top_now) >= CAL_SWEEP_MIN_DEG);
    bool ritual_complete = top_plausible && bottom_valid && sweep_ok;

    /* ---- Cold-start (RAM-only, no prior calibration). ---- */
    if (!have_calib) {
        if (ritual_complete) {
            r.decision = CAL_SEED; r.reason = CAL_REASON_OK;
            r.new_top = top_now; r.new_bottom = bottom_candidate; r.apply = true;
        } else {
            r.decision = CAL_REJECT; r.reason = CAL_REASON_COLD_START_DEFAULT; r.apply = false;
        }
        return r;
    }

    /* ---- Have prior calibration. ---- */
    if (ritual_complete) {
        bool top_moved = fabsf(top_now          - prior_top)    >= CAL_MIN_DELTA;
        bool bot_moved = fabsf(bottom_candidate  - prior_bottom) >= CAL_MIN_DELTA;
        if (!top_moved && !bot_moved) {
            r.decision = CAL_REJECT; r.reason = CAL_REASON_BELOW_MIN_DELTA; r.apply = false;
        } else {
            r.decision = CAL_ADOPT; r.reason = CAL_REASON_OK;
            r.new_top    = cal_blend(prior_top,    top_now);
            r.new_bottom = cal_blend(prior_bottom, bottom_candidate);
            r.apply      = true;
        }
        return r;
    }

    /* Ritual incomplete. Mid-air (plausible top, NO rest captured this session) ->
     * shadow only: log the would-be coupled translation, APPLY NOTHING. */
    if (top_plausible && !bottom_valid) {
        r.decision = CAL_SHADOW_TRANSLATE; r.reason = CAL_REASON_MID_AIR_SHADOW;
        r.shadow_bottom = top_now + (prior_bottom - prior_top);  /* preserve prior span */
        r.apply = false;
        return r;
    }

    /* Otherwise a hard reject; pick the most specific reason. */
    r.decision = CAL_REJECT; r.apply = false;
    if (!top_plausible)    r.reason = CAL_REASON_IMPLAUSIBLE_TOP;
    else if (bottom_valid) r.reason = CAL_REASON_INSUFFICIENT_SWEEP; /* rest exists, sweep too small */
    else                   r.reason = CAL_REASON_NO_PLATEAU;         /* unreachable (mid-air covers it) */
    return r;
}
