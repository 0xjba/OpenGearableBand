#ifndef CURSOR_CALIB_H
#define CURSOR_CALIB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Natural (entry-time) cursor calibration -- PURE decision engine.
 * No Zephyr / no I/O / no threads, so it is host-unit-testable like
 * cursor_track.  The CALLER (gesture_mode) owns the vert history buffer,
 * linearises it into chronological order (oldest -> newest), and applies
 * the verdict via cursor_track_set_anchors().
 * Geometry: vert = acos(|gx|/|g|); SMALL when raised (screen top),
 * LARGE when flat (screen bottom).  So bottom > top in degrees.
 * See docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md.
 */

typedef enum {
    CAL_SEED,             /* cold-start: both anchors seeded from this entry      */
    CAL_ADOPT,            /* both anchors trusted, blended toward the new capture  */
    CAL_SHADOW_TRANSLATE, /* mid-air entry: NOTHING applied; shadow_bottom logged  */
    CAL_REJECT,           /* no change                                             */
} cursor_calib_decision_t;

typedef enum {
    CAL_REASON_OK,
    CAL_REASON_COLD_START_DEFAULT, /* cold-start + incomplete ritual -> run on defaults */
    CAL_REASON_NO_PLATEAU,
    CAL_REASON_INSUFFICIENT_SWEEP, /* a low-var rest existed but too close to top        */
    CAL_REASON_IMPLAUSIBLE_TOP,
    CAL_REASON_BELOW_MIN_DELTA,
    CAL_REASON_MID_AIR_SHADOW,
} cursor_calib_reason_t;

typedef struct {
    cursor_calib_decision_t decision;
    cursor_calib_reason_t   reason;
    bool  apply;             /* true => caller MUST cursor_track_set_anchors(new_top,new_bottom) */
    float new_top;           /* anchor to apply (valid when apply==true)         */
    float new_bottom;        /* anchor to apply (valid when apply==true)         */
    /* Diagnostics for the [CAL] log line: */
    bool  plateau_found;
    float plateau_var;       /* variance of the chosen plateau (deg^2)           */
    int   plateau_n;         /* length of the chosen plateau (samples)           */
    float bottom_candidate;  /* median vert of the plateau (deg); 0 if none      */
    float sweep_deg;         /* bottom_candidate - top_now (deg); 0 if no plateau */
    float shadow_bottom;     /* would-be translate (CAL_SHADOW_TRANSLATE only)   */
} cursor_calib_result_t;

/*
 * Decide whether/how to recalibrate the cursor anchors for this entry.
 *   have_calib    : false on the first entry since boot (RAM-only, no prior).
 *   prior_top/bottom : current anchors (defaults on cold start).
 *   top_now       : the snap-moment inclination (deg) = current_vert_deg().
 *   vert_chrono   : n samples of vert (deg) in CHRONOLOGICAL order (oldest first).
 * Pure; returns a verdict. Never mutates global state.
 */
cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n);

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_CALIB_H */
