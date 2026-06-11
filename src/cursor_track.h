#ifndef CURSOR_TRACK_H
#define CURSOR_TRACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Air-mouse pointing: turn the drift-free orientation into a relative mouse
 * delta.  PURE -- no Zephyr / cursor_pipeline deps, so it is host-unit-
 * testable; the CALLER injects the returned delta into cursor_pipeline.
 * (The spec draft showed cursor_track_update calling cursor_pipeline_inject_
 * motion directly; the plan revised it to out-params so the module stays
 * caller-injectable and testable without the pipeline.)
 * Relative/rate model.  Stateful singleton (one band / one cursor).
 * See docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md.
 */

/* Begin a session: capture the reference angles (no entry jump) and gate X
 * until the shadow clearly clears the cone. */
void cursor_track_start(float vert_deg, float roll_deg);

/* One tick (~100 Hz).
 *   vert_deg : angle-from-vertical (deg), the Y driver.  Use the GRAVITY-based
 *     inclination (acos(|gx|/|g|)), NOT Euler pitch -- hardware showed Euler
 *     pitch saturates at high roll (roll-contaminated) and freezes the cursor
 *     over the lower third of the stroke; the gravity inclination is immune.
 *   roll_deg : fused roll (orientation_get), the X driver.
 *   at_rest  : orientation stillness flag.
 *   shadow   : sqrt(gy^2+gz^2) from the GRAVITY-LPF (NEVER the fused
 *     quaternion -- inside the cone the fused roll drifts on gyro alone; the
 *     gate needs the raw gravity signal).
 * Writes the relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float vert_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy);

/* End the session (clears latched state). */
void cursor_track_stop(void);

/* Live gain tuning (px per deg of wrist angle).  Initialised from
 * CURSOR_GAIN_X/Y; the serial console adjusts these on-device so the
 * reachable span (gain * usable-wrist-range) can be dialled in without a
 * rebuild.  X gain may be negated to flip the X axis.  Y gain MUST remain
 * positive: a negative GAIN_Y zeroes max_counts() and freezes the servo
 * (target_counts always returns 0); flip Y by other means if ever needed. */
void  cursor_track_set_gain(float gain_x, float gain_y);
void  cursor_track_get_gain(float *gain_x, float *gain_y);

/* Introspection for telemetry + host tests. */
bool  cursor_track_is_slamming(void);
float cursor_track_cur_y(void);           /* Y position estimate, counts from top  */
float cursor_track_target_counts(void);   /* servo target, counts from top         */
float cursor_track_vert_top(void);        /* captured top anchor (deg)             */

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_TRACK_H */
