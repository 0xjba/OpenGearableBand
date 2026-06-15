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
void cursor_track_start(float vert_deg, float yaw_deg);

/* One tick (~100 Hz).
 *   vert_deg : angle-from-vertical (deg), the Y driver.  Use the GRAVITY-based
 *     inclination (acos(|gx|/|g|)), NOT Euler pitch -- hardware showed Euler
 *     pitch saturates at high roll (roll-contaminated) and freezes the cursor
 *     over the lower third of the stroke; the gravity inclination is immune.
 *   yaw_deg  : fused heading (orientation_get, bias-tracked, at-rest re-zeroed),
 *     the X driver.  X is a LINEAR angle->px map of the yaw delta (a forearm
 *     sweep about the elbow), so a given sweep angle moves a fixed screen
 *     distance regardless of speed.  No absolute heading reference exists on a
 *     6-axis IMU, so X is pseudo-absolute: cone-gated near vertical and
 *     re-anchored (prev resynced) at every at-rest pause to bound gyro drift.
 *   at_rest  : orientation stillness flag.
 *   shadow   : sqrt(gy^2+gz^2) from the GRAVITY-LPF (NEVER the fused
 *     quaternion -- it is the distance-from-vertical the cone gate needs; yaw
 *     degenerates at gimbal where shadow -> 0).
 * Writes the relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float vert_deg, float yaw_deg, bool at_rest,
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
float cursor_track_vert_bottom(void);     /* current bottom anchor (deg), clamped  */

/* Set both absolute-Y anchors at runtime (natural calibration, RAM-only).  MUST
 * be called BEFORE cursor_track_start() at entry: the entry slam is sized from
 * the anchors, so a stale anchor would slam the recalibrating entry against the
 * previous mount.  Does NOT itself re-slam.  See
 * docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md. */
void cursor_track_set_anchors(float vert_top, float vert_bottom);

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_TRACK_H */
