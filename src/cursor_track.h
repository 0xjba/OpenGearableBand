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
void cursor_track_start(float pitch_deg, float roll_deg);

/* One tick (~100 Hz).  pitch/roll: fused (orientation_get) angles in deg.
 * at_rest: orientation stillness flag.  shadow: sqrt(gy^2+gz^2) from the
 * GRAVITY-LPF (NEVER the fused quaternion -- inside the cone the fused roll
 * drifts on gyro alone; the gate needs the raw gravity signal).  Writes the
 * relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float pitch_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy);

/* End the session (clears latched state). */
void cursor_track_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_TRACK_H */
