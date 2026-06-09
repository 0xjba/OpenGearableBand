#ifndef GESTURE_POSES_H
#define GESTURE_POSES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pose identifiers.  POSE_NONE means no canonical pose currently
 * matched. */
typedef enum {
    POSE_NONE = 0,
    POSE_AIR_MOUSE,     /* forearm raised forward, band volar facing
                         * screen (away from user's face) */
    POSE_DICTATION,     /* forearm raised + rotated, band volar facing
                         * user's mouth */
    POSE_SURFACE,       /* wrist horizontal, band volar facing up
                         * (palm-down rest) */
    POSE_COUNT
} pose_id_t;

/* Canonical pose definition.  Gravity vector points DOWNWARD when
 * the wrist is in the canonical pose; we store the expected gravity
 * vector in band-frame g-units (±9.81 m/s^2 normalised to ±1.0).
 *
 * tolerance_cos is the minimum cosine of the angle between observed
 * gravity and canonical gravity to still count as a match.  Higher
 * value = tighter tolerance.  cos(30°) ≈ 0.866, cos(20°) ≈ 0.940.
 *
 * Numbers below are hand-tuned defaults for the developer's current
 * setup.  Productionization will replace these with per-user
 * calibration values from NVS. */
typedef struct {
    pose_id_t id;
    float gx;             /* canonical gravity x in band frame, g-units */
    float gy;             /* canonical gravity y */
    float gz;             /* canonical gravity z */
    float tolerance_cos;  /* min cos(angle) to canonical for match */
    const char *name;     /* for logging */
} canonical_pose_t;

/* Compute the match score for a given pose, given observed gravity
 * vector in band frame.  Returns continuous score in [0, 1]:
 *   - 1.0 = observed gravity is exactly the canonical
 *   - 0.0 = observed is outside the tolerance cone
 *   - intermediate = continuous falloff inside the cone
 *
 * Score formula: max(0, (cos_angle - tolerance_cos) /
 *                       (1.0 - tolerance_cos)).
 * This is 0 at the cone boundary and 1 at the canonical direction. */
float pose_score(const canonical_pose_t *p,
                 float gx, float gy, float gz);

/* Find the best-matching canonical pose for the observed gravity
 * vector.  Returns POSE_NONE if no pose has score above
 * pose_match_threshold (typically 0.5).  out_score (optional)
 * receives the winning score. */
pose_id_t pose_classify_best(float gx, float gy, float gz,
                              float pose_match_threshold,
                              float *out_score);

/* Access the canonical pose definitions (read-only).  Used by the
 * pose FSM. */
const canonical_pose_t *pose_get_canonical(pose_id_t id);
const char *pose_name(pose_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_POSES_H */
