#include "gesture_poses.h"
#include "gesture_thresholds.h"

#include <math.h>
#include <stddef.h>

/* Hand-tuned canonical pose definitions.
 *
 * IMPORTANT (productionization): these values are tuned for the
 * developer's current setup (open PCB, duct-tape mount).  They WILL
 * need recalibration when housing arrives and per-user when the
 * calibration ritual ships.  See
 * project_productionization_gesture_calibration memory. */
static const canonical_pose_t k_canonical_poses[POSE_COUNT] = {
    /* POSE_NONE: never matches; placeholder. */
    { POSE_NONE,      0.0f,  0.0f,  0.0f,  2.0f, "NONE" },

    /* Canonicals re-measured 2026-06-10 in the FINAL mount: RIGHT
     * wrist, VOLAR, THUMB-SIDE (radial).  See hardware_wear_position
     * memory.  Each value is the normalized mean of several held 'g'
     * readings; gravity reads ~+9.8 m/s^2 on the dominant axis,
     * normalized here to a unit vector (pose_score normalizes the
     * OBSERVED vector but assumes the canonical is already unit). */

    /* POSE_EAR: raise-to-ear (phone-call) pose. Canonical = the 2026-06-17
     * measured held-ear gravity, normalized (POSE_EAR_* in gesture_thresholds.h).
     * Tight tolerance so it does NOT match a generic forward raise (the old broad
     * AIR_MOUSE cone was removed). The MODE it enters (DICTATION) is gated by
     * voice-onset in gesture_mode, not by the pose alone. */
    { POSE_EAR,     POSE_EAR_GX, POSE_EAR_GY, POSE_EAR_GZ, POSE_EAR_TOL, "EAR" },
};

float pose_score(const canonical_pose_t *p,
                 float gx, float gy, float gz)
{
    if (p == NULL || p->id == POSE_NONE) return 0.0f;

    /* Normalise observed gravity. */
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (mag < 0.5f) {
        /* Free-fall or sensor problem; can't classify. */
        return 0.0f;
    }
    float ux = gx / mag;
    float uy = gy / mag;
    float uz = gz / mag;

    /* Canonical is already normalised (unit vector). */
    float cos_angle = ux * p->gx + uy * p->gy + uz * p->gz;

    if (cos_angle <= p->tolerance_cos) {
        return 0.0f;  /* outside tolerance cone */
    }
    return (cos_angle - p->tolerance_cos) /
           (1.0f - p->tolerance_cos);
}

pose_id_t pose_classify_best(float gx, float gy, float gz,
                              float pose_match_threshold,
                              float *out_score)
{
    pose_id_t best = POSE_NONE;
    float best_score = 0.0f;

    for (int i = 1; i < POSE_COUNT; i++) {  /* skip POSE_NONE */
        float s = pose_score(&k_canonical_poses[i], gx, gy, gz);
        if (s > best_score) {
            best_score = s;
            best = (pose_id_t)i;
        }
    }

    if (out_score) *out_score = best_score;
    if (best_score < pose_match_threshold) return POSE_NONE;
    return best;
}

const canonical_pose_t *pose_get_canonical(pose_id_t id)
{
    if (id <= POSE_NONE || id >= POSE_COUNT) return NULL;
    return &k_canonical_poses[id];
}

const char *pose_name(pose_id_t id)
{
    if (id <= POSE_NONE || id >= POSE_COUNT) return "?";
    return k_canonical_poses[id].name;
}
