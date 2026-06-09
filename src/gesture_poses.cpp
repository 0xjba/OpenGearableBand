#include "gesture_poses.h"

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

    /* POSE_AIR_MOUSE: forearm raised forward, band volar facing
     * screen (away from user's face).  Gravity vector is along the
     * band's NEGATIVE Y axis (pointing toward the floor through the
     * band's edge).  Tolerance ±30°. */
    { POSE_AIR_MOUSE, 0.0f, -1.0f,  0.0f,  0.866f, "AIR_MOUSE" },

    /* POSE_DICTATION: forearm raised + rotated, band volar facing
     * user's mouth.  Compared to AIR_MOUSE this is a roll about the
     * forearm axis -- band Y axis still points downward but band X
     * has rotated; gravity now has a non-trivial X component.
     * Approx (gx, gy, gz) = (+0.5, -0.85, 0).  Tolerance ±30°.
     *
     * Empirically verify and re-tune during hardware integration. */
    { POSE_DICTATION, 0.5f, -0.866f, 0.0f, 0.866f, "DICTATION" },

    /* POSE_SURFACE: wrist horizontal, band volar facing up (palm-
     * down rest position).  Gravity is along band POSITIVE Z (the
     * band is being "looked at from above").  Tolerance tighter
     * (±20°) to reduce lap false positives. */
    { POSE_SURFACE,   0.0f,  0.0f,  1.0f,  0.940f, "SURFACE" },
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
