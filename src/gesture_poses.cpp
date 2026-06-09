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

    /* Canonicals re-measured 2026-06-10 in the FINAL mount: RIGHT
     * wrist, VOLAR, THUMB-SIDE (radial).  See hardware_wear_position
     * memory.  Each value is the normalized mean of several held 'g'
     * readings; gravity reads ~+9.8 m/s^2 on the dominant axis,
     * normalized here to a unit vector (pose_score normalizes the
     * OBSERVED vector but assumes the canonical is already unit). */

    /* POSE_AIR_MOUSE: forearm raised forward, band volar facing
     * screen.  Gravity points along band +X (Y≈0).  Measured across
     * 4 raised angles: X +0.92..+0.99, Y -0.02..+0.05, Z -0.13..+0.38;
     * family spans ~16° from this centre, inside the ±30° cone. */
    { POSE_AIR_MOUSE, 0.99f, 0.01f, 0.12f, 0.866f, "AIR_MOUSE" },

    /* POSE_DICTATION: forearm raised + rotated so band volar faces
     * the mouth.  Same +X dominance as AIR_MOUSE but with a clear +Y
     * roll (0.30..0.45).  NOTE: only ~23° from AIR_MOUSE, so the user
     * must roll distinctly or a weak-roll dictation attempt falls
     * back to AIR_MOUSE (correct/safe -- AIR_MOUSE is the live mode;
     * DICTATION is log-only until the dictation feature lands).
     * pose_classify_best picks the nearer canonical. */
    { POSE_DICTATION, 0.92f, 0.39f, 0.03f, 0.866f, "DICTATION" },

    /* POSE_SURFACE: wrist horizontal on desk, band volar facing up.
     * Gravity along band +Z with a slight +X lean (measured centre
     * [0.18, 0.08, 0.98]; family spans ~11°).  Tolerance tighter
     * (±20°) to reduce lap false positives. */
    { POSE_SURFACE,   0.18f, 0.08f, 0.98f, 0.940f, "SURFACE" },
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
