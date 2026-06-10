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

    /* POSE_AIR_MOUSE: the RAISED-arm hemisphere.  Re-measured
     * 2026-06-10 across straight + max forward/left/right leans:
     *   straight [0.995, 0.008, -0.098]
     *   forward  [0.861, 0.189, +0.472]
     *   left     [0.919, -0.292, 0.265]
     *   right    [0.906, +0.409, -0.113]
     * All are +X-dominant (gravity ~along the forearm axis) with the
     * perpendicular (Y,Z) components spreading up to ~36° as the wrist
     * leans.  Centred on the pure forearm axis [1,0,0] with a wide
     * cone (tol 0.60 ≈ ±53°, arms within ~37° at the 0.5 threshold)
     * to cover the whole lean range.  Stays clear of SURFACE (80°
     * away) and NEUTRAL (~46°).
     *
     * IMPORTANT: this pose is "raised arm", which is air-mouse AND
     * dictation BOTH.  Gravity CANNOT separate them -- measured proof
     * 2026-06-10: an air-mouse max-right-lean [0.906,0.409,-0.113] is
     * only ~4° from a dictation pose [0.905,0.421,-0.045] (leaning the
     * raised hand right rolls the band the same way supinating for
     * dictation does -- same axis, same gravity).  So the MODE is
     * decided by the confirming gesture, not the pose: cadenced
     * double-tap -> AIR_MOUSE; voice -> DICTATION (see
     * decision_dictation_voice_gated_entry memory).  DICTATION is
     * disabled below until voice detection exists. */
    { POSE_AIR_MOUSE, 1.0f, 0.0f, 0.0f, 0.60f, "AIR_MOUSE" },

    /* POSE_DICTATION: DISABLED (tolerance 2.0 => pose_score always 0,
     * never matches).  Gravity cannot distinguish dictation from an
     * air-mouse raise (see AIR_MOUSE note above -- ~4° apart).  The
     * raised pose is shared; dictation will be split out at the
     * GESTURE stage via voice presence (band at the lips), not by a
     * separate gravity pose.  Re-enable as a real discriminator only
     * when voice-gated entry is built (decision_dictation_voice_gated_
     * entry memory). */
    { POSE_DICTATION, 0.92f, 0.39f, 0.03f, 2.0f, "DICTATION" },

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
