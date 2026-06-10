#include "orientation.h"

#include <math.h>

/* Sample period: acq runs at ~100 Hz. */
#define ORI_DT          0.01f
#define RAD2DEG         57.29578f

/* Mahony gains.  TWO_KP = proportional (how fast pitch/roll lock to
 * gravity); TWO_KI = integral (gyro-bias estimation rate).  Standard
 * starting values; tune from the stationary trace if drift is high. */
#define TWO_KP          (2.0f * 0.5f)
#define TWO_KI          (2.0f * 0.1f)

/* Stillness detection thresholds (empirical -- refine from this unit's
 * own still-vs-moving logs per the cursor spec's honesty note). */
#define STILL_GYRO_THRESH_RPS   0.10f   /* ~5.7 deg/s total */
#define STILL_ACC_RESID         0.80f   /* |a| within this of 9.81 m/s^2 */
#define STILL_DWELL_SAMPLES     30      /* 300 ms of quiet -> "at rest" */

/* Orientation quaternion (band -> world). */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* Integral feedback = running gyro-bias estimate (rad/s). */
static float ifb_x = 0.0f, ifb_y = 0.0f, ifb_z = 0.0f;

/* Yaw offset applied on re-zero so reported yaw is relative to the last
 * rest pose (the ZUPT anchor). */
static float yaw_offset_deg = 0.0f;

static int  still_count = 0;
static bool at_rest = false;
static bool was_rest = false;

static float raw_yaw_deg(void)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

void orientation_init(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    ifb_x = ifb_y = ifb_z = 0.0f;
    yaw_offset_deg = 0.0f;
    still_count = 0;
    at_rest = false;
    was_rest = false;
}

void orientation_rezero_yaw(void)
{
    yaw_offset_deg = raw_yaw_deg();
}

void orientation_update(float ax, float ay, float az,
                        float gx, float gy, float gz)
{
    /* --- Stillness detection: gate on BOTH linear and angular quiet,
     * per the ZUPT review (a too-loose detector injects error). --- */
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
    bool quiet = (fabsf(amag - 9.81f) < STILL_ACC_RESID) &&
                 (gmag < STILL_GYRO_THRESH_RPS);
    if (quiet) {
        if (still_count < STILL_DWELL_SAMPLES) still_count++;
    } else {
        still_count = 0;
    }
    at_rest = (still_count >= STILL_DWELL_SAMPLES);

    /* --- Mahony: correct the gyro toward gravity (pitch/roll lock) --- */
    if (amag > 1e-4f) {
        float axn = ax / amag, ayn = ay / amag, azn = az / amag;

        /* Gravity direction the current quaternion predicts. */
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Error = measured x estimated gravity. */
        float ex = ayn * vz - azn * vy;
        float ey = azn * vx - axn * vz;
        float ez = axn * vy - ayn * vx;

        /* Integral term tracks gyro bias (ZARU); converges hardest when
         * still (error is then pure bias, not real rotation). */
        ifb_x += TWO_KI * ex * ORI_DT;
        ifb_y += TWO_KI * ey * ORI_DT;
        ifb_z += TWO_KI * ez * ORI_DT;

        gx += ifb_x + TWO_KP * ex;
        gy += ifb_y + TWO_KP * ey;
        gz += ifb_z + TWO_KP * ez;
    }

    /* --- Integrate the (bias-corrected) rate into the quaternion. --- */
    float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qd1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qd2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qd3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += qd0 * ORI_DT;
    q1 += qd1 * ORI_DT;
    q2 += qd2 * ORI_DT;
    q3 += qd3 * ORI_DT;

    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n > 1e-4f) {
        q0 /= n; q1 /= n; q2 /= n; q3 /= n;
    }

    /* Yaw re-zero the instant the wrist comes to rest (ZUPT). */
    if (at_rest && !was_rest) {
        orientation_rezero_yaw();
    }
    was_rest = at_rest;
}

void orientation_get(orientation_state_t *out)
{
    out->qw = q0; out->qx = q1; out->qy = q2; out->qz = q3;

    /* Quaternion -> Euler (deg). */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    out->pitch_deg = asinf(sinp) * RAD2DEG;
    out->roll_deg  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                            1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;

    /* Yaw relative to the last rest re-zero, wrapped to [-180, 180]. */
    float y = raw_yaw_deg() - yaw_offset_deg;
    while (y > 180.0f)  y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    out->yaw_deg = y;

    out->at_rest = at_rest;
    out->gyro_bias_dps[0] = ifb_x * RAD2DEG;
    out->gyro_bias_dps[1] = ifb_y * RAD2DEG;
    out->gyro_bias_dps[2] = ifb_z * RAD2DEG;
}
