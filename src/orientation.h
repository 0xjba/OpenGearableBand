#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared IMU orientation-estimation foundation.
 *
 * A Mahony complementary filter fuses the accelerometer (gravity ->
 * absolute pitch/roll) with the gyroscope (angular rate -> all axes),
 * producing a quaternion + Euler angles.  On a 6-axis IMU (no
 * magnetometer) pitch/roll are drift-free; YAW has no absolute
 * reference and is gyro-integrated only, so it drifts -- it is held in
 * check by gyro-bias tracking (ZARU, via the filter's integral term)
 * and an automatic yaw re-zero whenever the wrist is detected at rest
 * (ZUPT analogue).
 *
 * Used for: (1) path-independent dictation-vs-air-mouse discrimination
 * (end-state yaw, not a path-dependent gyro integral), (2) steadier
 * pose detection, (3) the future drift-free air-mouse cursor.  See
 * docs/research/cursor-drift-mitigation.md and the
 * project_orientation_foundation memory.
 */
typedef struct {
    float qw, qx, qy, qz;     /* orientation quaternion */
    float pitch_deg;          /* gravity-locked, drift-free */
    float roll_deg;           /* gravity-locked, drift-free */
    float yaw_deg;            /* gyro-only; drifts; re-zeroed at rest */
    bool  at_rest;            /* stillness flag (accel + gyro quiet) */
    float gyro_bias_dps[3];   /* estimated gyro bias, deg/s */
    float gravity[3];         /* fused gravity DIRECTION in band frame, unit vector
                               * (gyro-driven, accel-corrected) -- the roll-immune
                               * source for the cursor's vertical driver */
} orientation_state_t;

void orientation_init(void);

/* Feed one fused sample at the acq rate (100 Hz).  accel may be in any
 * consistent unit (only its direction is used); gyro must be rad/s. */
void orientation_update(float ax, float ay, float az,
                        float gx_rps, float gy_rps, float gz_rps);

void orientation_get(orientation_state_t *out);

/* Force a yaw re-zero (ZUPT-style).  Also invoked automatically the
 * moment the filter detects the wrist has come to rest. */
void orientation_rezero_yaw(void);

/* Stationary gyro-bias trace.  Hold the device perfectly still and
 * call this; it accumulates raw gyro for n_samples (~100 Hz) and logs
 * mean (= bias, deg/s), stddev (= noise), and min/max per axis.  Run
 * warm AND cold to get this unit's real numbers before trusting the
 * filter's stillness / bias thresholds (per cursor-drift-mitigation
 * spec, section 8). */
void orientation_bias_trace_start(uint32_t n_samples);

#ifdef __cplusplus
}
#endif

#endif /* ORIENTATION_H */
