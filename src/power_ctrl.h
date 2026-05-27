/*
 * Peripheral power-control helpers.
 *
 * The Zephyr drivers for MAX30101 and LSM6DSL in NCS do not expose the
 * power-saving features we need (SHDN bit for the MAX, embedded
 * pedometer + significant-motion engines for the LSM).  These helpers
 * drive the relevant registers directly over the same I2C bus the
 * drivers use, without disturbing the driver's own state.
 *
 * Coexistence: we only touch registers the upstream drivers don't.
 *   - MAX30101 driver writes MODE_CFG (0x09) only at init.  We touch
 *     its SHDN bit (7) at runtime; the other bits are preserved via
 *     read-modify-write.
 *   - LSM6DSL driver does not touch CTRL10_C, INT1_CTRL, TAP_CFG,
 *     WAKE_UP_*, or MD1_CFG.  The embedded-function path is entirely
 *     ours to configure.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* MAX30102 ----------------------------------------------------------------
 * Shutdown drops the chip's Iq to ~0.7 uA.  Configuration registers
 * (LED PA, sample rate, mode, FIFO config) are preserved across
 * shutdown per the datasheet, so wake() needs no re-init -- the chip
 * resumes streaming into the FIFO immediately at the configured rate.
 *
 * Both functions return 0 on success, negative errno on I2C failure.
 */
int max30102_shutdown(void);
int max30102_wake(void);

/* LSM6DSL embedded functions ---------------------------------------------
 * The Zephyr lsm6dsl driver only exposes SENSOR_TRIG_DATA_READY (literal
 * __ASSERT_NO_MSG in its trigger code) and never touches CTRL10_C or
 * INT1_CTRL when CONFIG_LSM6DSL_TRIGGER_NONE=y (which is our setting).
 * We therefore configure the chip's built-in Significant Motion engine
 * directly: the algorithm is tuned by STMicroelectronics to fire only
 * on sustained walking-or-running-class kinematics (it ignores typing,
 * driving, brushing teeth) and routes a single edge to INT1 (= P0.11
 * on the Xiao Sense board DTS).
 *
 * lsm6dsl_enable_sign_motion() turns the engine on and routes the event
 * to INT1.  lsm6dsl_disable_sign_motion() unhooks it -- call this once
 * we've transitioned into WORKOUT so the engine doesn't keep firing
 * spuriously while we're already in continuous mode.
 *
 * Both return 0 on success, negative errno on I2C failure.
 */
int lsm6dsl_enable_sign_motion(void);
int lsm6dsl_disable_sign_motion(void);

#ifdef __cplusplus
}
#endif
