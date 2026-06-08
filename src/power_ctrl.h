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

#include <stdint.h>

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

/* LSM6DSL chip-embedded tap engine ---------------------------------------
 *
 * The chip's tap state machine generates SINGLE_TAP and DOUBLE_TAP events
 * via hardware-debounced shock/quiet/duration windows configured in
 * INT_DUR2.  Both events route to the same INT1 GPIO our sig-motion
 * already uses, sharing the pin (the chip ORs them at the pad).  The
 * INT1 consumer demuxes by reading TAP_SRC + FUNC_SRC1.
 *
 * Configuration choices baked into enable():
 *   - ODR_XL bumped to 416 Hz while tap is armed.  Industry-standard
 *     for tap on this chip family: ST's own Mbed LSM6DSL driver
 *     hardcodes 416 Hz inside its Enable_Single_Tap_Detection() and
 *     Enable_Double_Tap_Detection() functions, and every working
 *     community example (SparkFun, Ozzmaker) uses 416 Hz.  We restore
 *     the driver's 104 Hz on disable.
 *   - Full-scale stays at the driver-configured ±2 g (default)
 *   - LIR (latched interrupt) = 1, latched.  Empirically pivoted
 *     2026-06-08 from the LIR=0 community baseline after Test 4
 *     showed isolated single-tap events were race-lost.  Per ST
 *     AN4987 "critical for reliable tap detection applications."
 *     INT1 stays high until the consumer thread reads TAP_SRC --
 *     zero race risk regardless of wake latency.
 *   - SINGLE_DOUBLE_TAP = 0: chip emits ONLY SINGLE_TAP events.
 *     Firmware multi-tap counter in gesture_mode.cpp derives double-
 *     and triple-tap with its own timing window.  Pivoted 2026-06-08
 *     from SDT=1 because LIR=1 + SDT=1 only latches the double-tap
 *     line (per AN5040), losing single-tap reliability that we need
 *     for firmware-derived triple-tap.  Net: same effective vocabulary,
 *     cleaner architecture, no race risk.
 *   - Tap detection enabled on X, Y, and Z (wristband can be hit from
 *     any direction; the tap engine uses the |slope| of whichever
 *     axis dominates)
 *   - INTERRUPTS_ENABLE bit set -- the LSM6DS3TR-C variant gates ALL
 *     embedded-event interrupts on this bit, silent failure if missed
 *
 * tap_threshold: the TAP_THS field of TAP_THS_6D (5 bits, 0-31).  LSB
 *   value = FS_XL / 32.  At ±2 g, 1 LSB ≈ 62.5 mg.  Tuned empirically
 *   via calibration mode -- 0x08 (~500 mg) is the initial guess.
 *
 * Returns 0 on success, negative errno on I2C failure.
 */
int lsm6dsl_tap_engine_enable(uint8_t tap_threshold);
int lsm6dsl_tap_engine_disable(void);

/* Runtime threshold tuning during calibration mode.  Updates the TAP_THS
 * field of TAP_THS_6D without disturbing the 6D-orientation bits.
 * threshold: 0-31 (5-bit value). */
int lsm6dsl_tap_set_threshold(uint8_t threshold);

/* Source-register reads used by the INT1 dispatcher.  Both registers
 * are read-on-the-fly when INT1 fires; the chip clears them on read
 * (or after LIR), so each call returns the events that have fired
 * since the last read.
 *
 *   TAP_SRC (0x1C) bits:
 *     [6] TAP_IA       -- umbrella: any tap event fired
 *     [5] SINGLE_TAP   -- single tap detected this event
 *     [4] DOUBLE_TAP   -- double tap detected this event
 *     [3] TAP_SIGN     -- sign of slope at detection (1 = negative)
 *     [2] X_TAP        -- tap on X
 *     [1] Y_TAP        -- tap on Y
 *     [0] Z_TAP        -- tap on Z
 *
 *   FUNC_SRC1 (0x53) bit [6]: SIGN_MOTION_IA (significant motion)
 *
 * Returns 0 on success, negative errno on I2C failure. */
int lsm6dsl_read_tap_src(uint8_t *src);
int lsm6dsl_read_func_src1(uint8_t *src);

/* LSM6DSL FIFO bio-acoustic capture (Stage E) ---------------------------
 *
 * Enables the chip's internal 4 KB FIFO in continuous mode at
 * 1.66 kHz, accel-only.  Buffers ~410 ms of pre-event signal as a
 * ring; on a tap event, the consumer reads the entire FIFO to
 * snapshot the pre-event window for feature extraction.
 *
 * Why continuous-to-... wasn't used: the chip's continuous-to-FIFO
 * mode auto-switches on an "INT generator" event that's wired to a
 * specific subset of detectors (not single-tap on LSM6DSL).  Plain
 * continuous mode + MCU-driven read on tap event is the cleanest
 * path that ships on this chip.
 */
int lsm6dsl_fifo_enable_continuous(void);
int lsm6dsl_fifo_disable(void);

/* Returns current FIFO occupancy in 2-byte words.  An accel-only
 * sample is 3 words (X/Y/Z, each int16); caller divides by 3 to get
 * sample count. */
int lsm6dsl_fifo_get_word_count(uint16_t *count);

/* Burst-read want_words 2-byte words from FIFO_DATA_OUT.  Returns
 * actual words read in *got_words.  Caller supplies a buffer of at
 * least want_words * 2 bytes.  Byte order is little-endian (chip's
 * native format -- LSB first then MSB per axis). */
int lsm6dsl_fifo_read_words(uint8_t *buf, uint16_t want_words,
                            uint16_t *got_words);

#ifdef __cplusplus
}
#endif
