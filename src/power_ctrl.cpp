#include "power_ctrl.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(pwr, LOG_LEVEL_INF);

/* ---- MAX30102 on i2c1 @ 0x57 -------------------------------------------
 *
 * The MAX30101 Zephyr driver hangs off DT_NODELABEL(max30102) on the
 * i2c1 bus; we reach into the same bus by node label so we don't have
 * to crack open the driver's private state.
 */
#define MAX30102_I2C_ADDR              0x57
#define MAX30102_REG_MODE_CFG          0x09
#define MAX30102_MODE_CFG_SHDN_BIT     (1U << 7)

static const struct device *const max30102_bus =
    DEVICE_DT_GET(DT_NODELABEL(i2c1));

static int max30102_set_shdn(bool shutdown)
{
    if (!device_is_ready(max30102_bus)) {
        LOG_ERR("i2c1 bus not ready");
        return -ENODEV;
    }

    /* Read-modify-write: preserve the mode-select bits the driver
     * programmed at init, only flip SHDN.  Cheap (one read, one write)
     * and means future driver changes to other MODE_CFG bits won't
     * silently get clobbered.
     */
    uint8_t mode;
    int err = i2c_reg_read_byte(max30102_bus, MAX30102_I2C_ADDR,
                                MAX30102_REG_MODE_CFG, &mode);
    if (err) {
        LOG_ERR("MAX30102 MODE_CFG read failed (%d)", err);
        return err;
    }

    uint8_t before = mode;
    if (shutdown) {
        mode |= MAX30102_MODE_CFG_SHDN_BIT;
    } else {
        mode &= ~MAX30102_MODE_CFG_SHDN_BIT;
    }

    /* No-op if already in the requested state -- saves one bus
     * transaction on repeated calls (the state machine may try to
     * re-shutdown after a snapshot even if shutdown is already in
     * effect, e.g. after a soft error path).
     */
    if (mode == before) {
        return 0;
    }

    err = i2c_reg_write_byte(max30102_bus, MAX30102_I2C_ADDR,
                             MAX30102_REG_MODE_CFG, mode);
    if (err) {
        LOG_ERR("MAX30102 MODE_CFG write failed (%d)", err);
        return err;
    }

    LOG_INF("MAX30102 %s (MODE_CFG 0x%02x -> 0x%02x)",
            shutdown ? "shutdown" : "wake", before, mode);
    return 0;
}

int max30102_shutdown(void) { return max30102_set_shdn(true); }
int max30102_wake(void)     { return max30102_set_shdn(false); }

/* ---- LSM6DS3TR-C on i2c0 @ 0x6A -----------------------------------------
 *
 * Same trick as MAX30102: reach the chip via the bus node label so we
 * don't have to crack the driver open.  All register addresses and bit
 * masks come from /opt/nordic/ncs/zephyr/drivers/sensor/st/lsm6dsl/lsm6dsl.h
 * (i.e. the chip's actual register map, not a guess).
 */
#define LSM6DSL_I2C_ADDR               0x6A

#define LSM6DSL_REG_INT1_CTRL          0x0D
#define LSM6DSL_INT1_CTRL_SIGN_MOT_BIT (1U << 6)

#define LSM6DSL_REG_CTRL10_C           0x19
#define LSM6DSL_CTRL10_C_PEDO_EN_BIT          (1U << 4)
#define LSM6DSL_CTRL10_C_FUNC_EN_BIT          (1U << 2)
#define LSM6DSL_CTRL10_C_SIGN_MOTION_EN_BIT   (1U << 0)

/* CTRL1_XL (0x10): top 4 bits = ODR_XL, [3:2] = FS, [1] = LPF1_BW_SEL.
 * We only touch the ODR field; FS and the bandwidth-select bits stay
 * at whatever the driver configured (default ±2 g, BW auto). */
#define LSM6DSL_REG_CTRL1_XL           0x10
#define LSM6DSL_CTRL1_XL_ODR_MASK      0xF0
#define LSM6DSL_CTRL1_XL_ODR_104HZ     (4U << 4)   /* matches driver default */
#define LSM6DSL_CTRL1_XL_ODR_416HZ     (6U << 4)   /* AN5040-recommended for tap */

/* Tap-engine registers per AN5040 / LSM6DSL datasheet section 9.x. */
#define LSM6DSL_REG_TAP_CFG            0x58
#define LSM6DSL_TAP_CFG_INT_ENABLE     (1U << 7)   /* MUST be set on LSM6DS3TR-C
                                                     or no embedded event fires */
#define LSM6DSL_TAP_CFG_TAP_X_EN       (1U << 3)
#define LSM6DSL_TAP_CFG_TAP_Y_EN       (1U << 2)
#define LSM6DSL_TAP_CFG_TAP_Z_EN       (1U << 1)
#define LSM6DSL_TAP_CFG_LIR            (1U << 0)   /* 1 = latched (recommended)
                                                     -- INT1 stays high until
                                                     we read TAP_SRC, zero
                                                     race risk for the
                                                     consumer thread */

#define LSM6DSL_REG_TAP_THS_6D         0x59
#define LSM6DSL_TAP_THS_MASK           0x1F        /* lower 5 bits = TAP_THS */

#define LSM6DSL_REG_INT_DUR2           0x5A
/* DUR=7 (~538 ms inter-tap gap at 416 Hz), QUIET=3 (~29 ms),
 * SHOCK=3 (~58 ms).  Values from AN5040 reference + BerryIMU tutorial
 * + ST community guidance; to be empirically tuned. */
#define LSM6DSL_INT_DUR2_INITIAL       0x7F

#define LSM6DSL_REG_WAKE_UP_THS        0x5B
#define LSM6DSL_WAKE_UP_THS_SINGLE_DOUBLE_TAP   (1U << 7)

#define LSM6DSL_REG_MD1_CFG            0x5E
#define LSM6DSL_MD1_CFG_INT1_SINGLE_TAP   (1U << 6)
#define LSM6DSL_MD1_CFG_INT1_DOUBLE_TAP   (1U << 3)

/* Source registers read by the INT1 dispatcher. */
#define LSM6DSL_REG_TAP_SRC            0x1C
#define LSM6DSL_REG_FUNC_SRC1          0x53

/* FIFO registers for the bio-acoustic capture pipeline (Stage E).
 * FIFO is 4 KB total, holding up to 682 accel-only samples (each 6
 * bytes) at our intended high-ODR config of 1.66 kHz -- about 410 ms
 * of pre-event signal continuously buffered. */
#define LSM6DSL_REG_FIFO_CTRL1         0x06  /* FIFO threshold LSB */
#define LSM6DSL_REG_FIFO_CTRL2         0x07  /* FIFO threshold MSB + timer en */
#define LSM6DSL_REG_FIFO_CTRL3         0x08  /* per-sensor decimation */
#define LSM6DSL_REG_FIFO_CTRL4         0x09  /* decimation slaves + stop_on_fth */
#define LSM6DSL_REG_FIFO_CTRL5         0x0A  /* FIFO ODR + FIFO_MODE */
#define LSM6DSL_REG_FIFO_STATUS1       0x3A  /* DIFF_FIFO LSB */
#define LSM6DSL_REG_FIFO_STATUS2       0x3B  /* DIFF_FIFO MSB + status flags */
#define LSM6DSL_REG_FIFO_DATA_OUT_L    0x3E  /* burst-read from here */
#define LSM6DSL_REG_FIFO_DATA_OUT_H    0x3F

/* FIFO_CTRL5 layout: [7:3] ODR_FIFO, [2:0] FIFO_MODE.
 *
 *   FIFO_MODE values (datasheet table):
 *     000 = bypass (FIFO disabled, content reset)
 *     001 = FIFO mode (stops on full)
 *     011 = continuous-to-FIFO (continuous until trigger event)
 *     100 = bypass-to-continuous (bypass until trigger)
 *     110 = continuous (oldest overwritten on full)
 *
 *   ODR_FIFO values match ODR_XL encoding -- 1010 = 1.66 kHz.
 */
#define LSM6DSL_FIFO_MODE_BYPASS       0x00
#define LSM6DSL_FIFO_MODE_FIFO         0x01  /* stop on full */
#define LSM6DSL_FIFO_MODE_CONTINUOUS   0x06
#define LSM6DSL_FIFO_ODR_1_66KHZ       (0xA << 3)

/* FIFO_STATUS2 bit 6 = FIFO_FULL (FIFO at threshold).  Bits [10:0]
 * across STATUS1/2 = DIFF_FIFO (current word count, 1 word = 2 bytes). */
#define LSM6DSL_FIFO_STATUS2_FULL_BIT  (1U << 6)

/* FIFO_CTRL3 bits [2:0] = DEC_FIFO_XL (accel decimation).
 *   000 = sensor not in FIFO
 *   001 = no decimation (every sample)
 *   010 = decimation factor 2
 *   ... etc. */
#define LSM6DSL_FIFO_CTRL3_DEC_XL_NONE     0x01

static const struct device *const lsm6dsl_bus =
    DEVICE_DT_GET(DT_NODELABEL(i2c0));

static int lsm6dsl_update_bits(uint8_t reg, uint8_t mask, uint8_t set_bits)
{
    if (!device_is_ready(lsm6dsl_bus)) {
        LOG_ERR("i2c0 bus not ready");
        return -ENODEV;
    }
    uint8_t val;
    int err = i2c_reg_read_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR, reg, &val);
    if (err) {
        LOG_ERR("LSM6DSL reg 0x%02x read failed (%d)", reg, err);
        return err;
    }
    uint8_t newv = (val & ~mask) | (set_bits & mask);
    if (newv == val) {
        return 0;
    }
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR, reg, newv);
    if (err) {
        LOG_ERR("LSM6DSL reg 0x%02x write failed (%d)", reg, err);
        return err;
    }
    LOG_DBG("LSM6DSL reg 0x%02x: 0x%02x -> 0x%02x", reg, val, newv);
    return 0;
}

int lsm6dsl_enable_sign_motion(void)
{
    /* Significant Motion is the output of the Pedometer engine, which in
     * turn is gated by the master "embedded functions" enable.  All three
     * bits must be set together, otherwise the engine never runs and INT1
     * stays silent.
     *
     * The order matters: enable the engines BEFORE routing the event to
     * INT1, so a stale start-up flag in FUNC_SRC1 cannot generate a
     * spurious interrupt at the moment we enable the pin.
     */
    const uint8_t ctrl10_mask =
        LSM6DSL_CTRL10_C_FUNC_EN_BIT |
        LSM6DSL_CTRL10_C_PEDO_EN_BIT |
        LSM6DSL_CTRL10_C_SIGN_MOTION_EN_BIT;

    int err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL10_C,
                                  ctrl10_mask, ctrl10_mask);
    if (err) return err;

    err = lsm6dsl_update_bits(LSM6DSL_REG_INT1_CTRL,
                              LSM6DSL_INT1_CTRL_SIGN_MOT_BIT,
                              LSM6DSL_INT1_CTRL_SIGN_MOT_BIT);
    if (err) return err;

    LOG_INF("LSM6DSL: Significant Motion engine enabled, INT1 armed");
    return 0;
}

int lsm6dsl_disable_sign_motion(void)
{
    /* Unhook the INT1 routing first so we cannot get a final spurious
     * edge while we're tearing the engines down.
     */
    int err = lsm6dsl_update_bits(LSM6DSL_REG_INT1_CTRL,
                                  LSM6DSL_INT1_CTRL_SIGN_MOT_BIT, 0);
    if (err) return err;

    /* Disable just the SIGN_MOTION_EN bit; leave PEDO_EN / FUNC_EN
     * untouched so future re-enable is just one bit flip and the step
     * counter (a possible later feature) keeps running.
     */
    err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL10_C,
                              LSM6DSL_CTRL10_C_SIGN_MOTION_EN_BIT, 0);
    if (err) return err;

    LOG_INF("LSM6DSL: Significant Motion engine disabled");
    return 0;
}

/* ---- LSM6DSL chip-embedded tap engine ---------------------------------- */

int lsm6dsl_tap_engine_enable(uint8_t tap_threshold)
{
    /* Order matters here, same lesson as sig-motion: configure all the
     * engine state BEFORE routing the event to INT1 so a stale flag
     * cannot pulse the pin during arming.
     *
     * Step 1: bump accel ODR to 416 Hz so the tap engine's slope
     * filter, SHOCK/QUIET/DUR windows, and double-tap timing all run
     * at the cadence AN5040 documents.  The driver's 104 Hz output
     * stream remains accessible via the same registers -- we keep
     * polling at our usual rate; the chip just samples internally 4x
     * faster while tap is armed. */
    int err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL1_XL,
                                  LSM6DSL_CTRL1_XL_ODR_MASK,
                                  LSM6DSL_CTRL1_XL_ODR_416HZ);
    if (err) return err;

    /* Step 2: program the threshold.  Mask off the 6D-orientation bits
     * in the upper part of TAP_THS_6D -- those are unrelated to tap
     * sensitivity and we don't use the 6D classifier here (we have
     * our own gravity-LPF orientation work). */
    uint8_t ths = tap_threshold & LSM6DSL_TAP_THS_MASK;
    err = lsm6dsl_update_bits(LSM6DSL_REG_TAP_THS_6D,
                              LSM6DSL_TAP_THS_MASK, ths);
    if (err) return err;

    /* Step 3: program SHOCK/QUIET/DUR windows.  Single-byte register,
     * full overwrite. */
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_INT_DUR2,
                             LSM6DSL_INT_DUR2_INITIAL);
    if (err) {
        LOG_ERR("LSM6DSL INT_DUR2 write failed (%d)", err);
        return err;
    }

    /* Step 4: SINGLE_DOUBLE_TAP=0 -- chip emits ONLY SINGLE_TAP
     * events.  Firmware multi-tap counter in gesture_mode.cpp
     * derives double-tap and triple-tap from individual single-tap
     * arrivals with its own timing window.
     *
     * The pivot from SINGLE_DOUBLE_TAP=1 (industry default) is
     * empirically justified by Test 4 on 2026-06-08: with LIR=0 +
     * SDT=1, isolated single-tap events were race-lost (the chip's
     * ~2.4 ms pulse window was beyond the consumer thread's wake
     * latency).  Pivoting to SDT=0 + LIR=1 latches every individual
     * tap until we read TAP_SRC -- zero race risk -- at the cost of
     * losing the chip's hardware double-tap classifier.  We don't
     * lose functionality because the firmware counter derives the
     * same semantic from the SINGLE_TAP stream.
     *
     * Required by ST AN: under LIR=1 + SDT=0 the latch is applied
     * to the single-tap interrupt signal; the tap event must also be
     * routed to INT1 (done in Step 6) or the latch has no effect. */
    err = lsm6dsl_update_bits(LSM6DSL_REG_WAKE_UP_THS,
                              LSM6DSL_WAKE_UP_THS_SINGLE_DOUBLE_TAP, 0);
    if (err) return err;

    /* Step 5: enable the tap engine itself.  TAP_CFG bit 7
     * (INTERRUPTS_ENABLE) is the silent-failure gotcha on
     * LSM6DS3TR-C -- without it, all the routing and detection runs
     * but no INT1 ever fires.
     *
     * LIR=1 (latched): every SINGLE_TAP event holds INT1 high until
     * the consumer reads TAP_SRC.  Empirically justified pivot from
     * LIR=0 baseline (see SDT comment above).  ST AN4987 calls this
     * "critical for reliable tap detection applications." */
    const uint8_t tap_cfg =
        LSM6DSL_TAP_CFG_INT_ENABLE |
        LSM6DSL_TAP_CFG_TAP_X_EN |
        LSM6DSL_TAP_CFG_TAP_Y_EN |
        LSM6DSL_TAP_CFG_TAP_Z_EN |
        LSM6DSL_TAP_CFG_LIR;
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_TAP_CFG, tap_cfg);
    if (err) {
        LOG_ERR("LSM6DSL TAP_CFG write failed (%d)", err);
        return err;
    }

    /* Step 6: route single-tap to INT1.  We don't enable DOUBLE_TAP
     * route since the chip won't emit it under SDT=0, but leaving
     * the bit set is harmless.  Sig-motion lives in a different
     * routing register (INT1_CTRL bit 6) so this MD1_CFG write does
     * not disturb it.
     *
     * The single-tap route bit must be set for the LIR latch to take
     * effect (ST documented requirement). */
    err = lsm6dsl_update_bits(
        LSM6DSL_REG_MD1_CFG,
        LSM6DSL_MD1_CFG_INT1_SINGLE_TAP | LSM6DSL_MD1_CFG_INT1_DOUBLE_TAP,
        LSM6DSL_MD1_CFG_INT1_SINGLE_TAP);
    if (err) return err;

    LOG_INF("LSM6DSL: tap engine enabled (LIR=1 latched, "
            "SINGLE_DOUBLE_TAP=0, TAP_THS=0x%02x, ODR=416 Hz)", ths);
    return 0;
}

int lsm6dsl_tap_engine_disable(void)
{
    /* Reverse order: un-route INT1 first so no spurious edge can be
     * pulsed while we're tearing down. */
    int err = lsm6dsl_update_bits(
        LSM6DSL_REG_MD1_CFG,
        LSM6DSL_MD1_CFG_INT1_SINGLE_TAP | LSM6DSL_MD1_CFG_INT1_DOUBLE_TAP,
        0);
    if (err) return err;

    /* Disable the tap engine.  Leave the master INTERRUPTS_ENABLE bit
     * set -- sig-motion uses the same master gate via the embedded-
     * functions path, and turning it off here would break that. */
    err = lsm6dsl_update_bits(LSM6DSL_REG_TAP_CFG,
                              LSM6DSL_TAP_CFG_TAP_X_EN |
                              LSM6DSL_TAP_CFG_TAP_Y_EN |
                              LSM6DSL_TAP_CFG_TAP_Z_EN,
                              0);
    if (err) return err;

    err = lsm6dsl_update_bits(LSM6DSL_REG_WAKE_UP_THS,
                              LSM6DSL_WAKE_UP_THS_SINGLE_DOUBLE_TAP, 0);
    if (err) return err;

    /* Restore the driver's 104 Hz ODR. */
    err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL1_XL,
                              LSM6DSL_CTRL1_XL_ODR_MASK,
                              LSM6DSL_CTRL1_XL_ODR_104HZ);
    if (err) return err;

    LOG_INF("LSM6DSL: tap engine disabled, ODR restored to 104 Hz");
    return 0;
}

int lsm6dsl_tap_set_threshold(uint8_t threshold)
{
    uint8_t ths = threshold & LSM6DSL_TAP_THS_MASK;
    int err = lsm6dsl_update_bits(LSM6DSL_REG_TAP_THS_6D,
                                  LSM6DSL_TAP_THS_MASK, ths);
    if (err) return err;
    LOG_INF("LSM6DSL: TAP_THS = 0x%02x (~%d mg at FS=2g)",
            ths, ths * 62);  /* 1 LSB ≈ 62.5 mg at ±2 g */
    return 0;
}

int lsm6dsl_read_tap_src(uint8_t *src)
{
    if (!device_is_ready(lsm6dsl_bus)) {
        return -ENODEV;
    }
    return i2c_reg_read_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_TAP_SRC, src);
}

int lsm6dsl_read_func_src1(uint8_t *src)
{
    if (!device_is_ready(lsm6dsl_bus)) {
        return -ENODEV;
    }
    return i2c_reg_read_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FUNC_SRC1, src);
}

/* ---- LSM6DSL FIFO bio-acoustic capture pipeline -------------------------
 *
 * Enables the chip's internal FIFO at 1.66 kHz in continuous mode,
 * accel-only (no decimation, no gyro / external).  The FIFO ring-
 * buffers ~410 ms of pre-event signal continuously.  On a tap event,
 * the INT1 consumer reads the entire FIFO to capture the pre-event
 * window, then resumes for the next event.
 *
 * Power impact: ~0.3 mA additional draw versus the 416 Hz tap-only
 * config (chip runs at 1.66 kHz internally; we still poll at 100 Hz
 * for the main acq pipeline).
 */

int lsm6dsl_fifo_enable_continuous(void)
{
    /* Step 1: bump CTRL1_XL ODR to 1.66 kHz (FIFO ODR must be ≤
     * sensor ODR per AN5040).  This overrides the 416 Hz that
     * lsm6dsl_tap_engine_enable() left in place. */
    int err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL1_XL,
                                  LSM6DSL_CTRL1_XL_ODR_MASK,
                                  0xA << 4);   /* 1010 = 1.66 kHz */
    if (err) return err;

    /* Step 2: set FIFO threshold high (max 0x7FF in CTRL1+CTRL2),
     * but we don't actually use the watermark interrupt -- the tap
     * event is our trigger.  Write 0xFF/0x07 to keep the threshold
     * effectively disabled (chip won't fire watermark before tap). */
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_CTRL1, 0xFF);
    if (err) return err;
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_CTRL2, 0x07);
    if (err) return err;

    /* Step 3: enable accel-only in FIFO with no decimation (every
     * sample at 1.66 kHz makes it in).  Gyro / ext-sensor decimation
     * stays at 000 (sensor not in FIFO). */
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_CTRL3,
                             LSM6DSL_FIFO_CTRL3_DEC_XL_NONE);
    if (err) return err;
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_CTRL4, 0x00);
    if (err) return err;

    /* Step 4: FIFO mode = continuous (110b), FIFO ODR = 1.66 kHz.
     * This starts the FIFO actively writing. */
    err = i2c_reg_write_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_CTRL5,
                             LSM6DSL_FIFO_ODR_1_66KHZ |
                             LSM6DSL_FIFO_MODE_CONTINUOUS);
    if (err) return err;

    LOG_INF("LSM6DSL: FIFO enabled (continuous mode, 1.66 kHz, "
            "accel-only) -- ~410 ms pre-event window");
    return 0;
}

int lsm6dsl_fifo_disable(void)
{
    /* Bypass mode resets FIFO content and stops the engine. */
    int err = lsm6dsl_update_bits(LSM6DSL_REG_FIFO_CTRL5,
                                  0x07, LSM6DSL_FIFO_MODE_BYPASS);
    if (err) return err;

    /* Restore the tap-engine ODR (416 Hz) so tap detection still
     * works after FIFO is off. */
    err = lsm6dsl_update_bits(LSM6DSL_REG_CTRL1_XL,
                              LSM6DSL_CTRL1_XL_ODR_MASK,
                              LSM6DSL_CTRL1_XL_ODR_416HZ);
    if (err) return err;

    LOG_INF("LSM6DSL: FIFO disabled, ODR restored to 416 Hz");
    return 0;
}

int lsm6dsl_fifo_get_word_count(uint16_t *count)
{
    if (!device_is_ready(lsm6dsl_bus) || !count) {
        return -ENODEV;
    }
    uint8_t s1, s2;
    int err = i2c_reg_read_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                                LSM6DSL_REG_FIFO_STATUS1, &s1);
    if (err) return err;
    err = i2c_reg_read_byte(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                            LSM6DSL_REG_FIFO_STATUS2, &s2);
    if (err) return err;
    /* DIFF_FIFO spans bits [10:0] = s2[2:0] << 8 | s1.
     * Each "word" is 2 bytes; an accel sample is 3 words (XL/YL/ZL
     * pairs).  Caller divides by 3 to get sample count. */
    *count = ((uint16_t)(s2 & 0x07) << 8) | s1;
    return 0;
}

int lsm6dsl_fifo_read_words(uint8_t *buf, uint16_t want_words,
                            uint16_t *got_words)
{
    if (!device_is_ready(lsm6dsl_bus) || !buf || !got_words) {
        return -ENODEV;
    }
    *got_words = 0;

    /* FIFO_DATA_OUT auto-increments by 2 bytes per read.  We can
     * burst-read multiple words in one I2C transaction by reading
     * from FIFO_DATA_OUT_L with auto-address-increment (default
     * behaviour on this chip per IF_INC bit in CTRL3_C). */
    uint16_t bytes = want_words * 2;
    if (bytes == 0) return 0;

    int err = i2c_burst_read(lsm6dsl_bus, LSM6DSL_I2C_ADDR,
                             LSM6DSL_REG_FIFO_DATA_OUT_L,
                             buf, bytes);
    if (err) return err;

    *got_words = want_words;
    return 0;
}
