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
