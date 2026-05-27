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
