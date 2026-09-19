/*
 * battery -- see battery.h.
 */
#include "battery.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/* nPM13xx CHARGER.BCHGCHARGESTATUS bitmasks.
 * Source: nrf/samples/pmic/native/npm13xx_fuel_gauge/src/fuel_gauge.c (Nordic),
 * matched by Seeed's own example. An earlier version of this file guessed these
 * bit positions and every one was wrong, so the charge state it reported was
 * meaningless. Do not "simplify" these -- they are register bits. */
#define NPM13XX_CHG_STATUS_COMPLETE_MASK BIT(1)
#define NPM13XX_CHG_STATUS_TRICKLE_MASK  BIT(2)
#define NPM13XX_CHG_STATUS_CC_MASK       BIT(3)
#define NPM13XX_CHG_STATUS_CV_MASK       BIT(4)

/* nPM1300 VBUSIN.VBUSINSTATUS bit 0 = VBUSINPRESENT.
 * Source: nordicsemi/npmx adk/npm1300.h, VBUSIN_VBUSINSTATUS_VBUSINPRESENT_Msk.
 * See battery.h for why neither `!= 0` nor the CC-detect register is used. */
#define NPM1300_VBUSINSTATUS_PRESENT_MASK BIT(0)

static const struct device *charger;

/* The nPM13xx regulator PARENT device (the `regulators` node in Seeed's board
 * dtsi). It has no label of its own, so reach it as the parent of Seeed's
 * vsys_3v3 (BUCK2) node. */
static const struct device *const regulators =
	DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vsys_3v3)));

int battery_init(void)
{
	charger = DEVICE_DT_GET_ANY(nordic_npm1300_charger);
	if (charger == NULL || !device_is_ready(charger)) {
		LOG_WRN("nPM1300 charger not available -- battery reporting off");
		charger = NULL;
		return -ENODEV;
	}
	LOG_INF("battery: nPM1300 charger bound");
	return 0;
}

void battery_read(struct battery_status *out)
{
	*out = (struct battery_status){};
	if (charger == NULL) {
		return;
	}
	if (sensor_sample_fetch(charger) < 0) {
		LOG_WRN("PMIC sample failed");
		return;
	}

	struct sensor_value v {}, i {}, t {}, st {}, vb {}, er {};
	if (sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v) != 0) {
		return;
	}
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i);
	sensor_channel_get(charger, SENSOR_CHAN_DIE_TEMP, &t);
	sensor_channel_get(charger, (enum sensor_channel)
			   SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &st);
	sensor_channel_get(charger, (enum sensor_channel)
			   SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vb);
	sensor_channel_get(charger, (enum sensor_channel)
			   SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &er);

	out->valid         = true;
	out->millivolts    = v.val1 * 1000 + v.val2 / 1000;
	out->milliamps     = i.val1 * 1000 + i.val2 / 1000;
	out->die_temp_decidegc = t.val1 * 10 + t.val2 / 100000;
	out->vbus_status_raw = (uint8_t)vb.val1;
	out->vbus = (out->vbus_status_raw & NPM1300_VBUSINSTATUS_PRESENT_MASK) != 0;
	out->charger_error = (uint32_t)er.val1;

	/* Same precedence Nordic's sample uses. */
	const uint32_t s = (uint32_t)st.val1;
	if      (s & NPM13XX_CHG_STATUS_COMPLETE_MASK) out->state = BATTERY_STATE_COMPLETE;
	else if (s & NPM13XX_CHG_STATUS_TRICKLE_MASK)  out->state = BATTERY_STATE_TRICKLE;
	else if (s & NPM13XX_CHG_STATUS_CC_MASK)       out->state = BATTERY_STATE_CC;
	else if (s & NPM13XX_CHG_STATUS_CV_MASK)       out->state = BATTERY_STATE_CV;
	else                                            out->state = BATTERY_STATE_IDLE;
}

const char *battery_state_str(enum battery_charge_state s)
{
	switch (s) {
	case BATTERY_STATE_IDLE:     return "idle";
	case BATTERY_STATE_TRICKLE:  return "trickle";
	case BATTERY_STATE_CC:       return "charging CC";
	case BATTERY_STATE_CV:       return "charging CV";
	case BATTERY_STATE_COMPLETE: return "charged";
	default:                     return "unknown";
	}
}

int battery_enter_ship_mode(void)
{
	if (!device_is_ready(regulators)) {
		LOG_ERR("nPM13xx regulator device not ready -- cannot ship");
		return -ENODEV;
	}
	return regulator_parent_ship_mode(regulators);
}
