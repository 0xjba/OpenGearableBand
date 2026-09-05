/*
 * battery -- see battery.h.
 */
#include "battery.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static const struct device *charger;

/* [STRUCTURAL] LiPo open-circuit discharge curve, 4.2V nominal. Coarse on
 * purpose: voltage-based SoC is only good to about +/-10% under load, and a
 * finer table would imply an accuracy we do not have. A real fuel gauge
 * (coulomb counting) is the answer if the product needs better. */
static const struct { uint16_t mv; uint8_t pct; } curve[] = {
	{4200, 100}, {4100, 90}, {4000, 80}, {3900, 68}, {3850, 58},
	{3800, 48}, {3750, 38}, {3700, 28}, {3650, 18}, {3600, 10},
	{3500,  5}, {3400,  2}, {3300,  0},
};

static uint8_t mv_to_pct(uint32_t mv)
{
	if (mv >= curve[0].mv) return 100;
	for (size_t i = 1; i < ARRAY_SIZE(curve); i++) {
		if (mv >= curve[i].mv) {
			/* linear interpolation inside the bracket */
			uint32_t span = curve[i - 1].mv - curve[i].mv;
			uint32_t up   = mv - curve[i].mv;
			int      dp   = curve[i - 1].pct - curve[i].pct;
			return (uint8_t)(curve[i].pct + (up * dp) / span);
		}
	}
	return 0;
}

int battery_init(void)
{
	charger = DEVICE_DT_GET_ANY(nordic_npm1300_charger);
	if (charger == NULL || !device_is_ready(charger)) {
		LOG_WRN("nPM1300 charger not available -- battery reporting off");
		charger = NULL;
		return -ENODEV;
	}
	LOG_INF("battery: nPM1300 charger bound (read-only)");
	return 0;
}

void battery_read(struct battery_status *out)
{
	*out = (struct battery_status){};
	if (charger == NULL) {
		return;
	}
	if (sensor_sample_fetch(charger) != 0) {
		LOG_WRN("PMIC sample failed");
		return;
	}
	struct sensor_value v, i, st, vb;
	if (sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v) != 0) {
		return;
	}
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i);
	sensor_channel_get(charger, (enum sensor_channel)
			   SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &st);
	sensor_channel_get(charger, (enum sensor_channel)
			   SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vb);

	out->valid      = true;
	out->millivolts = (uint32_t)(v.val1 * 1000 + v.val2 / 1000);
	out->percent    = mv_to_pct(out->millivolts);
	out->vbus       = (vb.val1 != 0);

	/* nPM1300 CHARGER_STATUS bit 3 = trickle, 4 = constant current,
	 * 5 = constant voltage, 6 = charge complete. Anything in CC/CV means
	 * current is going in; complete with VBUS still present means charged. */
	uint32_t s = (uint32_t)st.val1;
	bool active   = (s & (BIT(3) | BIT(4) | BIT(5))) != 0;
	bool complete = (s & BIT(6)) != 0;

	/* [STRUCTURAL] Below this the reading is either an unconnected VBAT pin
	 * floating or a cell discharged past its safe floor (LiPo cutoff is ~3.0V).
	 * Either way it is NOT a charged battery, and reporting it as one hides
	 * exactly the fault we most need to see. */
	if (out->millivolts < 3000)     out->state = BATTERY_NONE;
	else if (active)                out->state = BATTERY_CHARGING;
	else if (out->vbus && complete) out->state = BATTERY_CHARGED;
	else if (out->vbus)             out->state = BATTERY_CHARGED;
	else                            out->state = BATTERY_DISCHARGING;
}

const char *battery_state_str(enum battery_charge_state s)
{
	switch (s) {
	case BATTERY_DISCHARGING: return "on battery";
	case BATTERY_CHARGING:    return "charging";
	case BATTERY_CHARGED:     return "charged";
	case BATTERY_NONE:        return "NO BATTERY/FLAT";
	default:                  return "unknown";
	}
}
