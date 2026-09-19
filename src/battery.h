/*
 * battery -- battery + charge state from the onboard nPM1300.
 *
 * REPORTS MEASUREMENTS ONLY. No state-of-charge percentage.
 *
 * WHY NO PERCENT: a trustworthy SoC needs nrf_fuel_gauge with a `battery_model`
 * characterising THIS cell, produced by Nordic's Battery Model Characterizer.
 * Nordic ships models only for their own EK cells. An earlier version of this
 * file used a hand-written voltage->percent curve; that was invented, and a
 * voltage-derived SoC is worth about +/-10% under load anyway. Until the cell
 * is characterised we report voltage, current, temperature and the charger's
 * own status -- all real measurements -- and no number we cannot defend.
 *
 * All values come from Seeed's board definition and Nordic's driver. Nothing
 * here is configured by this project.
 */
#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirrors the nPM13xx CHARGER.BCHGCHARGESTATUS register. */
enum battery_charge_state {
	BATTERY_STATE_UNKNOWN = 0,
	BATTERY_STATE_IDLE,       /* charger not active                     */
	BATTERY_STATE_TRICKLE,    /* pre-charge, cell below trickle level   */
	BATTERY_STATE_CC,         /* constant current                       */
	BATTERY_STATE_CV,         /* constant voltage                       */
	BATTERY_STATE_COMPLETE,   /* charge terminated                      */
};

struct battery_status {
	bool     valid;          /* false if the PMIC could not be read      */
	int32_t  millivolts;     /* SENSOR_CHAN_GAUGE_VOLTAGE                */
	int32_t  milliamps;      /* SENSOR_CHAN_GAUGE_AVG_CURRENT, signed    */
	/* PMIC DIE temperature, 0.1 C (SENSOR_CHAN_DIE_TEMP). NOT the cell.
	 * The cell-temperature channel (SENSOR_CHAN_GAUGE_TEMP) is deliberately
	 * not read: on the XIAO nRF54LM20A the nPM1300 NTC pin (D3) goes to R6, a
	 * FIXED 10K resistor to GND (Seeed schematic, sheet 03 Power), so it reads
	 * exactly 25.0 C forever. Reporting that would be a fake number that looks
	 * real. The die sits at the battery input, so it is the best real thermal
	 * signal the board has -- a proxy, not a cell measurement. */
	int32_t  die_temp_decidegc;
	/* USB present = VBUSINSTATUS bit 0, VBUSINPRESENT.
	 * Source: Nordic's nPM1300 register header, nordicsemi/npmx adk/npm1300.h
	 * (VBUSIN_VBUSINSTATUS_VBUSINPRESENT_Pos = 0). The register holds SIX
	 * independent flags (present, current-limit, over-voltage, under-voltage,
	 * suspend, VBUSOUT), so `!= 0` is NOT "USB present".
	 *
	 * History, so nobody reverts to either wrong answer:
	 *  - `status != 0` (what Seeed's example does): never false, so the
	 *    over-discharge cutoff never fired -- the board ran on battery through
	 *    three 68-104s unplugs on HW 2026-09-19.
	 *  - Nordic sample's VBUSIN DETECT (CC comparator) check: ALWAYS zero on
	 *    this board -- Seeed terminates the USB-C CC lines with 5.1K resistors
	 *    and does not route them to the PMIC -- so it reported "no USB" while
	 *    plugged in.
	 * FAIL-SAFE: if the PMIC read fails, report true, so a bus error can never
	 * trigger a low-battery shutdown. */
	bool     vbus;
	/* Raw VBUSINSTATUS (0x07), for diagnostics. */
	uint8_t  vbus_status_raw;
	uint32_t charger_error;  /* SENSOR_CHAN_NPM13XX_CHARGER_ERROR, 0=ok  */
	enum battery_charge_state state;
};

/* Bind the PMIC charger. Non-fatal: on failure the caller keeps running with
 * battery reporting disabled. */
int battery_init(void);

/* Sample the PMIC. Always fills `out`; `out->valid` says whether to trust it. */
void battery_read(struct battery_status *out);

const char *battery_state_str(enum battery_charge_state s);

/*
 * Put the nPM1300 into SHIP MODE: everything off, including the nRF54 and the
 * USB debugger. Per Seeed's pinout it wakes on "button press or external power"
 * -- i.e. plugging in USB. Uses Zephyr's regulator_parent_ship_mode() on the
 * nPM13xx regulator device, the same call Nordic's samples use
 * (nrf/samples/zephyr/drivers/regulator/ship_mode). On success it does NOT
 * return. Returns a negative errno if the PMIC refused.
 */
int battery_enter_ship_mode(void);

#ifdef __cplusplus
}
#endif
#endif /* BATTERY_H_ */
