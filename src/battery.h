/*
 * battery -- read-only battery + charge state from the XIAO's ONBOARD nPM1300.
 *
 * READ-ONLY BY DESIGN. The charger node carries no `charging-enable`, so this
 * module reports state and never reprograms charge current, termination voltage
 * or thermistor settings. The XIAO charges correctly with no firmware running
 * at all (the charge LED is hardware-driven), so there is nothing to gain by
 * writing those registers -- and getting them wrong on a LiPo is a safety
 * issue, not a bug. Revisit only with the cell's real capacity and NTC fitment.
 */
#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum battery_charge_state {
	BATTERY_UNKNOWN = 0,
	BATTERY_NONE,          /* no cell fitted, or one that is over-discharged */
	BATTERY_DISCHARGING,   /* on battery, no USB                */
	BATTERY_CHARGING,      /* USB present, current flowing in   */
	BATTERY_CHARGED,       /* USB present, charge complete      */
};

struct battery_status {
	bool     valid;         /* false if the PMIC could not be read */
	uint32_t millivolts;
	uint8_t  percent;       /* 0-100, from a LiPo discharge curve  */
	bool     vbus;          /* USB power present                   */
	enum battery_charge_state state;
};

/* Bind the PMIC charger. Returns 0 on success; non-fatal on failure -- the
 * caller keeps running with battery reporting disabled. */
int battery_init(void);

/* Sample the PMIC. Always fills `out`; `out->valid` says whether to trust it. */
void battery_read(struct battery_status *out);

const char *battery_state_str(enum battery_charge_state s);

#ifdef __cplusplus
}
#endif
#endif /* BATTERY_H_ */
