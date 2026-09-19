/*
 * resilience -- boot-survival services shared by every gestureband firmware.
 *
 * Exists because of the 2026-09-05 field failure: a peripheral stopped
 * responding and took the WHOLE device down -- no BLE, no console, no recovery,
 * indistinguishable from a brick. Worse, the build had no watchdog and no
 * RESET_ON_FATAL_ERROR, so a genuine hang would have hung forever, and nothing
 * recorded WHY the board restarted. This module fixes all three.
 *
 * Call order in main() matters -- see resilience_boot() docs.
 */
#ifndef RESILIENCE_H_
#define RESILIENCE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot outcome, decided before any optional hardware is touched. */
struct resilience_boot_info {
	uint32_t reset_cause;     /* raw RESETREAS; 0 = power-on or BROWNOUT   */
	const char *cause_str;    /* human-readable, safe to log               */
	uint16_t boot_count;      /* consecutive boots without a healthy run   */
	bool safe_mode;           /* true -> bring up comms ONLY               */
	bool user_forced;         /* safe mode entered by holding the button   */
};

/*
 * Read + clear the reset reason, bump the persistent boot counter, and decide
 * whether this boot is a safe-mode boot.
 *
 * Call this FIRST in main(), before the radio and long before any optional
 * peripheral. `button_held` is the debounced button state sampled at boot; pass
 * false if the product has no button.
 *
 * The counter lives in NVS, not RAM: a brownout loop wipes RAM, so a RAM
 * counter would reset to zero on every cycle and never trip.
 */
const struct resilience_boot_info *resilience_boot(bool button_held);

/*
 * Start the watchdog. Call AFTER the radio is up, so that a fault while
 * bringing up optional hardware reboots us into a state that can still be
 * reached. Returns 0 on success; a failure here is logged, not fatal.
 */
int resilience_watchdog_start(void);

/* Feed the watchdog. Call from the main loop at least twice per timeout. */
void resilience_watchdog_feed(void);

/*
 * Declare the boot good. Clears the boot counter so the NEXT failure starts
 * counting from zero. Called automatically once the device has been up for
 * RESILIENCE_HEALTHY_MS; exposed for a product that has a better definition of
 * "healthy" (e.g. first successful host connection).
 */
void resilience_mark_healthy(void);

/* True if this boot is running in safe mode. */
bool resilience_in_safe_mode(void);

/*
 * Record, in NVS, that the device is about to cut its own power for a low
 * battery. Waking from nPM1300 ship mode is a power-on reset (RESETREAS = 0),
 * which is indistinguishable from a BROWNOUT -- exactly the ambiguity that cost
 * an evening on 2026-09-05. With this note the next boot reports
 * "low-battery shutdown" instead. Call immediately before entering ship mode.
 */
void resilience_note_low_battery_shutdown(void);

#ifdef __cplusplus
}
#endif
#endif /* RESILIENCE_H_ */
