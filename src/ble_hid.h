/*
 * ble_hid -- HID-over-GATT (HOGP) mouse service
 *
 * Exposes the band as a standard BLE HID mouse so any host OS
 * (Windows / Mac / Linux / Android, iOS with caveats) can receive
 * cursor + click events without an app.
 *
 * Coexists with the existing Heart Rate Service and Battery Service.
 * Both are advertised; hosts subscribe only to what they care about.
 *
 * Security model:
 *   HID requires encrypted/bonded pairing (per the BLE HOGP spec --
 *   prevents a stranger nearby from making your computer act on a
 *   rogue mouse).  Our HRS remains open to keep fitness apps
 *   working without pairing prompts.
 *
 * Architecture reference: see docs/research/gesture-architecture.md
 * section 3 (Hardware capability check -- nRF52840 + Zephyr BLE HID)
 * and section 4 (Use Case 2 - air mouse).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Button bitmask values for ble_hid_send_report().
 * Match the HID report-map definition in the .cpp.
 */
#define BLE_HID_BTN_LEFT    (1U << 0)
#define BLE_HID_BTN_RIGHT   (1U << 1)
#define BLE_HID_BTN_MIDDLE  (1U << 2)

/*
 * Initialise the HID mouse service.  Called once from main() after
 * bt_enable() has completed (i.e. inside or after bt_ready callback).
 *
 * Registers the HOGP service into the GATT table alongside the
 * existing HRS / BAS.  Subsequent bt_le_adv_start should include the
 * HID UUID in the advertising or scan-response payload (handled in
 * main.cpp -- this module only registers the service).
 *
 * Returns 0 on success, negative errno on failure.
 */
int ble_hid_init(void);

/*
 * Send a mouse input report to the connected host (if any).
 *
 *   buttons -- bitmask using BLE_HID_BTN_* values
 *   dx, dy  -- signed relative cursor delta (-127 .. 127)
 *   scroll  -- signed scroll-wheel delta (-127 .. 127, 0 for no scroll)
 *
 * Thread-safe wrt Zephyr's BLE stack: the function blocks internally
 * if the lower-layer notification ring is full.  The cursor pipeline
 * thread should call this at the rate it wants reports published,
 * typically 100-125 Hz when actively cursor-bearing.
 *
 * Returns 0 on success, negative errno on failure (e.g. -ENOTCONN
 * when no host is connected, -EAGAIN when the notification was
 * dropped due to flow control).
 */
int ble_hid_send_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t scroll);

/*
 * Quick query for whether a host is currently subscribed to the
 * input-report notifications.  Cursor pipeline uses this to skip
 * computing reports when no one is listening.
 */
bool ble_hid_notifications_enabled(void);

#ifdef __cplusplus
}
#endif
