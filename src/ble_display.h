/*
 * ble_display -- minimal custom GATT service exposing one writable
 * "display-text" characteristic. A BLE central (phone/Mac) writes a UTF-8
 * string; the registered handler renders it (on this variant, to the OLED).
 *
 * Decoupled from the display: the app wires the received bytes to
 * display_oled_show(), so this module has no dependency on the panel.
 *
 * 128-bit UUIDs (custom, gestureband display service):
 *   service : e9a10001-4b2c-4d3e-9f5a-0123456789ab
 *   text    : e9a10002-4b2c-4d3e-9f5a-0123456789ab  (WRITE / WRITE_NO_RSP)
 */
#ifndef BLE_DISPLAY_H
#define BLE_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called on each write to the text characteristic. `data` is the raw payload
 * (not NUL-terminated), `len` its length. Runs in the BLE RX context. */
typedef void (*ble_display_text_cb)(const uint8_t *data, uint16_t len);

/* Register the write handler. Call once before advertising. The GATT service
 * itself is registered statically at boot. */
void ble_display_set_handler(ble_display_text_cb cb);

#ifdef __cplusplus
}
#endif

#endif /* BLE_DISPLAY_H */
