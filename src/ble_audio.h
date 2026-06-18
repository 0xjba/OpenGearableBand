/*
 * ble_audio -- minimal custom GATT service that streams LC3 dictation audio
 * over BLE notifications.
 *
 * Scope (sub-project B): ONE notify characteristic, nothing else. No control
 * char, no status char, no host command protocol -- streaming is gesture-gated
 * (the device decides when it dictates; see audio_stream / MODE_DICTATION). The
 * host only subscribes.
 *
 * This module does NOT bootstrap Bluetooth: main.cpp already owns bt_enable(),
 * advertising (HRS+BAS), and its own connection callbacks. ble_audio only adds
 * its GATT service (auto-registered via BT_GATT_SERVICE_DEFINE) and its own
 * connection-tracking callbacks (BT_CONN_CB_DEFINE -- Zephyr invokes every
 * registered set, so this coexists with main.cpp's).
 *
 * Wire format (gestureband-specific; intentionally NOT oneDiary-compatible):
 *   [seq: 2 bytes little-endian][payload: N bytes]
 * For B the payload is one 20 ms encoded block = 80 bytes (two 40-byte LC3
 * frames), so each notification is 82 bytes. The 2-byte sequence number lets the
 * test receiver detect dropped/reordered notifications.
 *
 * 128-bit UUIDs:
 *   Service:   47A10001-9B70-4C2E-8A1D-2F6B9E4A77C1
 *   Audio data 47A10002-9B70-4C2E-8A1D-2F6B9E4A77C1 (NOTIFY + CCC)
 */

#ifndef BLE_AUDIO_H
#define BLE_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 2-byte sequence header prepended to every notification. */
#define BLE_AUDIO_SEQ_HDR_SIZE 2

/* True when a central is connected AND has enabled notifications on the audio
 * data characteristic (CCC). audio_stream uses this as part of its gate. */
bool ble_audio_subscribed(void);

/* Send one notification: [seq16 LE][payload]. Non-blocking -- on a full TX
 * queue the frame is DROPPED (a drop counter is bumped) and -ENOMEM returned;
 * the caller (mic capture thread) never blocks on BLE. Returns 0 on success,
 * -ENOTCONN if not subscribed, -ENOMEM if dropped, other negative on error.
 * Increments the sequence number only on a successful send. */
int ble_audio_notify(const uint8_t *payload, uint16_t len);

/* Count of notifications dropped due to a full TX queue since boot. Read by the
 * bring-up instrumentation to size CONFIG_BT_L2CAP_TX_BUF_COUNT. */
uint32_t ble_audio_drop_count(void);

/* Request a connection interval suited to the current need:
 *   fast=true  -- short interval (~15-30 ms) so the host can sustain the ~50
 *                 notif/s audio rate; called when a dictation burst starts.
 *   fast=false -- relaxed interval (~30-50 ms) to save power; called when the
 *                 burst ends.
 * The central may honour, modify, or ignore the request (e.g. iOS) -- the actual
 * negotiated interval is logged via the le_param_updated callback. No-op if not
 * connected. Returns 0 if the request was issued, negative errno otherwise. */
int ble_audio_set_fast_conn(bool fast);

#ifdef __cplusplus
}
#endif

#endif /* BLE_AUDIO_H */
