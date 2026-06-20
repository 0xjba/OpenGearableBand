/*
 * ble_audio -- custom GATT service for the BLE audio path (uplink LC3 dictation
 * stream + downlink playback + buffer feedback).
 *
 * This module does NOT bootstrap Bluetooth: main.cpp owns bt_enable(), advertising
 * (HRS+BAS), and its own connection callbacks. ble_audio adds its GATT service
 * (auto-registered via BT_GATT_SERVICE_DEFINE) plus its own connection-tracking
 * callbacks (BT_CONN_CB_DEFINE -- Zephyr invokes every registered set, so this
 * coexists with main.cpp's).
 *
 * Uplink wire format (notifications, gestureband-specific):
 *   [seq: 2 bytes little-endian][payload: N bytes]   (B payload = one 80-byte LC3
 *   block, so each notification is 82 bytes; the seq lets the receiver detect drops.)
 *
 * 128-bit UUIDs (base ...-9B70-4C2E-8A1D-2F6B9E4A77C1):
 *   Service:           47A10001
 *   Uplink audio       47A10002  (NOTIFY + CCC)        device -> host LC3 stream
 *   Downlink audio     47A10003  (WRITE / WRITE_NR)    host -> device LC3 stream
 *   Downlink control   47A10004  (WRITE)               1-byte cmd; 0x01 = FLUSH
 *   Downlink status    47A10005  (NOTIFY + CCC)        device -> host buffer feedback
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

/* Downlink control characteristic commands (1 byte). */
#define BLE_AUDIO_CTRL_FLUSH 0x01   /* barge-in: stop + clear buffered audio */

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

/* Downlink status report length: [ring_used u16 LE][ring_capacity u16 LE][flags u8].
 * flags bit0=session active, bit1=overflowed-since-last, bit2=underran-since-last. */
#define BLE_AUDIO_STATUS_LEN 5

/* True when a central has subscribed (CCC) to the downlink status characteristic.
 * The downlink reporter gates its notifications on this. */
bool ble_audio_status_subscribed(void);

/* Notify the downlink status characteristic. payload must be BLE_AUDIO_STATUS_LEN
 * bytes. Non-blocking; returns 0 on success, -ENOTCONN if not subscribed, -EINVAL
 * on a bad payload, other negative on error. Distinct from ble_audio_notify() (the
 * uplink audio char). */
int ble_audio_notify_status(const uint8_t *payload, uint16_t len);

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
