/*
 * ble_audio -- minimal LC3 audio streaming GATT service. See ble_audio.h.
 *
 * Borrows the GATT/notify mechanics from oneDiary's ble_service.c but NOT its
 * Bluetooth bootstrap (bt_enable / advertising / mode/control protocol) -- those
 * are owned by main.cpp here. Trimmed to a single notify characteristic.
 */

#include "ble_audio.h"
#include "audio_downlink.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_audio, CONFIG_LOG_DEFAULT_LEVEL);

/* gestureband audio service UUIDs (distinct from oneDiary's base). */
static struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10001, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));

static struct bt_uuid_128 data_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10002, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));

/* Downlink audio (phone -> device): 47A10003-9B70-4C2E-8A1D-2F6B9E4A77C1 */
static struct bt_uuid_128 ble_audio_dl_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10003, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));
/* Downlink control: 47A10004-9B70-4C2E-8A1D-2F6B9E4A77C1 */
static struct bt_uuid_128 ble_audio_ctrl_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10004, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));

/* Connection / subscription state. Owned by this module (its own conn cbs). */
static struct bt_conn *audio_conn;
static bool   data_notifications_enabled;
static uint16_t seq_num;
static uint32_t drop_count;

/* ---- GATT ---- */

static void data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	data_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("audio notifications %s",
		data_notifications_enabled ? "enabled" : "disabled");
}

static ssize_t dl_audio_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset,
			      uint8_t flags)
{
	ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	const uint8_t *p = (const uint8_t *)buf;
	/* Wire format: [seq16 LE][LC3 frames]. Skip the seq, feed the LC3. */
	if (len > BLE_AUDIO_SEQ_HDR_SIZE) {
		audio_downlink_feed(p + BLE_AUDIO_SEQ_HDR_SIZE,
				    len - BLE_AUDIO_SEQ_HDR_SIZE);
	}
	return len;
}

static ssize_t dl_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset,
			     uint8_t flags)
{
	ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(offset); ARG_UNUSED(flags);
	if (len >= 1 && ((const uint8_t *)buf)[0] == BLE_AUDIO_CTRL_FLUSH) {
		audio_downlink_flush();
	}
	return len;
}

/* attrs: [0]=service, [1]=char decl, [2]=char value, [3]=CCC.
 * Notifications target attrs[2]. */
BT_GATT_SERVICE_DEFINE(ble_audio_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	BT_GATT_CHARACTERISTIC(&data_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(data_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* Downlink audio: Write-Without-Response (unacked, high-throughput). */
	BT_GATT_CHARACTERISTIC(&ble_audio_dl_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL, dl_audio_write, NULL),
	/* Downlink control (FLUSH/barge-in). */
	BT_GATT_CHARACTERISTIC(&ble_audio_ctrl_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, dl_ctrl_write, NULL),
);

/* ---- Connection tracking (own callback set; coexists with main.cpp's) ---- */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return; /* main.cpp logs the failure */
	}
	LOG_INF("connected");
	if (audio_conn == NULL) {
		audio_conn = bt_conn_ref(conn);
	}
	seq_num = 0;
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	/* Reason is the HCI disconnect code -- the key bring-up diagnostic for a
	 * drop-right-after-connect (0x08 = supervision timeout, 0x13 = remote user
	 * terminated, 0x3d = MIC failure, 0x22 = LL response timeout, etc.). */
	LOG_INF("disconnected, reason 0x%02x", reason);
	if (audio_conn) {
		bt_conn_unref(audio_conn);
		audio_conn = NULL;
	}
	data_notifications_enabled = false;
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval,
				uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	/* interval is in 1.25 ms units. Logged so the HW bring-up can confirm the
	 * host actually granted a short interval during dictation (the one open
	 * item the research flagged: iOS may negotiate a long interval). */
	LOG_INF("conn params: interval=%u (%u.%02u ms) latency=%u timeout=%u",
		interval, (interval * 5) / 4, ((interval * 5) % 4) * 25,
		latency, timeout);
}

BT_CONN_CB_DEFINE(ble_audio_conn_cb) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.le_param_updated = on_le_param_updated,
};

/* ---- Public API ---- */

bool ble_audio_subscribed(void)
{
	return (audio_conn != NULL) && data_notifications_enabled;
}

uint32_t ble_audio_drop_count(void)
{
	return drop_count;
}

int ble_audio_set_fast_conn(bool fast)
{
	if (audio_conn == NULL) {
		return -ENOTCONN;
	}
	/* Units of 1.25 ms: fast = 15-30 ms, relaxed = 30-50 ms. latency=0,
	 * supervision timeout 4 s. Static structs (C++ can't take the address of
	 * BT_LE_CONN_PARAM's temporary compound-literal array).
	 * [PRODUCTION] host-dependent + a power/throughput trade-off -- re-tune per
	 * the real host (phone vs Mac) and the production power budget. macOS honoured
	 * the fast request as 30 ms in bring-up (2026-06-18); other centrals may clamp
	 * differently. */
	static const struct bt_le_conn_param fast_param =
		BT_LE_CONN_PARAM_INIT(12, 24, 0, 400);
	static const struct bt_le_conn_param relax_param =
		BT_LE_CONN_PARAM_INIT(24, 40, 0, 400);
	int err = bt_conn_le_param_update(audio_conn,
					  fast ? &fast_param : &relax_param);
	if (err && err != -EALREADY) {
		LOG_WRN("conn param update (%s) failed: %d",
			fast ? "fast" : "relax", err);
	}
	return err;
}

int ble_audio_notify(const uint8_t *payload, uint16_t len)
{
	if (audio_conn == NULL || !data_notifications_enabled) {
		return -ENOTCONN;
	}

	/* [seq16 LE][payload]. Sized for the B payload (80-byte block) plus
	 * headroom; the static cap also documents the max packet we ever send. */
	uint8_t pkt[BLE_AUDIO_SEQ_HDR_SIZE + 128];
	uint16_t pkt_len = BLE_AUDIO_SEQ_HDR_SIZE + len;

	if (len == 0 || payload == NULL || pkt_len > sizeof(pkt)) {
		return -EINVAL;
	}

	pkt[0] = seq_num & 0xFF;
	pkt[1] = (seq_num >> 8) & 0xFF;
	memcpy(&pkt[BLE_AUDIO_SEQ_HDR_SIZE], payload, len);

	int err = bt_gatt_notify(audio_conn, &ble_audio_svc.attrs[2], pkt, pkt_len);
	if (err == -ENOMEM) {
		drop_count++;          /* TX queue full -- drop, never block */
		return -ENOMEM;
	} else if (err) {
		LOG_ERR("bt_gatt_notify failed: %d", err);
		return err;
	}

	seq_num++;
	return 0;
}
