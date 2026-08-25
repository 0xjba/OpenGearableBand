/*
 * ble_display -- see ble_display.h.
 */
#include "ble_display.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_display, LOG_LEVEL_INF);

/* 128-bit UUIDs (see header). */
#define BT_UUID_DISP_SVC_VAL \
	BT_UUID_128_ENCODE(0xe9a10001, 0x4b2c, 0x4d3e, 0x9f5a, 0x0123456789ab)
#define BT_UUID_DISP_TXT_VAL \
	BT_UUID_128_ENCODE(0xe9a10002, 0x4b2c, 0x4d3e, 0x9f5a, 0x0123456789ab)

static struct bt_uuid_128 disp_svc_uuid = BT_UUID_INIT_128(BT_UUID_DISP_SVC_VAL);
static struct bt_uuid_128 disp_txt_uuid = BT_UUID_INIT_128(BT_UUID_DISP_TXT_VAL);

static ble_display_text_cb text_cb;

void ble_display_set_handler(ble_display_text_cb cb)
{
	text_cb = cb;
}

static ssize_t txt_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);

	if (text_cb != NULL) {
		text_cb((const uint8_t *)buf, len);
	}
	return len;
}

/* Service: one writable text characteristic. */
BT_GATT_SERVICE_DEFINE(disp_svc,
	BT_GATT_PRIMARY_SERVICE(&disp_svc_uuid),
	BT_GATT_CHARACTERISTIC(&disp_txt_uuid.uuid,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, txt_write, NULL),
);
