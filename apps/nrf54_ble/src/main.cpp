/*
 * nRF54LM20A OLED variant -- M2: BLE -> display-text.
 *
 * Advertises a connectable peripheral (name + battery service + custom display
 * service). A central writes a UTF-8 string to the display-text characteristic
 * and it renders on the OLED. This is the host->wristband text path of the
 * product loop (Gemini text reply -> OLED); the mic/dictation uplink is M3.
 *
 * Serial console (standing rule): r reboot, b bootloader-info.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/bas.h>

#include "display_oled.h"
#include "ble_display.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Advertising: general discoverable, no BR/EDR, complete local name. */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* ---- BLE text -> OLED ---- */
static void on_text(const uint8_t *data, uint16_t len)
{
	/* Copy + NUL-terminate (bounded), split on the first newline into two
	 * lines; otherwise the whole string goes on line 1 (CFB clips overflow). */
	char buf[64];
	uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
	memcpy(buf, data, n);
	buf[n] = '\0';

	char *line2 = strchr(buf, '\n');
	if (line2 != NULL) {
		*line2 = '\0';
		line2++;
	}
	LOG_INF("display-text (%u B): \"%s\"%s%s", len, buf,
		line2 ? " / " : "", line2 ? line2 : "");
	display_oled_show(buf, line2);
}

/* ---- Connection lifecycle ---- */
/* Connectable, general-discoverable, fast advertising interval. */
#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, \
	BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

static void start_adv(void)
{
	int err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("advertising start failed: %d", err);
		display_oled_show("BLE adv", "FAILED");
	} else {
		LOG_INF("advertising as \"%s\"", DEVICE_NAME);
		display_oled_show("gband-OLED", "advertising");
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connect failed: %u", err);
		return;
	}
	LOG_INF("connected");
	display_oled_show("connected", "write text...");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected (reason 0x%02x)", reason);
	start_adv();   /* self-heal: re-advertise (mirrors main.cpp on beta) */
}

BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* ---- Serial console ---- */
/* The UART rx callback runs in ISR context, where a blocking I2C transfer isn't
 * allowed. Draw operations are therefore deferred to the system workqueue. */
static void estate_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	display_oled_estate_test();
}
static K_WORK_DEFINE(estate_work, estate_work_handler);

static void uart_rx_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t c;

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
		return;
	}
	while (uart_fifo_read(dev, &c, 1) == 1) {
		switch (c) {
		case 'r':
			LOG_INF("reboot (cold)");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 'b':
			LOG_INF("no UF2 bootloader on nRF54L -- flash via "
				"./flash_nrf54.sh (probe-rs); rebooting cold");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 'd':
			/* Screen-estate test: deferred to a thread (I2C can't run in
			 * this ISR context). */
			k_work_submit(&estate_work);
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M2 BLE->OLED ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
	}

	if (display_oled_init() == 0) {
		display_oled_show("gband-OLED", "BLE booting");
	}

	ble_display_set_handler(on_text);

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		display_oled_show("BLE", "init FAILED");
		return 0;
	}
	LOG_INF("bluetooth enabled");
	bt_bas_set_battery_level(100);
	start_adv();

	return 0;
}
