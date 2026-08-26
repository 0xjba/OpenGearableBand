/*
 * nRF54LM20A OLED variant -- M3c.2: mic -> LC3 -> BLE uplink.
 *
 * The full uplink path: mic_vad captures 20 ms PCM blocks and feeds audio_stream;
 * when dictation is active AND a host has subscribed to the audio characteristic,
 * the audio thread LC3-encodes each block and notifies it over BLE
 * (ble_audio, wire format [seq16][ts32][LC3], see docs/device-ble-contract.md).
 *
 * gesture_mode is stubbed here -- press `d` to force MODE_DICTATION on/off so the
 * uplink can be exercised without the full gesture FSM. audio_downlink is stubbed
 * (the OLED variant has no speaker/downlink).
 *
 * Console: r reboot, b boot-info, d toggle dictation (streaming gate).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "mic_vad.h"
#include "audio_stream.h"
#include "ble_audio.h"
#include "gesture_mode.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

extern "C" void gesture_mode_stub_set(GestureMode m);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static atomic_t dictation = ATOMIC_INIT(0);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, \
	BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

static void start_adv(void)
{
	int err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("advertising start failed: %d", err);
	} else {
		LOG_INF("advertising as \"%s\"", DEVICE_NAME);
	}
}

/* Advertising restart on disconnect (ble_audio has its own conn cb for the
 * audio_conn tracking; multiple conn callbacks coexist). */
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("disconnected (reason 0x%02x) -> re-advertising", reason);
	start_adv();
}
BT_CONN_CB_DEFINE(app_conn_cb) = {
	.disconnected = on_disconnected,
};

static void set_dictation(bool on)
{
	atomic_set(&dictation, on ? 1 : 0);
	gesture_mode_stub_set(on ? MODE_DICTATION : MODE_IDLE);
	LOG_INF("dictation %s (streams when host subscribed: %s)",
		on ? "ON" : "OFF",
		ble_audio_subscribed() ? "yes" : "not yet");
}

static void uart_rx_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t ch;

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
		return;
	}
	while (uart_fifo_read(dev, &ch, 1) == 1) {
		switch (ch) {
		case 'r':
			LOG_INF("reboot (cold)");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 'b':
			LOG_INF("no UF2 bootloader on nRF54L -- flash via "
				"./flash_nrf54.sh; rebooting cold");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 'd':
			set_dictation(!atomic_get(&dictation));
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M3c.2 LC3 uplink ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console: r reboot, b boot-info, d toggle dictation");
	}

	mic_vad_init();
	mic_vad_start();          /* capture continuously; feeds audio_stream */
	audio_stream_init();      /* LC3 codec init (audio thread auto-starts) */

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return 0;
	}
	LOG_INF("bluetooth enabled");
	start_adv();

	LOG_INF("ready: connect + subscribe to the audio char, press 'd', then speak");

	/* Periodic status so the streaming gate state is visible. */
	while (1) {
		if (atomic_get(&dictation)) {
			LOG_INF("[uplink] dictation=ON subscribed=%d drops=%u",
				ble_audio_subscribed(), ble_audio_drop_count());
		}
		k_sleep(K_SECONDS(2));
	}
	return 0;
}
