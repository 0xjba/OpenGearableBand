/*
 * nRF54LM20A OLED variant -- M4: converged firmware.
 *
 * The complete OLED product loop, input + output:
 *   IN : acq thread (100 Hz IMU) -> gesture_mode (orientation + POSE_EAR gate) ->
 *        mic_vad (voice-onset -> MODE_DICTATION) -> audio_stream -> LC3 ->
 *        ble_audio NOTIFY (mic uplink to the host / Gemini).
 *   OUT: ble_display "display-text" characteristic -> OLED (host renders Gemini's
 *        TEXT reply here; there is NO speaker/audio-downlink on this variant).
 *
 * No HR, no audio-downlink, no AEC/barge-in (structurally absent -- no speaker).
 * Console: r reboot, b boot-info, v verbose pose/mode trace.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "display_oled.h"
#include "ble_display.h"
#include "gesture_mode.h"
#include "mic_vad.h"
#include "audio_stream.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static atomic_t verbose = ATOMIC_INIT(0);
static atomic_t host_text_shown = ATOMIC_INIT(0);   /* host text overrides idle screen */
static atomic_t force_dict = ATOMIC_INIT(0);        /* bench: force dictation (bypass pose) */

/* ---- BLE display-text -> OLED ---- */
static void on_text(const uint8_t *data, uint16_t len)
{
	char buf[64];
	uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
	memcpy(buf, data, n);
	buf[n] = '\0';
	char *line2 = strchr(buf, '\n');
	if (line2) { *line2 = '\0'; line2++; }
	LOG_INF("display-text: \"%s\"%s%s", buf, line2 ? " / " : "", line2 ? line2 : "");
	atomic_set(&host_text_shown, 1);
	display_oled_show(buf, line2);
}

/* ---- advertising (deferred restart avoids the ble_audio teardown race) ---- */
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
static void adv_work_handler(struct k_work *w) { ARG_UNUSED(w); start_adv(); }
static K_WORK_DEFINE(adv_work, adv_work_handler);

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("disconnected (0x%02x) -> re-advertise (deferred)", reason);
	/* Defer: running bt_le_adv_start directly in the disconnect callback races
	 * ble_audio's conn teardown (seen as adv start -12 in M3c.2). */
	k_work_submit(&adv_work);
}
BT_CONN_CB_DEFINE(app_conn_cb) = { .disconnected = on_disconnected };

/* ---- acq thread: 100 Hz IMU -> gesture_mode (drives pose gate + mic uplink) ---- */
#define ACQ_STACK_SZ 2048
/* Below the audio thread (prio 8) so the 100 Hz IMU I2C + Mahony + OLED writes
 * can't preempt LC3 encode/BLE-notify. At prio 7 (above audio) it starved the
 * uplink -> encode+notify spiked 3->40 ms and audio_stream q_drops climbed.
 * mic capture stays prio 6 (highest). */
#define ACQ_PRIO     10
static K_THREAD_STACK_DEFINE(acq_stack, ACQ_STACK_SZ);
static struct k_thread acq_thread;

static void acq_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	if (!device_is_ready(imu)) {
		LOG_ERR("IMU not ready -- acq disabled");
		return;
	}

	GestureMode last_mode = MODE_IDLE;
	int64_t last_trace = 0;

	while (1) {
		struct sensor_value acc[3], gyr[3];
		if (sensor_sample_fetch(imu) == 0) {
			sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr);
			float ax = sensor_value_to_double(&acc[0]);
			float ay = sensor_value_to_double(&acc[1]);
			float az = sensor_value_to_double(&acc[2]);
			float gx = sensor_value_to_double(&gyr[0]);
			float gy = sensor_value_to_double(&gyr[1]);
			float gz = sensor_value_to_double(&gyr[2]);

			/* accel stashed, then gyro drives orientation + pose + ear gate. */
			gesture_mode_update_accel(ax, ay, az);
			gesture_mode_update_gyro(gx, gy, gz);
		}

		/* Reflect dictation state on the OLED (unless host text is showing). */
		GestureMode m = gesture_mode_get();
		if (m != last_mode) {
			last_mode = m;
			if (m == MODE_DICTATION) {
				atomic_set(&host_text_shown, 0);
				display_oled_show("listening", "...");
			} else if (!atomic_get(&host_text_shown)) {
				display_oled_show("gband-oled", "ready");
			}
			LOG_INF("mode -> %s", gesture_mode_str(m));
		}

		if (atomic_get(&verbose)) {
			int64_t now = k_uptime_get();
			if (now - last_trace >= 200) {
				last_trace = now;
				float ggx, ggy, ggz;
				gesture_mode_get_gravity(&ggx, &ggy, &ggz);
				LOG_INF("g=(%5.2f,%5.2f,%5.2f) pose=%s mode=%s",
					(double)ggx, (double)ggy, (double)ggz,
					pose_name(gesture_mode_armed_pose()),
					gesture_mode_str(m));
			}
		}
		k_sleep(K_MSEC(10));   /* ~100 Hz */
	}
}

/* ---- console ---- */
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
			LOG_INF("reboot (cold)"); sys_reboot(SYS_REBOOT_COLD); break;
		case 'b':
			LOG_INF("no UF2 on nRF54L -- ./flash_nrf54.sh; rebooting");
			sys_reboot(SYS_REBOOT_COLD); break;
		case 'v':
			atomic_set(&verbose, atomic_get(&verbose) ? 0 : 1);
			LOG_INF("verbose %s", atomic_get(&verbose) ? "ON" : "OFF");
			break;
		case 'd': {
			/* Bench test: force dictation on/off, bypassing POSE_EAR. */
			bool on = !atomic_get(&force_dict);
			atomic_set(&force_dict, on ? 1 : 0);
			gesture_mode_debug_force_dictation(on);
			LOG_INF("FORCE dictation %s (subscribe on host + speak)",
				on ? "ON" : "OFF");
			break;
		}
		default: break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M4 converged ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console: r reboot, b boot-info, v verbose, d force-dictation");
	}

	if (display_oled_init() == 0) {
		display_oled_show("gband-oled", "booting");
	}
	ble_display_set_handler(on_text);

	gesture_mode_init();
	mic_vad_init();          /* gesture_mode's ear gate starts/stops it */
	audio_stream_init();     /* LC3 codec (audio thread auto-starts) */

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		display_oled_show("BLE", "FAILED");
		return 0;
	}
	LOG_INF("bluetooth enabled");
	start_adv();

	if (device_is_ready(imu)) {
		k_thread_create(&acq_thread, acq_stack,
				K_THREAD_STACK_SIZEOF(acq_stack), acq_loop,
				NULL, NULL, NULL, ACQ_PRIO, 0, K_NO_WAIT);
		k_thread_name_set(&acq_thread, "acq");
		display_oled_show("gband-oled", "ready");
		LOG_INF("ready: raise-to-ear + speak -> mic uplink; host writes text -> OLED");
	} else {
		LOG_ERR("IMU not ready -- gesture path disabled");
		display_oled_show("IMU", "NOT READY");
	}

	return 0;
}
