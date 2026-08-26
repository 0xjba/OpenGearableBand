/*
 * nRF54LM20A OLED variant -- M3b: orientation + pose detection.
 *
 * A 100 Hz acq thread reads the LSM6DS3TR-C (accel m/s^2 + gyro rad/s), feeds
 * the shared Mahony orientation filter, and runs pose classification on the
 * normalized gravity vector -- the same pipeline as the nRF52840 `beta`
 * firmware (gesture_mode), ported to this board.
 *
 * NOTE: the POSE_EAR canonical in gesture_thresholds.h is tuned for the
 * nRF52840 mount/axes; it will NOT match on this board until re-calibrated for
 * the wrist mount. Use `c` to capture the current gravity as a candidate
 * canonical once the band is worn.
 *
 * Console: r reboot, b boot-info, v toggle verbose trace, c capture pose.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "orientation.h"
#include "gesture_poses.h"
#include "gesture_thresholds.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static atomic_t verbose = ATOMIC_INIT(0);

/* Latest normalized gravity (g-units), for the `c` calibration capture.
 * Written by the acq thread, read in the console cb -- benign race. */
static volatile float g_ngx = 0.0f, g_ngy = 0.0f, g_ngz = 1.0f;

/* ---- acq thread: 100 Hz IMU -> orientation + pose ---- */
#define ACQ_STACK_SZ 2048
#define ACQ_PRIO     7
static K_THREAD_STACK_DEFINE(acq_stack, ACQ_STACK_SZ);
static struct k_thread acq_thread;

static void acq_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (!device_is_ready(imu)) {
		LOG_ERR("IMU not ready -- acq thread exiting");
		return;
	}
	orientation_init();
	LOG_INF("acq thread running (100 Hz); orientation + pose active");

	int64_t last_print = 0;
	while (1) {
		struct sensor_value acc[3], gyr[3];
		if (sensor_sample_fetch(imu) == 0) {
			sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr);

			float ax = sensor_value_to_double(&acc[0]);
			float ay = sensor_value_to_double(&acc[1]);
			float az = sensor_value_to_double(&acc[2]);
			/* Zephyr gyro channels are rad/s -- feed directly. */
			float gx = sensor_value_to_double(&gyr[0]);
			float gy = sensor_value_to_double(&gyr[1]);
			float gz = sensor_value_to_double(&gyr[2]);

			orientation_update(ax, ay, az, gx, gy, gz);

			/* Normalize accel -> unit gravity direction for pose match. */
			float n = sqrtf(ax * ax + ay * ay + az * az);
			float ngx = (n > 0.0f) ? ax / n : 0.0f;
			float ngy = (n > 0.0f) ? ay / n : 0.0f;
			float ngz = (n > 0.0f) ? az / n : 1.0f;
			g_ngx = ngx; g_ngy = ngy; g_ngz = ngz;

			float score = 0.0f;
			pose_id_t best = pose_classify_best(ngx, ngy, ngz,
							    POSE_MATCH_THRESH, &score);

			int64_t now = k_uptime_get();
			int interval = atomic_get(&verbose) ? 200 : 1000;
			if (now - last_print >= interval) {
				last_print = now;
				orientation_state_t st;
				orientation_get(&st);
				LOG_INF("pitch=%6.1f roll=%6.1f yaw=%6.1f rest=%d | "
					"g=(%5.2f,%5.2f,%5.2f) | best=%s score=%.2f",
					(double)st.pitch_deg, (double)st.roll_deg,
					(double)st.yaw_deg, st.at_rest,
					(double)ngx, (double)ngy, (double)ngz,
					pose_name(best), (double)score);
			}
		}
		k_sleep(K_MSEC(10));   /* ~100 Hz */
	}
}

/* ---- console (ISR context: no I2C here; c/v only touch cached state) ---- */
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
		case 'v':
			atomic_set(&verbose, atomic_get(&verbose) ? 0 : 1);
			LOG_INF("verbose trace %s",
				atomic_get(&verbose) ? "ON (5 Hz)" : "OFF (1 Hz)");
			break;
		case 'c':
			/* Capture current gravity as a candidate pose canonical. */
			LOG_INF("POSE CAPTURE -> gx=%.4f gy=%.4f gz=%.4f "
				"(paste into gesture_thresholds.h POSE_*_G*)",
				(double)g_ngx, (double)g_ngy, (double)g_ngz);
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M3b pose ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console: r reboot, b boot-info, v verbose, c capture-pose");
	}

	if (!device_is_ready(imu)) {
		LOG_ERR(">>> IMU NOT READY -- pose disabled <<<");
		return 0;
	}
	LOG_INF(">>> IMU OK -- POSE_EAR canonical is nRF52840-tuned; recalibrate "
		"with `c` when wrist-mounted <<<");

	k_thread_create(&acq_thread, acq_stack, K_THREAD_STACK_SIZEOF(acq_stack),
			acq_loop, NULL, NULL, NULL, ACQ_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&acq_thread, "acq");

	return 0;
}
