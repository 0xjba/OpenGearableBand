/*
 * nRF54LM20A OLED variant -- M3a: power + IMU bring-up.
 *
 * The nPM1300 LDO1 (regulator-boot-on in the overlay) powers the IMU/mic rail;
 * the LSM6DS3TR-C is probed via the Zephyr sensor API (device_is_ready() == the
 * WHO_AM_I check passed). Prints the gravity vector so tilt is visible.
 *
 * Serial console: r reboot, b bootloader-info, g one-shot gravity read.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static void read_gravity(void)
{
	if (!device_is_ready(imu)) {
		LOG_ERR("IMU not ready (rail off? wrong addr? WHO_AM_I mismatch?)");
		return;
	}

	struct sensor_value acc[3];
	int rc = sensor_sample_fetch(imu);
	if (rc != 0) {
		LOG_ERR("sensor_sample_fetch failed: %d", rc);
		return;
	}
	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);

	double x = sensor_value_to_double(&acc[0]);
	double y = sensor_value_to_double(&acc[1]);
	double z = sensor_value_to_double(&acc[2]);
	LOG_INF("gravity: x=%6.2f y=%6.2f z=%6.2f  |a|=%5.2f m/s^2",
		x, y, z, sqrt(x * x + y * y + z * z));
}

/* 'g' deferred to a thread: I2C can't run in the UART ISR callback. */
static void grav_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	read_gravity();
}
static K_WORK_DEFINE(grav_work, grav_work_handler);

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
				"./flash_nrf54.sh; rebooting cold");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 'g':
			k_work_submit(&grav_work);
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M3a IMU bring-up ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console ready: r reboot, b boot-info, g gravity");
	}

	/* device_is_ready(imu) is true only if the lsm6dsl driver probed the chip
	 * successfully -- i.e. the rail is powered and WHO_AM_I matched. */
	if (device_is_ready(imu)) {
		LOG_INF(">>> IMU ALIVE: LSM6DS3TR-C probed OK (WHO_AM_I matched) <<<");
	} else {
		LOG_ERR(">>> IMU NOT READY -- check LDO1 rail / I2C addr / INT pin <<<");
	}

	/* Print gravity every 2 s so tilting the board is visible. */
	while (1) {
		read_gravity();
		k_sleep(K_SECONDS(2));
	}
	return 0;
}
