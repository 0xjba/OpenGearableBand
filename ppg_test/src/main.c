/*
 * MAX30105 GREEN-LED PPG probe (XIAO nRF52840 Sense).
 *
 * Lights ONLY the green LED (slot 1 = LED3, see prj.conf) and streams its raw
 * 18-bit FIFO counts over serial. Cover the sensor's optical window (fingertip
 * or wrist) and the count should rise (DC) and show a pulsatile wobble at your
 * heart rate. Green (~530 nm) is the wavelength wrist wearables use -- this is
 * the quick "is green better here than red/IR?" check.
 *
 * Console: 'b' = UF2 bootloader, 'r' = reboot.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(ppg, LOG_LEVEL_INF);

/* ---- standing 'b'/'r' console ---- */
static const struct device *console_uart =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static volatile uint8_t pending_cmd;

static void uart_rx_cb(const struct device *dev, void *ud)
{
	ARG_UNUSED(ud);
	uint8_t c;

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
		return;
	}
	while (uart_fifo_read(dev, &c, 1) == 1) {
		if (c == 'r' || c == 'R') {
			pending_cmd = 'r';
		} else if (c == 'b' || c == 'B') {
			pending_cmd = 'b';
		}
	}
}

static void service_console(void)
{
	uint8_t cmd = pending_cmd;

	if (cmd == 'r') {
		pending_cmd = 0;
		LOG_INF("rebooting...");
		k_sleep(K_MSEC(50));
		sys_reboot(SYS_REBOOT_COLD);
	} else if (cmd == 'b') {
		pending_cmd = 0;
		LOG_INF("entering UF2 bootloader...");
		k_sleep(K_MSEC(50));
		NRF_POWER->GPREGRET = 0x57;   /* Adafruit nRF52 "stay in UF2" magic */
		sys_reboot(SYS_REBOOT_COLD);
	}
}

int main(void)
{
	const struct device *max = DEVICE_DT_GET_ONE(maxim_max30101);

	if (!device_is_ready(console_uart)) {
		return 0;
	}
	uart_irq_callback_user_data_set(console_uart, uart_rx_cb, NULL);
	uart_irq_rx_enable(console_uart);

	if (!device_is_ready(max)) {
		LOG_ERR("MAX30105 not ready (wiring/I2C?)  'b'=bootloader 'r'=reboot");
		while (1) {
			service_console();
			k_msleep(50);
		}
	}

	LOG_INF("MAX30105 LED diagnostic -- all 3 LEDs on. Cover the sensor.");
	LOG_INF("A working LED -> counts in the THOUSANDS; ~30 = LED not emitting.");
	LOG_INF("'b'=UF2 bootloader  'r'=reboot");

	while (1) {
		service_console();

		struct sensor_value red, ir, green;

		if (sensor_sample_fetch(max) == 0 &&
		    sensor_channel_get(max, SENSOR_CHAN_RED, &red) == 0 &&
		    sensor_channel_get(max, SENSOR_CHAN_IR, &ir) == 0 &&
		    sensor_channel_get(max, SENSOR_CHAN_GREEN, &green) == 0) {
			LOG_INF("RED=%-7u IR=%-7u GREEN=%-7u",
				(unsigned)red.val1, (unsigned)ir.val1,
				(unsigned)green.val1);
		} else {
			LOG_WRN("read failed");
		}

		k_msleep(33);   /* ~30 prints/sec */
	}
}
