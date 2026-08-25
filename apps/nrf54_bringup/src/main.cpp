/*
 * nRF54LM20A OLED variant -- M1 bring-up app.
 *
 * Proves the custom board definition boots, the serial console works, and the
 * SSD1306 OLED renders text. No PMIC/sensor/BLE code (VOUT2 is autonomous, so
 * the CPU boots without touching the PMIC -- see docs/nrf54lm20a-board.md).
 *
 * Serial console (standing project rule -- every firmware/test app has these):
 *   r  reboot (sys_reboot COLD)
 *   b  UF2 bootloader -- N/A on nRF54L (no Adafruit UF2 loader); documented no-op
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "display_oled.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Interrupt-driven single-letter console. */
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
			/* nRF54L has no UF2 mass-storage bootloader; flashing is
			 * via the SAMD11 SWD bridge. Cold-reboot instead so the
			 * command is never a silent no-op. */
			LOG_INF("no UF2 bootloader on nRF54L -- rebooting cold; "
				"use `west flash` (SWD) to reflash");
			sys_reboot(SYS_REBOOT_COLD);
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M1 bring-up ===");

	if (!device_is_ready(console_dev)) {
		LOG_ERR("console device not ready");
	} else {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console ready: 'r' reboot, 'b' bootloader-info");
	}

	int rc = display_oled_init();
	if (rc == 0) {
		display_oled_show("gestureband", "OLED v1 M1");
		LOG_INF("OLED banner shown");
	} else {
		LOG_ERR("OLED init failed: %d (check I2C wiring/addr)", rc);
	}

	/* Heartbeat so the serial log shows the board is alive. */
	uint32_t beat = 0;
	while (1) {
		LOG_INF("alive %u", beat++);
		k_sleep(K_SECONDS(5));
	}
	return 0;
}
