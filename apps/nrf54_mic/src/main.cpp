/*
 * nRF54LM20A OLED variant -- M3c.1: PDM mic + mic_vad bring-up.
 *
 * Brings up the PDM mic (powered by nPM1300 LDO1) and the shared mic_vad
 * capture + voice-activity path (adaptive noise floor + M-of-N voiced-onset via
 * CMSIS rFFT). The mic runs continuously here so you can watch the [MIC] blocks
 * respond to speech; the LC3->BLE uplink is wired in M3c.2 (audio_stream stubbed).
 *
 * Console: r reboot, b boot-info, m toggle mic on/off.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

#include "mic_vad.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static atomic_t mic_on = ATOMIC_INIT(0);

static void set_mic(bool on)
{
	if (on) {
		mic_vad_start();
		atomic_set(&mic_on, 1);
		LOG_INF("mic ON (speak near the mic; watch [MIC] blocks)");
	} else {
		mic_vad_stop();
		atomic_set(&mic_on, 0);
		LOG_INF("mic OFF");
	}
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
		case 'm':
			set_mic(!atomic_get(&mic_on));
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	LOG_INF("=== gestureband nRF54LM20A OLED variant -- M3c.1 mic ===");

	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
		LOG_INF("console: r reboot, b boot-info, m toggle mic");
	}

	mic_vad_init();
	set_mic(true);   /* auto-start so speaking is immediately visible */

	/* Report voiced-onset latches (the key VAD output). */
	while (1) {
		if (atomic_get(&mic_on) && mic_vad_voice_onset()) {
			LOG_INF(">>> VOICE ONSET detected <<<");
		}
		k_sleep(K_MSEC(50));
	}
	return 0;
}
