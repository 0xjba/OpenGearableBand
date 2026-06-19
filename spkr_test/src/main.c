/*
 * SD-card WAV player test (MAX98357A) on the XIAO nRF52840 Sense.
 *
 * Mounts a FAT microSD over SPI, opens /SD:/music.wav, parses the WAV header,
 * and streams its 16-bit PCM straight to I2S on a loop. Mono is expanded to
 * stereo; the I2S frame rate follows the file's sample rate. Stock Zephyr
 * i2s_nrfx driver. Console: 'b' = UF2 bootloader, 'r' = reboot.
 *
 * Card: FAT32, 16-bit PCM WAV named music.wav at the root.
 * Wiring: I2S BCLK->D1 LRCK->D0 DIN->D2 ampSD->D7 ; SD CLK->D8 MISO->D9
 *         MOSI->D10 CS->D3 ; SD 3V3->3V3 GND->GND.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>
#include <string.h>

LOG_MODULE_REGISTER(spkr, LOG_LEVEL_INF);

/* ---- standing 'b'/'r' console (ISR sets a flag, loop acts on it) ---- */
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

/* Spin forever servicing the console -- used on fatal setup errors so 'b'/'r'
 * still work without a physical double-tap. */
static void idle_console(void)
{
	while (1) {
		service_console();
		k_msleep(50);
	}
}

/* ---- SD / FAT ---- */
#define WAV_PATH "/SD:/music.wav"

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};

/* ---- WAV ---- */
struct wav_info {
	uint32_t rate;
	uint16_t channels;
	uint16_t bits;
	uint32_t data_off;   /* byte offset of PCM start */
	uint32_t data_len;   /* PCM byte count */
};

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p)
{
	return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

/* Walk the RIFF chunks to find `fmt ` and `data`. */
static int parse_wav(struct fs_file_t *f, struct wav_info *w)
{
	uint8_t hdr[12];

	if (fs_read(f, hdr, 12) != 12) {
		return -1;
	}
	if (memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
		return -2;
	}

	bool have_fmt = false;

	while (1) {
		uint8_t ch[8];

		if (fs_read(f, ch, 8) != 8) {
			return -3;
		}
		uint32_t sz = rd32(ch + 4);

		if (!memcmp(ch, "fmt ", 4)) {
			uint8_t fmt[16];

			if (fs_read(f, fmt, 16) != 16) {
				return -4;
			}
			w->channels = rd16(fmt + 2);
			w->rate     = rd32(fmt + 4);
			w->bits     = rd16(fmt + 14);
			have_fmt = true;
			if (sz > 16) {
				fs_seek(f, sz - 16, FS_SEEK_CUR);
			}
		} else if (!memcmp(ch, "data", 4)) {
			w->data_len = sz;
			w->data_off = (uint32_t)fs_tell(f);
			return have_fmt ? 0 : -5;
		} else {
			fs_seek(f, sz + (sz & 1), FS_SEEK_CUR);   /* skip, word-align */
		}
	}
}

/* ---- I2S streaming ---- */
#define FRAMES_PER_BLOCK  512                              /* ~11.6 ms @ 44.1k */
#define I2S_BLOCK_BYTES   (FRAMES_PER_BLOCK * 2 * 2)       /* stereo 16-bit */
#define NUM_BLOCKS        6                                /* buffer SD latency */

K_MEM_SLAB_DEFINE(tx_slab, I2S_BLOCK_BYTES, NUM_BLOCKS, 4);

static struct fs_file_t wav;
static struct wav_info  wi;
static uint32_t         pos;                /* bytes consumed from data chunk */
static uint8_t          filebuf[I2S_BLOCK_BYTES];

/* Read one block of source PCM (looping at EOF) and lay it into the stereo
 * I2S block -- mono duplicated to L+R, stereo copied straight through. */
static void fill_block(int16_t *block)
{
	uint32_t want = FRAMES_PER_BLOCK * wi.channels * 2;   /* source bytes */
	uint32_t got = 0;

	while (got < want) {
		uint32_t remain = wi.data_len - pos;

		if (remain == 0) {
			fs_seek(&wav, wi.data_off, FS_SEEK_SET);   /* loop the track */
			pos = 0;
			continue;
		}
		uint32_t chunk = MIN(want - got, remain);
		int r = fs_read(&wav, filebuf + got, chunk);

		if (r <= 0) {                                  /* read error -> silence */
			memset(filebuf + got, 0, want - got);
			got = want;
			break;
		}
		got += r;
		pos += r;
	}

	if (wi.channels == 1) {
		const int16_t *src = (const int16_t *)filebuf;

		for (int i = 0; i < FRAMES_PER_BLOCK; i++) {
			block[2 * i] = block[2 * i + 1] = src[i];
		}
	} else {
		memcpy(block, filebuf, I2S_BLOCK_BYTES);
	}
}

int main(void)
{
	const struct device *i2s   = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(i2s) || !device_is_ready(gpio1) ||
	    !device_is_ready(console_uart)) {
		LOG_ERR("device not ready");
		return 0;
	}

	uart_irq_callback_user_data_set(console_uart, uart_rx_cb, NULL);
	uart_irq_rx_enable(console_uart);

	gpio_pin_configure(gpio1, 12, GPIO_OUTPUT_ACTIVE);   /* amp SD (D7) high */

	int rc = fs_mount(&mp);
	if (rc) {
		LOG_ERR("SD mount failed: %d (card inserted? FAT32? wiring?)", rc);
		idle_console();
	}
	LOG_INF("SD mounted");

	fs_file_t_init(&wav);
	rc = fs_open(&wav, WAV_PATH, FS_O_READ);
	if (rc) {
		LOG_ERR("open %s failed: %d", WAV_PATH, rc);
		idle_console();
	}

	rc = parse_wav(&wav, &wi);
	if (rc) {
		LOG_ERR("WAV parse failed: %d", rc);
		idle_console();
	}
	LOG_INF("WAV: %u Hz, %u ch, %u-bit, %u KB PCM",
		wi.rate, wi.channels, wi.bits, wi.data_len / 1024);

	if (wi.bits != 16 || wi.channels < 1 || wi.channels > 2) {
		LOG_ERR("need a 16-bit mono/stereo WAV");
		idle_console();
	}

	struct i2s_config cfg = {
		.word_size      = 16,
		.channels       = 2,               /* stereo I2S framing */
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = wi.rate,         /* follow the file's rate */
		.mem_slab       = &tx_slab,
		.block_size     = I2S_BLOCK_BYTES,
		.timeout        = 2000,
	};
	rc = i2s_configure(i2s, I2S_DIR_TX, &cfg);
	if (rc) {
		LOG_ERR("i2s_configure: %d", rc);
		idle_console();
	}

	fs_seek(&wav, wi.data_off, FS_SEEK_SET);
	pos = 0;
	LOG_INF("playing %s on loop  ('b'=bootloader 'r'=reboot)", WAV_PATH);

	int primed = 0;

	while (1) {
		service_console();

		void *block;
		rc = k_mem_slab_alloc(&tx_slab, &block, K_MSEC(200));
		if (rc) {
			LOG_WRN("slab alloc timeout");
			continue;
		}

		fill_block((int16_t *)block);

		rc = i2s_write(i2s, block, I2S_BLOCK_BYTES);
		if (rc) {
			LOG_ERR("i2s_write: %d", rc);
			k_mem_slab_free(&tx_slab, block);
			k_msleep(50);
			continue;
		}

		if (primed < 2 && ++primed == 2) {
			rc = i2s_trigger(i2s, I2S_DIR_TX, I2S_TRIGGER_START);
			if (rc) {
				LOG_ERR("START: %d", rc);
			} else {
				LOG_INF("I2S started -- music should be playing");
			}
		}
	}
}
