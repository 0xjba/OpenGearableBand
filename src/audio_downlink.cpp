/*
 * audio_downlink -- see audio_downlink.h. Decouples the BLE RX context from the
 * LC3 decode + I2S play (the discipline that fixed sub-project B's slab starvation).
 */
#include "audio_downlink.h"
#include "audio_out.h"
#include "lc3_codec.h"
#include "ble_audio.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(audio_downlink, LOG_LEVEL_INF);

#define DL_RATE_HZ      16000                       /* [UNIT] matches LC3 16 kHz */
#define DL_MAX_PAYLOAD  240                         /* [STRUCTURAL] up to 6 x 40 B frames per write */
#define DL_MSGQ_DEPTH   16                          /* [STRUCTURAL] downlink jitter queue */
#define DL_THREAD_PRIO  7                           /* [STRUCTURAL] decode thread priority (uplink-audio tier) */
/* [STRUCTURAL] decode thread stack. liblc3's lc3_decode is stack-hungry (MDCT +
 * temp buffers); 2048 overflowed on the first frame -> MPU stack-guard fault ->
 * silent halt (CONFIG_RESET_ON_FATAL_ERROR off). Measured peak under load = 2784 B
 * (Zephyr thread analyzer, 2026-06-21); 4096 = ~47% margin over that. Decode stack
 * is data-independent (no recursion), so the peak is stable across content. */
#define DL_THREAD_STACK 4096

struct dl_item {
	uint16_t len;
	uint8_t  data[DL_MAX_PAYLOAD];
};

K_MSGQ_DEFINE(dl_msgq, sizeof(struct dl_item), DL_MSGQ_DEPTH, 4);

static uint32_t dl_drops;
static int16_t  dl_pcm[LC3_FRAME_SAMPLES];          /* 160 samples, decode scratch */

static void dl_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct dl_item item;
	bool fast = false;

	while (1) {
		if (k_msgq_get(&dl_msgq, &item, K_MSEC(500)) != 0) {
			/* No frames for 500 ms -> stream idle: relax the BLE interval. */
			if (fast) {
				ble_audio_set_fast_conn(false);
				fast = false;
			}
			continue;
		}

		/* Auto-start a session on the first frame after idle. If start returns
		 * -EBUSY (a previous session's feeder is still in its ~50 ms cleanup),
		 * this frame's writes no-op-drop on audio_out's active check and the
		 * next frame retries -- a ~20 ms gap at a rapid stop->start, benign. */
		if (!audio_out_active()) {
			audio_out_start(DL_RATE_HZ);
		}
		if (!fast) {
			ble_audio_set_fast_conn(true);   /* sustain ~50 writes/s */
			fast = true;
		}

		for (uint16_t off = 0; off + LC3_FRAME_BYTES <= item.len;
		     off += LC3_FRAME_BYTES) {
			if (lc3_codec_decode_frame(item.data + off, dl_pcm) == 0) {
				audio_out_write(dl_pcm, LC3_FRAME_SAMPLES);
			}
		}
	}
}

K_THREAD_DEFINE(dl_thread_id, DL_THREAD_STACK, dl_thread, NULL, NULL, NULL,
		DL_THREAD_PRIO, 0, 0);

int audio_downlink_init(void)
{
	LOG_INF("audio_downlink ready");
	return 0;
}

void audio_downlink_feed(const uint8_t *lc3, size_t len)
{
	if (lc3 == NULL || len == 0 || len > DL_MAX_PAYLOAD) {
		dl_drops++;
		return;
	}
	struct dl_item item;
	item.len = (uint16_t)len;
	memcpy(item.data, lc3, len);
	if (k_msgq_put(&dl_msgq, &item, K_NO_WAIT) != 0) {
		dl_drops++;   /* queue full -- never block the BLE RX context */
	}
}

void audio_downlink_flush(void)
{
	k_msgq_purge(&dl_msgq);
	audio_out_flush();
	LOG_INF("downlink flush (barge-in)");
}

uint32_t audio_downlink_drop_count(void)
{
	return dl_drops;
}
