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
#define DL_THREAD_PRIO  8                           /* [STRUCTURAL] SOFT codec: BELOW the I2S feeder (7) and
                                                     * PDM capture (6). At 7 (equal to the feeder) a host
                                                     * burst made this thread decode a 16-item backlog without
                                                     * blocking, starving the hard-real-time feeder past its
                                                     * 64 ms DMA window -> I2S underrun -> session death.
                                                     * (2026-06-30 full-duplex stutter fix.) */
#define DL_PREBUF_MS    140                         /* [STRUCTURAL] downlink jitter buffer = host control setpoint (HW-tuned) */
#define DL_STATUS_PERIOD_MS 100                     /* [STRUCTURAL] buffer-feedback cadence */
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

/* Over/underflow events taken from audio_out but not yet delivered to the host.
 * Accumulated across ticks so a failed/dropped status notify never loses an event
 * (the host clock-recovery loop depends on seeing them). Cleared on a successful
 * notify and at session end. */
static uint8_t dl_pending_ev;

/* forward decl required: K_WORK_DELAYABLE_DEFINE takes the function pointer. */
static void dl_status_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dl_status_work, dl_status_fn);

/* Periodic downlink buffer-feedback reporter. Runs only while a session plays
 * (self-stops when audio_out goes idle); notifies only when the host is subscribed
 * to the status char. Payload: [used u16 LE][capacity u16 LE][flags u8]. */
static void dl_status_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!audio_out_active()) {
		dl_pending_ev = 0;        /* session ended: drop stale events, stop the chain */
		return;
	}
	/* Drain audio_out's latch into our accumulator every tick (so it's captured even
	 * when unsubscribed or when a notify fails); clear it only once delivered. */
	dl_pending_ev |= audio_out_take_event_flags();
	if (ble_audio_status_subscribed()) {
		uint16_t used = (uint16_t)audio_out_ring_used();
		uint16_t cap  = (uint16_t)audio_out_ring_capacity();
		uint8_t  flags = (uint8_t)(BLE_AUDIO_STATUS_FL_ACTIVE |
			((dl_pending_ev & AUDIO_OUT_EV_OVERFLOW) ? BLE_AUDIO_STATUS_FL_OVERFLOW : 0) |
			((dl_pending_ev & AUDIO_OUT_EV_UNDERRUN) ? BLE_AUDIO_STATUS_FL_UNDERRUN : 0));
		uint8_t pkt[BLE_AUDIO_STATUS_LEN] = {
			(uint8_t)(used & 0xFF), (uint8_t)(used >> 8),
			(uint8_t)(cap & 0xFF),  (uint8_t)(cap >> 8),
			flags,
		};
		if (ble_audio_notify_status(pkt, sizeof(pkt)) == 0) {
			dl_pending_ev = 0;    /* delivered -> clear; else keep for the next tick */
		}
	}
	k_work_reschedule(&dl_status_work, K_MSEC(DL_STATUS_PERIOD_MS));
}

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
			audio_out_start(DL_RATE_HZ, DL_PREBUF_MS);
			k_work_schedule(&dl_status_work, K_MSEC(DL_STATUS_PERIOD_MS));
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
