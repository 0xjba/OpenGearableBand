/*
 * audio_stream -- mic-capture -> LC3 -> BLE bridge for dictation. See header.
 *
 * Producer/consumer split (per the 2026-06-18 deep-research finding): the PDM
 * capture thread (mic_vad) MUST NOT do the LC3 encode + BLE notify itself --
 * Zephyr's nrfx PDM driver fail-STOPS the moment its slab alloc returns -ENOMEM
 * (K_NO_WAIT), so the slab has to be drained faster than it fills. Holding a slab
 * buffer through the variable-latency notify starved it (-12 / -EAGAIN, ~1 s
 * dropouts). So:
 *
 *   audio_stream_feed() [capture thread]: COPY the 320-sample block into a msgq
 *     and return immediately, letting mic_vad free the slab at once.
 *   audio_thread() [this module]: drain the msgq, LC3-encode, BLE-notify --
 *     entirely off the capture path. Lower priority than the capture thread so it
 *     can never delay the slab drain.
 *
 * This mirrors Nordic's own LE-Audio reference (a FIFO runs the audio datapath in
 * its own thread; block-based FIFO between stages).
 */

#include "audio_stream.h"
#include "lc3_codec.h"
#include "ble_audio.h"
#include "gesture_mode.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_stream, CONFIG_LOG_DEFAULT_LEVEL);

/* Block we expect from mic_vad: 320 samples (20 ms @ 16 kHz) = 2 LC3 frames. */
#define AUDIO_STREAM_BLOCK_SAMPLES (LC3_FRAME_SAMPLES * LC3_FRAMES_PER_PDM_BUF)
#define AUDIO_STREAM_BLOCK_BYTES   (AUDIO_STREAM_BLOCK_SAMPLES * 2)

/* PCM hand-off queue (capture thread -> audio thread). Each message is one raw
 * 20 ms PCM block; k_msgq copies it in, so the slab buffer is released the moment
 * audio_stream_feed() returns. Depth gives slack for transient encode/notify
 * jitter without ever back-pressuring (full -> drop, never block the producer). */
#define AUDIO_STREAM_Q_DEPTH 16   /* ~320 ms buffer. Deeper than 6 so a transient
                                   * BLE-notify stall (waiting on a conn event)
                                   * doesn't drop mic blocks -- dropped blocks make
                                   * gappy audio that fails host ASR. */
K_MSGQ_DEFINE(audio_pcm_q, AUDIO_STREAM_BLOCK_BYTES, AUDIO_STREAM_Q_DEPTH, 4);

/* Idle timeout: after this long with no PCM, treat the dictation burst as ended
 * and relax the BLE connection interval back to a low-power setting. */
#define AUDIO_STREAM_IDLE_MS 500

/* Bring-up instrumentation: log encode timing + drops every N blocks while
 * streaming (~50 blocks * 20 ms = ~1 s). Confirms the encode+notify cost and
 * that the producer/consumer split holds. */
#define AUDIO_STREAM_LOG_EVERY 50

static uint32_t q_drop_count;   /* PCM blocks dropped because the msgq was full */

int audio_stream_init(void)
{
	int rc = lc3_codec_init();
	if (rc) {
		LOG_ERR("audio_stream: LC3 codec init failed (%d)", rc);
		return rc;
	}
	LOG_INF("audio_stream ready (producer/consumer, q=%d)", AUDIO_STREAM_Q_DEPTH);
	return 0;
}

bool audio_stream_active(void)
{
	return (gesture_mode_get() == MODE_DICTATION) && ble_audio_subscribed();
}

void audio_stream_feed(const int16_t *pcm, size_t nsamp)
{
	if (!audio_stream_active()) {
		return;
	}
	if (nsamp != AUDIO_STREAM_BLOCK_SAMPLES || pcm == NULL) {
		return; /* defensive: only the known 20 ms block shape */
	}

	/* Copy into the queue and return at once (slab freed right after by the
	 * caller). Never block the capture thread: full queue -> drop. */
	if (k_msgq_put(&audio_pcm_q, pcm, K_NO_WAIT) != 0) {
		q_drop_count++;
	}
}

/* Consumer: encode + notify off the capture path. */
static void audio_thread(void *, void *, void *)
{
	int16_t pcm[AUDIO_STREAM_BLOCK_SAMPLES];
	bool fast_conn = false;
	uint32_t blk = 0;
	/* Cumulative captured mic-sample count -> ts32 in each uplink frame, so the host
	 * can estimate the device clock rate live for AEC drift compensation. Advances per
	 * block dequeued (even if the BLE notify later drops), so a dropped frame leaves a
	 * matching gap in ts32. Cumulative from boot; the host resets its estimator per
	 * connection, so the absolute value (and the ~74 h uint32 wrap) is irrelevant. */
	uint32_t mic_samples = 0;

	for (;;) {
		int rc = k_msgq_get(&audio_pcm_q, pcm, K_MSEC(AUDIO_STREAM_IDLE_MS));
		if (rc != 0) {
			/* Idle: dictation burst ended -> relax the link to save power. */
			if (fast_conn) {
				ble_audio_set_fast_conn(false);
				fast_conn = false;
			}
			continue;
		}

		/* ts32 for this frame = index of its first sample; advance per captured block. */
		uint32_t ts32 = mic_samples;
		mic_samples += AUDIO_STREAM_BLOCK_SAMPLES;

		/* First block of a burst: ask for a short connection interval so the
		 * host can sustain ~50 notif/s. (Whether iOS honours it is measured on
		 * HW via the le_param_updated log.) */
		if (!fast_conn) {
			ble_audio_set_fast_conn(true);
			fast_conn = true;
		}

		uint32_t t0 = k_cycle_get_32();

		uint8_t encoded[LC3_ENCODED_PDM_BUF_SIZE];
		uint16_t enc_len = 0;
		if (lc3_codec_encode_pcm_buffer(pcm, encoded, &enc_len) == 0) {
			/* Drop-on-ENOMEM handled inside ble_audio_notify (never blocks). */
			(void)ble_audio_notify(ts32, encoded, enc_len);
		} else {
			LOG_ERR("audio_stream: encode failed");
		}

		uint32_t dt_cyc = k_cycle_get_32() - t0;
		if ((++blk % AUDIO_STREAM_LOG_EVERY) == 0) {
			uint32_t us = (uint32_t)k_cyc_to_us_floor64(dt_cyc);
			LOG_INF("[AUD] encode+notify=%u us, ble_drops=%u q_drops=%u",
				us, ble_audio_drop_count(), q_drop_count);
		}
	}
}

/* Priority 8: SOFT codec work, strictly BELOW the PDM capture thread (6) AND the
 * I2S feeder (7). Both of those are hard-real-time -- capture fail-stops on slab
 * starvation, the feeder DMA-underruns (which KILLS the I2S session) -- whereas this
 * encode/notify is buffered (msgq + drop-on-full) and must yield to them. At 7 (equal
 * to the feeder) a burst starved this thread for 234 ms and the feeder past its 64 ms
 * window -> I2S session death + dropped mic blocks. (2026-06-30 full-duplex stutter fix.)
 * Stack 6144: measured high-water during LC3 encode+notify was 3520 B
 * (2026-06-18 thread-analyzer bring-up); 6144 leaves comfortable FPU margin. */
K_THREAD_DEFINE(audio_thread_id, 6144, audio_thread, NULL, NULL, NULL, 8, 0, 0);
