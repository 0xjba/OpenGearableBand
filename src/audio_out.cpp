/*
 * audio_out -- speaker output engine. See audio_out.h.
 *
 * All I2S/amp hardware access lives in the feeder thread; start()/stop() only
 * flip an atomic flag + signal a semaphore, and write() only fills the ring.
 * This keeps every i2s_* call on one thread (no cross-thread driver races) and
 * keeps the producer (BLE/test) decoupled from I2S timing -- the same discipline
 * that fixed the sub-project-B slab starvation.
 */
#include "audio_out.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_out, LOG_LEVEL_INF);

/* --- tunables (tag for housing/production retune) --- */
#define FRAMES_PER_BLOCK   256                          /* [STRUCTURAL] I2S block: latency vs jitter */
#define I2S_BLOCK_BYTES    (FRAMES_PER_BLOCK * 2 * 2)   /* stereo 16-bit */
#define NUM_BLOCKS         4                            /* [STRUCTURAL] mem-slab depth */
#define RING_BYTES         16384                        /* [STRUCTURAL] ~0.5 s @ 16 kHz mono jitter buffer */
#define IDLE_STOP_MS       400                          /* [STRUCTURAL] silence before auto-stop */
#define PREBUF_BYTES       (RING_BYTES / 2)             /* [STRUCTURAL] fill ring ~half before starting playback */
#define PREBUF_TRIES       60                           /* [STRUCTURAL] 60 x 5 ms = 300 ms cap (short clips) */
#define FEEDER_STACK       2048
#define FEEDER_PRIO        7                            /* same tier as the uplink audio thread */
#define AMP_SD_GPIO_PIN    12                           /* D7 / P1.12 (gpio1) */

K_MEM_SLAB_DEFINE(tx_slab, I2S_BLOCK_BYTES, NUM_BLOCKS, 4);
RING_BUF_DECLARE(pcm_ring, RING_BYTES);
K_MUTEX_DEFINE(ring_mutex);
K_SEM_DEFINE(run_sem, 0, 1);
K_SEM_DEFINE(idle_sem, 1, 1);   /* 1 = feeder idle; gates one session at a time */

static const struct device *i2s_dev;
static const struct device *amp_gpio;
static atomic_t active = ATOMIC_INIT(0);
static uint32_t session_rate = 16000;
static int16_t  mono_scratch[FRAMES_PER_BLOCK];
/* [USER] playback volume 0-100%. Dev default 75%: on the bare XIAO 3V3 rail (no
 * bulk cap), full-volume peaks sag the rail and crackle; 75% is clean for dev.
 * PRODUCTION: with a proper amp supply (boost/dedicated rail + decoupling) set
 * this to 100. See docs/research/2026-06-20-audio-storage-ble-architecture-review.md. */
static uint8_t  volume_pct = 75;

/* --- DIAGNOSTIC: per-session glitch counters, logged at session end. --- */
static uint32_t dbg_blocks;
static uint32_t dbg_underruns;            /* ring held < a full block -> silence-padded */
static uint32_t dbg_underruns_partial;    /* 0 < got < block: producer behind = AUDIBLE crackle
                                           * (vs got==0 empties = gaps / the drain tail) */
static uint32_t dbg_writeerr;
static int64_t  dbg_play_start_ms;        /* uptime at first real block (effective-rate calc) */
static uint32_t dbg_play_samples;         /* real mono samples clocked out (drift detector) */
static atomic_t dbg_overflow = ATOMIC_INIT(0);

static int i2s_setup(uint32_t rate)
{
	struct i2s_config cfg = {
		.word_size      = 16,
		.channels       = 2,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = rate,
		.mem_slab       = &tx_slab,
		.block_size     = I2S_BLOCK_BYTES,
		.timeout        = 2000,
	};
	return i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
}

/* MAX98357A SD is active-high (high = enabled). Raw pin on gpio1, so the
 * gpio_pin_set logical level equals the physical level. */
static inline void amp_enable(bool on)
{
	gpio_pin_set(amp_gpio, AMP_SD_GPIO_PIN, on ? 1 : 0);
}

/* Pull up to one block of mono samples from the ring, expand to stereo into
 * `block`, zero-padding any shortfall. Returns the number of real mono samples
 * pulled (0 = ring empty; < FRAMES_PER_BLOCK = underrun, silence-padded tail). */
static size_t fill_block(int16_t *block)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	uint32_t got_bytes = ring_buf_get(&pcm_ring, (uint8_t *)mono_scratch,
					  FRAMES_PER_BLOCK * 2);
	k_mutex_unlock(&ring_mutex);

	size_t got = got_bytes / 2;
	for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
		int16_t s = (i < got)
			? (int16_t)((int32_t)mono_scratch[i] * volume_pct / 100)
			: 0;
		block[2 * i]     = s;
		block[2 * i + 1] = s;
	}
	return got;
}

static void feeder(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (1) {
		k_sem_take(&run_sem, K_FOREVER);    /* wait for a session */

		if (i2s_setup(session_rate) != 0) {
			LOG_ERR("i2s_configure failed");
			atomic_set(&active, 0);
			k_sem_give(&idle_sem);          /* release the gate on failure */
			continue;
		}
		amp_enable(true);

		/* Pre-buffer: yield so the (lower-priority) producer can build a
		 * cushion before we start clocking I2S -- keeps the ring full from the
		 * first block, no opening underrun. Bounded so a slow/short producer
		 * can't hang the start. */
		for (int i = 0; i < PREBUF_TRIES && atomic_get(&active); i++) {
			if (ring_buf_space_get(&pcm_ring) <= (RING_BYTES - PREBUF_BYTES)) {
				break;
			}
			k_msleep(5);
		}

		uint32_t block_ms = (FRAMES_PER_BLOCK * 1000u) / session_rate;
		if (block_ms == 0) {
			block_ms = 1;
		}
		uint32_t silence_ms = 0;
		int primed = 0;
		dbg_blocks = 0;
		dbg_underruns = 0;
		dbg_underruns_partial = 0;
		dbg_writeerr = 0;
		dbg_play_start_ms = 0;
		dbg_play_samples = 0;
		atomic_set(&dbg_overflow, 0);

		while (atomic_get(&active)) {
			void *block;

			if (k_mem_slab_alloc(&tx_slab, &block, K_MSEC(100)) != 0) {
				continue;
			}
			size_t got = fill_block((int16_t *)block);
			bool all_silence = (got == 0);
			dbg_blocks++;
			if (got < FRAMES_PER_BLOCK) {
				dbg_underruns++;
				/* 0 < got < block = ring not yet empty but producer is
				 * behind -> this is the audible mid-stream crackle; got==0
				 * is a full gap / the post-stream drain tail (inaudible). */
				if (got > 0) {
					dbg_underruns_partial++;
				}
			}
			/* Effective I2S rate = real samples clocked / wall-clock. If it
			 * reads > session_rate, the I2S clock runs fast and steadily drains
			 * the jitter buffer (drift), vs burst jitter which wouldn't. */
			if (got > 0) {
				if (dbg_play_start_ms == 0) {
					dbg_play_start_ms = k_uptime_get();
				}
				dbg_play_samples += got;
			}

			if (i2s_write(i2s_dev, block, I2S_BLOCK_BYTES) != 0) {
				/* TX underrun/error leaves the nrfx driver in ERROR;
				 * end the session cleanly instead of spinning on
				 * perpetually-failing writes. */
				LOG_ERR("i2s_write failed; ending session");
				k_mem_slab_free(&tx_slab, block);
				dbg_writeerr++;
				atomic_set(&active, 0);
				break;
			}
			if (primed < 2 && ++primed == 2) {
				i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			}

			silence_ms = all_silence ? silence_ms + block_ms : 0;
			if (silence_ms >= IDLE_STOP_MS) {
				atomic_set(&active, 0);   /* end after a gap of silence */
			}
		}

		/* session end: stop + purge queued blocks + mute amp, then release
		 * the gate so the next audio_out_start() may arm. */
		i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
		amp_enable(false);
		/* Two rates over the play span:
		 *  - i2s_hz  = ALL samples clocked (real + silence pad) / time. Should
		 *    read ~session_rate; a steady deviation = I2S clock drift.
		 *  - audio_hz = REAL samples only / time. Below session_rate by the
		 *    fraction of time spent silence-padding (the underrun cost). */
		int64_t play_ms = dbg_play_start_ms ?
			(k_uptime_get() - dbg_play_start_ms) : 0;
		uint32_t i2s_hz = play_ms ?
			(uint32_t)(((int64_t)dbg_blocks * FRAMES_PER_BLOCK * 1000) / play_ms) : 0;
		uint32_t audio_hz = play_ms ?
			(uint32_t)(((int64_t)dbg_play_samples * 1000) / play_ms) : 0;
		LOG_INF("audio_out session: %u blocks, %u underruns (%u partial/audible), "
			"%ld overflow, %u write-err",
			dbg_blocks, dbg_underruns, dbg_underruns_partial,
			(long)atomic_get(&dbg_overflow), dbg_writeerr);
		LOG_INF("audio_out rate: req=%u Hz, i2s=%u Hz (drift check), audio=%u Hz "
			"over %lld ms", session_rate, i2s_hz, audio_hz, (long long)play_ms);
		k_sem_give(&idle_sem);
	}
}

K_THREAD_DEFINE(audio_out_tid, FEEDER_STACK, feeder, NULL, NULL, NULL,
		FEEDER_PRIO, 0, 0);

int audio_out_init(void)
{
	i2s_dev  = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	amp_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(i2s_dev) || !device_is_ready(amp_gpio)) {
		LOG_ERR("audio_out: device not ready");
		return -ENODEV;
	}
	/* amp SD low = MAX98357A shutdown (muted) until a session starts */
	gpio_pin_configure(amp_gpio, AMP_SD_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
	LOG_INF("audio_out ready");
	return 0;
}

int audio_out_start(uint32_t sample_rate)
{
	/* Gate: one session at a time, and a new session cannot arm until the
	 * previous session's feeder cleanup has fully finished -- this closes the
	 * start/stop/start lifecycle race (the feeder is guaranteed parked on
	 * run_sem before we set active and signal it). */
	if (k_sem_take(&idle_sem, K_MSEC(50)) != 0) {
		return -EBUSY;
	}
	session_rate = sample_rate;

	k_mutex_lock(&ring_mutex, K_FOREVER);
	ring_buf_reset(&pcm_ring);
	k_mutex_unlock(&ring_mutex);

	atomic_set(&active, 1);
	k_sem_give(&run_sem);
	return 0;
}

void audio_out_write(const int16_t *mono_pcm, size_t nsamp)
{
	/* active is read outside ring_mutex: a concurrent audio_out_flush() could
	 * reset the ring just after this check, leaving a stale frame queued -- but
	 * the next audio_out_start() resets the ring before playback, so no ghost
	 * audio reaches the speaker. */
	if (!atomic_get(&active) || mono_pcm == NULL || nsamp == 0) {
		return;
	}
	k_mutex_lock(&ring_mutex, K_FOREVER);
	uint32_t put = ring_buf_put(&pcm_ring, (const uint8_t *)mono_pcm,
				    nsamp * 2);
	k_mutex_unlock(&ring_mutex);

	if (put < nsamp * 2) {
		atomic_inc(&dbg_overflow);
		LOG_WRN("audio_out: ring overflow, dropped %u bytes",
			(unsigned)(nsamp * 2 - put));
	}
}

void audio_out_stop(void)
{
	atomic_set(&active, 0);   /* feeder stops I2S + mutes amp */
}

void audio_out_flush(void)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	ring_buf_reset(&pcm_ring);
	k_mutex_unlock(&ring_mutex);
	atomic_set(&active, 0);   /* feeder DROPs queued I2S + mutes amp */
}

size_t audio_out_ring_free(void)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	size_t freebytes = ring_buf_space_get(&pcm_ring);
	k_mutex_unlock(&ring_mutex);
	return freebytes;
}

size_t audio_out_ring_used(void)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	size_t used = ring_buf_size_get(&pcm_ring);
	k_mutex_unlock(&ring_mutex);
	return used;
}

bool audio_out_active(void)
{
	return atomic_get(&active) != 0;
}

void audio_out_set_volume(uint8_t percent)
{
	volume_pct = (percent > 100) ? 100 : percent;
}

uint8_t audio_out_get_volume(void)
{
	return volume_pct;
}
