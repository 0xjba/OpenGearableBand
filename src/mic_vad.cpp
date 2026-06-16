/*
 * mic_vad -- PDM-mic feasibility probe (dictation entry A.0).
 *
 * Captures 16 kHz mono audio from the onboard MSM261D3526H1CPM via Zephyr's
 * DMIC API on its own thread and logs short-term RMS energy ([MIC]) so the
 * wrist-mic voice-at-ear-vs-ambient separability can be measured before any
 * VAD/FSM is built.  Isolated like bio_acoustic: no dependency into the
 * gesture FSM.  Plan: docs/superpowers/plans/2026-06-17-dictation-entry-A0-
 * mic-feasibility.md.  The pure RMS helper lives in mic_vad_rms.cpp (host-
 * tested); this TU is Zephyr-only.
 */
#include "mic_vad.h"

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(mic_vad, LOG_LEVEL_INF);

/* 16 kHz mono, ~20 ms blocks (320 samples, 640 bytes). Known-good rate (pairs
 * with the driver's ~1.28 MHz PDM clock). */
#define MIC_PCM_RATE_HZ    16000
#define MIC_BLOCK_SAMPLES  (MIC_PCM_RATE_HZ * 20 / 1000)   /* 320 */
#define MIC_BLOCK_BYTES    (MIC_BLOCK_SAMPLES * 2)          /* 640 */
#define MIC_SLAB_BLOCKS    4

K_MEM_SLAB_DEFINE(mic_slab, MIC_BLOCK_BYTES, MIC_SLAB_BLOCKS, 4);

/* Resolved at COMPILE time, not in mic_vad_init().  The capture thread is
 * K_THREAD_DEFINE'd (runnable at boot) and main() yields (bt_enable, settings,
 * ...) before it reaches mic_vad_init() late in startup -- so a runtime
 * assignment would race: the thread could read a NULL mic_dev, log "not ready",
 * and exit permanently.  DEVICE_DT_GET is a static initializer; the PDM driver
 * inits at POST_KERNEL (before any app thread runs), so the pointer is valid
 * and the device is ready by the time the thread checks. */
static const struct device *mic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0));
static atomic_t mic_running = ATOMIC_INIT(0);

static struct dmic_cfg make_cfg(struct pcm_stream_cfg *stream)
{
    stream->pcm_width = 16;
    stream->pcm_rate  = MIC_PCM_RATE_HZ;
    stream->block_size = MIC_BLOCK_BYTES;
    stream->mem_slab   = &mic_slab;

    struct dmic_cfg cfg = {};
    cfg.io.min_pdm_clk_freq = 1000000;
    cfg.io.max_pdm_clk_freq = 3500000;
    cfg.io.min_pdm_clk_dc   = 40;
    cfg.io.max_pdm_clk_dc   = 60;
    cfg.streams = stream;
    cfg.channel.req_num_streams = 1;
    cfg.channel.req_num_chan    = 1;
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
    return cfg;
}

static void mic_thread(void *, void *, void *)
{
    if (!device_is_ready(mic_dev)) {
        LOG_ERR("[MIC] PDM device not ready");
        return;
    }
    struct pcm_stream_cfg stream;
    struct dmic_cfg cfg = make_cfg(&stream);
    /* A.0 lifecycle: configure once, then trigger START/STOP per probe toggle.
     * Adequate for the single start->measure->stop feasibility run.  A.1 NOTE:
     * if start/stop is cycled repeatedly, verify on HW that the nRFx PDM driver
     * survives STOP->START without a re-configure (re-call dmic_configure if not). */
    if (dmic_configure(mic_dev, &cfg) < 0) {
        /* A.1 NOTE: a boot-time configure failure kills this thread, so a later
         * mic_vad_start() sets the flag with nothing listening (no [MIC] log).
         * Diagnostic enough for the A.0 smoke test (see plan Task 4); give it a
         * proper retry/recover path when A.1 adds persistent lifecycle. */
        LOG_ERR("[MIC] dmic_configure failed");
        return;
    }

    float floor_rms = 0.0f;        /* slow ambient-floor estimate */
    int   log_ctr = 0;

    for (;;) {
        if (!atomic_get(&mic_running)) { k_msleep(50); continue; }

        if (dmic_trigger(mic_dev, DMIC_TRIGGER_START) < 0) {
            LOG_ERR("[MIC] trigger START failed"); k_msleep(200); continue;
        }
        LOG_INF("[MIC] capture started (16 kHz mono)");

        while (atomic_get(&mic_running)) {
            void *buf; size_t size;
            int rc = dmic_read(mic_dev, 0, &buf, &size, 1000);
            if (rc < 0) { LOG_ERR("[MIC] read err %d", rc); break; }

            float rms = mic_vad_block_rms((const int16_t *)buf, size / 2);
            k_mem_slab_free(&mic_slab, buf);   /* return the block */

            /* Slow floor tracker (ambient). */
            floor_rms = (floor_rms == 0.0f) ? rms : (0.98f * floor_rms + 0.02f * rms);
            float ratio = (floor_rms > 1.0f) ? (rms / floor_rms) : 0.0f;

            if ((++log_ctr % 5) == 0) {   /* ~10 Hz */
                LOG_INF("[MIC] rms=%d floor=%d ratio=%d.%02d",
                        (int)rms, (int)floor_rms,
                        (int)ratio, (int)((ratio - (int)ratio) * 100));
            }
        }
        dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
        LOG_INF("[MIC] capture stopped");
    }
}

K_THREAD_DEFINE(mic_thread_id, 2048, mic_thread, NULL, NULL, NULL, 6, 0, 0);

void mic_vad_init(void)
{
    /* mic_dev is resolved at compile time (see its definition).  This call just
     * reports readiness at boot; the probe stays idle until mic_vad_start(). */
    LOG_INF("[MIC] init (pdm0 %s)", device_is_ready(mic_dev) ? "ready" : "NOT READY");
}

void mic_vad_start(void) { atomic_set(&mic_running, 1); }
void mic_vad_stop(void)  { atomic_set(&mic_running, 0); }
