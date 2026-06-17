/*
 * mic_vad -- PDM-mic capture + voice features (dictation entry A.0 + A.1).
 *
 * Captures 16 kHz mono audio from the onboard MSM261D3526H1CPM via Zephyr's
 * DMIC API on its own thread.  A.0 logs short-term RMS energy ([MIC]); A.1
 * adds a voiced-band (300-3000 Hz) spectral-energy feature (CMSIS rFFT) to the
 * same [MIC] line so the speech-vs-ambient separability can be measured, ahead
 * of the latched-floor voice-onset detector.  Isolated like bio_acoustic: no
 * dependency into the gesture FSM.  Plans:
 * docs/superpowers/plans/2026-06-17-dictation-entry-A0-mic-feasibility.md and
 * .../2026-06-17-dictation-entry-A1-voice-onset-fsm.md.  The pure helpers
 * (mic_vad_block_rms, mic_vad_band_sum) live in mic_vad_rms.cpp (host-tested);
 * this TU is Zephyr-only.
 */
#include "mic_vad.h"

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <arm_math.h>

LOG_MODULE_REGISTER(mic_vad, LOG_LEVEL_INF);

/* 16 kHz mono, ~20 ms blocks (320 samples, 640 bytes). Known-good rate (pairs
 * with the driver's ~1.28 MHz PDM clock). */
#define MIC_PCM_RATE_HZ    16000
#define MIC_BLOCK_SAMPLES  (MIC_PCM_RATE_HZ * 20 / 1000)   /* 320 */
#define MIC_BLOCK_BYTES    (MIC_BLOCK_SAMPLES * 2)          /* 640 */
#define MIC_SLAB_BLOCKS    4

/* Voiced-band spectral feature. 512-pt real FFT over the 320-sample block
 * (Hann-windowed, zero-padded). Bin width = 16000/512 = 31.25 Hz.
 * Voiced band 300-3000 Hz => bins 10..96 (300/31.25=9.6->10, 3000/31.25=96). */
#define MIC_FFT_N        512
#define MIC_VOICED_LO    10
#define MIC_VOICED_HI    96

static arm_rfft_fast_instance_f32 mic_fft;
static float mic_hann[MIC_BLOCK_SAMPLES];       /* 320-pt Hann window */
static float mic_fft_in[MIC_FFT_N];             /* windowed + zero-padded */
static float mic_fft_out[MIC_FFT_N];            /* packed real FFT output */
static float mic_bin_e[MIC_FFT_N / 2];          /* per-bin energy (re^2+im^2) */
static bool  mic_fft_ready;                     /* set by a successful FFT init */

/* Spectral features of one int16 block: Hann-window, zero-pad to 512, rFFT,
 * then *out_voiced = sum |X|^2 over the voiced band (bins 10..96, 300-3000 Hz)
 * and *out_total = sum |X|^2 over bins 1..255 (DC excluded). The voiced/total
 * FRACTION is loudness-invariant -- it separates concentrated, harmonic speech
 * from broadband ambient (e.g. fan) without depending on absolute level. */
static void mic_spectral(const int16_t *s, size_t n,
                         float *out_voiced, float *out_total)
{
    *out_voiced = 0.0f;
    *out_total  = 0.0f;
    if (!mic_fft_ready) return;        /* init failed; don't read a bad instance */
    size_t m = (n < MIC_BLOCK_SAMPLES) ? n : MIC_BLOCK_SAMPLES;
    for (size_t i = 0; i < m; i++)         mic_fft_in[i] = (float)s[i] * mic_hann[i];
    for (size_t i = m; i < MIC_FFT_N; i++) mic_fft_in[i] = 0.0f;

    arm_rfft_fast_f32(&mic_fft, mic_fft_in, mic_fft_out, 0 /* forward */);

    /* Packed format: out[0]=DC real, out[1]=Nyquist real, then re,im pairs.
     * Same convention as bio_acoustic.cpp (verified against CMSIS source). */
    mic_bin_e[0] = mic_fft_out[0] * mic_fft_out[0];
    for (int k = 1; k < MIC_FFT_N / 2; k++) {
        float re = mic_fft_out[2 * k];
        float im = mic_fft_out[2 * k + 1];
        mic_bin_e[k] = re * re + im * im;
    }
    *out_voiced = mic_vad_band_sum(mic_bin_e, MIC_FFT_N / 2, MIC_VOICED_LO, MIC_VOICED_HI);
    *out_total  = mic_vad_band_sum(mic_bin_e, MIC_FFT_N / 2, 1, (MIC_FFT_N / 2) - 1);
}

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

            const int16_t *pcm = (const int16_t *)buf;
            size_t nsamp = size / 2;
            float rms = mic_vad_block_rms(pcm, nsamp);
            float ve, te;
            mic_spectral(pcm, nsamp, &ve, &te);
            k_mem_slab_free(&mic_slab, buf);   /* return the block */

            /* Display: scale voiced energy to millions (raw |X|^2 overflows an
             * int log cast), plus the loudness-invariant voiced/total fraction
             * in per-mille (0-1000). These are the two candidate discriminators
             * being measured before the onset threshold is set. */
            int veM  = (int)(ve / 1.0e6f);
            int frac = (te > 1.0f) ? (int)(1000.0f * ve / te) : 0;

            if ((++log_ctr % 5) == 0) {   /* ~10 Hz */
                LOG_INF("[MIC] rms=%d veM=%d frac=%d", (int)rms, veM, frac);
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
    arm_status fft_st = arm_rfft_fast_init_f32(&mic_fft, MIC_FFT_N);
    mic_fft_ready = (fft_st == ARM_MATH_SUCCESS);
    if (!mic_fft_ready) {
        LOG_ERR("[MIC] arm_rfft_fast_init_f32 failed (%d) -- voiced energy disabled",
                (int)fft_st);
    }
    for (int i = 0; i < (int)MIC_BLOCK_SAMPLES; i++) {
        mic_hann[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * i / (MIC_BLOCK_SAMPLES - 1)));
    }
}

void mic_vad_start(void) { atomic_set(&mic_running, 1); }
void mic_vad_stop(void)  { atomic_set(&mic_running, 0); }
