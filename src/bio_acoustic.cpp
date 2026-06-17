#include "bio_acoustic.h"
#include "gesture_thresholds.h"
#include "power_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>

#include <arm_math.h>
#include <arm_const_structs.h>

LOG_MODULE_REGISTER(bio_acoustic, LOG_LEVEL_INF);

/*
 * --- Bio-acoustic capture + feature extraction + classifier (Stage E) ---
 *
 * When the chip's tap engine fires INT1, the main dispatcher calls
 * bio_acoustic_on_tap().  That function reads the chip's FIFO (continuous
 * mode at 1.66 kHz; ~410 ms ring-buffer of recent accel samples) into a
 * static capture buffer, then signals a worker thread.  The worker computes
 * features and classifies the tap as snap-vs-band-tap.
 *
 * Architecture decisions (see docs/research/gesture-architecture.md §3
 * Bio-acoustic sensing path):
 *   - Capture: chip FIFO in continuous mode (chip-native pre-trigger
 *     ring buffer).  ~250 samples = ~150 ms pre-event window at
 *     1.66 kHz reading.
 *   - Worker: async thread (industry standard).  Acq thread keeps
 *     reading at 100 Hz unaffected.
 *   - Classifier: simple rules in v0.  Snap = anticipatory pre-event
 *     motion + broad axis distribution.  Band-tap = quiet pre-event +
 *     Z-axis dominance.  ML migration tracked in Item 8.
 *   - Output: log-only for now.  When classifier proves reliable on
 *     real data, the classification will feed mode-routing decisions.
 *
 * For v0 testing, the classifier output does NOT change FSM behavior
 * -- it only logs.  The multi-tap counter runs as unbound scaffolding
 * (POSE_EAR enters via voice, POSE_SURFACE is dormant) -- no tap-bound
 * mode after the air-mouse extraction.  This lets you validate the
 * classifier against ground truth (you know which gesture you did)
 * without breaking working paths.
 */

/* Capture buffer.  At 833 Hz, 512 samples = ~615 ms of signal,
 * a power of 2 so CMSIS-DSP arm_rfft_fast_f32 can transform it
 * directly.  Each sample is 3 axes * int16 = 6 bytes.  Total
 * 3072 bytes. */
#define TAP_CAPTURE_SAMPLES   512
#define TAP_CAPTURE_BYTES     (TAP_CAPTURE_SAMPLES * 6)
static uint8_t  tap_capture_buf[TAP_CAPTURE_BYTES];
static uint16_t tap_capture_sample_count = 0;

/* FFT scratch -- magnitude signal time-domain (input) and complex
 * frequency-domain (output).  CMSIS-DSP arm_rfft_fast_f32 produces
 * an N-element interleaved real/imag output for an N-element real
 * input.  At 833 Hz / 512 points, bin resolution is 1.63 Hz.
 *
 * Total memory: 2 * 512 * 4 = 4 KB.  Acceptable on our 256 KB RAM. */
#define FFT_SIZE              TAP_CAPTURE_SAMPLES
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static arm_rfft_fast_instance_f32 fft_inst;
static bool fft_initialised = false;

/* Worker synchronisation: dispatcher gives the semaphore after
 * filling the buffer; worker takes it and processes.  Binary
 * semaphore -- if a second tap arrives while the worker is still
 * processing, the dispatcher skips capture and logs "dropped"
 * rather than corrupting the in-flight buffer. */
static K_SEM_DEFINE(tap_capture_sem, 0, 1);
static atomic_t tap_capture_busy = ATOMIC_INIT(0);  /* 1 = buffer in use */

/* Most recent tap's mid_band_energy, updated by bio_acoustic_worker
 * after each FFT.  Read by bio_acoustic_last_was_hard_surface() to
 * gate SURFACE mode entry.  Hard surfaces produce elevated mid_band
 * from resonance feedback; soft surfaces damp it. */
static float last_tap_mid_band_energy = 0.0f;

/* Feature extraction output.  Computed by the worker from
 * tap_capture_buf, then fed to the classifier.  v2 features
 * (2026-06-10): spectral band energies replace pre_event_energy
 * which proved unreliable in real-world use. */
struct tap_features {
    int16_t peak_x;        /* signed peak (most positive or negative) */
    int16_t peak_y;
    int16_t peak_z;
    uint16_t peak_x_abs;   /* magnitudes for ratio comparisons */
    uint16_t peak_y_abs;
    uint16_t peak_z_abs;
    uint16_t peak_sample_idx;  /* where in buffer the peak is */
    float    low_band_energy;  /* energy in 20-200 Hz band (snap-favouring) */
    float    mid_band_energy;  /* energy in 200-400 Hz band (tap-favouring) */
    float    low_band_ratio;   /* low / (low + mid); near 1.0 = snap, near
                                  0.5 = broadband (tap) */
    char     dominant_axis;    /* 'X' / 'Y' / 'Z' */
};

/* Helper: parse one accel sample (6 bytes, little-endian) from buf. */
static inline void _parse_sample(const uint8_t *p,
                                 int16_t *x, int16_t *y, int16_t *z)
{
    *x = (int16_t)((p[1] << 8) | p[0]);
    *y = (int16_t)((p[3] << 8) | p[2]);
    *z = (int16_t)((p[5] << 8) | p[4]);
}

static inline uint16_t _abs16(int16_t v)
{
    return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

/* Locate the peak-magnitude sample.  Scans all samples; the one
 * with the largest |X|+|Y|+|Z| sum is treated as the impulse moment.
 * Records peak per axis and the index where the peak sits. */
static void _extract_peak(const uint8_t *buf, uint16_t sample_count,
                          struct tap_features *out)
{
    uint32_t max_sum = 0;
    uint16_t max_idx = 0;
    int16_t  px = 0, py = 0, pz = 0;

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t x, y, z;
        _parse_sample(buf + i * 6, &x, &y, &z);
        uint32_t s = (uint32_t)_abs16(x) +
                     (uint32_t)_abs16(y) +
                     (uint32_t)_abs16(z);
        if (s > max_sum) {
            max_sum = s;
            max_idx = i;
            px = x;
            py = y;
            pz = z;
        }
    }
    out->peak_x = px;
    out->peak_y = py;
    out->peak_z = pz;
    out->peak_x_abs = _abs16(px);
    out->peak_y_abs = _abs16(py);
    out->peak_z_abs = _abs16(pz);
    out->peak_sample_idx = max_idx;

    /* Dominant axis = whichever |axis| is largest at peak. */
    if (out->peak_z_abs >= out->peak_x_abs &&
        out->peak_z_abs >= out->peak_y_abs) {
        out->dominant_axis = 'Z';
    } else if (out->peak_x_abs >= out->peak_y_abs) {
        out->dominant_axis = 'X';
    } else {
        out->dominant_axis = 'Y';
    }
}

/* Compute spectral band energies of the impulse signal.
 *
 * Approach (ViBand-style, adapted for 833 Hz ODR):
 *   1. Build a magnitude signal mag[i] = sqrt(x^2 + y^2 + z^2)
 *      from the int16 accel samples.  Magnitude is pose-invariant
 *      (whichever axis the impulse hits, magnitude captures it).
 *   2. Subtract mean (DC removal) so gravity doesn't dominate the
 *      DC bin.  Saves dynamic range for the impulse content.
 *   3. Apply Hann window to reduce spectral leakage at the
 *      buffer edges.
 *   4. Run real FFT via CMSIS-DSP arm_rfft_fast_f32.
 *   5. Sum |X[k]|^2 over the two bands of interest.
 *
 * Bands (at 833 Hz / 512 points, bin resolution 1.63 Hz):
 *   - Low band: bins 12-122 = 19.5 - 199 Hz.  Bone-conducted
 *     snap impulses dominate here per ViBand.
 *   - Mid band: bins 122-246 = 199 - 400 Hz.  Direct mechanical
 *     impacts (tap on band housing) have more energy here.
 *
 * Discriminator: low_ratio = low / (low + mid).  Snap -> ratio
 * high (~0.7+); tap -> ratio lower (~0.5 or less).
 *
 * Band-edge constants (FFT_BIN_LOW_START/END, FFT_BIN_MID_START/END)
 * come from gesture_thresholds.h. */

static void _compute_band_energies(const uint8_t *buf,
                                   uint16_t sample_count,
                                   float *out_low,
                                   float *out_mid,
                                   float *out_ratio)
{
    *out_low = 0.0f;
    *out_mid = 0.0f;
    *out_ratio = 0.5f;  /* neutral default */

    if (sample_count < FFT_SIZE || !fft_initialised) {
        return;
    }

    /* Step 1+2: magnitude signal with DC removal. */
    float mean = 0.0f;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        int16_t x, y, z;
        _parse_sample(buf + i * 6, &x, &y, &z);
        float fx = (float)x;
        float fy = (float)y;
        float fz = (float)z;
        float mag = sqrtf(fx * fx + fy * fy + fz * fz);
        fft_input[i] = mag;
        mean += mag;
    }
    mean /= (float)FFT_SIZE;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        fft_input[i] -= mean;
    }

    /* Step 3: Hann window.  Reduces spectral leakage at the buffer
     * edges so the band-energy integrals are more accurate.  Half-
     * cosine envelope from 0 at the ends to 1 at the centre.
     * Local 2*PI constant -- M_PI isn't exposed in this build
     * configuration. */
    static const float TWO_PI = 6.28318530717958647692f;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        float w = 0.5f * (1.0f - cosf(TWO_PI * (float)i /
                                       (float)(FFT_SIZE - 1)));
        fft_input[i] *= w;
    }

    /* Step 4: FFT.  ifftFlag=0 means forward transform. */
    arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);

    /* Step 5: sum |X[k]|^2 over the two bands.  arm_rfft_fast_f32
     * output format: index 0 = DC (real), index 1 = Nyquist (real),
     * indices 2..N-1 = interleaved real/imag pairs for bins 1..N/2-1.
     *
     * For our band ranges (12-122 and 122-246) we're well clear of
     * bins 0 and Nyquist, so we read pairs starting at offset 2*k. */
    for (uint16_t k = FFT_BIN_LOW_START; k < FFT_BIN_LOW_END; k++) {
        float re = fft_output[2 * k];
        float im = fft_output[2 * k + 1];
        *out_low += re * re + im * im;
    }
    for (uint16_t k = FFT_BIN_MID_START; k < FFT_BIN_MID_END; k++) {
        float re = fft_output[2 * k];
        float im = fft_output[2 * k + 1];
        *out_mid += re * re + im * im;
    }

    float total = *out_low + *out_mid;
    if (total > 0.0f) {
        *out_ratio = *out_low / total;
    }
}

/* Surface-spectral confirmation -- v0 proxy.
 *
 * Literature-backed discriminator (PMC 2016, surface-stiffness
 * acceleration analysis): hard surfaces produce LONGER-DURATION
 * ringing -- more oscillation cycles before energy decays.  Soft
 * surfaces damp quickly.  The proper feature is post-event
 * ring-down duration.
 *
 * Our current FFT pipeline doesn't directly measure ring-down
 * duration -- the 512-sample capture is mostly PRE-impact, with
 * the impact at the end.  Instead we use mid_band_energy as a
 * proxy: hard surfaces couple some of their ringing into the
 * mid frequencies that ARE in our window (via reflected feedback
 * during the tap itself), giving elevated mid_band.  Soft
 * surfaces don't.
 *
 * This is imperfect; future work (F9, see plan) is to capture a
 * post-event window for direct ring-down measurement.
 *
 * Empirical threshold from desk-tap-vs-lap-tap data collection (to
 * be tuned during integration test).  Conservative initial value.
 *
 * IMPORTANT: this signal is housing-dependent.  Current value tuned
 * for open PCB; will need recalibration when housing arrives.
 *
 * SURFACE_RESONANCE_MID_BAND_THRESH is defined in gesture_thresholds.h. */

/* Returns true if the most recently captured tap shows desk-feedback
 * spectral content (mid_band above threshold).  Used to confirm
 * SURFACE entry: hard surface (desk) gives elevated mid_band from
 * resonance feedback; soft surface (lap) damps it. */
static bool surface_spectral_confirms_hard_surface(void)
{
    return last_tap_mid_band_energy >= SURFACE_RESONANCE_MID_BAND_THRESH;
}

/* Worker thread entry.  Blocks on tap_capture_sem, processes one
 * capture at a time, logs the result.  Sample count and buffer are
 * static and protected by the busy flag. */
static void bio_acoustic_worker(void *, void *, void *)
{
    while (1) {
        k_sem_take(&tap_capture_sem, K_FOREVER);

        uint16_t n = tap_capture_sample_count;
        if (n < 100) {
            LOG_INF("[BIO] capture too short (%u samples) -- skipping",
                    (unsigned)n);
            atomic_set(&tap_capture_busy, 0);
            continue;
        }

        struct tap_features f = {};
        _extract_peak(tap_capture_buf, n, &f);
        _compute_band_energies(tap_capture_buf, n,
                               &f.low_band_energy,
                               &f.mid_band_energy,
                               &f.low_band_ratio);
        uint32_t total_abs = (uint32_t)f.peak_x_abs +
                             (uint32_t)f.peak_y_abs +
                             (uint32_t)f.peak_z_abs;
        uint32_t z_ratio_pct = total_abs ?
                                (uint32_t)f.peak_z_abs * 100 / total_abs :
                                0;

        /* Log the full feature vector for diagnostics.  ratio is the
         * headline discriminator; low/mid absolute energies confirm we
         * have meaningful signal (vs noise floor).  No verdict -- pose
         * carries mode info; snap-vs-tap discrimination deferred to
         * future in-session fusion (F6). */
        LOG_INF("[BIO] features peak[%d,%d,%d] dom=%c z_ratio=%u%% "
                "low_band=%.0f mid_band=%.0f low_ratio=%.2f idx=%u/%u",
                (int)f.peak_x, (int)f.peak_y, (int)f.peak_z,
                f.dominant_axis,
                (unsigned)z_ratio_pct,
                (double)f.low_band_energy,
                (double)f.mid_band_energy,
                (double)f.low_band_ratio,
                (unsigned)f.peak_sample_idx, (unsigned)n);

        last_tap_mid_band_energy = f.mid_band_energy;

        atomic_set(&tap_capture_busy, 0);
    }
}

K_THREAD_DEFINE(bio_acoustic_worker_id, 1536,
                bio_acoustic_worker, NULL, NULL, NULL,
                7,  /* priority -- below acq but above background */
                0, 0);

/* Public API for main.cpp.  Call once at boot to enable the chip
 * FIFO; call on every chip tap event to snapshot + signal worker. */
void bio_acoustic_init(void)
{
    last_tap_mid_band_energy = 0.0f;

    /* Initialise CMSIS-DSP RFFT instance.  Length must be a
     * supported power of 2 (32..4096); 512 fits us perfectly.
     * arm_rfft_fast_init_f32 returns ARM_MATH_SUCCESS (0) on
     * success.  Stash the success flag so the worker can short-
     * circuit if init failed (shouldn't happen but defensive). */
    arm_status fft_status = arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    if (fft_status != ARM_MATH_SUCCESS) {
        LOG_ERR("[BIO] arm_rfft_fast_init_f32 failed (%d) -- spectral "
                "classifier disabled", (int)fft_status);
        fft_initialised = false;
    } else {
        fft_initialised = true;
    }

    int err = lsm6dsl_fifo_enable_continuous();
    if (err) {
        LOG_ERR("[BIO] FIFO enable failed (%d) -- bio-acoustic "
                "capture will not work", err);
    } else {
        LOG_INF("[BIO] worker thread started; FIFO armed for capture; "
                "FFT %s", fft_initialised ? "ready" : "DISABLED");
    }
}

void bio_acoustic_on_tap(void)
{
    /* Try to claim the capture buffer.  If the worker is still
     * processing a previous capture, drop this one with a log so the
     * loss is visible -- we'd rather skip than corrupt mid-process. */
    if (!atomic_cas(&tap_capture_busy, 0, 1)) {
        LOG_INF("[BIO] tap dropped -- worker still processing previous");
        return;
    }

    /* Read the LATEST samples from the FIFO -- this is where the tap
     * impulse lives (the chip wrote it most recently).  Previous
     * implementation read the OLDEST samples from FIFO head, which
     * meant the actual impulse was sitting just past our read window
     * (the unread tail).  We were "capturing" pre-event signal but
     * missing the impulse entirely, and the classifier was operating
     * on gravity vectors only.  Caught in hardware test 2026-06-09.
     *
     * Fix: figure out total occupancy, discard the older samples to
     * get to the latest TAP_CAPTURE_SAMPLES, then read those. */
    uint16_t words_in_fifo = 0;
    int err = lsm6dsl_fifo_get_word_count(&words_in_fifo);
    if (err) {
        LOG_WRN("[BIO] FIFO_STATUS read failed (%d)", err);
        atomic_set(&tap_capture_busy, 0);
        return;
    }

    /* Each accel sample is 3 words (X/Y/Z each = 1 word = 2 bytes).
     * Cap our keep-window at TAP_CAPTURE_SAMPLES. */
    uint16_t samples_in_fifo = words_in_fifo / 3;
    uint16_t want_samples = (samples_in_fifo > TAP_CAPTURE_SAMPLES)
                                ? TAP_CAPTURE_SAMPLES
                                : samples_in_fifo;
    uint16_t skip_samples = (samples_in_fifo > want_samples)
                                ? (samples_in_fifo - want_samples)
                                : 0;

    /* Discard the older samples one chunk at a time.  Reusing
     * tap_capture_buf as scratch is safe -- the worker isn't running
     * yet (we hold the busy flag) so the buffer's data will be
     * overwritten before being inspected.  Chunk size sized to one
     * I2C burst that fits comfortably. */
    uint16_t discard_chunk_samples = 64;  /* 64 samples * 6 bytes = 384 B */
    while (skip_samples > 0) {
        uint16_t chunk = (skip_samples > discard_chunk_samples)
                            ? discard_chunk_samples
                            : skip_samples;
        uint16_t got_discard = 0;
        err = lsm6dsl_fifo_read_words(tap_capture_buf, chunk * 3,
                                      &got_discard);
        if (err || got_discard == 0) {
            LOG_WRN("[BIO] FIFO discard-read failed (err=%d got=%u)",
                    err, (unsigned)got_discard);
            atomic_set(&tap_capture_busy, 0);
            return;
        }
        skip_samples -= got_discard / 3;
    }

    /* Now read the latest want_samples into the capture buffer. */
    uint16_t want_words = want_samples * 3;
    uint16_t got_words = 0;
    err = lsm6dsl_fifo_read_words(tap_capture_buf, want_words, &got_words);
    if (err || got_words == 0) {
        LOG_WRN("[BIO] FIFO read failed (err=%d got=%u)", err,
                (unsigned)got_words);
        atomic_set(&tap_capture_busy, 0);
        return;
    }

    tap_capture_sample_count = got_words / 3;
    k_sem_give(&tap_capture_sem);
}

bool bio_acoustic_last_was_hard_surface(void)
{
    return surface_spectral_confirms_hard_surface();
}
