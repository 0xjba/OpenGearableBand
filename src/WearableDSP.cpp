#include "WearableDSP.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);

// Skip the first AUTOCORR_TRANSIENT_SKIP samples of the band-pass IIR's
// output before correlating.  The 4th-order Butterworth IIR is re-init'd
// per window (forced by the 50% buffer overlap -- carrying filter state
// across the overlap-shift would misalign x[n-1] / x[n-2] vs the new
// sample positions), so the first ~70 samples of every window's output
// are dominated by the filter's impulse response rather than by signal.
// Those samples participate in autocorrelation at every lag and can
// create false correlation peaks -- observed: snapshot-1 trace had a
// fully clean signal (ppg_var=804, motion=STATIONARY) but autocorr's
// first WORN window returned raw=41.38 (lag 145) while Apple Watch
// reported 81-83 BPM.  80 samples = 0.8 s gives one extra time constant
// of margin past the filter's ~0.7 s settle.
#define AUTOCORR_TRANSIENT_SKIP   80

// ----------------------------------------------------------------------------
// PPG band-pass filter coefficients.
//
// 4th-order Butterworth band-pass, 0.6 - 3.3 Hz at SAMPLE_RATE = 100 Hz,
// realised as two biquad sections in cascade.
//
// Designed with scipy.signal.butter(N=2, Wn=[0.6, 3.3], btype='bandpass',
// fs=100, output='sos').  Verified frequency response (sosfreqz):
//   0.1 Hz  -> -34.5 dB  (kills baseline wander, respiration)
//   0.6 Hz  -> - 3.0 dB  (lower passband edge, by design)
//   1.0 Hz  -> - 0.1 dB
//   1.5 Hz  -> - 0.0 dB  (band center)
//   3.3 Hz  -> - 3.0 dB  (upper passband edge, by design)
//  10.0 Hz  -> -23.0 dB  (suppresses motion-band harmonics)
//
// CMSIS-DSP uses the ADDITIVE feedback convention:
//   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]
// whereas scipy's SOS rows use the subtractive form.  Each feedback
// coefficient below is therefore the NEGATIVE of scipy's a1 / a2.
// Verified against arm_biquad_cascade_df1_f32.c documentation.
//
// Layout per stage: { b0, b1, b2, a1, a2 }.  Length = 5 * numStages = 10.
static const float BP_COEFFS[10] = {
    // Stage 0
     6.4121937450e-03f,  1.2824387490e-02f,  6.4121937450e-03f,
     1.7921621587e+00f, -8.2296866107e-01f,
    // Stage 1
     1.0000000000e+00f, -2.0000000000e+00f,  1.0000000000e+00f,
     1.9541618917e+00f, -9.5592720067e-01f,
};

float KalmanFilter1D::update(float measurement) {
    p = p + q;
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1.0f - k) * p;
    return x;
}

WearableDSP::WearableDSP() {
    arm_rfft_fast_init_f32(&fft_inst, BUFFER_SIZE);
    resetAdaptiveFilters();
}

void WearableDSP::resetAdaptiveFilters() {
    // Kalman: neutral starting guess, full uncertainty.
    kalman.x = 75.0f;
    kalman.p = 1.0f;
    kalman.k = 0.0f;

    // Band-pass IIR: arm_biquad_cascade_df1_init_f32 memsets the state
    // buffer to zero (verified in CMSIS-DSP source), so this both
    // installs the coefficient/state pointers and clears history.
    arm_biquad_cascade_df1_init_f32(&bp_inst, 2, BP_COEFFS, bp_state);

    // Motion hysteresis: clear so a fresh wear starts in the "no recent
    // motion" state.  Otherwise a stale cooldown left over from before
    // unwear could force the first window to MICRO_MOTION inappropriately.
    motion_cooldown = 0;
}

float WearableDSP::processHeartRate(float* ppg_buffer, float* imu_smv) {
    // 1. Wear-state machine.  Check raw PPG mean BEFORE DC removal.
    float raw_mean;
    arm_mean_f32(ppg_buffer, BUFFER_SIZE, &raw_mean);
    bool mean_passes = (raw_mean >= WEAR_PPG_THRESHOLD);

    if (mean_passes) {
        // Edge: NOT_WORN -> STABILIZING.  Buffer still holds stale data,
        // but this is the moment we know the user just put the device
        // on.  Reset Kalman + band-pass so the eventual first WORN
        // window starts clean.
        if (wear_state == WEAR_NOT_WORN) {
            resetAdaptiveFilters();
            wear_pass_count = 1;
            wear_state = WEAR_STABILIZING;
            return 0.0f;
        }

        if (wear_pass_count < WEAR_PASSES_REQUIRED) {
            wear_pass_count++;
        }

        if (wear_pass_count >= WEAR_PASSES_REQUIRED) {
            wear_state = WEAR_WORN;
            // fall through to DSP processing
        } else {
            // Still stabilising; buffer not yet 100% post-wear data.
            wear_state = WEAR_STABILIZING;
            return 0.0f;
        }
    } else {
        // Mean failed.  Drop straight back to NOT_WORN; the next OFF->ON
        // edge will trigger a fresh resetAdaptiveFilters().  No fail-
        // counter hysteresis is needed because the worn/un-worn means
        // are separated by an order of magnitude (worn ~30-80k, off-
        // wrist < 2k) -- the threshold won't bounce on a brief sensor
        // lift unless the lift covers most of a 5.12 s window, in
        // which case treating it as off-wrist is correct.
        wear_pass_count = 0;
        wear_state = WEAR_NOT_WORN;
        last_motion = STATIONARY;
        return 0.0f;
    }

    // 2. Endpoint detrending for PPG.  Plain mean subtraction handles
    //    a flat DC offset, but during the first ~10 s of fresh band
    //    contact the optical baseline DRIFTS noticeably (snap-1 trace:
    //    ppg_dc rising 136964 -> 137047 -> 137283 over three windows
    //    as skin/capillaries/LED equilibrate).  Plain mean subtraction
    //    leaves that drift as a ~100-count linear ramp residual inside
    //    the buffer, which is a step input to the 4th-order band-pass
    //    IIR and triggers ringing at the lower cutoff (0.6 Hz, period
    //    167 samples -> clipped by our [30, 150] autocorr search to a
    //    spurious lag-145 peak = 41 BPM exactly).
    //
    //    Endpoint detrending subtracts a straight line drawn through
    //    the first and last buffer samples, forcing both endpoints to
    //    exactly 0.  No step input -> no IIR ring.
    //
    //    Slope is biased by cardiac AC at the endpoints (max bias
    //    ~ peak-cardiac-amplitude / N ~ 0.2 counts/sample), which is
    //    small relative to the actual drift rate (~0.4-0.8 counts/
    //    sample observed).  A least-squares fit would be more accurate
    //    but is overkill for this case.
    float ac_mean;  // kept for the dsp: diagnostic log
    arm_mean_f32(ppg_buffer, BUFFER_SIZE, &ac_mean);

    float first_val = ppg_buffer[0];
    float last_val  = ppg_buffer[BUFFER_SIZE - 1];
    float slope = (last_val - first_val) / (float)(BUFFER_SIZE - 1);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        ppg_buffer[i] -= (first_val + slope * (float)i);
    }

    // 2b. Band-pass to 0.6 - 3.3 Hz.  In-place is safe (CMSIS processes
    //     stage-by-stage across the block, reading each sample before
    //     overwriting it).
    //
    //     State is re-zeroed each window: with 50% acquisition overlap
    //     the "old half" of the buffer has already been filtered when
    //     it was the fresh half last window.  Carrying state would
    //     double-filter it.  A clean per-window state costs ~0.7 s of
    //     startup transient (out of 5.12 s), which both the lag >= 30
    //     autocorrelation and bin >= 3 FFT search tolerate.
    arm_biquad_cascade_df1_init_f32(&bp_inst, 2, BP_COEFFS, bp_state);
    arm_biquad_cascade_df1_f32(&bp_inst, ppg_buffer, ppg_buffer, BUFFER_SIZE);

    // 3. DC removal for IMU SMV (drops the gravity bias).  Used only
    //    for motion-state variance classification.
    float imu_dc;
    arm_mean_f32(imu_smv, BUFFER_SIZE, &imu_dc);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        imu_smv[i] -= imu_dc;
    }

    // 4. SQI: variance of the AC PPG.  Reject if too flat (signal is
    //    dead) or too wild (motion, light leak, contact loss).
    //
    //    Bounds are coupled to CONFIG_MAX30101_LED2_PA in prj.conf --
    //    variance scales ~quadratically with LED drive current.  At
    //    0x66 (~20 mA) a healthy wrist PPG sits in the 50 k - 250 k
    //    range, with brief physiological excursions to ~500 k during
    //    vigorous pulses.  Motion / band-removal pushes the variance
    //    into the millions and beyond, which we reject.  If LED2_PA
    //    is changed, retune both bounds proportionally.
    float ppg_var;
    arm_var_f32(ppg_buffer, BUFFER_SIZE, &ppg_var);
    bool sqi_passed = (ppg_var >= 10.0f && ppg_var <= 1000000.0f);

    // 5. Motion classifier: IMU SMV variance after gravity removal.
    //    SMV variance for a truly still wrist sits around 0.001-0.005;
    //    light wrist activity pushes it into 0.01+; sprint / heavy
    //    exercise hits 1+.
    //
    //    Hysteresis: after any motion-detected window, motion_cooldown
    //    is refilled.  While the cooldown is non-zero, the state is
    //    forced to at least MICRO_MOTION even if the IMU has gone
    //    quiet -- catches the case where PPG continues to show motion
    //    artifact for one window after the wrist has stopped moving.
    float imu_var;
    arm_var_f32(imu_smv, BUFFER_SIZE, &imu_var);
    MotionState motion;
    if      (imu_var < 0.01f) motion = STATIONARY;
    else if (imu_var < 0.5f)  motion = MICRO_MOTION;
    else                      motion = HEAVY_MOTION;

    if (motion != STATIONARY) {
        motion_cooldown = MOTION_COOLDOWN_WINDOWS;
    } else if (motion_cooldown > 0) {
        motion = MICRO_MOTION;
        motion_cooldown--;
    }
    last_motion = motion;

    // 6. Extract a raw BPM.  Production build only processes the
    //    STATIONARY path (autocorr + stationary FFT with dual-method
    //    cross-validation and harmonic-sum check).  MICRO and HEAVY
    //    paths hold Kalman -- the caller is expected to signal the
    //    user to hold still via needsSteady().  See the feature/
    //    motion-path-experiments branch for the chained-NLMS work.
    float raw_bpm = 0.0f;
    float ac_bpm = 0.0f;
    float fft_bpm = 0.0f;
    float stat_conf = 0.0f;
    const char *path = "skip";
    if (sqi_passed && motion == STATIONARY) {
        ac_bpm  = runAutocorrelation(ppg_buffer);
        fft_bpm = runStationaryFFT(ppg_buffer);
        raw_bpm = reconcileStationary(ac_bpm, fft_bpm, &stat_conf);

        // Harmonic-structure check.  Catches the case where BOTH
        // methods agreed on the sub-harmonic during the optical-
        // baseline transient at snapshot start.  See WearableDSP.h
        // for the full Apple-patent reference.
        float adjusted_bpm = raw_bpm;
        if (stat_conf >= 0.4f && raw_bpm > 0.0f) {
            adjusted_bpm = applyHarmonicCheck(raw_bpm);
        }
        bool harmonic_switched = (adjusted_bpm != raw_bpm);
        raw_bpm = adjusted_bpm;

        if (stat_conf >= 0.99f) {
            kalman.r = 0.5f;
            path = harmonic_switched ? "stat-h3" : "stat-agree";
        } else if (stat_conf >= 0.4f) {
            kalman.r = 2.0f;
            path = harmonic_switched ? "stat-h3" : "stat-harm";
        } else if (ppg_var > 5000.0f) {
            // Both methods unconvinced AND signal is noisy.  Hold.
            raw_bpm = 0.0f;
            path = "autocorr-settling";
        } else {
            // Clean signal but methods still disagreed without a 2:1
            // explanation.  Hold; log so we see why.
            raw_bpm = 0.0f;
            path = "stat-hold";
        }
    } else if (sqi_passed && motion != STATIONARY) {
        // Motion detected -- caller (main / BLE layer) signals user
        // to hold still.  Kalman not updated this window.
        path = (motion == HEAVY_MOTION) ? "motion-heavy" : "motion-micro";
    }
    // (sqi_passed = 0 falls through with path = "skip")

    bool bpm_in_range = (raw_bpm >= MIN_BPM && raw_bpm <= MAX_BPM);

    // One diagnostic line per window.  ac / fft / conf are zero on any
    // path that didn't run dual-method (motion or sqi-skip).
    LOG_INF("dsp: ppg_dc=%.0f ppg_var=%.0f imu_var=%.3f motion=%d sqi=%d "
            "path=%s ac=%.2f fft=%.2f conf=%.2f raw=%.2f in_range=%d kalman=%.2f",
            (double)ac_mean, (double)ppg_var, (double)imu_var,
            (int)motion, (int)sqi_passed, path,
            (double)ac_bpm, (double)fft_bpm, (double)stat_conf,
            (double)raw_bpm, (int)bpm_in_range, (double)kalman.x);

    if (!sqi_passed)   return kalman.x;
    if (!bpm_in_range) return kalman.x;
    return kalman.update(raw_bpm);
}

float WearableDSP::runAutocorrelation(float* ppg) {
    const int n = BUFFER_SIZE - AUTOCORR_TRANSIENT_SKIP;
    arm_correlate_f32(ppg + AUTOCORR_TRANSIENT_SKIP, n,
                      ppg + AUTOCORR_TRANSIENT_SKIP, n,
                      this->correlation);
    int delay_index = findSecondPeak(this->correlation, n);
    if (delay_index == 0) return 0;
    return (60.0f * SAMPLE_RATE) / (float)delay_index;
}

float WearableDSP::runStationaryFFT(float* ppg) {
    // Same band-passed buffer as autocorr.  The IIR transient (first
    // ~70 samples of impulse response) lands around 0.6 Hz, i.e. bin 3.
    // Our cardiac search range starts at bin 3, and MIN_BPM = 40 in
    // the range gate rejects anything below bin 3.4, so any transient
    // pollution at bin 3 falls out of the candidate set automatically.
    arm_rfft_fast_f32(&fft_inst, ppg, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, fft_magnitudes, BUFFER_SIZE / 2);

    int min_idx = (int)(0.6f * BUFFER_SIZE / SAMPLE_RATE);
    int max_idx = (int)(3.33f * BUFFER_SIZE / SAMPLE_RATE);

    float max_val = 0;
    int max_idx_found = 0;
    for (int i = min_idx; i <= max_idx; i++) {
        if (fft_magnitudes[i] > max_val) {
            max_val = fft_magnitudes[i];
            max_idx_found = i;
        }
    }
    return ((float)max_idx_found * SAMPLE_RATE / (float)BUFFER_SIZE) * 60.0f;
}

float WearableDSP::reconcileStationary(float bpm_ac, float bpm_fft,
                                       float *out_confidence) {
    if (bpm_ac <= 0.0f && bpm_fft <= 0.0f) {
        *out_confidence = 0.0f;
        return 0.0f;
    }
    if (bpm_ac <= 0.0f) {
        *out_confidence = 0.5f;
        return bpm_fft;
    }
    if (bpm_fft <= 0.0f) {
        *out_confidence = 0.5f;
        return bpm_ac;
    }

    // 10 BPM agreement window.  FFT bin width is constant 11.72 BPM;
    // autocorr lag-step BPM gaps are 3-7 at high HR.  The two methods
    // cannot round to within 5 BPM of each other at running cadences
    // even when pointing at the same physiological frequency.  10 BPM
    // absorbs the combined uncertainty without compromising the sub-
    // harmonic case (T vs 2T differ by T, far above 10).
    float delta = bpm_ac - bpm_fft;
    if (delta < 0.0f) delta = -delta;
    if (delta <= 10.0f) {
        *out_confidence = 1.0f;
        return 0.5f * (bpm_ac + bpm_fft);
    }

    float ratio = (bpm_ac > bpm_fft) ? (bpm_ac / bpm_fft)
                                     : (bpm_fft / bpm_ac);
    if (ratio >= 1.85f && ratio <= 2.15f) {
        *out_confidence = 0.5f;
        return (bpm_ac > bpm_fft) ? bpm_ac : bpm_fft;
    }

    *out_confidence = 0.0f;
    return 0.0f;
}

float WearableDSP::harmonicScore(int K) {
    int max_bin = BUFFER_SIZE / 2 - 1;
    if (K < 1 || K > max_bin) return 0.0f;
    float sum = fft_magnitudes[K];
    int b2 = 2 * K;
    int b3 = 3 * K;
    if (b2 <= max_bin) sum += fft_magnitudes[b2];
    if (b3 <= max_bin) sum += fft_magnitudes[b3];
    return sum;
}

float WearableDSP::applyHarmonicCheck(float candidate_bpm) {
    int K = (int)(candidate_bpm * (float)BUFFER_SIZE
                  / (SAMPLE_RATE * 60.0f) + 0.5f);

    int min_idx = (int)(0.6f * BUFFER_SIZE / SAMPLE_RATE);
    int max_idx = (int)(3.33f * BUFFER_SIZE / SAMPLE_RATE);

    if (K < min_idx || K > max_idx) return candidate_bpm;

    int bin_2K = 2 * K;
    if (bin_2K < min_idx || bin_2K > max_idx) {
        // 2K out of cardiac band -> K must be the fundamental.
        return candidate_bpm;
    }

    float score_K  = harmonicScore(K);
    float score_2K = harmonicScore(bin_2K);

    if (score_2K > score_K) {
        // Sub-harmonic detected.  Snap to 2K's BPM.
        return (float)bin_2K * SAMPLE_RATE / (float)BUFFER_SIZE * 60.0f;
    }
    return candidate_bpm;
}

int WearableDSP::findSecondPeak(float* data, int autocorr_n) {
    int zero_lag_index = autocorr_n - 1;
    int output_length = 2 * autocorr_n - 1;
    int min_lag = (int)((60.0f / MAX_BPM) * SAMPLE_RATE);
    int max_lag = (int)((60.0f / MIN_BPM) * SAMPLE_RATE);

    int start_index = zero_lag_index + min_lag;
    int end_index = zero_lag_index + max_lag;
    if (end_index >= output_length) {
        end_index = output_length - 1;
    }

    // Pass 1: in-range global max, used purely as a prominence reference.
    float max_val = 0.0f;
    for (int i = start_index; i <= end_index; i++) {
        if (data[i] > max_val) max_val = data[i];
    }
    if (max_val <= 0.0f) {
        return 0;
    }

    const float prominence_threshold = 0.5f * max_val;

    // Pass 2: return the FIRST local maximum that clears the threshold.
    //
    // Autocorrelation of a periodic signal has its strongest peak (after
    // lag 0) at the fundamental period T.  A signal with a strong 2x
    // harmonic -- like wrist PPG, whose sharp systolic upstroke + slower
    // diastolic decay naturally generates one -- also produces a peak
    // at T/2.  Walking the lag range left-to-right and stopping at the
    // first prominent peak yields T directly: by the time we reach lag
    // T, we've already walked past the T/2 harmonic peak and its
    // descending right-hand side, so the next local max we hit is the
    // fundamental.  The remaining "T -> 2T sub-harmonic when T-peak
    // is below 50 % of in-band max" failure mode is caught downstream
    // by the dual-method cross-validation + harmonic-sum check.
    for (int i = start_index + 1; i < end_index; i++) {
        if (data[i] > prominence_threshold &&
            data[i] > data[i - 1] &&
            data[i] > data[i + 1]) {
            return i - zero_lag_index;
        }
    }

    return 0;
}
