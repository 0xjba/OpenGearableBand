#include "WearableDSP.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);

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

    // NLMS: zero coefficients, re-init so internal state pointer is rewound.
    for (int i = 0; i < 32; i++) lms_coeffs[i] = 0.0f;
    arm_lms_norm_init_f32(&lms_inst, 32, lms_coeffs, lms_state, 0.01f, BUFFER_SIZE);

    // Band-pass IIR: arm_biquad_cascade_df1_init_f32 memsets the state
    // buffer to zero (verified in CMSIS-DSP source), so this both
    // installs the coefficient/state pointers and clears history.
    arm_biquad_cascade_df1_init_f32(&bp_inst, 2, BP_COEFFS, bp_state);

    // Motion hysteresis: clear so a fresh wear starts in the "no recent
    // motion" state and the first stationary window can use the autocorr
    // path immediately (rather than being forced into MICRO_MOTION by a
    // stale cooldown left over from before unwear).
    motion_cooldown = 0;
}

bool WearableDSP::checkSQI(float* ppg_data) {
    float mean, var;
    arm_mean_f32(ppg_data, BUFFER_SIZE, &mean);
    arm_var_f32(ppg_data, BUFFER_SIZE, &var);
    if (var < 10.0f || var > 100000.0f) return false;
    return true;
}

MotionState WearableDSP::getMotionState(float* imu_data) {
    float var;
    arm_var_f32(imu_data, BUFFER_SIZE, &var);
    if (var < 0.05f) return STATIONARY;
    if (var < 0.5f) return MICRO_MOTION;
    return HEAVY_MOTION;
}

float WearableDSP::processHeartRate(float* ppg_buffer, float* imu_buffer) {
    // 1. Wear-state machine.  Check raw PPG mean BEFORE DC removal.
    float raw_mean;
    arm_mean_f32(ppg_buffer, BUFFER_SIZE, &raw_mean);
    bool mean_passes = (raw_mean >= WEAR_PPG_THRESHOLD);

    if (mean_passes) {
        // Edge: NOT_WORN -> STABILIZING.  Buffer still holds stale data, but
        // this is the moment we know the user just put the device on.  Reset
        // adaptive filters so the eventual first WORN window starts clean.
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
        // Mean failed.  Drop straight back to NOT_WORN; the next OFF->ON edge
        // will trigger a fresh resetAdaptiveFilters().  No fail-counter
        // hysteresis is needed because the worn/un-worn means are separated
        // by an order of magnitude (worn ~30-80k, off-wrist < 2k) -- the
        // threshold won't bounce on a brief sensor lift unless the lift
        // covers most of a 5.12 s window, in which case treating it as
        // off-wrist is correct.
        wear_pass_count = 0;
        wear_state = WEAR_NOT_WORN;
        // Reset last_motion explicitly: returning early skips the motion
        // classifier, and a stale MICRO/HEAVY value left over from the
        // last worn window will fool the power state machine into
        // thinking the user is still exercising (proven by trace: 68/69
        // motion windows in VERIFY while the wear gate said NOT_WORN).
        last_motion = STATIONARY;
        return 0.0f;
    }

    // 2. DC removal for PPG so the band-pass IIR sees a near-zero-mean input
    //    (smaller startup transient than feeding it the raw 180k-DC signal).
    float ac_mean;
    arm_mean_f32(ppg_buffer, BUFFER_SIZE, &ac_mean);
    for (int i = 0; i < BUFFER_SIZE; i++) ppg_buffer[i] -= ac_mean;

    // 2b. Band-pass to 0.6 - 3.3 Hz.  In-place is safe (verified: CMSIS
    //     processes stage-by-stage across the block, reading each sample
    //     before overwriting it).
    //
    //     State is re-zeroed each window: with 50% acquisition overlap, the
    //     "old half" of the buffer has already been filtered when it was
    //     the fresh half last window.  Carrying state would double-filter
    //     it.  A clean per-window state costs us ~0.7 s of startup
    //     transient (out of 5.12 s), which the lag>=30 autocorrelation /
    //     bin>=3 FFT search both tolerate.
    arm_biquad_cascade_df1_init_f32(&bp_inst, 2, BP_COEFFS, bp_state);
    arm_biquad_cascade_df1_f32(&bp_inst, ppg_buffer, ppg_buffer, BUFFER_SIZE);

    // 3. DC removal for IMU (drops the gravity bias).  No band-pass on IMU
    //    -- NLMS wants the full motion spectrum as its reference.
    float imu_dc;
    arm_mean_f32(imu_buffer, BUFFER_SIZE, &imu_dc);
    for (int i = 0; i < BUFFER_SIZE; i++) imu_buffer[i] -= imu_dc;

    // 4. SQI: variance of the AC PPG.  Reject if too flat (signal is dead)
    //    or too wild (motion, light leak, contact loss).
    //
    // Bounds are coupled to CONFIG_MAX30101_LED2_PA in prj.conf -- variance
    // scales ~quadratically with LED drive current.  At 0x66 (~20 mA) a
    // healthy wrist PPG sits in the 50 k - 250 k range, with brief
    // physiological excursions to ~500 k during vigorous pulses or
    // mild perfusion changes.  Motion / band-removal pushes the variance
    // into the millions and beyond, which is what we actually want to
    // reject.  If LED2_PA is changed, retune both bounds proportionally.
    float ppg_var;
    arm_var_f32(ppg_buffer, BUFFER_SIZE, &ppg_var);
    bool sqi_passed = (ppg_var >= 10.0f && ppg_var <= 1000000.0f);

    // 5. Motion classifier: IMU variance after gravity removal.
    //
    // The IMU is now the Signal Magnitude Vector (SMV) of all 3 accel axes
    // (computed in main.cpp), so motion energy from any direction is
    // captured.  Threshold note: SMV variance for a truly still wrist sits
    // around 0.001-0.005; light wrist activity pushes it into the 0.01+
    // range.  Anything above the 0.01 floor gets routed through NLMS so
    // the IMU-correlated component can be subtracted from the PPG.
    //
    // Hysteresis: after any motion-detected window, motion_cooldown is
    // refilled.  While the cooldown is non-zero, the state is forced to
    // at least MICRO_MOTION even if the IMU has gone quiet -- this catches
    // the case where PPG continues to show the motion artifact for one
    // window after the wrist has stopped moving (proven by the 17:17
    // trace: imu_var=0.046 but raw_bpm=139.53 from a still-ringing PPG).
    float imu_var;
    arm_var_f32(imu_buffer, BUFFER_SIZE, &imu_var);
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

    // 6. Extract a raw BPM using the path that matches the motion state.
    //    Only run this path if SQI passed -- otherwise raw_bpm stays 0 and
    //    the diagnostic log makes that explicit.
    float raw_bpm = 0.0f;
    const char *path = "skip";
    if (sqi_passed) {
        switch (motion) {
            case STATIONARY:
                raw_bpm = runAutocorrelation(ppg_buffer);
                kalman.r = 0.5f;
                path = "autocorr";
                break;
            case MICRO_MOTION:
                // Plain FFT used to live here, but with the 0.6-3.3 Hz
                // band-pass, any wrist motion at 1-3 Hz lands inside the
                // pass band as a peak just as tall as the heart-rate peak
                // -- FFT would obediently pick it.  Route through NLMS so
                // the IMU-correlated component is subtracted out of the
                // PPG before the FFT bin search runs.
                raw_bpm = runAdaptiveNLMS(ppg_buffer, imu_buffer);
                kalman.r = 2.0f;
                path = "nlms-micro";
                break;
            case HEAVY_MOTION:
                raw_bpm = runAdaptiveNLMS(ppg_buffer, imu_buffer);
                kalman.r = 5.0f;
                path = "nlms-heavy";
                break;
        }
    }

    bool bpm_in_range = (raw_bpm >= MIN_BPM && raw_bpm <= MAX_BPM);

    // 7. One diagnostic line per window: every gate's verdict + the raw BPM
    //    the path produced.  Lets us see whether a stuck Kalman is caused by
    //    SQI rejection, peak-finder returning 0, or out-of-range BPM.
    LOG_INF("dsp: ppg_dc=%.0f ppg_var=%.0f imu_var=%.3f motion=%d sqi=%d "
            "path=%s raw=%.2f in_range=%d kalman=%.2f",
            (double)ac_mean, (double)ppg_var, (double)imu_var,
            (int)motion, (int)sqi_passed, path,
            (double)raw_bpm, (int)bpm_in_range, (double)kalman.x);

    if (!sqi_passed)    return kalman.x;
    if (!bpm_in_range)  return kalman.x;
    return kalman.update(raw_bpm);
}

float WearableDSP::runAutocorrelation(float* ppg) {
    arm_correlate_f32(ppg, BUFFER_SIZE, ppg, BUFFER_SIZE, this->correlation);
    int delay_index = findSecondPeak(correlation, BUFFER_SIZE * 2 - 1); 
    if(delay_index == 0) return 0;
    return (60.0f * SAMPLE_RATE) / (float)delay_index;
}

float WearableDSP::runFFT(float* ppg) {
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

float WearableDSP::runAdaptiveNLMS(float* ppg, float* imu) {
    arm_lms_norm_f32(&lms_inst, imu, ppg, this->clean_signal, this->error, BUFFER_SIZE);
    return runFFT(error);
}

int WearableDSP::findSecondPeak(float* data, int length) {
    // Zero lag sits at index BUFFER_SIZE - 1 in the full correlation array.
    // Search window covers plausible heart-rate periods:
    //   min_lag = 60/MAX_BPM * SR =  30 samples (200 BPM)
    //   max_lag = 60/MIN_BPM * SR = 150 samples ( 40 BPM)
    int zero_lag_index = BUFFER_SIZE - 1;
    int min_lag = (int)((60.0f / MAX_BPM) * SAMPLE_RATE);
    int max_lag = (int)((60.0f / MIN_BPM) * SAMPLE_RATE);

    int start_index = zero_lag_index + min_lag;
    int end_index = zero_lag_index + max_lag;
    if (end_index >= length) {
        end_index = length - 1;
    }

    // Pass 1: in-range global max, used purely as a prominence reference.
    float max_val = 0.0f;
    for (int i = start_index; i <= end_index; i++) {
        if (data[i] > max_val) max_val = data[i];
    }
    if (max_val <= 0.0f) {
        // No positive correlation anywhere in the plausible band -- the
        // signal isn't periodic at any HR frequency.  Reject.
        return 0;
    }

    const float prominence_threshold = 0.5f * max_val;

    // Pass 2: return the FIRST local maximum that clears the threshold.
    //
    // Autocorrelation of a periodic signal has its strongest peak (after
    // lag 0) at the fundamental period T.  A signal with a strong 2x
    // harmonic -- like wrist PPG, whose sharp systolic upstroke + slower
    // diastolic decay naturally generates one -- also produces a peak at
    // T/2.  The old "global max" approach picked T/2 because the
    // unnormalised correlation also has more terms summed at shorter
    // lags, doubly biasing the answer toward higher BPM.  Walking the lag
    // range left-to-right and stopping at the first prominent peak yields
    // T directly: by the time we reach lag T, we've already walked past
    // the T/2 harmonic peak and its descending right-hand side, so the
    // next local max we hit is the fundamental.
    for (int i = start_index + 1; i < end_index; i++) {
        if (data[i] > prominence_threshold &&
            data[i] > data[i - 1] &&
            data[i] > data[i + 1]) {
            return i - zero_lag_index;
        }
    }

    // No qualifying peak -- signal too noisy or dominated by a single
    // out-of-band component.  Caller will fall back to kalman.x.
    return 0;
}
