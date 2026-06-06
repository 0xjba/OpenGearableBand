#include "WearableDSP.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);

// Skip the first AUTOCORR_TRANSIENT_SKIP samples of the band-pass IIR's
// output before correlating.  The 4th-order Butterworth IIR is re-init'd
// per window (forced by the 50% buffer overlap -- carrying filter state
// across the overlap-shift would misalign x[n-1]/x[n-2] vs the new
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

    // Three independent NLMS filters: zero coefficients, re-init so
    // each instance's internal state-buffer pointer is rewound.  Same
    // step size (0.01) and tap count (32) across all three -- gives
    // them comparable adaptation behavior on their respective axes.
    for (int i = 0; i < 32; i++) {
        lms_coeffs_x[i] = 0.0f;
        lms_coeffs_y[i] = 0.0f;
        lms_coeffs_z[i] = 0.0f;
    }
    arm_lms_norm_init_f32(&lms_inst_x, 32, lms_coeffs_x, lms_state_x, 0.01f, BUFFER_SIZE);
    arm_lms_norm_init_f32(&lms_inst_y, 32, lms_coeffs_y, lms_state_y, 0.01f, BUFFER_SIZE);
    arm_lms_norm_init_f32(&lms_inst_z, 32, lms_coeffs_z, lms_state_z, 0.01f, BUFFER_SIZE);

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

float WearableDSP::processHeartRate(float* ppg_buffer,
                                    float* imu_smv,
                                    float* imu_x,
                                    float* imu_y,
                                    float* imu_z) {
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

    // 2. Endpoint detrending for PPG.  Plain mean subtraction handles
    //    a flat DC offset, but during the first ~10 s of fresh band
    //    contact the optical baseline DRIFTS noticeably (snap-1 trace:
    //    ppg_dc rising 136964 -> 137047 -> 137283 over three windows
    //    as skin/capillaries/LED equilibrate).  Plain mean subtraction
    //    leaves that drift as a ~100-count linear ramp residual inside
    //    the buffer -- positive at the start, negative at the end --
    //    which is a step input to the 4th-order band-pass IIR and
    //    triggers ringing at the lower cutoff (0.6 Hz, whose period
    //    of 167 samples gets clipped by our [30, 150] autocorr search
    //    to a spurious lag-145 peak = 41 BPM exactly).
    //
    //    Endpoint detrending subtracts a straight line drawn through
    //    the first and last buffer samples, forcing both endpoints to
    //    exactly 0.  No step input -> no IIR ring.
    //
    //    The 2-point slope estimate is biased by cardiac AC at the
    //    endpoints (max bias ~ peak-cardiac-amplitude / N ~ 0.2
    //    counts/sample), which is small relative to the actual drift
    //    rate (~0.4-0.8 counts/sample observed).  A least-squares
    //    fit would be more accurate but is overkill for this case.
    float ac_mean;  // kept for the dsp: diagnostic log
    arm_mean_f32(ppg_buffer, BUFFER_SIZE, &ac_mean);

    float first_val = ppg_buffer[0];
    float last_val  = ppg_buffer[BUFFER_SIZE - 1];
    float slope = (last_val - first_val) / (float)(BUFFER_SIZE - 1);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        ppg_buffer[i] -= (first_val + slope * (float)i);
    }

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

    // 3. DC removal for IMU (drops the gravity bias).  No band-pass on
    //    IMU -- NLMS wants the full motion spectrum as its reference.
    //
    //    All four signals get the gravity bias stripped:
    //      * SMV (motion-state variance metric -- ~1 g DC at rest)
    //      * X, Y, Z axes individually (NLMS references; without DC
    //        removal the filter would burn coefficients modeling the
    //        ~1 g component split across whichever axes are aligned
    //        with gravity, leaving little capacity for the actual
    //        kinetic AC content it's supposed to subtract).
    float imu_dc, dc_x, dc_y, dc_z;
    arm_mean_f32(imu_smv, BUFFER_SIZE, &imu_dc);
    arm_mean_f32(imu_x,   BUFFER_SIZE, &dc_x);
    arm_mean_f32(imu_y,   BUFFER_SIZE, &dc_y);
    arm_mean_f32(imu_z,   BUFFER_SIZE, &dc_z);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        imu_smv[i] -= imu_dc;
        imu_x[i]   -= dc_x;
        imu_y[i]   -= dc_y;
        imu_z[i]   -= dc_z;
    }

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

    // 6. Find the IMU stride frequency (only when we'll need it, i.e.
    //    when motion is non-stationary and the NLMS path is going to
    //    feed FFT).  Used for spectral exclusion in runFFT below.
    //    stride_bin <= 0 means "no exclusion" (passed through unused
    //    in the autocorr path).
    int stride_bin = -1;
    if (sqi_passed && motion != STATIONARY) {
        stride_bin = findStrideBin(imu_x, imu_y, imu_z);
    }

    // 7. Extract a raw BPM using the path that matches the motion state.
    //    Only run this path if SQI passed -- otherwise raw_bpm stays 0 and
    //    the diagnostic log makes that explicit.
    float raw_bpm = 0.0f;
    float ac_bpm = 0.0f;          // dual-method log: autocorr pick
    float fft_bpm = 0.0f;         // dual-method log: stationary-FFT pick
    float stat_conf = 0.0f;       // dual-method log: reconciliation confidence
    const char *path = "skip";
    if (sqi_passed) {
        switch (motion) {
            case STATIONARY:
                // STAGE-2 dual-method.  Run autocorr + stationary FFT on
                // every STATIONARY window, including high-ppg_var ones.
                //
                // The previous (pre-Stage-2) settling gate refused to run
                // the DSP at all when ppg_var > 5000, on the theory that
                // high variance meant unstable contact and any single-
                // method estimate would be garbage (lag-100+ peaks ->
                // fake ~50 BPM).  With dual-method, the original failure
                // mode is caught structurally: if autocorr and FFT
                // disagree, we hold; if they agree, the agreement itself
                // is evidence the signal is real regardless of variance.
                //
                // First trace post-Stage-2 confirmed this -- post-workout
                // ppg_var stayed in the 5000-19000 range for 90 s while
                // perfusion recovered, freezing Kalman at 123.55 while
                // Apple Watch dropped from 124 to 101.  Cardiac signal
                // was demonstrably present (FFT and autocorr both could
                // see it) but the variance gate refused to look.
                //
                // New behavior:
                //   stat_conf == 1.0 (agree)   -> update Kalman, r=0.5
                //   stat_conf == 0.5 (partial) -> update Kalman, r=2.0
                //   stat_conf == 0.0 AND ppg_var > 5000  -> "settling"
                //                                            (high noise
                //                                            AND no
                //                                            agreement)
                //   stat_conf == 0.0 AND ppg_var <= 5000 -> "stat-hold"
                //                                            (low noise
                //                                            but methods
                //                                            disagreed)
            {
                ac_bpm  = runAutocorrelation(ppg_buffer);
                fft_bpm = runStationaryFFT(ppg_buffer);
                raw_bpm = reconcileStationary(ac_bpm, fft_bpm, &stat_conf);

                // STAGE-2 polish: harmonic-structure check.  Catches the
                // case where BOTH methods agreed on the sub-harmonic --
                // observed in snapshot-start traces where the optical
                // baseline transient causes the IIR to ring at ~0.6 Hz,
                // creating coherent low-frequency content visible to both
                // autocorr and FFT but lacking real cardiac harmonic
                // structure.  applyHarmonicCheck switches the pick to 2K
                // if 2K shows richer harmonic content.  No-op otherwise.
                // Only run when we'd actually commit the value (conf >=
                // 0.4) -- pointless on hold-Kalman windows.
                float adjusted_bpm = raw_bpm;
                if (stat_conf >= 0.4f && raw_bpm > 0.0f) {
                    adjusted_bpm = applyHarmonicCheck(raw_bpm);
                }
                bool harmonic_switched = (adjusted_bpm != raw_bpm);
                raw_bpm = adjusted_bpm;

                if (stat_conf >= 0.99f) {
                    kalman.r = 0.5f;          // high confidence
                    path = harmonic_switched ? "stat-h3" : "stat-agree";
                } else if (stat_conf >= 0.4f) {
                    kalman.r = 2.0f;          // medium
                    path = harmonic_switched ? "stat-h3" : "stat-harm";
                } else if (ppg_var > 5000.0f) {
                    // Both methods unconvinced AND signal is noisy.
                    // This is the original "settling" case -- hold.
                    raw_bpm = 0.0f;
                    path = "autocorr-settling";
                } else {
                    // Clean signal, methods still disagreed without a
                    // 2:1 explanation.  Hold; log so we see why.
                    raw_bpm = 0.0f;
                    path = "stat-hold";
                }
                break;
            }
            case MICRO_MOTION:
                // Plain FFT used to live here, but with the 0.6-3.3 Hz
                // band-pass, any wrist motion at 1-3 Hz lands inside the
                // pass band as a peak just as tall as the heart-rate peak
                // -- FFT would obediently pick it.  Route through the
                // 3-axis chained NLMS so the X/Y/Z-correlated motion is
                // subtracted out of the PPG before the FFT bin search.
                // findStrideBin (called above) populated the dynamic
                // imu_shadow_mask which runFFT uses to shadow any bin
                // the IMU spectrum lit up as a motion peak -- catches
                // residuals that the chained NLMS leaves behind.
                raw_bpm = runAdaptiveNLMS(ppg_buffer, imu_x, imu_y, imu_z, stride_bin);
                kalman.r = 2.0f;
                path = "nlms-micro";
                break;
            case HEAVY_MOTION:
                raw_bpm = runAdaptiveNLMS(ppg_buffer, imu_x, imu_y, imu_z, stride_bin);
                kalman.r = 5.0f;
                path = "nlms-heavy";
                break;
        }
    }

    bool bpm_in_range = (raw_bpm >= MIN_BPM && raw_bpm <= MAX_BPM);

    // Symmetric slew-rate limiter.  Reject NLMS raws whose absolute
    // delta from current Kalman state implies either an impossible
    // jump or an impossible drop.  Symmetric rather than asymmetric
    // to avoid the "mathematical ratchet" failure mode: a one-sided
    // gate lets noise spikes pull Kalman one way without allowing the
    // legitimate counter-correction back, biasing the filter.  With
    // the 2x stride harmonic now spectrally excluded in runFFT, the
    // raws that reach this gate are mostly cardiac-cluster anyway --
    // the gate is here to catch the residual artifacts that slip
    // through.  STATIONARY/autocorr is exempt either way.
    float delta_abs = raw_bpm - kalman.x;
    if (delta_abs < 0.0f) delta_abs = -delta_abs;
    bool delta_ok = true;
    if (motion == MICRO_MOTION) {
        delta_ok = (delta_abs <= MAX_DELTA_MICRO_BPM);
    } else if (motion == HEAVY_MOTION) {
        delta_ok = (delta_abs <= MAX_DELTA_HEAVY_BPM);
    }

    // 7. One diagnostic line per window: every gate's verdict + the raw BPM
    //    the path produced.  Lets us see whether a stuck Kalman is caused by
    //    SQI rejection, peak-finder returning 0, out-of-range BPM, or the
    //    new delta gate rejecting a physiologically-impossible jump.
    LOG_INF("dsp: ppg_dc=%.0f ppg_var=%.0f imu_var=%.3f motion=%d sqi=%d "
            "path=%s stride=%d ac=%.2f fft=%.2f conf=%.2f raw=%.2f "
            "in_range=%d delta_ok=%d kalman=%.2f",
            (double)ac_mean, (double)ppg_var, (double)imu_var,
            (int)motion, (int)sqi_passed, path, stride_bin,
            (double)ac_bpm, (double)fft_bpm, (double)stat_conf,
            (double)raw_bpm, (int)bpm_in_range, (int)delta_ok, (double)kalman.x);

    if (!sqi_passed)    return kalman.x;
    if (!bpm_in_range)  return kalman.x;
    if (!delta_ok)      return kalman.x;
    return kalman.update(raw_bpm);
}

float WearableDSP::runAutocorrelation(float* ppg) {
    // Discard the first AUTOCORR_TRANSIENT_SKIP samples of the band-
    // passed buffer (see the comment on the constant for why).  The
    // PPG data is still valid -- we just don't include the IIR's
    // settle window in the lag-correlation sum.  Result is shorter
    // (n=432 instead of 512) but still well-conditioned for the
    // lag range [30, 150] we care about (zero_lag_index = 431,
    // search runs to index 581, output length is 863).
    const int n = BUFFER_SIZE - AUTOCORR_TRANSIENT_SKIP;
    arm_correlate_f32(ppg + AUTOCORR_TRANSIENT_SKIP, n,
                      ppg + AUTOCORR_TRANSIENT_SKIP, n,
                      this->correlation);
    int delay_index = findSecondPeak(this->correlation, n);
    if (delay_index == 0) return 0;
    return (60.0f * SAMPLE_RATE) / (float)delay_index;
}

float WearableDSP::runFFT(float* ppg, int stride_bin) {
    arm_rfft_fast_f32(&fft_inst, ppg, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, fft_magnitudes, BUFFER_SIZE / 2);

    int min_idx = (int)(0.6f * BUFFER_SIZE / SAMPLE_RATE);
    int max_idx = (int)(3.33f * BUFFER_SIZE / SAMPLE_RATE);

    // Hybrid spectral exclusion: dynamic IMU mask + predictive harmonic
    // mask.  Each catches what the other misses.
    //
    // The dynamic mask (imu_shadow_mask, populated by findStrideBin)
    // is REACTIVE -- it only shadows bins where the IMU FFT magnitude
    // exceeded 3x the in-band mean.  Good for catching the stride
    // fundamental and anything the IMU itself sees clearly.
    //
    // But the PPG has a strong 2x stride harmonic from biomechanical
    // doubling -- foot strike asymmetry, wrist flexion at heel strike,
    // and optical blood-volume doubling at each pulse-coupling event.
    // The IMU's OWN 2x harmonic does NOT mirror this at comparable
    // strength because the kinematic 2x is weaker than the optical 2x.
    // Sprint trace in v0.3: stride=8 detected stably, but PPG FFT
    // picked bins 14-17 (2x stride zone) every window while IMU 2x
    // at bin 16 stayed below 3x mean and was NOT shadowed.  Kalman
    // stuck at 85 while Apple Watch read 130-138 -- raws were
    // 164/175/187/199 BPM, all in the 2x stride cluster.
    //
    // So we ALSO shadow predictively, regardless of what the IMU shows:
    //   stride       +/-1  -- IMU fundamental + leakage skirt
    //   stride/3     +/-1  -- sub-harmonic from breathing / nonlinear
    //                         pickup (observed in earlier jogging trace
    //                         with stride=13, picks at bin 4)
    //   2*stride     +/-2  -- biomechanical 2x.  +/-2 because stride
    //                         detection bounces +/-1 window-to-window
    //                         AND rectangular-window leakage spreads
    //                         each peak +/-1-2 bins; the product means
    //                         2x of stride+/-1 already lives in +/-2
    //                         of nominal.
    //
    // stride_bin <= 0 disables the predictive masks.  In practice
    // runFFT is only called from the NLMS path, which always has a
    // stride detected, so this is just defensive.
    int sub_bin = (stride_bin > 0) ? (stride_bin / 3) : -1;
    int x2_bin  = (stride_bin > 0) ? (stride_bin * 2) : -1;

    float max_val = 0;
    int max_idx_found = 0;

    for (int i = min_idx; i <= max_idx; i++) {
        // Dynamic mask: whatever the IMU FFT lit up.
        if (imu_shadow_mask & (1U << i)) {
            continue;
        }
        // Predictive masks: shadowed regardless of IMU strength
        // because the PPG sees these even when the IMU doesn't.
        if (stride_bin > 0 &&
            i >= stride_bin - 1 && i <= stride_bin + 1) {
            continue;
        }
        if (sub_bin > 0 &&
            i >= sub_bin - 1 && i <= sub_bin + 1) {
            continue;
        }
        if (x2_bin > 0 &&
            i >= x2_bin - 2 && i <= x2_bin + 2) {
            continue;
        }
        if (fft_magnitudes[i] > max_val) {
            max_val = fft_magnitudes[i];
            max_idx_found = i;
        }
    }
    return ((float)max_idx_found * SAMPLE_RATE / (float)BUFFER_SIZE) * 60.0f;
}

float WearableDSP::runStationaryFFT(float* ppg) {
    // Dual-method companion to runAutocorrelation().  Same band-passed
    // PPG buffer, no NLMS, no stride masking -- we're stationary so
    // there's no motion content to subtract or shadow.
    //
    // The IIR transient (first ~70 samples of impulse response) lands
    // around 0.6 Hz, i.e. bin 3.  Our cardiac search range starts at
    // bin 3 and the MIN_BPM=40 range-check rejects anything below
    // bin 3.4 anyway, so any transient pollution at bin 3 falls out
    // of the candidate set automatically.
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
    // Dual-method reconciliation per the industry-standard pattern
    // (Apple US patent 9,826,940 "weighted combination" + Analog Devices
    // reference design + the harmonic-pair search documented across the
    // PPG HR estimation patent corpus).  We compute both autocorr and
    // FFT, then resolve:
    //
    //   Agree within +/-5 BPM    -> high confidence, return average
    //   One returned 0           -> medium confidence, trust the other
    //   2:1 ratio between them   -> medium confidence, return the LARGER
    //                               (the smaller is the 2T sub-harmonic;
    //                               this is the textbook failure mode
    //                               we saw in field traces at rest with
    //                               weak cardiac SNR)
    //   Disagree without 2:1     -> low confidence, return 0 (caller
    //                               should hold Kalman -- something
    //                               unexplained is going on, refuse
    //                               to commit a guess to state)
    //
    // The 2:1 ratio is checked within [1.85, 2.15] to tolerate spectral-
    // leakage bin rounding (autocorr lag at 71 corresponds to FFT bin
    // 7.04; both methods can round their picks +/- 1 bin / 1 sample).

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

    // Agreement tolerance must absorb the inherent resolution mismatch
    // between the two methods:
    //   FFT bin width = SAMPLE_RATE / BUFFER_SIZE * 60 = 100/512*60
    //                 = 11.72 BPM, constant across the cardiac band
    //   Autocorr lag width = N - (N/(1 + 1/N)) varies with HR.  At lag
    //   41 (= 146.34 BPM) adjacent lags are 142.86 / 150.00 / 139.53,
    //   i.e. 3-7 BPM apart at typical heavy-exercise HR.
    // Combined uncertainty at HR=140 is ~12 BPM; the methods *cannot*
    // round to within 5 BPM of each other at high cardiac rates even
    // when they are pointing at the same physiological frequency.
    // First trace with the 5 BPM gate showed seven "false hold" windows
    // where both methods reported essentially the same answer but
    // differed by 5-9 BPM and got rejected (e.g. ac=122.45 / fft=128.91
    // both = bin 11 territory).  Bump to 10 BPM, which still keeps the
    // sub-harmonic case clearly distinct (|T - 2T| = T, far above 10).
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
    // Compute bin index from BPM (round to nearest).  bin * SR/N * 60 = BPM
    // -> bin = BPM * N / (SR * 60).  Round via +0.5 trick.
    int K = (int)(candidate_bpm * (float)BUFFER_SIZE
                  / (SAMPLE_RATE * 60.0f) + 0.5f);

    int min_idx = (int)(0.6f * BUFFER_SIZE / SAMPLE_RATE);
    int max_idx = (int)(3.33f * BUFFER_SIZE / SAMPLE_RATE);

    // If the candidate falls outside the cardiac search band, don't
    // touch it (range gate downstream will reject it anyway).
    if (K < min_idx || K > max_idx) return candidate_bpm;

    // We only check upward (2K).  Checking K/2 would risk false-
    // downgrading a real fundamental to its sub-harmonic and is rarely
    // useful in the STATIONARY path -- our reconcileStationary already
    // handles the "ac picked 2T while FFT picked T" case via the 2:1
    // ratio rule.  The remaining failure mode is "both methods agreed
    // on the sub-harmonic", which is unidirectional and what this check
    // catches.
    int bin_2K = 2 * K;
    if (bin_2K < min_idx || bin_2K > max_idx) {
        // 2K is out of cardiac band -> K must be the fundamental
        return candidate_bpm;
    }

    float score_K  = harmonicScore(K);
    float score_2K = harmonicScore(bin_2K);

    if (score_2K > score_K) {
        // Sub-harmonic detected: 2K has richer harmonic structure than K,
        // so the real cardiac fundamental is at 2K and our candidate was
        // the 2T peak.  Snap to 2K's BPM.
        return (float)bin_2K * SAMPLE_RATE / (float)BUFFER_SIZE * 60.0f;
    }
    return candidate_bpm;
}

int WearableDSP::findStrideBin(float* imu_x, float* imu_y, float* imu_z) {
    // Locate the user's stride frequency from the IMU.  Earlier version
    // used SMV; that broke for fast sprints (3 Hz stride -> 6 Hz in SMV
    // -- out of the [0.6, 3.33 Hz] search range -- so findStrideBin
    // locked onto the sub-harmonic at bin 8 instead of the real stride
    // at bin 15).  Raw axes preserve the fundamental frequency.
    //
    // Pick the axis with the highest variance for THIS window.  Wrist
    // orientation is user/situation-dependent: arm swing projects mostly
    // onto Y when the wrist is palm-up at the start of the swing, X when
    // turned, etc.  Picking the dominant axis dynamically captures stride
    // regardless of orientation.
    //
    // Caller must have already DC-removed the axes so gravity isn't the
    // dominant "variance" (which would always select whichever axis
    // happens to be gravity-aligned).
    float var_x, var_y, var_z;
    arm_var_f32(imu_x, BUFFER_SIZE, &var_x);
    arm_var_f32(imu_y, BUFFER_SIZE, &var_y);
    arm_var_f32(imu_z, BUFFER_SIZE, &var_z);

    float *selected = imu_x;
    if (var_y >= var_x && var_y >= var_z) selected = imu_y;
    else if (var_z >= var_x && var_z >= var_y) selected = imu_z;

    arm_rfft_fast_f32(&fft_inst, selected, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, fft_magnitudes, BUFFER_SIZE / 2);

    int min_idx = (int)(0.6f * BUFFER_SIZE / SAMPLE_RATE);
    int max_idx = (int)(3.33f * BUFFER_SIZE / SAMPLE_RATE);

    float max_val = 0;
    int max_bin = 0;
    float sum = 0.0f;
    for (int i = min_idx; i <= max_idx; i++) {
        sum += fft_magnitudes[i];
        if (fft_magnitudes[i] > max_val) {
            max_val = fft_magnitudes[i];
            max_bin = i;
        }
    }

    // Build the dynamic shadow mask.  Replaces the older "shadow at
    // stride, stride/3, 2x stride +/- 1 bin" rule which had three
    // failure modes in field traces:
    //   (a) stride bin bounces +/-1 window-to-window for fast cadence
    //       (variance-based axis selection can flip dominant axes),
    //       so the 2x position is off by +/-2 bins -- enough to miss.
    //   (b) rectangular-window spectral leakage spreads each motion
    //       peak across +/-2-3 bins; +/-1 shadow misses the skirts
    //       and the cardiac search picks the skirt as "the" peak.
    //   (c) motion harmonics are not always cleanly integer (asymm
    //       gait + wrist rotation produce fractional inter-harmonics).
    //
    // Threshold = 3x in-band mean.  A typical jogging IMU spectrum has
    // 2-3 prominent peaks (fundamental + 1-2 harmonics) plus a noise
    // floor; 3x mean cleanly separates the peaks from the floor without
    // catching small wiggles.  Each flagged bin is widened by +/-1 to
    // catch the leakage skirts.
    float mean = sum / (float)(max_idx - min_idx + 1);
    float threshold = 3.0f * mean;
    imu_shadow_mask = 0;
    for (int i = min_idx; i <= max_idx; i++) {
        if (fft_magnitudes[i] >= threshold) {
            int lo = (i > 0)  ? i - 1 : 0;
            int hi = (i < 31) ? i + 1 : 31;
            for (int j = lo; j <= hi; j++) {
                imu_shadow_mask |= (1U << j);
            }
        }
    }

    return max_bin;
}

float WearableDSP::runAdaptiveNLMS(float* ppg,
                                   float* imu_x,
                                   float* imu_y,
                                   float* imu_z,
                                   int stride_bin)
{
    // Chained 3-stage NLMS multi-reference adaptive filter.
    //
    // arm_lms_norm_f32 signature is misleading -- naming-wise:
    //   pSrc  = reference signal (the noise we want to subtract)
    //   pRef  = desired signal (the noisy input)
    //   pOut  = ESTIMATED NOISE produced by the filter (not "clean")
    //   pErr  = ERROR = desired - estimated_noise = the cleaned signal
    //
    // So we feed each stage:
    //   reference = one IMU axis (already DC-removed by caller)
    //   desired   = the residual from the previous stage (or raw PPG
    //               for stage 1)
    //   pOut      = nlms_noise_estimate (scratch sink, we never read it)
    //   pErr      = the residual for the next stage
    //
    // After stage 3, nlms_final holds the PPG with X-, Y-, and Z-
    // correlated motion content subtracted out.  FFT picks the
    // strongest bin in the [0.6, 3.3] Hz band on that residual.
    arm_lms_norm_f32(&lms_inst_x, imu_x, ppg,
                     nlms_noise_estimate, nlms_residual_xy, BUFFER_SIZE);
    arm_lms_norm_f32(&lms_inst_y, imu_y, nlms_residual_xy,
                     nlms_noise_estimate, nlms_residual_yz, BUFFER_SIZE);
    arm_lms_norm_f32(&lms_inst_z, imu_z, nlms_residual_yz,
                     nlms_noise_estimate, nlms_final, BUFFER_SIZE);
    return runFFT(nlms_final, stride_bin);
}

int WearableDSP::findSecondPeak(float* data, int autocorr_n) {
    // arm_correlate_f32 on N samples produces a (2N-1)-length output
    // with zero lag at index N-1.  Caller passes N (the autocorrelation
    // INPUT length); we derive zero-lag and output bounds from it so
    // this function works for any input length, not just the full
    // BUFFER_SIZE.  Search window covers plausible HR periods:
    //   min_lag = 60/MAX_BPM * SR =  30 samples (200 BPM)
    //   max_lag = 60/MIN_BPM * SR = 150 samples ( 40 BPM)
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
