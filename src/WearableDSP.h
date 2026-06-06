#pragma once
#include <arm_math.h>
#include <stdint.h>

#define BUFFER_SIZE 512      // 5.12 seconds of data at 100Hz
#define SAMPLE_RATE 100.0f
#define MIN_BPM 40.0f
#define MAX_BPM 200.0f

enum MotionState {
    STATIONARY,
    MICRO_MOTION,
    HEAVY_MOTION
};

// Wear state with hysteresis.  The 50%-overlap window means a fresh OFF->ON
// transition contaminates the next two windows (the first holds ~50% stale
// off-wrist data; the second can still hold the leading edge).  Producing
// BPM on those windows yields garbage because the DC step dominates the
// FFT/autocorrelation and corrupts the NLMS coefficients.  We therefore
// require N consecutive "mean above threshold" windows before declaring WORN.
enum WearState {
    WEAR_NOT_WORN,    // PPG mean below threshold; no BPM produced
    WEAR_STABILIZING, // mean just passed, but buffer may still hold stale samples
    WEAR_WORN         // buffer is guaranteed 100% post-wear; produce BPM
};

// PPG raw-count threshold (MAX30102 IR channel, LED2_PA=0x33 ~ 10 mA).
// Typical worn signal: 30k-80k.  Ambient / no contact: < 2k.
#define WEAR_PPG_THRESHOLD       10000.0f

// Symmetric slew-rate limiter on raw_bpm vs. current Kalman state.
//
// History of this gate:
//   * Symmetric +/-50 first tried with broken stride detection ->
//     FFT picking 175-199 every window -> kalman stuck at 68.
//   * Asymmetric (rises free, drops gated) tried as a fix ->
//     kalman ratcheted UPWARD to 152 because unshadowed 2x stride
//     harmonics (164-199 BPM bins) passed the rise gate freely while
//     legitimate cardiac drops back to truth were rejected.  This is
//     the "mathematical ratchet" failure mode.
//   * The asymmetric design was philosophically right (HRR is real)
//     but tactically wrong: a one-sided gate cannot distinguish
//     "user's HR is genuinely rising" from "FFT picked a noise bin"
//     and the Kalman gain alone can't repair the bias because the
//     gated drops can't undo the un-gated rises.
//
// Current design: SYMMETRIC.  Both directions gated identically.  No
// ratchet.  Combined with the dynamic IMU shadow mask in runFFT
// (see imu_shadow_mask), the gate's job is purely to reject extreme
// outliers; the spectral path does most of the heavy lifting.
//
// Bound calibration from the trace:
//   * Apple Watch peak HR ~120-130 during exercise.
//   * Kalman state can be stale (snapshot at rest = 80-90) when
//     workout begins -- the first cardiac-correct FFT pick then
//     differs from kalman by 30-50 BPM.  This is LEGITIMATE
//     convergence, not artifact.
//   * Therefore HEAVY bound must be at LEAST 50 to let stale-state
//     onset find the truth.  +/-12 (as initially proposed) would
//     reject every cardiac-correct raw in the trace's onset window.
//   * MICRO is tighter because by the time motion is light, kalman
//     has been tracking through HEAVY and is close to truth -- the
//     legit deltas should be small, and a tighter gate rejects more
//     transition-window noise.
//
// STATIONARY (autocorr) is NOT gated -- that path is +/-1 BPM
// accurate independently and we want it to converge fast.
#define MAX_DELTA_MICRO_BPM      25.0f
#define MAX_DELTA_HEAVY_BPM      50.0f
// Number of consecutive passing windows required for WORN.  Buffer is
// BUFFER_SIZE samples = 2 overlap-windows long, so 2 passes guarantees
// the entire buffer is post-wear data.  Bumped from 3 to 4 to give the
// skin one more 2.5 s window to physiologically settle (skin compresses,
// capillaries equilibrate, LED warms the surface) before we start
// trusting the math.  Observed in v0.2 boot snapshots: ppg_dc rises
// 100-300 counts per window for the first 2-3 worn windows as the
// optical baseline drifts; that drift is large enough to dominate
// the cardiac AC and ring the band-pass IIR.
#define WEAR_PASSES_REQUIRED     4

// Number of windows the motion-state classifier remains "warm" after any
// motion event.  1 window = one 50%-overlap step (~2.56 s) -- long enough
// to cover the post-motion settle, short enough that we return to the
// clean autocorr path quickly once the wrist is truly still.
#define MOTION_COOLDOWN_WINDOWS  1

struct KalmanFilter1D {
    float q = 0.05f; 
    float r = 2.0f;  
    float x = 75.0f; 
    float p = 1.0f;  
    float k = 0.0f;  

    float update(float measurement);
};

class WearableDSP {
private:
    KalmanFilter1D kalman;
    arm_rfft_fast_instance_f32 fft_inst;

    // Chained multi-axis NLMS for motion-artifact cancellation.
    // SMV = sqrt(X^2 + Y^2 + Z^2) has frequency-doubled AC content
    // relative to the original wrist-motion frequency (an arm swing at
    // 2.5 Hz produces 5 Hz in SMV due to the magnitude operation), so
    // NLMS with SMV reference cannot subtract the 2.5 Hz PPG motion
    // artifact.  Raw X, Y, Z axes preserve the fundamental frequency.
    // Chain three single-reference NLMS filters: each axis scrubs its
    // own correlated content from the residual of the prior stage.
    arm_lms_norm_instance_f32 lms_inst_x;
    arm_lms_norm_instance_f32 lms_inst_y;
    arm_lms_norm_instance_f32 lms_inst_z;
    float lms_state_x[BUFFER_SIZE + 32];
    float lms_state_y[BUFFER_SIZE + 32];
    float lms_state_z[BUFFER_SIZE + 32];
    float lms_coeffs_x[32];
    float lms_coeffs_y[32];
    float lms_coeffs_z[32];

    // 4th-order Butterworth band-pass IIR (0.6 - 3.3 Hz @ 100 Hz Fs),
    // 2 biquad sections in cascade.  State buffer is 4 floats per stage.
    arm_biquad_casd_df1_inst_f32 bp_inst;
    float bp_state[4 * 2];

    float fft_output[BUFFER_SIZE];
    float fft_magnitudes[BUFFER_SIZE / 2];

    // Large buffers moved from stack to member variables to prevent stack overflow
    float correlation[BUFFER_SIZE * 2 - 1];
    // Intermediate buffers for the 3-stage NLMS chain:
    //   nlms_noise_estimate  -- "pOut" parameter, the LMS's predicted
    //                            noise term (we don't use it but the
    //                            CMSIS call requires a non-NULL pointer)
    //   nlms_residual_xy     -- output of stage 1 (X-cleaned), input
    //                            to stage 2
    //   nlms_residual_yz     -- output of stage 2 (X+Y-cleaned), input
    //                            to stage 3
    //   nlms_final           -- output of stage 3 (X+Y+Z-cleaned), this
    //                            is what we FFT
    float nlms_noise_estimate[BUFFER_SIZE];
    float nlms_residual_xy[BUFFER_SIZE];
    float nlms_residual_yz[BUFFER_SIZE];
    float nlms_final[BUFFER_SIZE];

    // Wear-state machine state
    WearState wear_state = WEAR_NOT_WORN;
    int wear_pass_count = 0;

    // Dynamic spectral shadow mask, recomputed each window by
    // findStrideBin().  Bit i is set if the IMU FFT magnitude at bin i
    // exceeded 3x the in-band mean (i.e. bin i is a "significant motion
    // peak", regardless of whether it's the stride fundamental, an
    // integer harmonic, a sub-harmonic, or just spectral leakage).
    // runFFT skips set bins when searching for the cardiac peak.  This
    // replaces the earlier fixed shadowing of stride / sub / 2x because
    // (a) stride detection bounces +/-1 bin window to window, (b)
    // spectral leakage spreads each peak across +/-2 bins, and (c)
    // motion isn't always cleanly integer-harmonic.  Capacity = 32
    // bins which comfortably covers our [3, 17] search range.
    uint32_t imu_shadow_mask = 0;

    // Last motion classification produced by processHeartRate().  Exposed
    // via getMotionState() so the power state machine can observe how
    // active the user is without re-running the IMU variance calc.
    MotionState last_motion = STATIONARY;

    // Motion-state hysteresis.  When IMU variance crosses into MICRO or
    // HEAVY in window N, we cannot trust the IMU's "quiet now" reading in
    // window N+1 -- mechanical settle of the blood column, IIR-filter
    // ringing leaking across the 50% overlap, and the autocorrelation
    // sampling the tail of the motion event all conspire to corrupt the
    // immediately-following stationary classification.  motion_cooldown
    // counts down from MOTION_COOLDOWN_WINDOWS after every motion-detected
    // window; while > 0, the path is forced to at least MICRO_MOTION
    // regardless of the current IMU variance.
    int motion_cooldown = 0;

    bool checkSQI(float* ppg_data);
    MotionState getMotionState(float* imu_data);
    float runAutocorrelation(float* ppg);
    // Stationary-only FFT.  Runs on the band-passed PPG buffer (no NLMS,
    // no stride masking).  Returns the BPM at the strongest bin in the
    // cardiac band [0.6, 3.33 Hz].  Used by the STATIONARY path as a
    // second opinion to cross-validate runAutocorrelation() and catch
    // the sub-harmonic failure mode (autocorr can lock onto the 2T peak
    // when the cardiac T-peak is below 50% of the in-band global max).
    float runStationaryFFT(float* ppg);
    // Reconcile autocorr and FFT picks for the STATIONARY path.
    // Returns the chosen BPM (or 0 to signal "hold Kalman") and writes
    // a confidence in [0, 1] to *out_confidence:
    //   1.0  -- both methods agreed within +/-10 BPM (average returned)
    //   0.5  -- one method silent / 2:1 ratio resolved (sub-harmonic case)
    //   0.0  -- disagree without explanation (caller should hold Kalman)
    float reconcileStationary(float bpm_ac, float bpm_fft, float *out_confidence);
    // Harmonic-sum score for FFT bin K: fft_magnitudes[K] + [2K] + [3K].
    // Out-of-range harmonic terms are omitted from the sum.  Real cardiac
    // peaks show rich harmonic structure (2nd harmonic typically 30-70 %
    // of fundamental in wrist PPG, 3rd often visible); IIR-ring artifacts
    // and sub-harmonic peaks do not.  Required state: fft_magnitudes
    // must hold the current window's FFT (populated by runStationaryFFT).
    float harmonicScore(int K);
    // Harmonic-structure post-check applied after reconcileStationary
    // agrees on a candidate BPM.  Computes the harmonic-sum score for
    // the bin matching the candidate vs the bin at 2x the candidate.
    // If 2K's score is higher, the candidate was a sub-harmonic (cardiac
    // is actually at 2K) -- return 2K's BPM.  Otherwise return the
    // candidate unchanged.
    //
    // This is the documented commercial pattern (Apple US patents
    // 10,736,575 / 11,744,520 / 10,123,746 -- "the metric is the sum of
    // fundamental spectral peak, second harmonic peak and third harmonic
    // peak").  Also the academic state-of-the-art (TROIKA / JOSS
    // spectral peak tracking).  Catches the failure mode where BOTH
    // autocorr and FFT agree on the sub-harmonic during the optical-
    // baseline transient at the start of each snapshot -- the IIR rings
    // at ~0.6 Hz creating coherent low-frequency content visible to
    // both methods, but lacking the harmonic structure of real cardiac.
    float applyHarmonicCheck(float candidate_bpm);
    // runFFT performs HYBRID spectral exclusion:
    //   * dynamic imu_shadow_mask (populated by findStrideBin) shadows
    //     any bin where IMU FFT magnitude >= 3x in-band mean (+/-1),
    //   * AND predictive shadows at stride+/-1, stride/3+/-1, and
    //     2*stride+/-2 -- catches the PPG's biomechanical 2x harmonic
    //     which the IMU's own 2x doesn't strongly mirror.
    // Pass stride_bin <= 0 to disable predictive shadowing (the
    // dynamic mask still runs).
    float runFFT(float* ppg, int stride_bin);
    // Chained 3-stage NLMS: cleans X-, Y-, then Z-correlated motion
    // from the PPG and FFTs the residual.  imu_x / imu_y / imu_z must
    // be DC-removed by the caller so gravity doesn't dominate the
    // adaptation -- otherwise the filter spends its coefficient budget
    // modeling the 1 g offset instead of the kinetic AC content.
    float runAdaptiveNLMS(float* ppg,
                          float* imu_x, float* imu_y, float* imu_z,
                          int stride_bin);
    // Find the dominant IMU motion-frequency bin (the user's stride
    // cadence during walking/running).  Uses the highest-variance
    // raw axis (X, Y, or Z) -- NOT SMV.  SMV's magnitude operation
    // doubles the AC frequency of the underlying motion, which moves
    // a 3 Hz sprint stride out to 6 Hz (out of our [0.6, 3.33 Hz]
    // search range) and causes findStrideBin to lock onto the wrong
    // sub-harmonic.  Raw axes preserve the fundamental frequency
    // regardless of cadence.  Side effect: overwrites fft_output[]
    // and fft_magnitudes[] (those buffers get overwritten again by
    // runFFT later in the same window, so no real harm).
    int findStrideBin(float* imu_x, float* imu_y, float* imu_z);
    // autocorr_n is the LENGTH OF THE INPUT to arm_correlate_f32, not
    // the length of the output array.  Output has 2*autocorr_n - 1
    // elements with zero lag at index autocorr_n - 1.
    int findSecondPeak(float* data, int autocorr_n);

    // Reset Kalman + NLMS to their initial values.  Called on the
    // NOT_WORN -> STABILIZING edge so the first BPM after wearing isn't
    // pulled toward a stale Kalman state and NLMS coefficients aren't
    // poisoned by transition-window data.
    void resetAdaptiveFilters();

public:
    WearableDSP();
    // imu_smv is used ONLY for motion-state variance classification
    // (still the right metric for that: gravity-removed SMV variance
    // cleanly separates stationary / micro / heavy regardless of axis
    // orientation).  imu_x / imu_y / imu_z are the references for the
    // chained NLMS motion-artifact cancellation.
    float processHeartRate(float* ppg_buffer,
                           float* imu_smv,
                           float* imu_x,
                           float* imu_y,
                           float* imu_z);
    WearState getWearState() const { return wear_state; }
    MotionState getMotionState() const { return last_motion; }
};
