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

// Slew-rate limiter on raw_bpm vs. current Kalman state.
//
// Without this, the v0.3 jogging trace showed Kalman dropping 119 -> 95
// in ~25 s after the user stopped running -- pulled down by noisy NLMS
// raws (single-window values of 46, 58, 70 BPM) while the user's actual
// HR was decaying gradually.  Apple Watch's "sticky" post-workout
// behavior is exactly this construct: reject raws that imply
// biologically impossible per-window changes.
//
// Cardiology calibration: Heart Rate Recovery (HRR) for a fit adult is
// 20-30 BPM in the FIRST FULL MINUTE after stopping exercise.  Per
// 2.56 s window that's ~1.0-1.3 BPM.  Our 30 / 50 BPM bounds are an
// order of magnitude above true HRR, so legitimate recovery is fully
// trackable; the bounds only fire on motion-artifact spikes (the
// observed bad raws were 25-70 BPM step changes in a single window).
//
// 50 BPM in 2.5 s = 1200 BPM/min rate of change.  Real exercise onset
// peaks at maybe 60 BPM/min, so 50 in a single window is the edge of
// plausible -- generous enough to track aggressive HR changes but
// rejects the clear motion-artifact spikes the NLMS path produces.
// MICRO is tighter because the user is no longer actively exercising
// when motion is light; real HR change should be slower.
//
// STATIONARY (autocorr) is NOT gated -- that path is independently
// validated at +/-1 BPM accuracy and we want it to converge quickly
// once the wrist is still.
#define MAX_DELTA_MICRO_BPM      30.0f
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
    // runFFT takes an optional stride_bin parameter for spectral
    // exclusion: any bin within +/-1 of stride_bin (or its /3 sub-
    // harmonic) is shadowed from the peak search.  Pass -1 to disable.
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
    // cadence during walking/running).  Returns the FFT bin index of
    // the maximum magnitude in the [0.6, 3.33 Hz] band of the IMU SMV
    // spectrum.  Side effect: overwrites fft_output[] and fft_magnitudes[]
    // (those buffers get overwritten again by runFFT later in the same
    // window, so no real harm).
    int findStrideBin(float* imu_smv);
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
