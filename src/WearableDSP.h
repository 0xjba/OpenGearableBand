#pragma once
#include <arm_math.h>
#include <stdint.h>

#define BUFFER_SIZE 512      // 5.12 seconds of data at 100Hz
#define SAMPLE_RATE 100.0f
#define MIN_BPM 40.0f
#define MAX_BPM 200.0f

// Motion classification driven by IMU SMV variance.  STATIONARY is the
// only state for which we currently produce a BPM update -- the MICRO /
// HEAVY motion-artifact-cancellation paths have been parked on the
// `feature/motion-path-experiments` branch pending further work.  In
// this build, MICRO and HEAVY both hold Kalman and signal "stay steady"
// to the caller.
enum MotionState {
    STATIONARY,
    MICRO_MOTION,
    HEAVY_MOTION
};

// Wear state with hysteresis.  The 50%-overlap window means a fresh OFF->ON
// transition contaminates the next two windows (the first holds ~50% stale
// off-wrist data; the second can still hold the leading edge).  Producing
// BPM on those windows yields garbage because the DC step dominates the
// FFT / autocorrelation.  We therefore require N consecutive "mean above
// threshold" windows before declaring WORN.
enum WearState {
    WEAR_NOT_WORN,    // PPG mean below threshold; no BPM produced
    WEAR_STABILIZING, // mean just passed, but buffer may still hold stale samples
    WEAR_WORN         // buffer is guaranteed 100% post-wear; produce BPM
};

// PPG raw-count threshold (MAX30102 IR channel, LED2_PA=0x66 ~ 20 mA).
// Typical worn signal: 30k-80k.  Ambient / no contact: < 2k.
#define WEAR_PPG_THRESHOLD       10000.0f

// Number of consecutive passing windows required for WORN.  Buffer is
// BUFFER_SIZE samples = 2 overlap-windows long, so 2 passes guarantees
// the entire buffer is post-wear data.  4 gives the skin one more
// 2.5 s window to physiologically settle (skin compresses, capillaries
// equilibrate, LED warms the surface) before we start trusting the math.
// Observed in v0.2 boot snapshots: ppg_dc rises 100-300 counts per
// window for the first 2-3 worn windows as the optical baseline drifts.
#define WEAR_PASSES_REQUIRED     4

// Number of windows the motion-state classifier remains "warm" after
// any motion event.  1 window = one 50%-overlap step (~2.56 s) --
// long enough to cover the post-motion settle, short enough that we
// return to the clean STATIONARY path quickly once the wrist is still.
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

    // 4th-order Butterworth band-pass IIR (0.6 - 3.3 Hz @ 100 Hz Fs),
    // 2 biquad sections in cascade.  State buffer is 4 floats per stage.
    arm_biquad_casd_df1_inst_f32 bp_inst;
    float bp_state[4 * 2];

    float fft_output[BUFFER_SIZE];
    float fft_magnitudes[BUFFER_SIZE / 2];

    // Autocorrelation output: arm_correlate_f32 on N samples produces
    // (2N - 1) samples with zero lag at index N - 1.  Moved off stack
    // to prevent overflow.
    float correlation[BUFFER_SIZE * 2 - 1];

    // Wear-state machine state
    WearState wear_state = WEAR_NOT_WORN;
    int wear_pass_count = 0;

    // Last motion classification produced by processHeartRate().
    // Exposed via getMotionState() so callers can react (e.g. the
    // power state machine, or the BLE layer signalling "stay steady").
    MotionState last_motion = STATIONARY;

    // Motion-state hysteresis.  When IMU variance crosses into MICRO or
    // HEAVY in window N, we cannot trust the IMU's "quiet now" reading
    // in window N + 1 -- mechanical settle of the blood column and IIR
    // ringing across the 50% overlap conspire to corrupt the immediately-
    // following stationary classification.  motion_cooldown counts down
    // from MOTION_COOLDOWN_WINDOWS after every motion-detected window;
    // while > 0, the path is forced to at least MICRO_MOTION regardless
    // of current IMU variance.
    int motion_cooldown = 0;

    float runAutocorrelation(float* ppg);
    // Stationary-only FFT.  Runs on the band-passed PPG buffer.
    // Returns the BPM at the strongest bin in the cardiac band
    // [0.6, 3.33 Hz].  Used as the second opinion to cross-validate
    // runAutocorrelation() and catch the sub-harmonic failure mode
    // (autocorr can lock onto the 2T peak when the cardiac T-peak is
    // below 50 % of the in-band global max).
    float runStationaryFFT(float* ppg);
    // Reconcile autocorr and FFT picks for the STATIONARY path.
    // Returns the chosen BPM (or 0 to signal "hold Kalman") and writes
    // a confidence in [0, 1] to *out_confidence:
    //   1.0  -- both methods agreed within +/-10 BPM (average returned)
    //   0.5  -- one method silent / 2:1 ratio resolved (sub-harmonic)
    //   0.0  -- disagree without explanation (caller should hold Kalman)
    float reconcileStationary(float bpm_ac, float bpm_fft, float *out_confidence);
    // Harmonic-sum score for FFT bin K: fft_magnitudes[K] + [2K] + [3K].
    // Out-of-range harmonic terms are omitted.  Real cardiac peaks
    // show rich harmonic structure (2nd harmonic typically 30-70 % of
    // fundamental in wrist PPG, 3rd often visible); IIR-ring artifacts
    // and sub-harmonic peaks do not.  Requires fft_magnitudes to hold
    // the current window's FFT (populated by runStationaryFFT).
    float harmonicScore(int K);
    // Harmonic-structure post-check applied after reconcileStationary
    // agrees on a candidate.  If 2K's harmonic-sum score is higher than
    // K's, the candidate was a sub-harmonic and we snap to 2K instead.
    // Documented commercial pattern (Apple US patents 10,736,575 /
    // 11,744,520 / 10,123,746 -- "the metric is the sum of fundamental
    // spectral peak, second harmonic peak and third harmonic peak");
    // also the TROIKA / JOSS academic state-of-the-art.
    float applyHarmonicCheck(float candidate_bpm);
    // autocorr_n is the LENGTH OF THE INPUT to arm_correlate_f32, not
    // the length of the output array.  Output has 2*autocorr_n - 1
    // elements with zero lag at index autocorr_n - 1.
    int findSecondPeak(float* data, int autocorr_n);

    // Reset Kalman + band-pass state to initial values.  Called on the
    // NOT_WORN -> STABILIZING edge so the first BPM after wearing isn't
    // pulled toward a stale Kalman state.
    void resetAdaptiveFilters();

public:
    WearableDSP();
    // imu_smv is the Signal Magnitude Vector of the accelerometer with
    // gravity removed by the caller, used purely for motion-state
    // variance classification.  Returns the filtered BPM (or the
    // current Kalman state if the window is non-stationary / not worn /
    // signal quality fails).
    float processHeartRate(float* ppg_buffer, float* imu_smv);
    WearState getWearState() const { return wear_state; }
    MotionState getMotionState() const { return last_motion; }
    // True when the last processed window detected motion (MICRO or
    // HEAVY) on a worn device.  Callers signalling the user to hold
    // still (e.g. the BLE layer suppressing HR notifications) check
    // this after every processHeartRate() call.
    bool needsSteady() const {
        return wear_state == WEAR_WORN && last_motion != STATIONARY;
    }
};
