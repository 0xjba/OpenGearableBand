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
// Number of consecutive passing windows required for WORN.  Buffer is
// BUFFER_SIZE samples = 2 overlap-windows long, so 2 passes guarantees the
// entire buffer is post-wear data.  Use 3 for one extra margin window.
#define WEAR_PASSES_REQUIRED     3

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
    arm_lms_norm_instance_f32 lms_inst;

    // 4th-order Butterworth band-pass IIR (0.6 - 3.3 Hz @ 100 Hz Fs),
    // 2 biquad sections in cascade.  State buffer is 4 floats per stage.
    arm_biquad_casd_df1_inst_f32 bp_inst;
    float bp_state[4 * 2];

    float lms_state[BUFFER_SIZE + 32];
    float lms_coeffs[32];
    float fft_output[BUFFER_SIZE];
    float fft_magnitudes[BUFFER_SIZE / 2];

    // Large buffers moved from stack to member variables to prevent stack overflow
    float correlation[BUFFER_SIZE * 2 - 1];
    float clean_signal[BUFFER_SIZE];
    float error[BUFFER_SIZE];

    // Wear-state machine state
    WearState wear_state = WEAR_NOT_WORN;
    int wear_pass_count = 0;

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
    float runFFT(float* ppg);
    float runAdaptiveNLMS(float* ppg, float* imu);
    int findSecondPeak(float* data, int length);

    // Reset Kalman + NLMS to their initial values.  Called on the
    // NOT_WORN -> STABILIZING edge so the first BPM after wearing isn't
    // pulled toward a stale Kalman state and NLMS coefficients aren't
    // poisoned by transition-window data.
    void resetAdaptiveFilters();

public:
    WearableDSP();
    float processHeartRate(float* ppg_buffer, float* imu_buffer);
    WearState getWearState() const { return wear_state; }
};
