/*
 * gesture_mode -- wrist-orientation classification + mode FSM
 *
 * Reads the accelerometer at the acquisition rate and maintains a
 * low-pass-filtered gravity vector to determine wrist orientation.
 * Detects trigger gestures (raise-to-air, flat-to-surface, wrist
 * flick, double-tap-on-band via LSM6DSL chip event) and transitions
 * a mode finite-state-machine accordingly.
 *
 * The mode is published via atomic_t and read by any future
 * gesture-classifier layer to decide whether their output is bound to
 * custom GATT or discarded.
 *
 * Architecture reference: see docs/research/gesture-architecture.md
 * sections 2 and 3.  This module implements the "MODE DETECTOR" box
 * in the unified architecture diagram.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gesture_poses.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Top-level operating mode.  Set by trigger gestures, observed by the
 * classifier output stages to decide what to do with detected gesture
 * events.
 *
 * MODE_IDLE:
 *   No gesture-driven output.  Chip-embedded sig-motion / tap events
 *   still fire (we detect + log them), but the gesture classifier
 *   (when later wired) does NOT publish custom-GATT events.  This is
 *   the default state on boot and after any cancel / timeout.
 *
 * MODE_GESTURE_AMBIENT:
 *   Reserved for a future ambient-gesture mode that needs no explicit
 *   trigger.  Not yet wired; declared so the enum reflects the roadmap
 *   and a future mode is a small addition to the FSM.
 */
typedef enum {
    MODE_IDLE = 0,
    MODE_GESTURE_AMBIENT,
} GestureMode;

/*
 * Intermediate wrist-orientation classification derived from the
 * low-pass-filtered accelerometer vector.  Used internally to drive
 * mode transitions; exposed for diagnostics.
 *
 * WRIST_DOWN_FLAT:
 *   Forearm horizontal, palm-down -- consistent with the wrist
 *   resting on a desk surface.  Gravity points strongly into +Z (or
 *   -Z, depending on mount orientation -- handled by the dominant-
 *   axis classifier).
 *
 * WRIST_UP_RAISED:
 *   Forearm raised, screen-side facing the user, palm-side facing
 *   downward / forward.  Consistent with the "raise to check the
 *   watch" / "raise to use air mouse" position.
 *
 * WRIST_NEUTRAL:
 *   Anything else: transitioning, vertical, hand at the side, etc.
 *   Mode FSM never auto-transitions into a cursor-bearing mode from
 *   NEUTRAL -- only DOWN_FLAT or UP_RAISED can trigger that.
 */
typedef enum {
    WRIST_NEUTRAL = 0,
    WRIST_DOWN_FLAT,
    WRIST_UP_RAISED,
} WristOrientation;

/*
 * Initialise the mode-detector internal state (filter coefficients,
 * trigger gesture state machines, mode = MODE_IDLE).  Called once
 * from main() at boot before any sensor data arrives.
 */
void gesture_mode_init(void);

/*
 * Feed one accelerometer sample to the mode detector.  Should be
 * called from the acquisition thread at the sample rate (100 Hz).
 *
 * Inputs are in m/s^2 (gravity = ~9.81).  The detector low-passes
 * internally; caller does NOT need to pre-filter.
 *
 * Side effects:
 *   - Updates internal gravity-vector estimate
 *   - Reclassifies orientation (cheap; runs every call)
 *   - Runs trigger-gesture detectors (raise-to-air, flat-to-surface,
 *     wrist-flick-to-cancel)
 *   - Transitions mode FSM if a trigger fires
 *   - Logs every mode transition via LOG_INF
 */
void gesture_mode_update_accel(float ax, float ay, float az);

/*
 * Feed one gyroscope sample (rad/s) to the mode detector.  Used for
 * wrist-flick cancel detection.  Optional -- the detector still
 * works on accel alone if gyro is unavailable, just without flick
 * detection.
 *
 * Called from the acquisition thread at the sample rate.
 */
void gesture_mode_update_gyro(float gx, float gy, float gz);

/*
 * Inform the mode detector that the LSM6DSL chip-embedded double-tap
 * interrupt fired.  After the air-mouse extraction a committed
 * double-tap is detect + log only (no mode bound) -- the detection is
 * kept live so a future mode can be wired here.
 *
 * Called from the GPIO INT1 callback in main.cpp once Stage 2 wires
 * the chip-embedded tap engine.  Stage 1 wires it to the serial 't'
 * test command.
 */
void gesture_mode_on_chip_double_tap(void);

/*
 * Inform the mode detector that the LSM6DSL chip-embedded SINGLE_TAP
 * interrupt fired (without a co-asserted DOUBLE_TAP bit -- the
 * dispatcher in main.cpp prefers DOUBLE_TAP when both are set on
 * the same TAP_SRC read).
 *
 * This entry point feeds a firmware-side multi-tap counter.  The
 * counter classifies the sequence (single / double / triple) and,
 * post air-mouse extraction, the commit handler DETECTS + LOGS the
 * gesture but binds it to no mode (the cursor that consumed taps was
 * removed; the detection stays live as a hook for future modes).
 *   - 2 single-taps inside a window -> "double-tap" gesture (log-only).
 *   - 3 inside a window -> "triple-tap" gesture (log-only).
 *   - 1 with no follow-up -> no-op.
 *
 * The counter is gated by an activity check (must have been still
 * for ~500 ms prior) to suppress gait-driven false positives.
 *
 * In stage B, this entry is a stub that only logs the arrival --
 * lets us validate that chip taps reach the firmware before we
 * layer the counter logic on top.
 *
 * Called from the INT1 dispatcher in main.cpp after reading TAP_SRC.
 *   peak_axis: 'X', 'Y', or 'Z' (axis where slope dominated)
 *   tap_sign:  '+' or '-' (sign of the slope at detection)
 */
void gesture_mode_on_chip_single_tap(char peak_axis, char tap_sign);

/*
 * Inform the mode detector that a triple-tap was detected.  Post
 * air-mouse extraction this is UNBOUND: it detects + logs the gesture
 * but enters no mode (SURFACE was retired and air-mouse extracted).
 * Kept as a free trigger for a future mode to bind.
 *
 * Wired to the serial 'y' test command and to the firmware multi-tap
 * counter when 3 single-taps land inside the window.
 */
void gesture_mode_on_chip_triple_tap(void);

/* Pose state machine query.  Returns the currently-armed pose, or
 * POSE_NONE if no pose is armed.  Used by the chip-tap path to
 * decide whether to accept a tap as a mode-entry trigger. */
pose_id_t gesture_mode_armed_pose(void);

/* True if the user is mid-gesture: a pose is currently armed OR a
 * chip-tap occurred within RECENT_GESTURE_GUARD_MS.  The power state
 * machine uses this to suppress the significant-motion -> WORKOUT_VERIFY
 * transition during gestures (rapid taps must not read as exercise). */
bool gesture_mode_recent_activity(void);

/* Start a pose-observability trace: hold a pose still and this logs
 * gravity + Y-Z shadow + forearm-from-vertical + pitch/roll at ~5 Hz for
 * n_samples (100 Hz acq).  Measures the roll-observability cone on the
 * current mount; assumes nothing from prior sessions. */
void gesture_mode_pose_trace_start(uint32_t n_samples);

/*
 * Read the current mode.  Thread-safe (uses atomic_t internally).
 * Cheap; safe to call from any thread including ISRs.
 */
GestureMode gesture_mode_get(void);

/*
 * Read the current orientation classification.  Diagnostics-only --
 * mode transitions happen automatically inside the detector based on
 * this, callers don't need to drive it manually.
 */
WristOrientation gesture_mode_get_orientation(void);

/*
 * Convert a mode enum value to a human-readable string for logging.
 * Returns a pointer to a static string; caller does not free.
 */
const char *gesture_mode_str(GestureMode mode);
const char *wrist_orientation_str(WristOrientation o);

/*
 * Calibration helper: read the current filtered gravity vector.
 * Used by the serial 'g' test command to dump the live reading so
 * the user can hold the band in their intended pose and capture the
 * (gx, gy, gz) triplet to update the orientation classifier mapping.
 *
 * Values are in m/s^2.  Caller passes non-NULL out_* pointers.
 */
void gesture_mode_get_gravity(float *out_gx, float *out_gy, float *out_gz);

/*
 * Acquisition-request callback signature.  Registered by main.cpp so
 * gesture_mode can ask the power-state machine to keep the IMU sampling
 * pipeline alive.  Under the always-on policy the gesture mode requests
 * continuous IMU samples so the pose FSM keeps running in MODE_IDLE
 * (the always-listening trigger state) even when the HR power state is
 * IDLE.
 *
 *   needs = true   -- ensure acq is running.
 *   needs = false  -- acq may stop if no other consumer needs it
 *                     (typically: power state is IDLE).
 *
 * The callback is invoked from inside gesture_mode (acq thread context
 * or wherever the registering call ran).  Implementations should be
 * quick and safe to call from those contexts.
 */
typedef void (*gesture_acq_request_cb_t)(bool needs);

/*
 * Register the acquisition-request callback.  Called once from main()
 * at boot, after the acq pipeline is constructed.  Passing NULL clears
 * the callback (used in unit tests / shutdown).
 */
void gesture_mode_set_acq_request_cb(gesture_acq_request_cb_t cb);

#ifdef __cplusplus
}
#endif
