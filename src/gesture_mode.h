/*
 * gesture_mode -- wrist-orientation classification + mode FSM
 *
 * Reads the accelerometer at the acquisition rate and maintains a
 * low-pass-filtered gravity vector to determine wrist orientation.
 * Detects trigger gestures (raise-to-air, flat-to-surface, wrist
 * flick, double-tap-on-band via LSM6DSL chip event) and transitions
 * a mode finite-state-machine accordingly.
 *
 * The mode is published via atomic_t and read by the cursor pipeline
 * and any future gesture-classifier layer to decide whether their
 * output is bound to BLE HID, custom GATT, or discarded.
 *
 * Architecture reference: see docs/research/gesture-architecture.md
 * sections 2 and 3.  This module implements the "MODE DETECTOR" box
 * in the unified architecture diagram.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Top-level operating mode.  Set by trigger gestures, observed by
 * cursor / classifier / BLE output stages to decide what to do with
 * detected gesture events.
 *
 * MODE_IDLE:
 *   No gesture-driven output.  Chip-embedded sig-motion / tap events
 *   still fire (we use them to wake into other modes), but the
 *   cursor pipeline does NOT publish HID reports and the gesture
 *   classifier (when later wired) does NOT publish custom-GATT events.
 *   This is the default state on boot and after any cancel / timeout.
 *
 * MODE_SURFACE:
 *   Wrist is resting flat on a surface (typical desk position).
 *   Cursor pipeline publishes HID reports.  LSM6 tap engine is
 *   retuned (later) to detect impulses propagating through the desk.
 *
 * MODE_AIR_MOUSE:
 *   Wrist is raised in air.  Cursor pipeline publishes HID reports.
 *   Pinch classifier (when later wired) generates click events.
 *
 * MODE_GESTURE_AMBIENT:
 *   Reserved for a future ambient-gesture mode that needs no explicit
 *   trigger.  Not used in Item 0; declared so the enum reflects the
 *   roadmap.
 */
typedef enum {
    MODE_IDLE = 0,
    MODE_SURFACE,
    MODE_AIR_MOUSE,
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
 * interrupt fired.  Double-tap is the AIR_MOUSE entry trigger -- the
 * raised "drawing on whiteboard" pose.  Called from IDLE -> enters
 * AIR_MOUSE.  Called from any non-IDLE state -> ignored (logged).
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
 * This entry point feeds a firmware-side multi-tap counter (added
 * in stage C).  Per the design:
 *   - 3 single-tap events inside a window -> chained to
 *     _on_chip_triple_tap() (SURFACE entry).  This is the firmware-
 *     side path because the chip is native single + double only.
 *   - 1 event, no follow-up within window -> currently a no-op; the
 *     surface-tap re-engage path (later) will hook here.
 *   - 2 single-taps without a co-asserted DOUBLE_TAP would be
 *     unusual (the chip's hardware double-tap classifier should
 *     fire on the second shock) -- if observed, gets counted toward
 *     the 3-for-triple window.
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
 * Inform the mode detector that a triple-tap was detected.  Triple-tap
 * is the SURFACE entry trigger -- the wrist-on-desk "touchpad" pose.
 * Called from IDLE -> enters SURFACE.  Called from any non-IDLE state
 * -> ignored (logged).
 *
 * Wired to the serial 'y' test command and to the firmware multi-tap
 * counter (stage C) when 3 single-taps land inside the window.
 */
void gesture_mode_on_chip_triple_tap(void);

/*
 * Read the current mode.  Thread-safe (uses atomic_t internally).
 * Cheap; safe to call from any thread including ISRs.
 */
GestureMode gesture_mode_get(void);

/*
 * Force the mode to a specific value.  Used by serial test commands
 * and unit-test scaffolding.  Production callers should rely on
 * trigger-gesture-driven transitions, not this.
 */
void gesture_mode_set(GestureMode mode);

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
 * Diagnostic: cooldown samples remaining (each sample = 10 ms at
 * 100 Hz acquisition).  Zero means cooldown is closed and the user
 * must double-tap to re-enter AIR_MOUSE; nonzero means the
 * orientation-only re-engage path is still open.
 */
int gesture_mode_get_cursor_cooldown_remaining(void);

/*
 * Acquisition-request callback signature.  Registered by main.cpp
 * so gesture_mode can ask the power-state machine to keep the IMU
 * sampling pipeline alive while AIR_MOUSE / SURFACE modes need it.
 *
 *   needs = true   -- gesture mode is entering a state that requires
 *                     continuous IMU samples (cursor + orientation).
 *                     Callback should ensure acq is running.
 *   needs = false  -- gesture mode is leaving such a state.  Callback
 *                     may stop acq if no other consumer needs it
 *                     (typically: power state is IDLE).
 *
 * The callback is invoked from inside _transition_to (i.e. acq thread
 * context or wherever gesture_mode_on_chip_double_tap was called from).
 * Implementations should be quick and safe to call from those contexts.
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
