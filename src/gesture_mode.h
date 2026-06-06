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
 * interrupt fired.  Used to cycle modes or cancel current mode --
 * exact semantics defined in the .cpp implementation (typically:
 * double-tap from IDLE arms the next-trigger gesture; double-tap
 * from a cursor mode returns to IDLE).
 *
 * Called from the GPIO INT1 callback in main.cpp.
 */
void gesture_mode_on_chip_double_tap(void);

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
int gesture_mode_get_air_mouse_cooldown_remaining(void);

#ifdef __cplusplus
}
#endif
