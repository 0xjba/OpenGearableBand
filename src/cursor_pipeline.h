/*
 * cursor_pipeline -- IMU/test -> HID mouse cursor publish pipeline
 *
 * Provides the data path from "we have a cursor delta to send" to
 * "HID notification transmitted to the connected host."  Does NOT
 * compute the cursor delta from gyro -- that's a later roadmap item
 * (Item 2: air mouse cursor + IMU pinch).  This module is the
 * scaffolding so subsequent items just call cursor_pipeline_inject_*
 * and don't worry about rate control / smoothing / HID details.
 *
 * Threading model:
 *   - Any thread can call cursor_pipeline_inject_*() to accumulate
 *     pending deltas / set button state.  Atomic / mutex-protected
 *     internally.
 *   - A dedicated low-priority publish thread (started by
 *     cursor_pipeline_init) runs at CURSOR_PUBLISH_HZ, applies
 *     sensitivity + dead zone + int8 quantization, calls
 *     ble_hid_send_report, then zeroes pending deltas.
 *
 * Architecture reference: see docs/research/gesture-architecture.md
 * Item 0 section ("Component C: Cursor pipeline scaffolding").
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Publish thread cadence.  125 Hz matches the standard wired-mouse
 * USB polling rate; HOGP can comfortably sustain this with one host.
 */
#define CURSOR_PUBLISH_HZ           125

/*
 * Dead zone (pixels per publish tick).  Pending delta magnitudes
 * below this are not transmitted, preventing gyro-noise jitter from
 * showing as cursor drift when the user holds still.
 *
 * In int8 units: small enough to feel responsive on intentional
 * motion, large enough to suppress sub-pixel jitter.
 */
#define CURSOR_DEAD_ZONE_PX         1

/*
 * Linear sensitivity for the test path.  Real tracking (Item 2) will
 * use a proper acceleration curve.  For now: 1.0 = pass-through.
 */
#define CURSOR_SENSITIVITY          1.0f

/*
 * Initialise pipeline state and start the publish thread.  Called
 * once from main() after ble_hid_init().  Returns 0 on success.
 *
 * The publish thread runs forever and gates output based on the
 * current gesture mode (only publishes in MODE_AIR_MOUSE / SURFACE),
 * cursor pipeline thus respects the mode FSM automatically.
 */
int cursor_pipeline_init(void);

/*
 * Inject pending cursor motion.  Deltas are floats so calling layers
 * can accumulate fractional motion across publish ticks (e.g. gyro
 * integration produces sub-pixel motion per ms).  The publish thread
 * quantizes the accumulator to int8 at send time.
 *
 * Thread-safe.  Multiple producers can inject deltas concurrently;
 * they accumulate additively.  Cheap (one mutex acquire + add).
 */
void cursor_pipeline_inject_motion(float dx, float dy);

/*
 * Inject pending scroll.  Same semantics as motion.
 */
void cursor_pipeline_inject_scroll(float scroll);

/*
 * Set the current button state (combination of BLE_HID_BTN_* flags
 * from ble_hid.h).  Absolute, not relative -- pass 0 to release all
 * buttons, BLE_HID_BTN_LEFT to hold left only, etc.
 *
 * The publish thread sends the current button state in every report,
 * so callers can hold a button by leaving the bit set across multiple
 * publishes (e.g. for drag operations).
 */
void cursor_pipeline_set_buttons(uint8_t buttons);

/*
 * Generate a "click" -- transient press + release on the specified
 * button(s).  Internally sets the buttons, sleeps for one publish
 * tick (~8 ms at 125 Hz), then clears them.  Blocks the calling
 * thread; should be called from a thread context that can sleep
 * (e.g. the serial console test command path, not an ISR).
 */
void cursor_pipeline_click(uint8_t buttons);

/*
 * Diagnostic: get current pending-motion magnitude for logging.
 * Cheap; safe from any thread.
 */
void cursor_pipeline_get_pending(float *out_dx, float *out_dy,
                                 float *out_scroll);

#ifdef __cplusplus
}
#endif
