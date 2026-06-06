#include "cursor_pipeline.h"
#include "ble_hid.h"
#include "gesture_mode.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(cursor, LOG_LEVEL_INF);

/*
 * --- Shared state ----------------------------------------------------
 *
 * Floats so callers can accumulate sub-pixel motion across publish
 * ticks.  Quantization to int8 happens in the publish thread right
 * before bt_gatt_notify.  Protected by a single mutex; lock-hold
 * time is tiny so contention is not a real concern at our rates.
 */
static K_MUTEX_DEFINE(state_mutex);
static float pending_dx = 0.0f;
static float pending_dy = 0.0f;
static float pending_scroll = 0.0f;
static uint8_t current_buttons = 0;

/*
 * --- Helpers ---------------------------------------------------------
 */

static inline int8_t clamp_to_int8(float v)
{
    if (v >  127.0f) return  127;
    if (v < -127.0f) return -127;
    return (int8_t)v;
}

/*
 * Apply sensitivity + dead zone + int8 clamp.  Updates *remaining to
 * carry any sub-pixel residual to the next tick.
 *
 * Example: caller injects 2.7 pixels of motion at 1.0 sensitivity.
 * We send 2 pixels, carry 0.7 to next tick.  Over time the cursor
 * tracks the intended motion with no rounding-down bias.
 */
static int8_t consume_axis(float *pending, float sensitivity)
{
    float scaled = *pending * sensitivity;
    if (fabsf(scaled) < (float)CURSOR_DEAD_ZONE_PX) {
        /* Below dead zone -- send nothing, retain full accumulator
         * for next tick. */
        return 0;
    }
    int8_t out = clamp_to_int8(scaled);
    /* Carry the residual.  out is what we actually sent, scaled is
     * what we wanted to send.  The difference (in pre-sensitivity
     * units) stays in the accumulator. */
    *pending -= (float)out / sensitivity;
    return out;
}

/*
 * Whether the current mode wants cursor reports published.
 */
static bool mode_allows_cursor(void)
{
    GestureMode m = gesture_mode_get();
    return (m == MODE_AIR_MOUSE) || (m == MODE_SURFACE);
}

/*
 * --- Publish thread --------------------------------------------------
 */

static void cursor_publish_thread(void *, void *, void *)
{
    LOG_INF("Cursor publish thread started at %u Hz", CURSOR_PUBLISH_HZ);
    const uint32_t period_ms = 1000U / CURSOR_PUBLISH_HZ;

    while (1) {
        k_sleep(K_MSEC(period_ms));

        /* Snapshot + reset under mutex.  Lock-hold time is microseconds. */
        int8_t out_dx, out_dy, out_scroll;
        uint8_t out_buttons;
        bool have_motion;
        k_mutex_lock(&state_mutex, K_FOREVER);
            out_dx     = consume_axis(&pending_dx,     CURSOR_SENSITIVITY);
            out_dy     = consume_axis(&pending_dy,     CURSOR_SENSITIVITY);
            out_scroll = consume_axis(&pending_scroll, CURSOR_SENSITIVITY);
            out_buttons = current_buttons;
            have_motion = (out_dx != 0) || (out_dy != 0) ||
                          (out_scroll != 0) || (out_buttons != 0);
        k_mutex_unlock(&state_mutex);

        /* Only emit a report if the mode allows it AND something is
         * actually happening.  Mode gating means cursor commands
         * inject silently while in MODE_IDLE -- no host gets spammed
         * with cursor reports we don't want. */
        if (!mode_allows_cursor()) {
            continue;
        }
        if (!have_motion) {
            continue;
        }
        if (!ble_hid_notifications_enabled()) {
            /* No host listening; don't waste a notify call. */
            continue;
        }

        (void)ble_hid_send_report(out_buttons, out_dx, out_dy, out_scroll);
    }
}

/*
 * The thread stack lives in BSS; ~1 KB is enough since the thread
 * itself does very little (no recursion, small autos, no big stack
 * buffers).  Priority 8 is below the DSP thread (priority 7) so
 * cursor publish never delays an HR window.
 */
#define CURSOR_THREAD_STACK_SIZE   1024
#define CURSOR_THREAD_PRIORITY     8

K_THREAD_STACK_DEFINE(cursor_thread_stack, CURSOR_THREAD_STACK_SIZE);
static struct k_thread cursor_thread_data;

/*
 * --- Public API -----------------------------------------------------
 */

int cursor_pipeline_init(void)
{
    k_thread_create(&cursor_thread_data, cursor_thread_stack,
                    K_THREAD_STACK_SIZEOF(cursor_thread_stack),
                    cursor_publish_thread, NULL, NULL, NULL,
                    CURSOR_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&cursor_thread_data, "cursor_publish");
    return 0;
}

void cursor_pipeline_inject_motion(float dx, float dy)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    pending_dx += dx;
    pending_dy += dy;
    k_mutex_unlock(&state_mutex);
}

void cursor_pipeline_inject_scroll(float scroll)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    pending_scroll += scroll;
    k_mutex_unlock(&state_mutex);
}

void cursor_pipeline_set_buttons(uint8_t buttons)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    current_buttons = buttons & 0x07;
    k_mutex_unlock(&state_mutex);
}

void cursor_pipeline_click(uint8_t buttons)
{
    cursor_pipeline_set_buttons(buttons);
    /* Hold long enough for at least one publish tick to capture the
     * press, then release.  2 * period_ms guarantees one publish
     * regardless of where we are in the tick. */
    k_sleep(K_MSEC(2 * (1000U / CURSOR_PUBLISH_HZ)));
    cursor_pipeline_set_buttons(0);
}

void cursor_pipeline_get_pending(float *out_dx, float *out_dy,
                                 float *out_scroll)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    if (out_dx)     *out_dx     = pending_dx;
    if (out_dy)     *out_dy     = pending_dy;
    if (out_scroll) *out_scroll = pending_scroll;
    k_mutex_unlock(&state_mutex);
}
