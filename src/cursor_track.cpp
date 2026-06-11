#include "cursor_track.h"
#include "gesture_thresholds.h"

#include <math.h>

static_assert(CURSOR_ROLL_SHADOW_REVALIDATE >= CURSOR_ROLL_SHADOW_INVALIDATE,
              "CURSOR_ROLL_SHADOW_REVALIDATE must be >= INVALIDATE (cone-gate hysteresis)");

static float s_prev_pitch = 0.0f;
static float s_prev_roll  = 0.0f;
static bool  s_roll_valid = false;
static bool  s_started    = false;

/* Wrap an angle delta into [-180,180] so the roll (atan2) discontinuity at
 * +/-180 doesn't produce a ~360 deg jump. */
static float wrap180(float d)
{
    /* if (not while): the raw delta of two bounded angles is in [-360,+360],
     * so a single step lands in [-180,+180].  A while loop would hang on a
     * non-finite input -- fatal on an MCU with no watchdog here. */
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

void cursor_track_start(float pitch_deg, float roll_deg)
{
    s_prev_pitch = pitch_deg;
    s_prev_roll  = roll_deg;
    s_roll_valid = false;   /* gate X until the shadow clearly clears the cone */
    s_started    = true;
}

void cursor_track_update(float pitch_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
    if (!s_started) { *out_dx = 0.0f; *out_dy = 0.0f; return; }

    float dpitch = wrap180(pitch_deg - s_prev_pitch);
    float droll  = wrap180(roll_deg  - s_prev_roll);

    /* (3c) Euler-wrap / glitch guard: a real wrist move is << this per tick. */
    if (fabsf(dpitch) > CURSOR_MAX_DELTA_DEG) dpitch = 0.0f;
    if (fabsf(droll)  > CURSOR_MAX_DELTA_DEG) droll  = 0.0f;

    /* (3a) Cone-gate the X axis with hysteresis (latched). */
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_roll_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_roll_valid = false;
    /* between the two thresholds: s_roll_valid holds */

    /* (3b) Asymmetric freeze: dwell-engage via at_rest, immediate release on
     * any per-tick motion (more sensitive than at_rest's gyro threshold, so
     * slow precision moves are not eaten). */
    float ang_speed = fabsf(dpitch) + fabsf(droll);
    bool frozen = at_rest && (ang_speed < CURSOR_FREEZE_RELEASE_DELTA);

    float dx = 0.0f, dy = 0.0f;
    if (!frozen) {
        dy = CURSOR_GAIN_Y * dpitch;
        dx = s_roll_valid ? (CURSOR_GAIN_X * droll) : 0.0f;
    }

    *out_dx = dx;
    *out_dy = dy;

    /* Always re-sync prev -> deltas are tick-to-tick; a frozen/gated tick
     * discards that motion instead of accumulating a jump. */
    s_prev_pitch = pitch_deg;
    s_prev_roll  = roll_deg;
}

void cursor_track_stop(void)
{
    s_roll_valid = false;
    s_started    = false;
}
