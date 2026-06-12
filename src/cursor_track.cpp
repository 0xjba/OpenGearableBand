#include "cursor_track.h"
#include "gesture_thresholds.h"

#include <math.h>

static_assert(CURSOR_ROLL_SHADOW_REVALIDATE >= CURSOR_ROLL_SHADOW_INVALIDATE,
              "CURSOR_ROLL_SHADOW_REVALIDATE must be >= INVALIDATE (cone-gate hysteresis)");
static_assert(CURSOR_PIN_LEAVE_DEG >= CURSOR_PIN_ENTER_DEG,
              "CURSOR_PIN_LEAVE_DEG must be >= ENTER (re-pin hysteresis)");

/* Threading: cursor_track_start/stop run in transition context (workqueue /
 * power thread); cursor_track_update runs in acq_thread.  These statics are
 * unsynchronised on purpose -- the worst race is one harmless zero-inject
 * (the s_started guard + cursor_pipeline's own mode gate are a double guard,
 * so no stale value reaches the host).
 * NOTE (Task 2): the absolute-Y state added here -- s_slam_remaining and
 * s_cur_y -- is also unsynchronised across the transition thread (start/stop)
 * and the acq thread (update).  Still benign: a one-off mis-read of the slam
 * count is absorbed by the slam's deliberate over-travel, and s_cur_y is
 * initialised on the transition path then owned by acq from the first servo
 * tick onward (the s_started gate bounds the window).  Revisit if Y gains a
 * smoothing filter or a host-fed position correction. */
static float s_prev_roll      = 0.0f;
static bool  s_roll_valid     = false;
static bool  s_started        = false;

/* Absolute-Y state. */
/* Absolute-Y anchors -- now RUNTIME-VARIABLE (natural calibration sets them via
 * cursor_track_set_anchors).  Seeded from the compile-time defaults, which also
 * serve as the cold-start values until the first complete entry ritual. */
static float s_vert_top    = CURSOR_VERT_TOP_DEG;
static float s_vert_bottom = CURSOR_VERT_TOP_DEG + CURSOR_VERT_SPAN_DEG;
static float s_cur_y          = 0.0f;                    /* counts from top         */
static float s_target_counts  = 0.0f;                    /* last servo target, counts from top */
static int   s_slam_remaining = 0;                       /* >0 => slamming          */
static int   s_slam_sign      = -1;                      /* -1 up(top), +1 down     */
static bool  s_at_top         = false;                   /* re-pin hysteresis latch */

/* Runtime gains, seeded from the compile-time defaults.  Tuned live over
 * serial (reachable span = gain * usable-wrist-range), so they are variables
 * rather than the #define constants used directly. */
static float s_gain_x = CURSOR_GAIN_X;
static float s_gain_y = CURSOR_GAIN_Y;

void cursor_track_set_gain(float gain_x, float gain_y)
{
    s_gain_x = gain_x;
    s_gain_y = gain_y;
}

void cursor_track_get_gain(float *gain_x, float *gain_y)
{
    *gain_x = s_gain_x;
    *gain_y = s_gain_y;
}

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

/* vert_bottom = runtime anchor, clamped to the comfort maximum. */
static float vert_bottom(void)
{
    float vb = s_vert_bottom;                       /* runtime anchor (was top + SPAN) */
    if (vb > CURSOR_VERT_BOTTOM_MAX) vb = CURSOR_VERT_BOTTOM_MAX;
    return vb;
}

static float max_counts(void)
{
    float mc = s_gain_y * (vert_bottom() - s_vert_top);
    return (mc > 0.0f) ? mc : 0.0f;
}

static float target_counts(float vert_deg)
{
    float t  = s_gain_y * (vert_deg - s_vert_top);
    float mc = max_counts();
    if (t < 0.0f) t = 0.0f;
    if (t > mc)   t = mc;
    return t;
}

/* Arm a slam burst.  sign -1 => up (to top), +1 => down (to bottom).  Size is a
 * GAIN_Y-independent floor max'd with margin*max_counts so it reaches the edge
 * even untuned; over-travel is harmless (OS clamps).
 * NOTE: cursor_pipeline drains pending_dy at <=127 counts per publish tick
 * (CURSOR_PUBLISH_HZ), so an N-report slam reaches the host over ~N publish
 * ticks -- a visible crawl to the edge, not an instant snap, and s_cur_y is
 * zeroed (here) before the host cursor finishes moving.  The over-travel
 * margin (SLAM_MARGIN/FLOOR) guarantees the final OS edge-clamp regardless, so
 * the transient decouple self-reconciles.  "Cursor crawls to top on entry" is
 * EXPECTED, not a regression. */
static void start_slam(int sign)
{
    float counts = CURSOR_SLAM_MARGIN * max_counts();
    if (counts < CURSOR_SLAM_FLOOR_COUNTS) counts = CURSOR_SLAM_FLOOR_COUNTS;
    int n = (int)ceilf(counts / 127.0f);
    if (n > CURSOR_SLAM_MAX_REPORTS) n = CURSOR_SLAM_MAX_REPORTS;
    s_slam_remaining = n;
    s_slam_sign      = sign;
}

void cursor_track_start(float vert_deg, float roll_deg)
{
    /* Fixed anchor (Amendment A.1): the entry inclination no longer sets the
     * top; vert_deg is unused here now but the arg is kept for API symmetry
     * (the slam + roll seed below still run on entry). */
    (void)vert_deg;
    s_prev_roll  = roll_deg;
    s_roll_valid = false;
    s_cur_y      = 0.0f;
    s_at_top     = true;     /* we enter at the top */
    start_slam(-1);          /* entry slam to the top edge */
    s_started    = true;
}

/* Top re-pin: latched hysteresis around the top anchor.  Entering the
 * [vert_top, vert_top+ENTER] band re-arms a to-top slam ONCE; the latch clears
 * only after leaving past vert_top+LEAVE, so a hover near the edge can't
 * chatter.  LEAVE >= ENTER (see gesture_thresholds.h). */
static void update_top_repin(float vert_deg)
{
    if (!s_at_top && vert_deg <= s_vert_top + CURSOR_PIN_ENTER_DEG) {
        s_at_top = true;
        start_slam(-1);
    } else if (s_at_top && vert_deg > s_vert_top + CURSOR_PIN_LEAVE_DEG) {
        s_at_top = false;
    }
}

void cursor_track_update(float vert_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
    if (!s_started) { *out_dx = 0.0f; *out_dy = 0.0f; return; }

    bool slamming = (s_slam_remaining > 0);   /* snapshot before the Y branch decrements */

    /* ---- Y axis: slam-then-servo (servo owns Y stillness; no freeze gate) ---- */
    float dy = 0.0f;
    if (slamming) {
        dy = (float)(s_slam_sign * 127);   /* emit ONLY the slam burst */
        s_slam_remaining--;
        if (s_slam_remaining == 0) {
            /* s_slam_sign < 0: slam to top -> pin at 0.
             * s_slam_sign > 0: slam to bottom -> pin at max_counts().
             * The +1 (down) path and max_counts() pin are currently
             * unexercised -- reserved for the spec §7 bottom-re-pin
             * upgrade.  Every current caller of start_slam() uses -1. */
            s_cur_y = (s_slam_sign < 0) ? 0.0f : max_counts();
        }
    } else {
        float target = target_counts(vert_deg);
        s_target_counts = target;
        float err = target - s_cur_y;
        if (err >  127.0f) err =  127.0f;
        if (err < -127.0f) err = -127.0f;
        dy = err;
        s_cur_y += dy;
        update_top_repin(vert_deg);   /* may re-arm a to-top slam */
    }

    /* ---- X axis: relative roll (UNCHANGED), but SUPPRESSED during a slam so
     * the slam emits only its Y burst (spec §5).  prev stays synced so X has no
     * jump when tracking resumes. ---- */
    float droll = wrap180(roll_deg - s_prev_roll);
    if (fabsf(droll) > CURSOR_MAX_DELTA_DEG) droll = 0.0f;
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_roll_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_roll_valid = false;
    bool x_frozen = at_rest && (fabsf(droll) < CURSOR_FREEZE_RELEASE_DELTA);
    float dx = 0.0f;
    if (!slamming && !x_frozen && s_roll_valid) dx = s_gain_x * droll;

    *out_dx = dx;
    *out_dy = dy;
    s_prev_roll = roll_deg;
}

bool  cursor_track_is_slamming(void)     { return s_slam_remaining > 0; }
float cursor_track_cur_y(void)           { return s_cur_y; }
float cursor_track_target_counts(void)   { return s_target_counts; }
float cursor_track_vert_top(void)        { return s_vert_top; }
float cursor_track_vert_bottom(void)     { return vert_bottom(); }

void cursor_track_set_anchors(float vert_top, float vert_bottom_in)
{
    s_vert_top    = vert_top;
    s_vert_bottom = vert_bottom_in;
}

void cursor_track_stop(void)
{
    s_roll_valid = false;
    s_started    = false;
    /* Note: absolute-Y state (s_at_top, s_cur_y, s_slam_remaining, s_slam_sign)
     * is intentionally NOT reset here -- cursor_track_start resets it on the
     * next entry.  Do not add redundant resets here; the s_started gate prevents
     * any stale value from reaching cursor_track_update before start is called. */
}
