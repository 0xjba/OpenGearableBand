#include "gesture_mode.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>

LOG_MODULE_REGISTER(gesture_mode, LOG_LEVEL_INF);

/*
 * --- Tuning constants ----------------------------------------------------
 *
 * Picked deliberately for our acquisition rate (100 Hz) and the human
 * timing of wrist movements.  Defined here at the top so they can be
 * located and tweaked in one place; mostly NOT exposed via Kconfig
 * because gesture-mode tuning is an internal detail, not a build-time
 * option.
 */

/*
 * Gravity-vector low-pass filter alpha.
 *
 * Implemented as g[n] = (1 - alpha) * g[n-1] + alpha * a[n].  At
 * alpha = 0.01 and 100 Hz sample rate, the effective time constant
 * is ~1 s (1 / (alpha * fs) = 1 / 1 = 1 s).  That's slow enough to
 * filter out cardiac micro-motion and high-frequency jitter while
 * still tracking deliberate orientation changes within ~1 second.
 */
#define GRAVITY_LP_ALPHA            0.01f

/*
 * Dominant-axis dwell required before declaring a new orientation.
 * Prevents flicker when the wrist is in a borderline position --
 * orientation only switches if the new classification holds for at
 * least DWELL_SAMPLES consecutive samples (~250 ms at 100 Hz).
 */
#define ORIENTATION_DWELL_SAMPLES   25

/*
 * The dominant axis must be at least this many times larger than the
 * next-largest axis to count as truly dominant.  1.3x ~ 30 % gap.
 *
 * Without this, a pose like (4.5, 4.5, 6.0) would be classified as
 * Z-dominant; with it, three-axis-balanced poses correctly resolve
 * to NEUTRAL.
 *
 * NOTE on the lack of an absolute magnitude threshold:
 *   Earlier versions also gated on the dominant axis's absolute
 *   magnitude.  The threshold value (5.0 m/s^2) was a guess at "how
 *   tilted does a pose have to be to count as intentional?" and
 *   dropping it in favour of just the ratio is more principled --
 *   the ratio answers the right question ("is one axis meaningfully
 *   larger than the others?") without picking an arbitrary number.
 *   Gravity always sums to ~9.81 m/s^2 in magnitude; the ratio
 *   handles balanced poses correctly regardless of where along the
 *   tilt-from-vertical spectrum the wrist is.
 */
#define DOMINANCE_RATIO             1.3f

/*
 * Cooldown window after exiting a cursor mode via orientation-drop.
 * During this window, returning to the same mode's pose auto-re-
 * engages without requiring another tap trigger.  Outside this
 * window, an explicit tap is required.
 *
 * Sized for the realistic case of a user actively using a cursor
 * mode who steps away briefly (rest a fatigued arm for AIR_MOUSE;
 * lift hand off the desk for SURFACE) then comes back.  Shoulder
 * fatigue recovery during sustained sub-shoulder-height pointing is
 * roughly 15-30 s; 2000 samples = 20 s at 100 Hz lands in the middle
 * of that range and works for both modes.
 *
 * Tradeoff: too short and a tired user has to re-tap to resume.
 * Too long and an unrelated movement (lifting hand to drink coffee,
 * etc.) accidentally re-engages.  The 500 ms re-engage dwell
 * (COOLDOWN_REENGAGE_DWELL) protects against casual movements
 * regardless of cooldown length, so we lean toward "longer" here.
 */
#define CURSOR_COOLDOWN_SAMPLES     2000

/*
 * Orientation dwell required during cooldown to re-engage.  Shorter
 * than the cold-start raise dwell because the user already signalled
 * intent recently.  50 samples = 500 ms.
 */
#define COOLDOWN_REENGAGE_DWELL     50

/*
 * Dwell required for orientation-drop exit from a cursor mode.  Pose
 * must NOT be the mode's expected one (UP_RAISED for AIR_MOUSE,
 * DOWN_FLAT for SURFACE) for this many consecutive samples before
 * the FSM transitions out.  50 samples = 500 ms tolerates brief
 * unintentional tilts without bouncing out of the mode.
 *
 * Same value for both modes -- the tolerance for "did the user
 * actually move out of pose" doesn't depend on which mode we're in.
 */
#define CURSOR_EXIT_DWELL           50

/*
 * Entry-grace window after a cursor mode is entered while NOT already
 * in the expected pose.  The user has this many samples to assume
 * the pose (raise into UP_RAISED for AIR_MOUSE, place wrist flat for
 * SURFACE); if they don't, the FSM bounces back to IDLE without ever
 * having armed exit detection.
 *
 * Sized to accommodate the natural human reaction time + orientation
 * dwell + comfortable margin:
 *   - Press trigger: ~0 ms
 *   - Initiate movement: 200-400 ms
 *   - Reach pose: 500-1000 ms
 *   - Orientation classifier dwell to register the new pose: 250 ms
 *   - Margin for slow / hesitant movement
 *
 * 500 samples = 5 s at 100 Hz.  Same window for both modes -- both
 * require comparable time for a user to deliberately assume the pose
 * after pressing the entry trigger.
 */
#define CURSOR_ENTRY_GRACE          500

/* (Previously: FLAT_DWELL_SAMPLES = 100 -- auto-triggered SURFACE
 * mode from DOWN_FLAT pose dwell.  Removed: user feedback established
 * SURFACE entry is explicit via triple-tap, matching the AIR_MOUSE
 * double-tap entry pattern, so no orientation auto-trigger.) */


/*
 * Wrist-flick cancel trigger: gyro magnitude burst followed by
 * reverse-direction motion within a short window.  FLICK_BURST_THRESH
 * is the peak gyro magnitude (rad/s) for the burst; FLICK_WINDOW is
 * the number of samples within which the reversal must complete.
 */
#define FLICK_BURST_THRESH_RPS      8.0f
#define FLICK_WINDOW_SAMPLES        25

/*
 * --- Internal state ----------------------------------------------------
 */

/* Atomically published mode; read by other threads. */
static atomic_t mode_atomic = ATOMIC_INIT(MODE_IDLE);

/* Filtered gravity vector (low-pass of accel). */
static float gx_filt = 0.0f;
static float gy_filt = 0.0f;
static float gz_filt = -9.81f;   /* assume face-up at boot */

/* Filter initialised flag (so first sample seeds the state instead
 * of being heavily attenuated by the LP filter). */
static bool filter_initialised = false;

/* Current orientation + dwell tracker. */
static WristOrientation orientation_current = WRIST_NEUTRAL;
static WristOrientation orientation_candidate = WRIST_NEUTRAL;
static int orientation_candidate_dwell = 0;

/* Trigger gesture dwell counters. */
/* Re-engage-dwell counter used during the cursor-mode cooldown to
 * detect the user holding the expected pose long enough to re-engage
 * the previously-active cursor mode.  Counts up while orientation
 * matches the cooldown's mode's expected pose. */
static int reengage_dwell = 0;

/* Counts non-expected-pose samples while in a cursor mode.  When it
 * hits CURSOR_EXIT_DWELL, FSM exits to IDLE + starts cooldown. */
static int cursor_exit_dwell = 0;

/* Cooldown after a cursor mode exits via orientation-drop.  During
 * the cooldown window, the user can return to the same mode by
 * holding the expected pose for COOLDOWN_REENGAGE_DWELL samples --
 * no fresh trigger required.  Outside the cooldown, an explicit
 * double-tap (AIR_MOUSE) / triple-tap (SURFACE) is needed.
 *
 * cooldown_remaining: samples remaining; decremented every accel
 *                     sample.  0 = window closed.
 * cooldown_mode:      which mode to re-engage when the re-engage
 *                     dwell completes.  Tracks the last cursor
 *                     mode that exited via orientation drop. */
static int cursor_cooldown_remaining = 0;
static GestureMode cursor_cooldown_mode = MODE_IDLE;

/* "Has the user actually reached the expected pose since entering
 * the current cursor mode?"  Reset on every transition INTO a cursor
 * mode; set the first time orientation matches the mode's expected
 * pose after that.
 *
 * Purpose: the natural sequence is "press trigger -> assume pose,"
 * not "be in pose -> press trigger."  Without this latch the exit
 * dwell would start counting immediately on entry while the user
 * is still in whatever pose they were in.  500 ms later the FSM
 * exits before the user has had a chance to reach the pose.
 *
 * With this latch, exit detection only arms after we've SEEN the
 * expected pose at least once.  If the user never reaches it
 * within the CURSOR_ENTRY_GRACE window (see entry_grace_remaining),
 * the FSM bounces back to IDLE without ever arming exit dwell --
 * prevents the band sitting in a cursor mode indefinitely if the
 * user pressed the entry trigger and got distracted. */
static bool cursor_has_reached_pose = false;

/* Counts down from CURSOR_ENTRY_GRACE on every accel sample while
 * we're in a cursor mode AND haven't yet reached the expected pose.
 * Hits zero -> the FSM bounces back to IDLE (no cooldown -- the user
 * never engaged, no point allowing quick re-engage). */
static int entry_grace_remaining = 0;

/* Acquisition-request callback registered by main.cpp. */
static gesture_acq_request_cb_t s_acq_request_cb = NULL;

/* Was the previous mode a "needs IMU continuously" mode?  Tracks the
 * acq-request edges so we only call the callback on transitions. */
static bool s_prev_needs_acq = false;

static inline bool _mode_needs_continuous_imu(GestureMode m)
{
    return (m == MODE_AIR_MOUSE) || (m == MODE_SURFACE);
}

/* Wrist-flick state. */
static int flick_burst_samples_remaining = 0;
static float flick_burst_sign = 0.0f;

/*
 * --- Internal helpers --------------------------------------------------
 */

static const char *_mode_str(GestureMode m)
{
    switch (m) {
    case MODE_IDLE:             return "IDLE";
    case MODE_SURFACE:          return "SURFACE";
    case MODE_AIR_MOUSE:        return "AIR_MOUSE";
    case MODE_GESTURE_AMBIENT:  return "GESTURE_AMBIENT";
    default:                    return "UNKNOWN";
    }
}

static const char *_orientation_str(WristOrientation o)
{
    switch (o) {
    case WRIST_NEUTRAL:         return "NEUTRAL";
    case WRIST_DOWN_FLAT:       return "DOWN_FLAT";
    case WRIST_UP_RAISED:       return "UP_RAISED";
    default:                    return "UNKNOWN";
    }
}

/*
 * Classify the current gravity vector into an orientation.  Returns
 * NEUTRAL if no axis is sufficiently dominant.
 *
 * Body-frame convention (verified against the existing IMU traces in
 * the HR logs): with the device worn palm-side-up on the wrist,
 * gravity registers approximately:
 *   - Palm-down on desk (DOWN_FLAT):    +X axis dominant
 *   - Forearm raised (UP_RAISED):       -Y axis dominant (varies by
 *                                       individual wrist twist)
 *
 * NOTE: these are first-cut classifications.  They will need
 * empirical adjustment once the band is on a wrist and the LOG_INF
 * lines below show what the actual filtered gravity vector looks
 * like in each pose.  See TODOs in the architecture doc.
 */
static WristOrientation _classify_orientation(float gx, float gy, float gz)
{
    float ax = fabsf(gx);
    float ay = fabsf(gy);
    float az = fabsf(gz);

    /* Sort magnitudes to find the largest and second-largest.  We
     * keep track of which axis is largest so we can check its sign
     * below.  Compact pattern: at most 3 comparisons. */
    float largest_mag, second_mag;
    int largest_axis;   /* 0=X, 1=Y, 2=Z */
    float largest_signed;

    if (ax >= ay && ax >= az) {
        largest_mag = ax; largest_axis = 0; largest_signed = gx;
        second_mag = (ay >= az) ? ay : az;
    } else if (ay >= az) {
        largest_mag = ay; largest_axis = 1; largest_signed = gy;
        second_mag = (ax >= az) ? ax : az;
    } else {
        largest_mag = az; largest_axis = 2; largest_signed = gz;
        second_mag = (ax >= ay) ? ax : ay;
    }

    /* NEUTRAL when no axis is clearly dominant over the others
     * (DOMINANCE_RATIO gate -- prevents 3-way-balanced poses from
     * being mis-classified to whichever is fractionally largest).
     * No absolute magnitude threshold by design -- see DOMINANCE_RATIO
     * comment for rationale. */
    if (second_mag > 0.0f && largest_mag < DOMINANCE_RATIO * second_mag) {
        return WRIST_NEUTRAL;
    }

    /* Classify based on which axis dominates + its sign.
     *
     * Empirical mapping from field calibration (this user, this
     * board mounting):
     *   - X+ dominant ~ 6.2 m/s^2  ==> AIR_MOUSE raised pose
     *     (captured during two test attempts at g=[6.18,-4.23,3.66]
     *     and g=[6.23,-3.98,3.24])
     *   - Z+ dominant ~ 8.3 m/s^2  ==> band flat / surface
     *     (captured during snapshot windows when band was sitting
     *     on desk: accel=(0.47,-5.04,8.27) etc.)
     *   - Y axis dominance: no current empirical mapping; was a
     *     placeholder guess for raised pose in earlier code, now
     *     stripped.  Will need re-test if the user adopts a third
     *     distinct gesture pose where Y dominates.
     *
     * If the user's board mounting changes (e.g. switching wrists
     * or rotating the band on the strap), these mappings will need
     * to be re-captured.  Procedure: hold the band in each intended
     * pose, run 'g' over serial, read raw_g and update the
     * conditions below to match. */
    switch (largest_axis) {
    case 0:  /* X-axis dominant */
        /* Positive X dominant -> wrist raised in AIR_MOUSE pose. */
        return (largest_signed > 0.0f) ? WRIST_UP_RAISED : WRIST_NEUTRAL;
    case 1:  /* Y-axis dominant */
        /* No mapping yet -- not enough field data.  Stay NEUTRAL so
         * the FSM doesn't fire on poses we don't recognize. */
        return WRIST_NEUTRAL;
    case 2:  /* Z-axis dominant */
        /* Positive Z dominant -> band flat (palm-down on surface
         * or armrest level).  Used as the disengage zone for
         * AIR_MOUSE exit and as the entry condition for SURFACE
         * mode. */
        return (largest_signed > 0.0f) ? WRIST_DOWN_FLAT : WRIST_NEUTRAL;
    default:
        return WRIST_NEUTRAL;
    }
}

static void _transition_to(GestureMode new_mode)
{
    GestureMode old = (GestureMode)atomic_get(&mode_atomic);
    if (old == new_mode) {
        return;
    }
    atomic_set(&mode_atomic, (atomic_val_t)new_mode);
    LOG_INF("Mode transition: %s -> %s",
            _mode_str(old), _mode_str(new_mode));
    /* Reset dwell counters on every transition so the next trigger
     * starts cleanly. */
    reengage_dwell = 0;
    cursor_exit_dwell = 0;

    /* Manage the cursor-mode entry-grace state machine.  Same logic
     * for AIR_MOUSE and SURFACE -- only the "expected pose" varies.
     *
     * Entering a cursor mode while already in the expected pose:
     *   - cursor_has_reached_pose = TRUE immediately
     *   - entry_grace_remaining = 0 (never counts down)
     *   - exit detection armed right away
     *
     * Entering from any other orientation:
     *   - cursor_has_reached_pose = FALSE; set when orientation matches
     *   - entry_grace_remaining loaded with CURSOR_ENTRY_GRACE;
     *     if it counts down to zero without the user reaching the pose,
     *     the FSM bounces back to IDLE
     *
     * Leaving a cursor mode (any cause):
     *   - cursor_has_reached_pose cleared
     *   - entry_grace_remaining cleared
     */
    if (_mode_needs_continuous_imu(new_mode)) {
        WristOrientation expected =
            (new_mode == MODE_AIR_MOUSE) ? WRIST_UP_RAISED
                                         : WRIST_DOWN_FLAT;
        cursor_has_reached_pose = (orientation_current == expected);
        const char *mode_str = _mode_str(new_mode);
        const char *pose_str =
            (new_mode == MODE_AIR_MOUSE) ? "raised wrist (palm-facing)"
                                         : "flat wrist (palm-down)";
        if (cursor_has_reached_pose) {
            entry_grace_remaining = 0;
            LOG_INF("%s entered while already in %s pose -- "
                    "exit detection armed immediately",
                    mode_str, pose_str);
        } else {
            entry_grace_remaining = CURSOR_ENTRY_GRACE;
            LOG_INF("%s entered -- assume %s pose within %d ms to "
                    "engage, or it auto-exits to IDLE",
                    mode_str, pose_str, CURSOR_ENTRY_GRACE * 10);
        }
    } else {
        cursor_has_reached_pose = false;
        entry_grace_remaining = 0;
    }

    /* Notify the acquisition-request callback on edges between
     * "needs continuous IMU" and "doesn't."  Only fires on real edges
     * so the callback doesn't get spammed on every transition. */
    bool now_needs = _mode_needs_continuous_imu(new_mode);
    if (s_acq_request_cb && now_needs != s_prev_needs_acq) {
        s_acq_request_cb(now_needs);
        s_prev_needs_acq = now_needs;
    }
}

/*
 * --- Public API --------------------------------------------------------
 */

void gesture_mode_init(void)
{
    atomic_set(&mode_atomic, (atomic_val_t)MODE_IDLE);
    gx_filt = 0.0f;
    gy_filt = 0.0f;
    gz_filt = -9.81f;
    filter_initialised = false;
    orientation_current = WRIST_NEUTRAL;
    orientation_candidate = WRIST_NEUTRAL;
    orientation_candidate_dwell = 0;
    reengage_dwell = 0;
    cursor_exit_dwell = 0;
    cursor_cooldown_remaining = 0;
    cursor_cooldown_mode = MODE_IDLE;
    cursor_has_reached_pose = false;
    entry_grace_remaining = 0;
    flick_burst_samples_remaining = 0;
    flick_burst_sign = 0.0f;
    LOG_INF("gesture_mode initialised: mode=IDLE orientation=NEUTRAL");
}

void gesture_mode_update_accel(float ax, float ay, float az)
{
    /* Seed the LP filter with the first sample so we don't have to
     * wait ~5 time constants for the filter to converge. */
    if (!filter_initialised) {
        gx_filt = ax;
        gy_filt = ay;
        gz_filt = az;
        filter_initialised = true;
        return;
    }

    /* Standard 1-pole IIR low-pass per axis. */
    gx_filt += GRAVITY_LP_ALPHA * (ax - gx_filt);
    gy_filt += GRAVITY_LP_ALPHA * (ay - gy_filt);
    gz_filt += GRAVITY_LP_ALPHA * (az - gz_filt);

    /* Reclassify orientation based on filtered gravity. */
    WristOrientation new_classification = _classify_orientation(
        gx_filt, gy_filt, gz_filt);

    /* Orientation dwell: only update orientation_current after the
     * classification has held for ORIENTATION_DWELL_SAMPLES. */
    if (new_classification == orientation_candidate) {
        if (orientation_candidate_dwell < ORIENTATION_DWELL_SAMPLES) {
            orientation_candidate_dwell++;
        }
    } else {
        orientation_candidate = new_classification;
        orientation_candidate_dwell = 1;
    }

    if (orientation_candidate_dwell >= ORIENTATION_DWELL_SAMPLES &&
        orientation_candidate != orientation_current) {
        WristOrientation old_o = orientation_current;
        orientation_current = orientation_candidate;
        LOG_INF("Orientation: %s -> %s (g=[%.2f, %.2f, %.2f])",
                _orientation_str(old_o),
                _orientation_str(orientation_current),
                (double)gx_filt, (double)gy_filt, (double)gz_filt);
    }

    /* --- Mode-transition logic --- */

    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);

    /* Decrement AIR_MOUSE cooldown each sample.  Once it hits 0, the
     * orientation-only re-engage path is closed and the user has to
     * deliberately double-tap to re-enter AIR_MOUSE. */
    if (cursor_cooldown_remaining > 0) {
        cursor_cooldown_remaining--;
    }

    /* Helper: which orientation does the given cursor mode expect? */
    auto expected_pose_for = [](GestureMode m) -> WristOrientation {
        return (m == MODE_AIR_MOUSE) ? WRIST_UP_RAISED : WRIST_DOWN_FLAT;
    };

    /* Cooldown re-engage: in IDLE with cooldown open, the user
     * holding the previously-active mode's expected pose for
     * COOLDOWN_REENGAGE_DWELL re-enters that mode without a fresh
     * trigger.  This covers "I was using AIR_MOUSE, briefly lowered
     * to rest, raising again" and the analogous SURFACE case. */
    if (current_mode == MODE_IDLE &&
        cursor_cooldown_remaining > 0 &&
        cursor_cooldown_mode != MODE_IDLE &&
        orientation_current == expected_pose_for(cursor_cooldown_mode)) {
        if (reengage_dwell < COOLDOWN_REENGAGE_DWELL) {
            reengage_dwell++;
            if (reengage_dwell == COOLDOWN_REENGAGE_DWELL) {
                LOG_INF("Cooldown re-engage (%s): %d ms remaining when fired",
                        _mode_str(cursor_cooldown_mode),
                        cursor_cooldown_remaining * 10);
                GestureMode target = cursor_cooldown_mode;
                cursor_cooldown_remaining = 0;
                _transition_to(target);
            }
        }
    } else {
        reengage_dwell = 0;
    }

    /* The remaining per-sample logic only matters while we're in a
     * cursor mode.  Compute "what pose does the current mode expect"
     * once and use it below. */
    bool in_cursor_mode = (current_mode == MODE_AIR_MOUSE ||
                           current_mode == MODE_SURFACE);
    WristOrientation expected = in_cursor_mode
        ? expected_pose_for(current_mode)
        : WRIST_NEUTRAL;

    /* Arm "has reached pose" latch the first time orientation matches
     * the expected pose for the current cursor mode.  Exit detection
     * below only runs when the latch is true -- the user has the
     * entry_grace_remaining window to assume the pose. */
    if (in_cursor_mode && !cursor_has_reached_pose &&
        orientation_current == expected) {
        cursor_has_reached_pose = true;
        entry_grace_remaining = 0;  /* engaged -- stop the timeout */
        LOG_INF("%s: expected pose reached -- exit detection armed",
                _mode_str(current_mode));
    }

    /* Entry-grace timeout: counts down while in a cursor mode and
     * the user hasn't reached the pose.  Hits zero -> bounce back to
     * IDLE so the band doesn't sit in a cursor mode indefinitely.
     * No cooldown started -- the user never actually engaged. */
    if (in_cursor_mode && !cursor_has_reached_pose &&
        entry_grace_remaining > 0) {
        entry_grace_remaining--;
        if (entry_grace_remaining == 0) {
            LOG_INF("%s entry grace expired -- user never assumed "
                    "pose, exiting to IDLE", _mode_str(current_mode));
            _transition_to(MODE_IDLE);
            /* No cooldown -- explicit re-tap required */
        }
    }

    /* Exit via orientation drop: pose is no longer the expected one
     * for CURSOR_EXIT_DWELL samples -> exit to IDLE and start the
     * cooldown.  Only runs after the user has assumed the pose at
     * least once. */
    if (in_cursor_mode && cursor_has_reached_pose &&
        orientation_current != expected) {
        if (cursor_exit_dwell < CURSOR_EXIT_DWELL) {
            cursor_exit_dwell++;
            if (cursor_exit_dwell == CURSOR_EXIT_DWELL) {
                LOG_INF("%s exit: orientation dropped, starting %d ms "
                        "re-engage cooldown",
                        _mode_str(current_mode),
                        CURSOR_COOLDOWN_SAMPLES * 10);
                cursor_cooldown_mode = current_mode;
                cursor_cooldown_remaining = CURSOR_COOLDOWN_SAMPLES;
                _transition_to(MODE_IDLE);
            }
        }
    } else {
        cursor_exit_dwell = 0;
    }
}

void gesture_mode_update_gyro(float gx_rps, float gy_rps, float gz_rps)
{
    /* Flick detection: look for a sharp burst followed by a reversal.
     * We pick the largest-magnitude axis for the burst direction.
     * Once a burst is detected, we count down FLICK_WINDOW_SAMPLES
     * looking for a sign-reversed burst.  If we get one inside the
     * window, that's a flick.
     *
     * For now we only act on flicks from a cursor mode -> IDLE
     * (the cancel direction).  Future extensions could add directional
     * flicks for app-side commands. */

    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);

    /* Find largest-magnitude axis. */
    float ax = fabsf(gx_rps);
    float ay = fabsf(gy_rps);
    float az = fabsf(gz_rps);
    float largest_mag;
    float largest_val;
    if (ax >= ay && ax >= az) { largest_mag = ax; largest_val = gx_rps; }
    else if (ay >= az)        { largest_mag = ay; largest_val = gy_rps; }
    else                      { largest_mag = az; largest_val = gz_rps; }

    if (flick_burst_samples_remaining > 0) {
        flick_burst_samples_remaining--;
        /* Look for a reversal of the original burst direction. */
        if (largest_mag >= FLICK_BURST_THRESH_RPS &&
            ((largest_val > 0.0f) != (flick_burst_sign > 0.0f))) {
            if (current_mode == MODE_AIR_MOUSE ||
                current_mode == MODE_SURFACE) {
                LOG_INF("Wrist flick detected -- cancelling mode");
                _transition_to(MODE_IDLE);
            }
            flick_burst_samples_remaining = 0;
        }
    } else if (largest_mag >= FLICK_BURST_THRESH_RPS) {
        /* New burst -- arm the reversal window. */
        flick_burst_samples_remaining = FLICK_WINDOW_SAMPLES;
        flick_burst_sign = largest_val;
    }
}

void gesture_mode_on_chip_double_tap(void)
{
    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);
    /* Double-tap is the AIR_MOUSE ENTRY trigger ONLY.  Per design
     * discussion: exit is by lowering the wrist, not by re-tapping.
     * Double-tap while already in a non-IDLE mode is a no-op (logged
     * for visibility so accidental taps during cursor use show up in
     * the trace).
     *
     * Why entry-only:
     *   - The user defined the model: tap to engage, lower to
     *     disengage, raise within cooldown to re-engage
     *   - Toggle-on-tap would let an accidental tap mid-cursor-use
     *     yank the user out of AIR_MOUSE mid-action
     *   - There's no benefit to having a redundant exit path that
     *     bypasses the natural orientation-based one
     */
    if (current_mode == MODE_IDLE) {
        LOG_INF("Chip double-tap from IDLE -- entering AIR_MOUSE");
        cursor_cooldown_remaining = 0;  /* explicit entry skips cooldown */
        cursor_cooldown_mode = MODE_IDLE;
        _transition_to(MODE_AIR_MOUSE);
    } else {
        LOG_INF("Chip double-tap from %s -- ignored (double-tap is "
                "entry-only; exit by lowering the wrist)",
                _mode_str(current_mode));
    }
}

void gesture_mode_on_chip_triple_tap(void)
{
    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);
    /* Triple-tap is the SURFACE ENTRY trigger ONLY.  Symmetric to
     * double-tap: explicit, entry-only, exit by orientation.  Triple-
     * tap while in any non-IDLE mode is a no-op (logged). */
    if (current_mode == MODE_IDLE) {
        LOG_INF("Chip triple-tap from IDLE -- entering SURFACE");
        cursor_cooldown_remaining = 0;  /* explicit entry skips cooldown */
        cursor_cooldown_mode = MODE_IDLE;
        _transition_to(MODE_SURFACE);
    } else {
        LOG_INF("Chip triple-tap from %s -- ignored (triple-tap is "
                "entry-only; exit by lifting the wrist off the surface)",
                _mode_str(current_mode));
    }
}

GestureMode gesture_mode_get(void)
{
    return (GestureMode)atomic_get(&mode_atomic);
}

void gesture_mode_set(GestureMode mode)
{
    _transition_to(mode);
}

WristOrientation gesture_mode_get_orientation(void)
{
    return orientation_current;
}

const char *gesture_mode_str(GestureMode mode)
{
    return _mode_str(mode);
}

const char *wrist_orientation_str(WristOrientation o)
{
    return _orientation_str(o);
}

void gesture_mode_get_gravity(float *out_gx, float *out_gy, float *out_gz)
{
    if (out_gx) *out_gx = gx_filt;
    if (out_gy) *out_gy = gy_filt;
    if (out_gz) *out_gz = gz_filt;
}

int gesture_mode_get_cursor_cooldown_remaining(void)
{
    return cursor_cooldown_remaining;
}

void gesture_mode_set_acq_request_cb(gesture_acq_request_cb_t cb)
{
    s_acq_request_cb = cb;
}
