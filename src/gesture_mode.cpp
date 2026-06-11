#include "gesture_mode.h"
#include "gesture_poses.h"
#include "orientation.h"
#include "gesture_thresholds.h"
#include "cursor_track.h"
#include "cursor_pipeline.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <limits.h>

#include <arm_math.h>
#include <arm_const_structs.h>

#include "power_ctrl.h"

LOG_MODULE_REGISTER(gesture_mode, LOG_LEVEL_INF);

/* Tuning constants are centralized in gesture_thresholds.h (included above).
 * See that header for rationale, tags, and the productionization catalog. */

/* (Previously: FLAT_DWELL_SAMPLES = 100 -- auto-triggered SURFACE
 * mode from DOWN_FLAT pose dwell.  Removed: user feedback established
 * SURFACE entry is explicit via triple-tap, matching the AIR_MOUSE
 * double-tap entry pattern, so no orientation auto-trigger.) */


/*
 * --- Internal state ----------------------------------------------------
 */

/* Atomically published mode; read by other threads. */
static atomic_t mode_atomic = ATOMIC_INIT(MODE_IDLE);

/* Filtered gravity vector (low-pass of accel). */
static float gx_filt = 0.0f;
static float gy_filt = 0.0f;
static float gz_filt = -9.81f;   /* assume face-up at boot */

/* Angle-from-vertical (deg) from the GRAVITY-LPF: the cursor Y driver.
 * Roll-immune (a scalar projection of gravity onto the forearm axis), unlike
 * Euler pitch which saturates at high roll -- see cursor_track.h. */
static inline float current_vert_deg(void)
{
    float mag = sqrtf(gx_filt * gx_filt + gy_filt * gy_filt + gz_filt * gz_filt);
    return (mag > 0.1f)
        ? acosf(fminf(1.0f, fabsf(gx_filt) / mag)) * (180.0f / 3.14159265f)
        : 0.0f;
}

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
 * exceeds the per-mode exit dwell (AIR_MOUSE_EXIT_DWELL or
 * SURFACE_EXIT_DWELL via _exit_dwell_for()), the FSM exits to IDLE +
 * starts cooldown. */
static int cursor_exit_dwell = 0;

/* Motion-burst exit detector (SURFACE only).  Counts consecutive
 * samples with motion-residual magnitude above
 * SURFACE_MOTION_BURST_THRESH.  Reaches SURFACE_MOTION_BURST_DWELL
 * -> SURFACE exit + cooldown.  Used to catch wrist-transport
 * gestures (e.g., desk -> lap) that don't change palm orientation. */
static int surface_motion_burst_dwell = 0;

/* Peak motion-residual seen since the last periodic stats log
 * (reset every SURFACE_MOTION_LOG_PERIOD samples).  Used both for
 * the periodic calibration stream and as the value reported in the
 * exit log.  Lets the user read off actual peaks during gliding
 * (should be small) vs transport (should be large) to tune the
 * threshold empirically without further code changes. */
static float surface_motion_peak = 0.0f;
static int   surface_motion_log_counter = 0;
#define      SURFACE_MOTION_LOG_PERIOD   200    /* 2 s at 100 Hz */

/* Pose FSM state.  Updated each accel sample from update_accel. */
static pose_id_t pose_armed_state = POSE_NONE;
static int64_t   pose_armed_time_ms = 0;

/* Multi-tap counter state.  See MULTI_TAP_WINDOW_MS in gesture_thresholds.h. */
static int     multi_tap_count = 0;
static int64_t multi_tap_last_time_ms = 0;
/* Diagnostic: last tap's axis/sign characters, included in the
 * commit log so we can spot patterns (e.g., all-Z = direct band
 * hit; mixed-axis = snap or surface-transmitted). */
static char    multi_tap_first_axis = '?';
static char    multi_tap_first_sign = '?';

/* k_work_delayable that fires MULTI_TAP_WINDOW_MS after the last
 * tap arrival to commit the accumulated sequence.  Rescheduled on
 * each tap (resets the timer), so the work only fires once the
 * sequence has gone quiet.  Independent of the acq pipeline --
 * fires regardless of whether gesture_mode_update_accel is being
 * called, which is what the per-sample check used to depend on
 * (and the bug 2026-06-08 that this work item fixes). */
static void multi_tap_commit_handler(struct k_work *work_arg);
static K_WORK_DELAYABLE_DEFINE(multi_tap_commit_work,
                               multi_tap_commit_handler);

static int64_t last_chip_tap_time_ms = 0;

/* Time of the chip-tap event before the current one.  Used by
 * last_two_tap_interval_ms() to compute the inter-tap interval for
 * cadenced-double-tap detection.  Updated by
 * gesture_mode_on_chip_single_tap() after the new tap is accepted.
 * Zero means there isn't a "previous" tap to compare against. */
static int64_t prev_chip_tap_time_ms = 0;

/* Inter-tap interval (ms) between the most-recent and second-most-
 * recent chip-tap events.  Returns 0 if there is no recent previous
 * tap to compare against (cold start). */
static int last_two_tap_interval_ms(void)
{
    if (prev_chip_tap_time_ms == 0) return 0;
    /* last_chip_tap_time_ms is the existing module-state set after
     * the most-recent accepted chip-tap.  prev_chip_tap_time_ms is
     * the value last_chip_tap_time_ms had BEFORE the most-recent
     * update -- so (last - prev) is the gap between the two most
     * recent chip-tap events. */
    int64_t delta = last_chip_tap_time_ms - prev_chip_tap_time_ms;
    if (delta < 0 || delta > INT_MAX) return 0;
    return (int)delta;
}

/* True if the given inter-tap interval falls within the cadenced
 * double-tap window. */
static bool is_cadenced_double_tap_window(int interval_ms)
{
    return (interval_ms >= CADENCE_MIN_MS) &&
           (interval_ms <= CADENCE_MAX_MS);
}

/* Activity gate state.  See ACTIVITY_GATE_* comment block.
 * samples_since_activity counts up per accel sample (saturated at
 * ACTIVITY_GATE_DWELL since the precise high value doesn't matter
 * once the gate is open), and resets to 0 when motion residual
 * exceeds ACTIVITY_GATE_THRESH.  Initialised at the saturation
 * value so the gate is open at boot. */
static int     samples_since_activity = ACTIVITY_GATE_DWELL;

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
 * within the per-mode entry-grace window (see entry_grace_remaining),
 * the FSM bounces back to IDLE without ever arming exit dwell --
 * prevents the band sitting in a cursor mode indefinitely if the
 * user pressed the entry trigger and got distracted. */
static bool cursor_has_reached_pose = false;

/* Counts down from the per-mode entry grace on every accel sample
 * while we're in a cursor mode AND haven't yet reached the expected
 * pose.  Hits zero -> the FSM bounces back to IDLE (no cooldown --
 * the user never engaged, no point allowing quick re-engage). */
static int entry_grace_remaining = 0;

/* Reference gravity vector captured at the moment a cursor mode
 * first engages (cursor_has_reached_pose flips false -> true).
 *
 * For SURFACE this is the "desk plane reference" -- the gravity
 * orientation at the instant the wrist was placed on the desk.
 * Item 2 (cursor tracking) will compute cursor motion as deviations
 * from this reference, so gliding the wrist around at desk level
 * produces relative motion correctly even though every position is
 * roughly DOWN_FLAT to the orientation classifier.  Lifting the
 * wrist will then read as "gravity left the reference plane,"
 * giving us the mouse-lift semantic precisely.
 *
 * For AIR_MOUSE the same idea applies but for the raised pose --
 * tracking is relative to the at-engagement orientation rather than
 * absolute angles. */

/* Acquisition-request callback registered by main.cpp. */
static gesture_acq_request_cb_t s_acq_request_cb = NULL;

/* Most recent tap's mid_band_energy, updated by bio_acoustic_worker
 * after each FFT.  Read by surface_spectral_confirms_hard_surface()
 * to gate SURFACE mode entry.  Declared here (before
 * gesture_mode_init) so the init reset compiles without a forward
 * reference; the companion constant and predicate live further down
 * near the bio-acoustic section. */
static float last_tap_mid_band_energy = 0.0f;

/* Forward declaration -- implementation lives near the bio-acoustic
 * section later in the file. */
static bool surface_spectral_confirms_hard_surface(void);

/* Was the previous mode a "needs IMU continuously" mode?  Tracks the
 * acq-request edges so we only call the callback on transitions. */
static bool s_prev_needs_acq = false;

static inline bool _mode_needs_continuous_imu(GestureMode m)
{
    return (m == MODE_AIR_MOUSE) || (m == MODE_SURFACE);
}

/* Per-mode timing accessors -- centralise the mode-vs-constant
 * lookups so the FSM body doesn't sprinkle conditionals throughout. */
static inline int _entry_grace_for(GestureMode m)
{
    return (m == MODE_AIR_MOUSE) ? AIR_MOUSE_ENTRY_GRACE
                                 : SURFACE_ENTRY_GRACE;
}
static inline int _exit_dwell_for(GestureMode m)
{
    return (m == MODE_AIR_MOUSE) ? AIR_MOUSE_EXIT_DWELL
                                 : SURFACE_EXIT_DWELL;
}

/* Wrist-flick state. */
static int flick_burst_samples_remaining = 0;
static float flick_burst_sign = 0.0f;

/* Gyro rotation-signature buffer (dictation-flip prototype, 2026-06-10).
 * Rolling ~1.5 s window of gyro samples in rad/s.  At pose-arm we
 * integrate it to report the net rotation about each band axis during
 * the move-into-pose -- the supination flip (roll about band X, the
 * forearm axis) for dictation should show a large net X that an
 * air-mouse lean does not.  150 samples * 3 * 4 B = 1800 B. */
#define GYRO_HIST_SAMPLES   150          /* 1.5 s at 100 Hz */
#define GYRO_DT_S           0.01f        /* 100 Hz sample period */
#define RAD_TO_DEG          57.29578f
static float gyro_hist[GYRO_HIST_SAMPLES][3];
static int   gyro_hist_idx = 0;

/* Latest raw accel (m/s^2), stashed by gesture_mode_update_accel so the
 * orientation filter can fuse it with the gyro sample that arrives in
 * the immediately-following gesture_mode_update_gyro() call. */
static float last_raw_ax = 0.0f, last_raw_ay = 0.0f, last_raw_az = 9.81f;

/* Integrate the gyro buffer -> net rotation (deg) and peak rate (dps)
 * about each band axis over the last ~1.5 s. */
static void gyro_signature(float *net_deg, float *peak_dps)
{
    float sum[3] = {0, 0, 0};
    float peak[3] = {0, 0, 0};
    for (int i = 0; i < GYRO_HIST_SAMPLES; i++) {
        for (int a = 0; a < 3; a++) {
            float w = gyro_hist[i][a];
            sum[a] += w;
            float m = fabsf(w);
            if (m > peak[a]) peak[a] = m;
        }
    }
    for (int a = 0; a < 3; a++) {
        net_deg[a]  = sum[a] * GYRO_DT_S * RAD_TO_DEG;
        peak_dps[a] = peak[a] * RAD_TO_DEG;
    }
}

/*
 * --- Internal helpers --------------------------------------------------
 */

/* Internal: check pose-arm timeout and disarm if expired.  Shared
 * by gesture_mode_armed_pose() (the public query) and
 * pose_fsm_update() (the per-sample updater).  Logs the expiry
 * event so the cause of a silent drop is visible. */
static void pose_check_timeout(void)
{
    if (pose_armed_state == POSE_NONE) return;
    int64_t now = k_uptime_get();
    int64_t elapsed = now - pose_armed_time_ms;
    if (elapsed > POSE_ARM_WINDOW_MS) {
        LOG_INF("Pose arm expired (%s, %lld ms elapsed) -> NONE",
                pose_name(pose_armed_state), (long long)elapsed);
        pose_armed_state = POSE_NONE;
    }
}

pose_id_t gesture_mode_armed_pose(void)
{
    pose_check_timeout();
    return pose_armed_state;
}

bool gesture_mode_recent_activity(void)
{
    /* Mid-gesture if a pose is armed... */
    if (gesture_mode_armed_pose() != POSE_NONE) {
        return true;
    }
    /* ...or a chip-tap landed within the guard window. */
    if (last_chip_tap_time_ms != 0 &&
        (k_uptime_get() - last_chip_tap_time_ms) < RECENT_GESTURE_GUARD_MS) {
        return true;
    }
    return false;
}

/* Update pose FSM based on current gravity vector.  Called every
 * accel sample from gesture_mode_update_accel().
 *
 * Pose arming is purely "currently matching a canonical pose":
 *   - If currently armed AND still matching the same pose: refresh
 *     the arm timestamp so the window stays open as long as the
 *     user holds the pose.
 *   - If not armed AND a canonical pose matches: arm.
 *
 * The deliberate-intent filter is the gesture (single tap or
 * cadenced double-tap) added in later tasks -- the pose alone just
 * selects which mode is being entered.  Requiring motion-into-pose
 * was over-engineering: it broke the natural case of "user already
 * has wrist on desk and wants to enter SURFACE." */
static void pose_fsm_update(float gx, float gy, float gz)
{
    /* Flush expired arm first. */
    pose_check_timeout();

    /* Classify current gravity. */
    float score = 0.0f;
    pose_id_t best = pose_classify_best(gx, gy, gz,
                                          POSE_MATCH_THRESH, &score);

    /* NOTE: do NOT re-add a roll/gz-based AIR_MOUSE<->DICTATION split here.
     * A held max-right air-mouse is gravity-identical to dictation (measured
     * 2026-06-11); pose-only discrimination is impossible.  The confirming
     * GESTURE decides the mode (see 2026-06-11-pose-trigger-realignment spec
     * + decision_dictation_voice_gated_entry).  The raised hemisphere arms
     * as POSE_AIR_MOUSE; dictation entry is future (clench + voice).
     * Pose logic operates on the gravity-LPF components (gx/gy/gz) only --
     * NEVER orientation_get().roll_deg (the quaternion Euler roll lags ~10
     * deg in motion; it is for the cursor/logging, not pose decisions). */

    if (pose_armed_state != POSE_NONE) {
        /* Already armed.  If the user is still in the SAME pose,
         * refresh the arm timestamp.  Otherwise leave the existing
         * arm alone (don't break user intent on a transient
         * mis-classification or wrist wobble). */
        if (best == pose_armed_state) {
            pose_armed_time_ms = k_uptime_get();
        }
        return;
    }

    /* Not armed.  Pose match is sufficient to arm. */
    if (best == POSE_NONE) {
        return;
    }

    pose_armed_state = best;
    pose_armed_time_ms = k_uptime_get();
    LOG_INF("Pose ARMED: %s (score=%.2f, gravity=(%.2f, %.2f, %.2f))",
            pose_name(best), (double)score,
            (double)gx, (double)gy, (double)gz);

    /* Dictation-flip prototype: dump the gyro rotation signature for
     * the move-into-pose.  net[X] is the net roll about the forearm
     * axis (supination) -- expected LARGE for a dictation flip, SMALL
     * for an air-mouse raise/lean.  Collect raises vs flips and compare
     * net_X / peak_X to design the discriminator. */
    float net_deg[3], peak_dps[3];
    gyro_signature(net_deg, peak_dps);
    LOG_INF("  GYRO-SIG net_deg[X=%.0f Y=%.0f Z=%.0f] peak_dps[X=%.0f Y=%.0f Z=%.0f]",
            (double)net_deg[0], (double)net_deg[1], (double)net_deg[2],
            (double)peak_dps[0], (double)peak_dps[1], (double)peak_dps[2]);

    /* Orientation-filter estimate at arm.  YAW (relative to the last
     * rest re-zero) is the path-independent dictation-vs-air-mouse
     * discriminator we're testing: a palm-to-face flip should land at a
     * very different yaw than a palm-to-screen raise, regardless of the
     * path taken to get there.  pitch/roll are gravity-locked. */
    orientation_state_t ori;
    orientation_get(&ori);
    LOG_INF("  ORI pitch=%.0f roll=%.0f yaw=%.0f at_rest=%d bias_dps[%.1f %.1f %.1f]",
            (double)ori.pitch_deg, (double)ori.roll_deg, (double)ori.yaw_deg,
            (int)ori.at_rest,
            (double)ori.gyro_bias_dps[0], (double)ori.gyro_bias_dps[1],
            (double)ori.gyro_bias_dps[2]);
}

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

static WristOrientation _classify_orientation(float gx, float gy, float gz)
{
    /* Unified gravity-component geometry (docs/research/observability-aware-
     * pose-and-cursor-design.md section 3.5): gx = forearm elevation, gy =
     * left-right SWEEP axis, gz = volar-normal.  RAISED ignores gy so a wide
     * air-mouse sweep (gy large at the extremes) stays UP_RAISED instead of
     * flapping to NEUTRAL.  (gy is unbounded in the RAISED test on purpose:
     * a large gy with gx > |gz| is physically unreachable in the volar-radial
     * wrist mount -- revisit if the mount orientation changes.)  Thresholds
     * are empirical (regression test: the 2026-06-11 left-right sweep must
     * produce 0 UP_RAISED<->NEUTRAL transitions). */

    /* RAISED: forearm up and clearly above flat (gx dominates |gz|). */
    if (gx > 0.0f && gx > RAISED_ELEVATION_RATIO * fabsf(gz)) {
        return WRIST_UP_RAISED;
    }

    /* FLAT: volar-normal up and dominant over BOTH other axes. */
    if (gz > 0.0f &&
        gz > DOMINANCE_RATIO * fabsf(gx) &&
        gz > DOMINANCE_RATIO * fabsf(gy)) {
        return WRIST_DOWN_FLAT;
    }

    return WRIST_NEUTRAL;
}

/* Forward decl -- defined immediately after _transition_to.  Called
 * from inside _transition_to to notify acq edges. */
static void _update_acq_request(void);

/* Flag set by the cooldown re-engage path before calling
 * _transition_to(), so the "entered while already in pose" log can
 * be suppressed -- the re-engage path already logged "Cooldown
 * re-engage (MODE): N ms remaining when fired" which provides full
 * context.  Without this, cooldown re-engages produce a misleading
 * "entered while already in pose" line that reads like a fresh cold
 * entry.  Cleared inside _transition_to after the check. */
static bool s_transition_via_cooldown_reengage = false;

static void _transition_to(GestureMode new_mode)
{
    GestureMode old = (GestureMode)atomic_get(&mode_atomic);
    if (old == new_mode) {
        return;
    }
    atomic_set(&mode_atomic, (atomic_val_t)new_mode);
    /* Air-mouse cursor: start tracking on entry (capture the reference
     * angles so there's no jump), stop on any transition away. */
    if (new_mode == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        cursor_track_start(current_vert_deg(), ori.roll_deg);
    } else {
        cursor_track_stop();
    }
    LOG_INF("Mode transition: %s -> %s",
            _mode_str(old), _mode_str(new_mode));
    /* Reset dwell counters on every transition so the next trigger
     * starts cleanly. */
    reengage_dwell = 0;
    cursor_exit_dwell = 0;
    surface_motion_burst_dwell = 0;
    surface_motion_peak = 0.0f;
    surface_motion_log_counter = 0;
    /* Multi-tap counter and activity gate persist across transitions
     * by design: a tap arriving DURING the disambiguation window
     * that triggers a mode change (e.g., AIR_MOUSE entry on tap 2)
     * should still be folded if a 3rd tap arrives quickly, upgrading
     * to triple-tap.  We rely on _check_tap_timeout() in the per-sample
     * update to commit and reset cleanly.  No reset here. */

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
     *   - entry_grace_remaining loaded from _entry_grace_for(new_mode);
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
            /* During a cooldown re-engage the user has just returned
             * to the expected pose, so cursor_has_reached_pose is
             * naturally true.  The re-engage path already logged
             * "Cooldown re-engage" with context, so suppress the
             * otherwise-misleading "already in pose" line.  Only log
             * for fresh cold entries (e.g. user double-taps the band
             * while their wrist happens to already be raised). */
            if (!s_transition_via_cooldown_reengage) {
                LOG_INF("%s entered while already in %s pose -- "
                        "exit detection armed immediately",
                        mode_str, pose_str);
            }
        } else {
            entry_grace_remaining = _entry_grace_for(new_mode);
            LOG_INF("%s entered -- assume %s pose within %d ms to "
                    "engage, or it auto-exits to IDLE",
                    mode_str, pose_str, entry_grace_remaining * 10);
        }
    } else {
        cursor_has_reached_pose = false;
        entry_grace_remaining = 0;
    }

    /* Clear the cooldown-reengage flag unconditionally so it never
     * leaks across transitions (the entry-grace -> IDLE path skips
     * the branches above that would otherwise consume it). */
    s_transition_via_cooldown_reengage = false;

    /* Re-evaluate the acq-request edge.  Acq must stay alive while a
     * cursor mode is active OR a cooldown is open -- without samples
     * during cooldown the decrement freezes and the re-engage check
     * goes blind (the 2026-06-07 bug).  See _update_acq_request(). */
    _update_acq_request();
}

/* Evaluate whether the acquisition pipeline needs to be alive RIGHT
 * NOW and notify the registered callback on edges.
 *
 * ALWAYS-ON policy (2026-06-10 fix): the acq pipeline must stay alive
 * continuously so the pose FSM receives accel samples in MODE_IDLE
 * (the always-listening trigger state).  Previously this was gated on
 * _mode_needs_continuous_imu(m) || cursor_cooldown_remaining > 0,
 * which meant pose detection only ran during AIR_MOUSE/SURFACE/
 * SNAPSHOT states -- broken for the trigger-detection use case since
 * the user is in MODE_IDLE when they want to trigger.
 *
 * Power cost: ~30-50 µA additional vs gated-acq IDLE.  The IMU is
 * already running at 833 Hz continuously (for the chip tap engine);
 * this only adds the MCU's per-sample read + processing at 100 Hz.
 * Acceptable for an always-listening gesture device.
 *
 * Reference m is preserved (via (void)m) for future per-state
 * optimisation (e.g., lower acq rate in IDLE specifically).
 *
 * Called from _transition_to (mode changes) and from the cooldown
 * decrement site (cooldown crossing zero).  Both are real edges of
 * the "needs alive" predicate; both must notify. */
static void _update_acq_request(void)
{
    if (!s_acq_request_cb) {
        return;
    }
    /* Acq pipeline must stay alive continuously so the pose FSM
     * receives accel samples in IDLE (the always-listening trigger
     * state).  Previously this was gated on
     * _mode_needs_continuous_imu() || cursor_cooldown_remaining > 0,
     * which meant pose detection only ran during AIR_MOUSE/SURFACE/
     * SNAPSHOT states -- broken for the trigger detection use case
     * since the user is in MODE_IDLE when they want to trigger.
     *
     * Power cost: ~30-50 µA additional vs gated-acq IDLE.  The
     * IMU is already running at 833 Hz continuously (for the chip
     * tap engine); this only adds the MCU's per-sample read+
     * processing at 100 Hz.  Acceptable for an always-listening
     * gesture device.
     *
     * Reference m for future per-state optimisation if needed
     * (e.g., lower acq rate in IDLE specifically). */
    GestureMode m = (GestureMode)atomic_get(&mode_atomic);
    (void)m;  /* not currently used for gating; preserved for future */
    bool now_needs = true;
    if (now_needs != s_prev_needs_acq) {
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
    surface_motion_burst_dwell = 0;
    surface_motion_peak = 0.0f;
    surface_motion_log_counter = 0;
    pose_armed_state = POSE_NONE;
    pose_armed_time_ms = 0;
    multi_tap_count = 0;
    multi_tap_last_time_ms = 0;
    multi_tap_first_axis = '?';
    multi_tap_first_sign = '?';
    last_chip_tap_time_ms = 0;
    prev_chip_tap_time_ms = 0;
    samples_since_activity = ACTIVITY_GATE_DWELL;
    flick_burst_samples_remaining = 0;
    flick_burst_sign = 0.0f;
    for (int i = 0; i < GYRO_HIST_SAMPLES; i++) {
        gyro_hist[i][0] = 0.0f;
        gyro_hist[i][1] = 0.0f;
        gyro_hist[i][2] = 0.0f;
    }
    gyro_hist_idx = 0;
    last_raw_ax = 0.0f; last_raw_ay = 0.0f; last_raw_az = 9.81f;
    orientation_init();
    last_tap_mid_band_energy = 0.0f;
    LOG_INF("gesture_mode initialised: mode=IDLE orientation=NEUTRAL");
}

/* --- Pose-observability trace -------------------------------------------
 * Periodic logger to MEASURE the roll-observability cone on the CURRENT
 * mount (re-taped per session, so assume nothing from prior days).  Hold a
 * pose still; logs gravity, the Y-Z "shadow" magnitude sqrt(gy^2+gz^2)
 * (small => forearm near vertical => roll unobservable), the forearm angle
 * from vertical, and the filter's pitch/roll.  Used to derive the cone
 * threshold + the bring-to-face dictation signature empirically. */
static uint32_t pose_trace_remaining = 0;
static uint32_t pose_trace_div = 0;

void gesture_mode_pose_trace_start(uint32_t n_samples)
{
    pose_trace_remaining = n_samples;
    pose_trace_div = 0;
    LOG_INF("POSE-TRACE: hold the pose STILL; logging ~%u s "
            "(g, shadow, vert=deg-from-vertical, pitch, roll)...",
            n_samples / 100u);
}

static void pose_trace_tick(void)
{
    if (pose_trace_remaining == 0) return;
    pose_trace_remaining--;
    if (++pose_trace_div < 20) return;   /* ~5 Hz at 100 Hz acq */
    pose_trace_div = 0;

    float mag = sqrtf(gx_filt * gx_filt + gy_filt * gy_filt + gz_filt * gz_filt);
    float shadow = sqrtf(gy_filt * gy_filt + gz_filt * gz_filt);
    float vert = (mag > 0.1f)
        ? acosf(fminf(1.0f, fabsf(gx_filt) / mag)) * RAD_TO_DEG : 0.0f;
    orientation_state_t ori;
    orientation_get(&ori);
    LOG_INF("POSE-TRACE g=(%.2f,%.2f,%.2f) shadow=%.2f vert=%.0f "
            "pitch=%.0f roll=%.0f at_rest=%d",
            (double)gx_filt, (double)gy_filt, (double)gz_filt,
            (double)shadow, (double)vert,
            (double)ori.pitch_deg, (double)ori.roll_deg, (int)ori.at_rest);
}

void gesture_mode_update_accel(float ax, float ay, float az)
{
    /* Stash raw accel for the orientation filter (fused with the gyro
     * sample in the update_gyro call that immediately follows).  Done
     * before any early return so it's always current. */
    last_raw_ax = ax;
    last_raw_ay = ay;
    last_raw_az = az;

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

    /* Observability-cone measurement trace (no-op unless armed via 'v'). */
    pose_trace_tick();

    /* Reclassify orientation based on filtered gravity. */
    WristOrientation new_classification = _classify_orientation(
        gx_filt, gy_filt, gz_filt);

    /* Counter caps at ORIENTATION_LEAVE_DWELL, so the NEUTRAL (leave) path
     * can only commit if LEAVE >= ENTER.  Enforce at compile time. */
    BUILD_ASSERT(ORIENTATION_LEAVE_DWELL >= ORIENTATION_ENTER_DWELL,
                 "ORIENTATION_LEAVE_DWELL must be >= ORIENTATION_ENTER_DWELL "
                 "(dwell counter caps at LEAVE)");

    /* Orientation dwell with HYSTERESIS: a candidate must hold before it
     * commits, and LEAVING a pose to NEUTRAL needs a longer hold than
     * entering a definite pose -- so a brief gx~=gy dip at a sweep extreme
     * can't flip UP_RAISED->NEUTRAL. */
    if (new_classification == orientation_candidate) {
        if (orientation_candidate_dwell < ORIENTATION_LEAVE_DWELL) {
            orientation_candidate_dwell++;
        }
    } else {
        orientation_candidate = new_classification;
        orientation_candidate_dwell = 1;
    }

    int required_dwell = (orientation_candidate == WRIST_NEUTRAL)
        ? ORIENTATION_LEAVE_DWELL    /* leaving a pose: sticky */
        : ORIENTATION_ENTER_DWELL;   /* entering UP_RAISED/DOWN_FLAT: responsive */

    if (orientation_candidate_dwell >= required_dwell &&
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

    /* Decrement cursor-mode cooldown each sample.  Once it hits 0,
     * the orientation-only re-engage path is closed and the user has
     * to deliberately re-tap (double-tap for AIR_MOUSE, triple-tap
     * for SURFACE) to re-enter.  At that crossing we also release
     * the acq-keep-alive hold so the IMU pipeline can suspend back
     * to whatever the underlying power state wants. */
    if (cursor_cooldown_remaining > 0) {
        cursor_cooldown_remaining--;
        if (cursor_cooldown_remaining == 0) {
            LOG_INF("Cooldown for %s expired -- explicit re-tap required",
                    _mode_str(cursor_cooldown_mode));
            cursor_cooldown_mode = MODE_IDLE;
            reengage_dwell = 0;
            _update_acq_request();
        }
    }

    /* Activity gate maintenance.  Compute motion residual once and
     * reset samples_since_activity to 0 if motion is "active"; tick
     * up to the saturation point otherwise.  See ACTIVITY_GATE_*
     * comment block for design intent. */
    {
        float rx = ax - gx_filt;
        float ry = ay - gy_filt;
        float rz = az - gz_filt;
        float r_mag = sqrtf(rx * rx + ry * ry + rz * rz);
        if (r_mag > ACTIVITY_GATE_THRESH) {
            samples_since_activity = 0;
        } else if (samples_since_activity < ACTIVITY_GATE_DWELL) {
            samples_since_activity++;
        }
    }

    /* Update pose state machine.  Arms on canonical pose match alone. */
    pose_fsm_update(gx_filt, gy_filt, gz_filt);

    /* Multi-tap commit-on-timeout MOVED to a k_work_delayable
     * scheduled from gesture_mode_on_chip_single_tap().  Reason:
     * gesture_mode_update_accel runs from the acq pipeline, which
     * is NOT active in IDLE with no cursor mode and no cooldown.
     * Without acq running, the per-sample timeout check would
     * never fire and multi-tap sequences would never commit (real
     * bug observed 2026-06-08).  See multi_tap_commit_handler. */

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
                /* Flag the upcoming transition so _transition_to
                 * suppresses the "entered while already in pose"
                 * log -- the line above already explains we're
                 * coming back from cooldown. */
                s_transition_via_cooldown_reengage = true;
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
     * for _exit_dwell_for(current_mode) samples -> exit to IDLE and
     * start the cooldown.  Only runs after the user has assumed the
     * pose at least once.
     *
     * Note on per-mode exit semantics:
     *   AIR_MOUSE: "lowering wrist" -- deliberate, slow.  Dwell = 500 ms.
     *   SURFACE  : "lifting wrist off desk" -- like lifting a mouse to
     *              reposition.  Dwell = 0 (only the orientation
     *              classifier's own 250 ms dwell remains), so the mode
     *              exits almost instantly when the wrist comes off the
     *              desk plane.  The 20 s cooldown then lets the user
     *              put the wrist back down and resume seamlessly.
     *
     *   For SURFACE specifically: the user might also LOWER (e.g. drop
     *   arm to lap) rather than LIFT.  Either departure from DOWN_FLAT
     *   triggers exit + cooldown; the orientation classifier doesn't
     *   distinguish between "lifted up" and "dropped down", just
     *   "no longer flat."  This matches how a mouse behaves: any way
     *   of losing surface contact pauses tracking.
     */
    int exit_dwell_target = _exit_dwell_for(current_mode);
    if (in_cursor_mode && cursor_has_reached_pose &&
        orientation_current != expected) {
        if (cursor_exit_dwell <= exit_dwell_target) {
            cursor_exit_dwell++;
            if (cursor_exit_dwell > exit_dwell_target) {
                LOG_INF("%s exit: %s -- starting %d ms re-engage cooldown",
                        _mode_str(current_mode),
                        (current_mode == MODE_SURFACE)
                            ? "wrist left desk plane (lift or drop)"
                            : "wrist lowered out of raised pose",
                        CURSOR_COOLDOWN_SAMPLES * 10);
                cursor_cooldown_mode = current_mode;
                cursor_cooldown_remaining = CURSOR_COOLDOWN_SAMPLES;
                _transition_to(MODE_IDLE);
            }
        }
    } else {
        cursor_exit_dwell = 0;
    }

    /* SURFACE motion-burst exit detector.  See
     * SURFACE_MOTION_BURST_THRESH comment block for rationale.  Only
     * runs in SURFACE (AIR_MOUSE has no equivalent "transport without
     * orientation change" failure mode -- raising the arm always
     * rotates the wrist).  Runs after cursor_has_reached_pose is
     * latched -- pre-engagement motion is just the user placing the
     * wrist down. */
    if (current_mode == MODE_SURFACE && cursor_has_reached_pose) {
        float rx = ax - gx_filt;
        float ry = ay - gy_filt;
        float rz = az - gz_filt;
        float r_mag = sqrtf(rx * rx + ry * ry + rz * rz);

        /* Track windowed peak (resets each periodic log). */
        if (r_mag > surface_motion_peak) {
            surface_motion_peak = r_mag;
        }

        /* Periodic stats log -- a calibration stream the user can
         * read off while gliding (low values expected, ~0.2-0.8) vs
         * transporting (high values expected, 2-5+).  Helps tune
         * SURFACE_MOTION_BURST_THRESH from real data. */
        surface_motion_log_counter++;
        if (surface_motion_log_counter >= SURFACE_MOTION_LOG_PERIOD) {
            LOG_INF("SURFACE motion stats: peak |a-g|=%.2f m/s^2 "
                    "in last %d ms (burst thresh=%.2f)",
                    (double)surface_motion_peak,
                    SURFACE_MOTION_LOG_PERIOD * 10,
                    (double)SURFACE_MOTION_BURST_THRESH);
            surface_motion_peak = 0.0f;
            surface_motion_log_counter = 0;
        }

        if (r_mag > SURFACE_MOTION_BURST_THRESH) {
            if (surface_motion_burst_dwell < SURFACE_MOTION_BURST_DWELL) {
                surface_motion_burst_dwell++;
                if (surface_motion_burst_dwell == SURFACE_MOTION_BURST_DWELL) {
                    LOG_INF("SURFACE exit: motion burst |a-g|=%.2f m/s^2 "
                            "(thresh=%.2f, window peak=%.2f) -- "
                            "starting %d ms re-engage cooldown",
                            (double)r_mag,
                            (double)SURFACE_MOTION_BURST_THRESH,
                            (double)surface_motion_peak,
                            CURSOR_COOLDOWN_SAMPLES * 10);
                    cursor_cooldown_mode = MODE_SURFACE;
                    cursor_cooldown_remaining = CURSOR_COOLDOWN_SAMPLES;
                    _transition_to(MODE_IDLE);
                }
            }
        } else {
            surface_motion_burst_dwell = 0;
        }
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

    /* Update the orientation filter, fusing this gyro sample with the
     * accel stashed in the immediately-preceding update_accel call. */
    orientation_update(last_raw_ax, last_raw_ay, last_raw_az,
                       gx_rps, gy_rps, gz_rps);

    /* Air-mouse cursor tracking: fused angles -> relative delta -> pipeline.
     * Cone gate uses the GRAVITY-LPF shadow (gy_filt/gz_filt), NEVER the
     * fused quaternion (inside the cone the fused roll drifts on gyro alone). */
    if ((GestureMode)atomic_get(&mode_atomic) == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        float shadow = sqrtf(gy_filt * gy_filt + gz_filt * gz_filt);
        float vert = current_vert_deg();      /* roll-immune Y driver */
        float dx = 0.0f, dy = 0.0f;
        cursor_track_update(vert, ori.roll_deg, ori.at_rest,
                            shadow, &dx, &dy);
        cursor_pipeline_inject_motion(dx, dy);

        /* Throttled cursor telemetry (~10 Hz).  Exposes absolute-Y internal
         * state (vert_top anchor, cur_y position, slam flag) alongside the
         * motion deltas for tuning and acceptance testing. */
        static int cursor_tel_ctr = 0;
        if ((dx != 0.0f || dy != 0.0f || cursor_track_is_slamming()) &&
            (++cursor_tel_ctr % 10 == 0)) {
            LOG_INF("[CURSOR] vert=%d vtop=%d cur_y=%d target=%d slam=%d roll=%d "
                    "shadow=%d dx=%d dy=%d",
                    (int)vert, (int)cursor_track_vert_top(),
                    (int)cursor_track_cur_y(), (int)cursor_track_target_counts(),
                    (int)cursor_track_is_slamming(),
                    (int)ori.roll_deg, (int)shadow, (int)dx, (int)dy);
        }
    }

    /* Push into the rotation-signature ring buffer (dictation-flip
     * prototype).  Stored in rad/s; gyro_signature() converts. */
    gyro_hist[gyro_hist_idx][0] = gx_rps;
    gyro_hist[gyro_hist_idx][1] = gy_rps;
    gyro_hist[gyro_hist_idx][2] = gz_rps;
    gyro_hist_idx = (gyro_hist_idx + 1) % GYRO_HIST_SAMPLES;

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

void gesture_mode_on_chip_single_tap(char peak_axis, char tap_sign)
{
    /* Stage C (2026-06-08): firmware multi-tap counter.  Each
     * SINGLE_TAP arrival from the chip is gated by ringing-
     * rejection refractory, then the POSE gate, then folded into a
     * running count.  The k_work_delayable commit handler commits
     * the accumulated count to a mode-entry decision
     * MULTI_TAP_WINDOW_MS after the last arrival.
     *
     * NOTE (2026-06-10): the old motion "activity gate" was REMOVED
     * from this path.  It rejected taps when accel motion-residual
     * was high recently -- but a tap's own impulse IS a large accel
     * transient, so the gate rejected the very taps it was meant to
     * pass (hardware test: every cadenced double-tap got "GATED by
     * recent motion (0 ms ago)").  The pose gate below + the cadence
     * requirement in the commit handler are strictly stronger
     * deliberate-intent filters, so the activity gate is now
     * redundant (taps without a pose are rejected by the pose gate
     * anyway) and was actively blocking legitimate triggers.  See
     * the pose-gated trigger redesign spec. */

    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);
    int64_t now = k_uptime_get();

    /* Ringing-rejection refractory: at 833 Hz the chip's QUIET
     * window is ~14.4 ms which is borderline for some band-ringing
     * patterns.  Reject duplicate events within RINGING_REFRACTORY_MS
     * as ringing leftover from the previous shock. */
    if (last_chip_tap_time_ms != 0 &&
        (now - last_chip_tap_time_ms) < RINGING_REFRACTORY_MS) {
        LOG_INF("Chip single-tap SUPPRESSED as ringing duplicate "
                "(%d ms after previous, axis=%c sign=%c)",
                (int)(now - last_chip_tap_time_ms),
                peak_axis, tap_sign);
        return;
    }
    /* Push current to previous BEFORE updating last_chip_tap_time_ms.
     * This is what enables last_two_tap_interval_ms() to compute the
     * gap between the two most-recent events. */
    prev_chip_tap_time_ms = last_chip_tap_time_ms;
    last_chip_tap_time_ms = now;

    /* Pose gate: chip-tap is only a mode-entry candidate if we're
     * currently in a pose-armed state.  Without an armed pose, the
     * tap is logged for diagnostics but does NOT advance multi-tap
     * state or trigger mode entry.
     *
     * This is what prevents skin-taps near the band, desk slaps, thigh
     * slaps, lap-rest impacts, etc. from triggering modes. */
    pose_id_t armed = gesture_mode_armed_pose();
    if (armed == POSE_NONE) {
        LOG_INF("Chip single-tap IGNORED: no pose armed "
                "(axis=%c sign=%c).  Mode entry requires pose-first.",
                peak_axis, tap_sign);
        return;
    }

    bool starts_new_sequence =
        (multi_tap_count == 0) ||
        ((now - multi_tap_last_time_ms) > MULTI_TAP_WINDOW_MS);

    if (starts_new_sequence) {
        multi_tap_count = 1;
        multi_tap_first_axis = peak_axis;
        multi_tap_first_sign = tap_sign;
    } else {
        multi_tap_count++;
    }
    multi_tap_last_time_ms = now;

    /* (Re)schedule the commit-on-timeout work item.  Each tap
     * arrival pushes the timeout out by MULTI_TAP_WINDOW_MS.  When
     * the tap sequence goes quiet, the work fires once and commits. */
    k_work_reschedule(&multi_tap_commit_work, K_MSEC(MULTI_TAP_WINDOW_MS));

    LOG_INF("Chip single-tap: axis=%c sign=%c (sequence count=%d, "
            "first axis=%c, mode=%s orient=%s)",
            peak_axis, tap_sign,
            multi_tap_count,
            multi_tap_first_axis,
            _mode_str(current_mode),
            _orientation_str(orientation_current));
}

/* Multi-tap commit-on-timeout handler.  Fires once MULTI_TAP_WINDOW_MS
 * after the last tap arrival.  Walks the accumulated count to a
 * 1/2/3+-tap action and chains to the existing chip-event handlers
 * (_on_chip_double_tap for AIR_MOUSE entry, _on_chip_triple_tap for
 * SURFACE entry).
 *
 * Runs in system workqueue context.  Concurrent access with
 * gesture_mode_on_chip_single_tap (which runs in the run_idle thread)
 * is benign: the worst case is the work fires concurrently with a
 * new tap arriving, which would commit count N and immediately start
 * a fresh sequence -- correct behavior.  The k_work_reschedule
 * already takes care of cancel-and-restart races. */
static void multi_tap_commit_handler(struct k_work *work_arg)
{
    ARG_UNUSED(work_arg);

    /* The accumulation window must cover the whole cadence range, or a valid
     * slow double-tap commits its first tap alone and is rejected. */
    BUILD_ASSERT(MULTI_TAP_WINDOW_MS >= CADENCE_MAX_MS,
                 "MULTI_TAP_WINDOW_MS must be >= CADENCE_MAX_MS "
                 "(else a slow double-tap splits into two single taps)");

    int count = multi_tap_count;
    char axis = multi_tap_first_axis;
    char sign = multi_tap_first_sign;

    /* Snapshot armed pose at commit time. */
    pose_id_t armed = gesture_mode_armed_pose();

    /* Reset multi-tap counter regardless of outcome.  Pose disarm
     * happens at the bottom of this function for cleanliness. */
    multi_tap_count = 0;
    multi_tap_first_axis = '?';
    multi_tap_first_sign = '?';

    if (armed == POSE_NONE) {
        /* Should not happen (the pose gate in on_chip_single_tap
         * should have rejected these taps), but defensive. */
        LOG_INF("Multi-tap commit ABORT: no pose armed (count=%d, "
                "axis=%c sign=%c)", count, axis, sign);
        return;
    }

    /* Common pre-check: ALL modes require cadenced double-tap.
     * Single tap is rejected as too easy to produce accidentally
     * even in a deliberate pose. */
    int interval = 0;  /* declared before any goto to avoid jump-crosses-init */
    if (count < 2) {
        LOG_INF("Mode entry rejected (%s): need double-tap "
                "(got count=%d)", pose_name(armed), count);
        goto disarm;
    }
    interval = last_two_tap_interval_ms();
    if (!is_cadenced_double_tap_window(interval)) {
        LOG_INF("Mode entry rejected (%s): inter-tap=%d ms outside "
                "cadence window [%d, %d]",
                pose_name(armed), interval,
                CADENCE_MIN_MS, CADENCE_MAX_MS);
        goto disarm;
    }

    /* Apply per-pose extra checks then trigger mode entry. */
    switch (armed) {
    case POSE_AIR_MOUSE:
        LOG_INF("MODE ENTRY: AIR_MOUSE (pose + cadenced double-tap)");
        _transition_to(MODE_AIR_MOUSE);
        break;

    case POSE_SURFACE:
        /* SURFACE has an extra check: tap must show desk-feedback
         * spectral signature (hard surface, not lap). */
        if (!surface_spectral_confirms_hard_surface()) {
            LOG_INF("SURFACE entry rejected: spectral signature "
                    "indicates soft surface (mid_band=%.0f < %.0f)",
                    (double)last_tap_mid_band_energy,
                    (double)SURFACE_RESONANCE_MID_BAND_THRESH);
            goto disarm;
        }
        LOG_INF("MODE ENTRY: SURFACE (pose + cadenced double-tap + "
                "hard-surface spectral confirmed)");
        /* TODO: trigger mode transition (future task F2). */
        break;

    default:
        LOG_INF("Mode entry ABORT: unknown armed pose %d", (int)armed);
        break;
    }

disarm:
    /* Disarm pose after a gesture attempt (whether successful or
     * not) -- one shot per arm window.  Forces user to re-pose
     * for the next attempt, which prevents accidentally chaining
     * gestures. */
    pose_armed_state = POSE_NONE;
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

    /* Kick the acq-request evaluation NOW that the callback is wired.
     *
     * Root cause of the 2026-06-10 "pose only arms during SNAPSHOT"
     * bug: _update_acq_request() is otherwise only called from
     * _transition_to() (mode changes) and the cooldown-expiry edge.
     * In the IDLE steady state neither fires, so gesture_needs_acq
     * stayed false, acq stopped after each SNAPSHOT, and
     * gesture_mode_update_accel() never ran in IDLE -- freezing the
     * gravity LPF and the pose FSM.
     *
     * Under the always-on policy (_update_acq_request now always
     * returns needs=true), this one call at boot sets
     * gesture_needs_acq=true and starts acquisition.  From then on
     * stop_acquisition() defers indefinitely (gesture_needs_acq
     * stays true), so acq runs continuously and the pose FSM gets
     * accel samples in every power state. */
    _update_acq_request();
}

/*
 * --- Bio-acoustic capture + feature extraction + classifier (Stage E) ---
 *
 * When the chip's tap engine fires INT1, the main dispatcher calls
 * gesture_mode_bio_acoustic_on_tap().  That function reads the chip's
 * FIFO (continuous mode at 1.66 kHz; ~410 ms ring-buffer of recent
 * accel samples) into a static capture buffer, then signals a worker
 * thread.  The worker computes features and classifies the tap as
 * snap-vs-band-tap.
 *
 * Architecture decisions (see docs/research/gesture-architecture.md §3
 * Bio-acoustic sensing path):
 *   - Capture: chip FIFO in continuous mode (chip-native pre-trigger
 *     ring buffer).  ~250 samples = ~150 ms pre-event window at
 *     1.66 kHz reading.
 *   - Worker: async thread (industry standard).  Acq thread keeps
 *     reading at 100 Hz unaffected.
 *   - Classifier: simple rules in v0.  Snap = anticipatory pre-event
 *     motion + broad axis distribution.  Band-tap = quiet pre-event +
 *     Z-axis dominance.  ML migration tracked in Item 8.
 *   - Output: log-only for now.  When classifier proves reliable on
 *     real data, the classification will feed mode-routing decisions.
 *
 * For v0 testing, the classifier output does NOT change FSM behavior
 * -- it only logs.  The existing multi-tap counter continues to drive
 * AIR_MOUSE / SURFACE entry as before.  This lets you validate the
 * classifier against ground truth (you know which gesture you did)
 * without breaking working paths.
 */

/* Capture buffer.  At 833 Hz, 512 samples = ~615 ms of signal,
 * a power of 2 so CMSIS-DSP arm_rfft_fast_f32 can transform it
 * directly.  Each sample is 3 axes * int16 = 6 bytes.  Total
 * 3072 bytes. */
#define TAP_CAPTURE_SAMPLES   512
#define TAP_CAPTURE_BYTES     (TAP_CAPTURE_SAMPLES * 6)
static uint8_t  tap_capture_buf[TAP_CAPTURE_BYTES];
static uint16_t tap_capture_sample_count = 0;

/* FFT scratch -- magnitude signal time-domain (input) and complex
 * frequency-domain (output).  CMSIS-DSP arm_rfft_fast_f32 produces
 * an N-element interleaved real/imag output for an N-element real
 * input.  At 833 Hz / 512 points, bin resolution is 1.63 Hz.
 *
 * Total memory: 2 * 512 * 4 = 4 KB.  Acceptable on our 256 KB RAM. */
#define FFT_SIZE              TAP_CAPTURE_SAMPLES
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static arm_rfft_fast_instance_f32 fft_inst;
static bool fft_initialised = false;

/* Worker synchronisation: dispatcher gives the semaphore after
 * filling the buffer; worker takes it and processes.  Binary
 * semaphore -- if a second tap arrives while the worker is still
 * processing, the dispatcher skips capture and logs "dropped"
 * rather than corrupting the in-flight buffer. */
static K_SEM_DEFINE(tap_capture_sem, 0, 1);
static atomic_t tap_capture_busy = ATOMIC_INIT(0);  /* 1 = buffer in use */

/* Feature extraction output.  Computed by the worker from
 * tap_capture_buf, then fed to the classifier.  v2 features
 * (2026-06-10): spectral band energies replace pre_event_energy
 * which proved unreliable in real-world use. */
struct tap_features {
    int16_t peak_x;        /* signed peak (most positive or negative) */
    int16_t peak_y;
    int16_t peak_z;
    uint16_t peak_x_abs;   /* magnitudes for ratio comparisons */
    uint16_t peak_y_abs;
    uint16_t peak_z_abs;
    uint16_t peak_sample_idx;  /* where in buffer the peak is */
    float    low_band_energy;  /* energy in 20-200 Hz band (snap-favouring) */
    float    mid_band_energy;  /* energy in 200-400 Hz band (tap-favouring) */
    float    low_band_ratio;   /* low / (low + mid); near 1.0 = snap, near
                                  0.5 = broadband (tap) */
    char     dominant_axis;    /* 'X' / 'Y' / 'Z' */
};

/* Helper: parse one accel sample (6 bytes, little-endian) from buf. */
static inline void _parse_sample(const uint8_t *p,
                                 int16_t *x, int16_t *y, int16_t *z)
{
    *x = (int16_t)((p[1] << 8) | p[0]);
    *y = (int16_t)((p[3] << 8) | p[2]);
    *z = (int16_t)((p[5] << 8) | p[4]);
}

static inline uint16_t _abs16(int16_t v)
{
    return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

/* Locate the peak-magnitude sample.  Scans all samples; the one
 * with the largest |X|+|Y|+|Z| sum is treated as the impulse moment.
 * Records peak per axis and the index where the peak sits. */
static void _extract_peak(const uint8_t *buf, uint16_t sample_count,
                          struct tap_features *out)
{
    uint32_t max_sum = 0;
    uint16_t max_idx = 0;
    int16_t  px = 0, py = 0, pz = 0;

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t x, y, z;
        _parse_sample(buf + i * 6, &x, &y, &z);
        uint32_t s = (uint32_t)_abs16(x) +
                     (uint32_t)_abs16(y) +
                     (uint32_t)_abs16(z);
        if (s > max_sum) {
            max_sum = s;
            max_idx = i;
            px = x;
            py = y;
            pz = z;
        }
    }
    out->peak_x = px;
    out->peak_y = py;
    out->peak_z = pz;
    out->peak_x_abs = _abs16(px);
    out->peak_y_abs = _abs16(py);
    out->peak_z_abs = _abs16(pz);
    out->peak_sample_idx = max_idx;

    /* Dominant axis = whichever |axis| is largest at peak. */
    if (out->peak_z_abs >= out->peak_x_abs &&
        out->peak_z_abs >= out->peak_y_abs) {
        out->dominant_axis = 'Z';
    } else if (out->peak_x_abs >= out->peak_y_abs) {
        out->dominant_axis = 'X';
    } else {
        out->dominant_axis = 'Y';
    }
}

/* Compute spectral band energies of the impulse signal.
 *
 * Approach (ViBand-style, adapted for 833 Hz ODR):
 *   1. Build a magnitude signal mag[i] = sqrt(x^2 + y^2 + z^2)
 *      from the int16 accel samples.  Magnitude is pose-invariant
 *      (whichever axis the impulse hits, magnitude captures it).
 *   2. Subtract mean (DC removal) so gravity doesn't dominate the
 *      DC bin.  Saves dynamic range for the impulse content.
 *   3. Apply Hann window to reduce spectral leakage at the
 *      buffer edges.
 *   4. Run real FFT via CMSIS-DSP arm_rfft_fast_f32.
 *   5. Sum |X[k]|^2 over the two bands of interest.
 *
 * Bands (at 833 Hz / 512 points, bin resolution 1.63 Hz):
 *   - Low band: bins 12-122 = 19.5 - 199 Hz.  Bone-conducted
 *     snap impulses dominate here per ViBand.
 *   - Mid band: bins 122-246 = 199 - 400 Hz.  Direct mechanical
 *     impacts (tap on band housing) have more energy here.
 *
 * Discriminator: low_ratio = low / (low + mid).  Snap -> ratio
 * high (~0.7+); tap -> ratio lower (~0.5 or less).
 *
 * Band-edge constants (FFT_BIN_LOW_START/END, FFT_BIN_MID_START/END)
 * come from gesture_thresholds.h. */

static void _compute_band_energies(const uint8_t *buf,
                                   uint16_t sample_count,
                                   float *out_low,
                                   float *out_mid,
                                   float *out_ratio)
{
    *out_low = 0.0f;
    *out_mid = 0.0f;
    *out_ratio = 0.5f;  /* neutral default */

    if (sample_count < FFT_SIZE || !fft_initialised) {
        return;
    }

    /* Step 1+2: magnitude signal with DC removal. */
    float mean = 0.0f;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        int16_t x, y, z;
        _parse_sample(buf + i * 6, &x, &y, &z);
        float fx = (float)x;
        float fy = (float)y;
        float fz = (float)z;
        float mag = sqrtf(fx * fx + fy * fy + fz * fz);
        fft_input[i] = mag;
        mean += mag;
    }
    mean /= (float)FFT_SIZE;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        fft_input[i] -= mean;
    }

    /* Step 3: Hann window.  Reduces spectral leakage at the buffer
     * edges so the band-energy integrals are more accurate.  Half-
     * cosine envelope from 0 at the ends to 1 at the centre.
     * Local 2*PI constant -- M_PI isn't exposed in this build
     * configuration. */
    static const float TWO_PI = 6.28318530717958647692f;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        float w = 0.5f * (1.0f - cosf(TWO_PI * (float)i /
                                       (float)(FFT_SIZE - 1)));
        fft_input[i] *= w;
    }

    /* Step 4: FFT.  ifftFlag=0 means forward transform. */
    arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);

    /* Step 5: sum |X[k]|^2 over the two bands.  arm_rfft_fast_f32
     * output format: index 0 = DC (real), index 1 = Nyquist (real),
     * indices 2..N-1 = interleaved real/imag pairs for bins 1..N/2-1.
     *
     * For our band ranges (12-122 and 122-246) we're well clear of
     * bins 0 and Nyquist, so we read pairs starting at offset 2*k. */
    for (uint16_t k = FFT_BIN_LOW_START; k < FFT_BIN_LOW_END; k++) {
        float re = fft_output[2 * k];
        float im = fft_output[2 * k + 1];
        *out_low += re * re + im * im;
    }
    for (uint16_t k = FFT_BIN_MID_START; k < FFT_BIN_MID_END; k++) {
        float re = fft_output[2 * k];
        float im = fft_output[2 * k + 1];
        *out_mid += re * re + im * im;
    }

    float total = *out_low + *out_mid;
    if (total > 0.0f) {
        *out_ratio = *out_low / total;
    }
}

/* Surface-spectral confirmation -- v0 proxy.
 *
 * Literature-backed discriminator (PMC 2016, surface-stiffness
 * acceleration analysis): hard surfaces produce LONGER-DURATION
 * ringing -- more oscillation cycles before energy decays.  Soft
 * surfaces damp quickly.  The proper feature is post-event
 * ring-down duration.
 *
 * Our current FFT pipeline doesn't directly measure ring-down
 * duration -- the 512-sample capture is mostly PRE-impact, with
 * the impact at the end.  Instead we use mid_band_energy as a
 * proxy: hard surfaces couple some of their ringing into the
 * mid frequencies that ARE in our window (via reflected feedback
 * during the tap itself), giving elevated mid_band.  Soft
 * surfaces don't.
 *
 * This is imperfect; future work (F9, see plan) is to capture a
 * post-event window for direct ring-down measurement.
 *
 * Empirical threshold from desk-tap-vs-lap-tap data collection (to
 * be tuned during integration test).  Conservative initial value.
 *
 * IMPORTANT: this signal is housing-dependent.  Current value tuned
 * for open PCB; will need recalibration when housing arrives.
 *
 * SURFACE_RESONANCE_MID_BAND_THRESH is defined near the top of the
 * static-variable block (before multi_tap_commit_handler) so it is
 * visible to that function without a forward reference. */

/* Returns true if the most recently captured tap shows desk-feedback
 * spectral content (mid_band above threshold).  Used to confirm
 * SURFACE entry: hard surface (desk) gives elevated mid_band from
 * resonance feedback; soft surface (lap) damps it. */
static bool surface_spectral_confirms_hard_surface(void)
{
    return last_tap_mid_band_energy >= SURFACE_RESONANCE_MID_BAND_THRESH;
}

/* Worker thread entry.  Blocks on tap_capture_sem, processes one
 * capture at a time, logs the result.  Sample count and buffer are
 * static and protected by the busy flag. */
static void bio_acoustic_worker(void *, void *, void *)
{
    while (1) {
        k_sem_take(&tap_capture_sem, K_FOREVER);

        uint16_t n = tap_capture_sample_count;
        if (n < 100) {
            LOG_INF("[BIO] capture too short (%u samples) -- skipping",
                    (unsigned)n);
            atomic_set(&tap_capture_busy, 0);
            continue;
        }

        struct tap_features f = {};
        _extract_peak(tap_capture_buf, n, &f);
        _compute_band_energies(tap_capture_buf, n,
                               &f.low_band_energy,
                               &f.mid_band_energy,
                               &f.low_band_ratio);
        uint32_t total_abs = (uint32_t)f.peak_x_abs +
                             (uint32_t)f.peak_y_abs +
                             (uint32_t)f.peak_z_abs;
        uint32_t z_ratio_pct = total_abs ?
                                (uint32_t)f.peak_z_abs * 100 / total_abs :
                                0;

        /* Log the full feature vector for diagnostics.  ratio is the
         * headline discriminator; low/mid absolute energies confirm we
         * have meaningful signal (vs noise floor).  No verdict -- pose
         * carries mode info; snap-vs-tap discrimination deferred to
         * future in-session fusion (F6). */
        LOG_INF("[BIO] features peak[%d,%d,%d] dom=%c z_ratio=%u%% "
                "low_band=%.0f mid_band=%.0f low_ratio=%.2f idx=%u/%u",
                (int)f.peak_x, (int)f.peak_y, (int)f.peak_z,
                f.dominant_axis,
                (unsigned)z_ratio_pct,
                (double)f.low_band_energy,
                (double)f.mid_band_energy,
                (double)f.low_band_ratio,
                (unsigned)f.peak_sample_idx, (unsigned)n);

        last_tap_mid_band_energy = f.mid_band_energy;

        atomic_set(&tap_capture_busy, 0);
    }
}

K_THREAD_DEFINE(bio_acoustic_worker_id, 1536,
                bio_acoustic_worker, NULL, NULL, NULL,
                7,  /* priority -- below acq but above background */
                0, 0);

/* Public API for main.cpp.  Call once at boot to enable the chip
 * FIFO; call on every chip tap event to snapshot + signal worker. */
void gesture_mode_bio_acoustic_init(void)
{
    /* Initialise CMSIS-DSP RFFT instance.  Length must be a
     * supported power of 2 (32..4096); 512 fits us perfectly.
     * arm_rfft_fast_init_f32 returns ARM_MATH_SUCCESS (0) on
     * success.  Stash the success flag so the worker can short-
     * circuit if init failed (shouldn't happen but defensive). */
    arm_status fft_status = arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    if (fft_status != ARM_MATH_SUCCESS) {
        LOG_ERR("[BIO] arm_rfft_fast_init_f32 failed (%d) -- spectral "
                "classifier disabled", (int)fft_status);
        fft_initialised = false;
    } else {
        fft_initialised = true;
    }

    int err = lsm6dsl_fifo_enable_continuous();
    if (err) {
        LOG_ERR("[BIO] FIFO enable failed (%d) -- bio-acoustic "
                "capture will not work", err);
    } else {
        LOG_INF("[BIO] worker thread started; FIFO armed for capture; "
                "FFT %s", fft_initialised ? "ready" : "DISABLED");
    }
}

void gesture_mode_bio_acoustic_on_tap(void)
{
    /* Try to claim the capture buffer.  If the worker is still
     * processing a previous capture, drop this one with a log so the
     * loss is visible -- we'd rather skip than corrupt mid-process. */
    if (!atomic_cas(&tap_capture_busy, 0, 1)) {
        LOG_INF("[BIO] tap dropped -- worker still processing previous");
        return;
    }

    /* Read the LATEST samples from the FIFO -- this is where the tap
     * impulse lives (the chip wrote it most recently).  Previous
     * implementation read the OLDEST samples from FIFO head, which
     * meant the actual impulse was sitting just past our read window
     * (the unread tail).  We were "capturing" pre-event signal but
     * missing the impulse entirely, and the classifier was operating
     * on gravity vectors only.  Caught in hardware test 2026-06-09.
     *
     * Fix: figure out total occupancy, discard the older samples to
     * get to the latest TAP_CAPTURE_SAMPLES, then read those. */
    uint16_t words_in_fifo = 0;
    int err = lsm6dsl_fifo_get_word_count(&words_in_fifo);
    if (err) {
        LOG_WRN("[BIO] FIFO_STATUS read failed (%d)", err);
        atomic_set(&tap_capture_busy, 0);
        return;
    }

    /* Each accel sample is 3 words (X/Y/Z each = 1 word = 2 bytes).
     * Cap our keep-window at TAP_CAPTURE_SAMPLES. */
    uint16_t samples_in_fifo = words_in_fifo / 3;
    uint16_t want_samples = (samples_in_fifo > TAP_CAPTURE_SAMPLES)
                                ? TAP_CAPTURE_SAMPLES
                                : samples_in_fifo;
    uint16_t skip_samples = (samples_in_fifo > want_samples)
                                ? (samples_in_fifo - want_samples)
                                : 0;

    /* Discard the older samples one chunk at a time.  Reusing
     * tap_capture_buf as scratch is safe -- the worker isn't running
     * yet (we hold the busy flag) so the buffer's data will be
     * overwritten before being inspected.  Chunk size sized to one
     * I2C burst that fits comfortably. */
    uint16_t discard_chunk_samples = 64;  /* 64 samples * 6 bytes = 384 B */
    while (skip_samples > 0) {
        uint16_t chunk = (skip_samples > discard_chunk_samples)
                            ? discard_chunk_samples
                            : skip_samples;
        uint16_t got_discard = 0;
        err = lsm6dsl_fifo_read_words(tap_capture_buf, chunk * 3,
                                      &got_discard);
        if (err || got_discard == 0) {
            LOG_WRN("[BIO] FIFO discard-read failed (err=%d got=%u)",
                    err, (unsigned)got_discard);
            atomic_set(&tap_capture_busy, 0);
            return;
        }
        skip_samples -= got_discard / 3;
    }

    /* Now read the latest want_samples into the capture buffer. */
    uint16_t want_words = want_samples * 3;
    uint16_t got_words = 0;
    err = lsm6dsl_fifo_read_words(tap_capture_buf, want_words, &got_words);
    if (err || got_words == 0) {
        LOG_WRN("[BIO] FIFO read failed (err=%d got=%u)", err,
                (unsigned)got_words);
        atomic_set(&tap_capture_busy, 0);
        return;
    }

    tap_capture_sample_count = got_words / 3;
    k_sem_give(&tap_capture_sem);
}
