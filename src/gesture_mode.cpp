#include "gesture_mode.h"
#include "gesture_poses.h"
#include "orientation.h"
#include "gesture_thresholds.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <limits.h>

#include "bio_acoustic.h"

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

/* Filtered gravity vector (gx/gy/gz_filt): SLOW ~1 s LPF (GRAVITY_LP_ALPHA).
 * Stability IS the feature here -- used by pose classification, the orientation
 * classifier, and `shadow`.  Keep it slow. */
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

/* Acquisition-request callback registered by main.cpp. */
static gesture_acq_request_cb_t s_acq_request_cb = NULL;

/* Tracks the acq-request edges so we only call the callback on
 * transitions. */
static bool s_prev_needs_acq = false;

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

    /* SURFACE mode was retired (2026-06-13): do NOT arm POSE_SURFACE in the live
     * FSM.  An armed SURFACE pose has no consumer now (it can enter no mode -- see
     * the log-only case in multi_tap_commit_handler) and only produces noise.  The
     * canonical (gesture_poses.cpp k_canonical_poses) and the hard-surface spectral
     * detector (surface_spectral_confirms_hard_surface) are intentionally KEPT in
     * code as ready scaffolding: when the high-ODR bio-acoustic surface-tap feature
     * is built (the ViBand path -- see docs/research/gesture-architecture.md §12
     * feasibility + docs/research/hr-algorithm-decisions.md §10 roadmap), RE-ARM
     * by deleting this demotion and
     * wiring a real consumer in multi_tap_commit_handler. */
    if (best == POSE_SURFACE) {
        best = POSE_NONE;
    }

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

static void _transition_to(GestureMode new_mode)
{
    GestureMode old = (GestureMode)atomic_get(&mode_atomic);
    if (old == new_mode) {
        return;
    }
    atomic_set(&mode_atomic, (atomic_val_t)new_mode);
    LOG_INF("Mode transition: %s -> %s",
            _mode_str(old), _mode_str(new_mode));
    /* Multi-tap counter and activity gate persist across transitions
     * by design; the multi-tap commit handler resets them cleanly.
     * No reset here. */

    /* Re-evaluate the acq-request edge so the pose FSM keeps getting
     * accel samples (always-on policy -- see _update_acq_request()). */
    _update_acq_request();
}

/* Evaluate whether the acquisition pipeline needs to be alive RIGHT
 * NOW and notify the registered callback on edges.
 *
 * ALWAYS-ON policy (2026-06-10 fix): the acq pipeline must stay alive
 * continuously so the pose FSM receives accel samples in MODE_IDLE
 * (the always-listening trigger state).  Previously this was gated on
 * the active mode, which meant pose detection only ran during
 * SNAPSHOT-type states -- broken for the trigger-detection use case
 * since the user is in MODE_IDLE when they want to trigger.
 *
 * Power cost: ~30-50 µA additional vs gated-acq IDLE.  The IMU is
 * already running at 833 Hz continuously (for the chip tap engine);
 * this only adds the MCU's per-sample read + processing at 100 Hz.
 * Acceptable for an always-listening gesture device.
 *
 * Called from _transition_to (mode changes) and once at boot when the
 * callback is registered. */
static void _update_acq_request(void)
{
    if (!s_acq_request_cb) {
        return;
    }
    /* Acq pipeline must stay alive continuously so the pose FSM
     * receives accel samples in IDLE (the always-listening trigger
     * state).  Previously this was gated on the active mode, which
     * meant pose detection only ran during SNAPSHOT-type states --
     * broken for the trigger detection use case since the user is in
     * MODE_IDLE when they want to trigger.
     *
     * Power cost: ~30-50 µA additional vs gated-acq IDLE.  The
     * IMU is already running at 833 Hz continuously (for the chip
     * tap engine); this only adds the MCU's per-sample read+
     * processing at 100 Hz.  Acceptable for an always-listening
     * gesture device. */
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
        gx_filt = ax;    gy_filt = ay;    gz_filt = az;
        filter_initialised = true;
        return;
    }

    /* SLOW 1-pole IIR (pose / classifier / cone gate / shadow) -- keep slow. */
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

    /* Motion residual (accel - gravity); drives the activity gate. */
    float rx_resid = ax - gx_filt;
    float ry_resid = ay - gy_filt;
    float rz_resid = az - gz_filt;
    float r_mag = sqrtf(rx_resid * rx_resid + ry_resid * ry_resid + rz_resid * rz_resid);
    if (r_mag > ACTIVITY_GATE_THRESH) {
        samples_since_activity = 0;
    } else if (samples_since_activity < ACTIVITY_GATE_DWELL) {
        samples_since_activity++;
    }

    /* Update pose state machine.  Arms on canonical pose match alone. */
    pose_fsm_update(gx_filt, gy_filt, gz_filt);

    /* Multi-tap commit-on-timeout lives in a k_work_delayable scheduled
     * from gesture_mode_on_chip_single_tap().  Reason: this function
     * runs from the acq pipeline; the work item fires regardless of
     * whether acq is ticking, so multi-tap sequences always commit (the
     * bug fixed 2026-06-08).  See multi_tap_commit_handler. */
}

void gesture_mode_update_gyro(float gx_rps, float gy_rps, float gz_rps)
{
    /* Flick detection: look for a sharp burst followed by a reversal.
     * We pick the largest-magnitude axis for the burst direction.
     * Once a burst is detected, we count down FLICK_WINDOW_SAMPLES
     * looking for a sign-reversed burst.  If we get one inside the
     * window, that's a flick.
     *
     * After the air-mouse extraction the flick is detect + log only (no
     * mode bound) -- kept live so a future mode can wire it. */

    /* Update the orientation filter, fusing this gyro sample with the
     * accel stashed in the immediately-preceding update_accel call. */
    orientation_update(last_raw_ax, last_raw_ay, last_raw_az,
                       gx_rps, gy_rps, gz_rps);

    /* Push into the rotation-signature ring buffer (dictation-flip
     * prototype).  Stored in rad/s; gyro_signature() converts. */
    gyro_hist[gyro_hist_idx][0] = gx_rps;
    gyro_hist[gyro_hist_idx][1] = gy_rps;
    gyro_hist[gyro_hist_idx][2] = gz_rps;
    gyro_hist_idx = (gyro_hist_idx + 1) % GYRO_HIST_SAMPLES;

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
            LOG_INF("Wrist flick detected (no mode bound)");
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
    /* The double-tap previously entered AIR_MOUSE.  After the air-mouse
     * extraction the detection is kept live but routes to no mode --
     * log only, ready to wire a future mode here. */
    LOG_INF("Chip double-tap detected from %s -- gesture detected "
            "(no mode bound)", _mode_str(current_mode));
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
 * 1/2/3-tap gesture.  Post air-mouse extraction these are UNBOUND:
 * a committed double-/triple-tap is DETECTED + LOGGED ("no mode bound")
 * via _on_chip_double_tap / _on_chip_triple_tap, which no longer enter
 * any mode -- kept live as hooks for a future mode to bind.
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

    /* The committed pose + cadenced double-tap is detected and logged
     * but routes to NO mode after the air-mouse extraction -- ready to
     * wire a future mode here.  Pose + tap detection stays fully alive. */
    switch (armed) {
    case POSE_AIR_MOUSE:
        LOG_INF("GESTURE: raised-pose + cadenced double-tap (no mode bound)");
        break;

    case POSE_SURFACE:
        /* SURFACE MODE was dropped (roadmap 2026-06-13).  The pose classifier +
         * hard-surface spectral detector are KEPT as useful logic, but a SURFACE
         * pose + double-tap no longer ENTERS a mode (it is unbound, like triple-tap).
         * Log the detector result for diagnostics; enter nothing. */
        LOG_INF("SURFACE pose + double-tap (unbound) -- detector live "
                "(hard_surface=%d), no mode entry (SURFACE removed)",
                (int)bio_acoustic_last_was_hard_surface());
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
    /* SURFACE mode was dropped (roadmap 2026-06-13).  Triple-tap is KEPT as a free,
     * unbound trigger to repurpose later -- log-only for now, enters no mode. */
    LOG_INF("Chip triple-tap (unbound) from %s -- no mode entry (SURFACE removed)",
            _mode_str(current_mode));
}

GestureMode gesture_mode_get(void)
{
    return (GestureMode)atomic_get(&mode_atomic);
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

void gesture_mode_set_acq_request_cb(gesture_acq_request_cb_t cb)
{
    s_acq_request_cb = cb;

    /* Kick the acq-request evaluation NOW that the callback is wired.
     *
     * Root cause of the 2026-06-10 "pose only arms during SNAPSHOT"
     * bug: _update_acq_request() is otherwise only called from
     * _transition_to() (mode changes).  In the IDLE steady state that
     * never fires, so gesture_needs_acq
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

