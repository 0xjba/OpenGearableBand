#include "gesture_mode.h"
#include "gesture_poses.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>

#include <arm_math.h>
#include <arm_const_structs.h>

#include "power_ctrl.h"

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
 * Per-mode dwell required for orientation-drop exit.  Pose must NOT
 * be the mode's expected one (UP_RAISED for AIR_MOUSE, DOWN_FLAT
 * for SURFACE) for this many consecutive samples before the FSM
 * transitions out.
 *
 * AIR_MOUSE_EXIT_DWELL = 50 (500 ms):
 *   Lowering the wrist out of the raised pose is a slow, deliberate
 *   motion -- "I'm done air-mousing".  Half a second of dwell
 *   tolerates brief unintentional tilts without bouncing out.
 *
 * SURFACE_EXIT_DWELL = 0 (~250 ms effective due to ORIENTATION_DWELL):
 *   The model is "lift the mouse 1-2 cm to reposition" -- a real
 *   optical mouse pauses tracking within ~50 ms of leaving the
 *   surface.  We can't match 50 ms because the orientation classifier
 *   has its own 250 ms dwell; setting SURFACE_EXIT_DWELL to 0 makes
 *   the orientation dwell the only delay (~250 ms total), which is
 *   the closest we get to mouse-like tactile response.
 *
 *   This means SURFACE exits and starts cooldown almost the instant
 *   the user lifts their wrist, mimicking a mouse losing surface
 *   contact.  The 20 s cooldown then handles "they'll come back."
 */
#define AIR_MOUSE_EXIT_DWELL        50
#define SURFACE_EXIT_DWELL          0

/*
 * SURFACE motion-burst exit detector.
 *
 * Problem the orientation-drop exit alone doesn't solve: a wrist
 * palm-down on a desk and a wrist palm-down on the user's lap have
 * the SAME gravity-vector signature (both are Z+ dominant ->
 * WRIST_DOWN_FLAT).  So moving the wrist from desk to lap without
 * rotating the palm doesn't change orientation_current, and SURFACE
 * stays engaged when it should exit.
 *
 * We have no altimeter and accel dead-reckoning drifts in seconds,
 * so we can't measure "height."  But the TRANSITION from desk to
 * lap involves a brief acceleration burst (lift up, translate, set
 * down) -- 2-5 m/s^2 motion residual for ~500 ms.  Steady gliding
 * on the desk (the intended SURFACE use) stays below ~1 m/s^2.
 * That gap lets us detect "the user is transporting the wrist
 * somewhere" without false-triggering on actual surface use.
 *
 * Motion residual = (a - gravity_lpf), magnitude per sample.
 * SURFACE_MOTION_BURST_THRESH (m/s^2) is the gating magnitude;
 * SURFACE_MOTION_BURST_DWELL is the consecutive-sample count the
 * residual must stay above the threshold before firing.
 *
 * Initial values are physics-motivated estimates (a 30 cm vertical
 * translation in 0.5 s requires ~2.4 m/s^2 peak); empirical tuning
 * happens after first hardware test.  The exit log prints the
 * actual peak magnitude so the user can read it back and we can
 * tighten.
 */
#define SURFACE_MOTION_BURST_THRESH 2.0f
#define SURFACE_MOTION_BURST_DWELL  15      /* 150 ms at 100 Hz */

/*
 * Firmware multi-tap counter (Stage C, 2026-06-08).
 *
 * The chip is now configured in SDT=0 mode -- it emits ONLY
 * SINGLE_TAP events, one per shock.  Firmware groups consecutive
 * arrivals within MULTI_TAP_WINDOW_MS into multi-tap sequences:
 *   1 tap, no follow-up    -> currently no-op (surface-tap re-engage
 *                              will hook here in a later stage)
 *   2 taps within window   -> chained to gesture_mode_on_chip_double_tap
 *                              (AIR_MOUSE entry)
 *   3+ taps within window  -> chained to gesture_mode_on_chip_triple_tap
 *                              (SURFACE entry)
 *
 * The window value follows industry guidance (Agent 3 research:
 * iOS VoiceOver 250 ms default, Arduino 101 CurieIMU 250 ms,
 * Apple accessibility 500 ms).  250 ms is the human-comfortable
 * default.
 *
 * Commit-on-timeout pattern: each tap arrival updates last_tap_time
 * and increments count.  The per-sample update polls "has
 * MULTI_TAP_WINDOW_MS elapsed since the last tap?"  If yes, commits
 * the sequence and resets.  This gives the disambiguation latency
 * documented in Choice 3 (Stage A): action fires ~250 ms after the
 * last tap, accepting that delay so a third tap can still upgrade
 * the sequence.
 */
#define MULTI_TAP_WINDOW_MS         250

/*
 * Activity gate (Choice 4, Stage C, 2026-06-08).
 *
 * Reject chip-tap events if motion residual was high in the
 * preceding ~500 ms.  Kills the dominant false-positive source
 * documented in patent literature (gait impacts, wrist-on-surface
 * micro-contacts during typing).  See AN5040 / ADXL367 AN-2554.
 *
 * Implementation: each per-sample update where motion residual
 * exceeds ACTIVITY_GATE_THRESH resets samples_since_activity to 0.
 * Otherwise it ticks up.  A tap is accepted only if
 * samples_since_activity >= ACTIVITY_GATE_DWELL.
 */
/* Initial value was 1.0 m/s^2 -- too aggressive.  Hardware test
 * 2026-06-09 showed natural arm motion (lowering wrist from
 * AIR_MOUSE pose) was gating legitimate snap events.  Bumped to
 * 2.0 m/s^2 to match SURFACE_MOTION_BURST_THRESH (already
 * empirically validated as a clean separator between gliding and
 * transport motion).  Tune lower again only if gait-driven
 * false positives reappear. */
#define ACTIVITY_GATE_THRESH        2.0f   /* m/s^2 motion residual */
#define ACTIVITY_GATE_DWELL         50     /* 500 ms at 100 Hz */

/*
 * Per-mode entry-grace window after a cursor mode is entered while
 * NOT already in the expected pose.  Counted down per accel sample.
 * If it hits zero before the user reaches the expected pose, the
 * FSM bounces back to IDLE without ever arming exit detection.
 *
 * AIR_MOUSE_ENTRY_GRACE = 500 (5 s):
 *   Raising the arm to shoulder height takes deliberate effort plus
 *   settling time.  5 s leaves margin for repositioning, getting
 *   comfortable in the pose, or hesitation before commitment.
 *
 * SURFACE_ENTRY_GRACE = 300 (3 s):
 *   Placing the wrist flat on a desk is faster -- typically a single
 *   motion from wherever the hand was.  3 s is enough for the user
 *   to settle the wrist into a comfortable touchpad position.
 */
#define AIR_MOUSE_ENTRY_GRACE       500
#define SURFACE_ENTRY_GRACE         300

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

/* Pose-arm window: gesture must arrive within this many milliseconds
 * of pose-arm to count as a trigger. */
#define POSE_ARM_WINDOW_MS   3000

/* Minimum pose-match score to consider the user in a pose. */
#define POSE_MATCH_THRESH    0.5f

/* Multi-tap counter state.  See MULTI_TAP_WINDOW_MS comment block. */
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

/* Firmware ringing-rejection refractory.  At higher ODR the chip's
 * QUIET window shrinks proportionally (14.4 ms at 833 Hz vs ~29 ms
 * at 416 Hz).  Band housing mechanical ringing post-impact can
 * survive shorter QUIET windows and cause double-firing of the
 * same physical tap.  This firmware-side guard rejects any chip-
 * tap event arriving within RINGING_REFRACTORY_MS of the previous
 * one -- treating it as ringing duplicate, not a real second tap.
 *
 * 50 ms is comfortably above the ~3-30 ms ringing we observed at
 * 1.66 kHz, and well below the ~150-250 ms minimum human inter-tap
 * gap, so real double-taps are unaffected. */
#define RINGING_REFRACTORY_MS    50
static int64_t last_chip_tap_time_ms = 0;

/* Activity gate state.  See ACTIVITY_GATE_* comment block.
 * samples_since_activity counts up per accel sample (saturated at
 * ACTIVITY_GATE_DWELL since the precise high value doesn't matter
 * once the gate is open), and resets to 0 when motion residual
 * exceeds ACTIVITY_GATE_THRESH.  Initialised at the saturation
 * value so the gate is open at boot. */
static int     samples_since_activity = ACTIVITY_GATE_DWELL;

/* Motion-into-pose detection.  We track the last ~500 ms of motion-
 * residual samples to detect a "moved into pose" event: residual was
 * elevated, then dropped to near-zero, AND the current gravity vector
 * matches a canonical pose.
 *
 * 50 samples at 100 Hz = 500 ms ring buffer.  Each entry is the
 * motion-residual magnitude (already computed in update_accel). */
#define MOTION_HISTORY_SAMPLES   50
static float    motion_history_buf[MOTION_HISTORY_SAMPLES];
static int      motion_history_idx = 0;

/* Threshold above which a sample counts as "user moving into pose"
 * activity.  Tuned conservatively: well above sensor noise floor,
 * below sustained gait/typing motion. */
#define MOTION_INTO_POSE_ACTIVITY_THRESH   1.5f   /* m/s^2 */

/* Minimum activity samples to count as "user was just moving". */
#define MOTION_INTO_POSE_MIN_ACTIVE        5

/* Maximum residual to count as "now still in pose". */
#define MOTION_INTO_POSE_STILL_THRESH      0.4f   /* m/s^2 */

/* Number of recent samples that must be still to count as "settled". */
#define MOTION_INTO_POSE_STILL_DWELL       30     /* 300 ms at 100 Hz */

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
 * absolute angles.
 *
 * Captured here but NOT yet consumed in Item 0; cursor_pipeline.cpp
 * will read these once cursor tracking lands. */
static float cursor_reference_gx = 0.0f;
static float cursor_reference_gy = 0.0f;
static float cursor_reference_gz = 0.0f;
static bool  cursor_reference_valid = false;

/* Acquisition-request callback registered by main.cpp. */
static gesture_acq_request_cb_t s_acq_request_cb = NULL;

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

/*
 * --- Internal helpers --------------------------------------------------
 */

/* Returns true if the recent motion history shows a "moved into
 * pose" pattern: at least MOTION_INTO_POSE_MIN_ACTIVE older samples
 * had activity, AND the most recent MOTION_INTO_POSE_STILL_DWELL
 * samples are all still.
 *
 * This is the "user just moved and then settled" signal. */
static bool motion_into_pose_detected(void)
{
    int active_count = 0;
    int recent_still_count = 0;

    /* Walk the ring buffer from oldest to newest.  Index just before
     * motion_history_idx is the newest sample. */
    for (int i = 0; i < MOTION_HISTORY_SAMPLES; i++) {
        int real_idx = (motion_history_idx + i) % MOTION_HISTORY_SAMPLES;
        float r = motion_history_buf[real_idx];
        int samples_from_newest =
            MOTION_HISTORY_SAMPLES - 1 - i;  /* 0 = newest */

        if (samples_from_newest < MOTION_INTO_POSE_STILL_DWELL) {
            /* Tail of buffer = recent samples; require these still. */
            if (r < MOTION_INTO_POSE_STILL_THRESH) {
                recent_still_count++;
            }
        } else {
            /* Older part of buffer = where activity should have been. */
            if (r >= MOTION_INTO_POSE_ACTIVITY_THRESH) {
                active_count++;
            }
        }
    }

    return (active_count >= MOTION_INTO_POSE_MIN_ACTIVE) &&
           (recent_still_count >= MOTION_INTO_POSE_STILL_DWELL);
}

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

/* Update pose FSM based on current gravity vector.  Called every
 * accel sample from gesture_mode_update_accel().  Transitions:
 *   - NONE -> *_ARMED when motion-into-pose detected AND gravity
 *     matches a canonical pose
 *   - *_ARMED -> NONE handled lazily via pose_check_timeout() on
 *     every sample, or via explicit gesture acceptance (next task) */
static void pose_fsm_update(float gx, float gy, float gz)
{
    /* Flush expired arm first. */
    pose_check_timeout();

    /* Classify current gravity. */
    float score = 0.0f;
    pose_id_t best = pose_classify_best(gx, gy, gz,
                                          POSE_MATCH_THRESH, &score);

    if (pose_armed_state != POSE_NONE) {
        /* Already armed.  If the user is still in the SAME pose,
         * refresh the arm timestamp so they stay armed as long as
         * they hold the position.  Otherwise leave the existing
         * arm alone (don't break user intent on a transient
         * mis-classification or wrist wobble). */
        if (best == pose_armed_state) {
            pose_armed_time_ms = k_uptime_get();
        }
        return;
    }

    /* Not armed.  Require motion-into-pose AND a canonical pose
     * match to start arming. */
    if (!motion_into_pose_detected()) {
        return;
    }
    if (best == POSE_NONE) {
        return;
    }

    pose_armed_state = best;
    pose_armed_time_ms = k_uptime_get();
    LOG_INF("Pose ARMED: %s (score=%.2f, gravity=(%.2f, %.2f, %.2f))",
            pose_name(best), (double)score,
            (double)gx, (double)gy, (double)gz);
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
        cursor_reference_valid = false;
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
 * NOW and notify the registered callback on edges.  "Needs alive"
 * means EITHER (a) the current mode is a cursor mode that consumes
 * IMU samples, OR (b) a cooldown is currently open -- because the
 * cooldown countdown is driven by accel-sample callbacks and the
 * re-engage detector needs fresh orientation classifications to
 * notice the user returning to the expected pose.
 *
 * Called from _transition_to (mode changes) and from the cooldown
 * decrement site (cooldown crossing zero).  Both are real edges of
 * the "needs alive" predicate; both must notify. */
static void _update_acq_request(void)
{
    if (!s_acq_request_cb) {
        return;
    }
    GestureMode m = (GestureMode)atomic_get(&mode_atomic);
    bool now_needs = _mode_needs_continuous_imu(m) ||
                     cursor_cooldown_remaining > 0;
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
    cursor_reference_valid = false;
    cursor_reference_gx = 0.0f;
    cursor_reference_gy = 0.0f;
    cursor_reference_gz = 0.0f;
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
    samples_since_activity = ACTIVITY_GATE_DWELL;
    for (int i = 0; i < MOTION_HISTORY_SAMPLES; i++) {
        motion_history_buf[i] = 0.0f;
    }
    motion_history_idx = 0;
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
        /* Push into motion history ring buffer for motion-into-pose
         * detection. */
        motion_history_buf[motion_history_idx] = r_mag;
        motion_history_idx = (motion_history_idx + 1) % MOTION_HISTORY_SAMPLES;
    }

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

        /* Capture the engagement gravity vector as the reference plane.
         * For SURFACE this records the desk-plane orientation at the
         * instant of contact; for AIR_MOUSE it records the raised-pose
         * baseline.  Item 2 (cursor pipeline) will use this to compute
         * cursor motion relative to the engagement orientation rather
         * than absolute angles. */
        cursor_reference_gx = gx_filt;
        cursor_reference_gy = gy_filt;
        cursor_reference_gz = gz_filt;
        cursor_reference_valid = true;

        LOG_INF("%s: expected pose reached -- exit detection armed, "
                "ref g=[%.3f, %.3f, %.3f]",
                _mode_str(current_mode),
                (double)cursor_reference_gx,
                (double)cursor_reference_gy,
                (double)cursor_reference_gz);
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
     * rejection refractory, then activity gate, then folded into
     * a running count.  The k_work_delayable commit handler
     * commits the accumulated count to a 1/2/3-tap action
     * MULTI_TAP_WINDOW_MS after the last arrival. */

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
    last_chip_tap_time_ms = now;

    /* Activity gate: reject the event if motion residual was high
     * recently.  Kills gait + typing false-positives that the chip's
     * SHOCK+QUIET windows alone won't filter (those are debounce
     * within a tap, not "is this a tap or background motion"). */
    if (samples_since_activity < ACTIVITY_GATE_DWELL) {
        LOG_INF("Chip single-tap GATED by recent motion "
                "(%d ms ago, axis=%c sign=%c mode=%s)",
                samples_since_activity * 10,
                peak_axis, tap_sign,
                _mode_str(current_mode));
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
    if (multi_tap_count == 0) {
        return;
    }
    int n = multi_tap_count;
    char first_axis = multi_tap_first_axis;
    char first_sign = multi_tap_first_sign;
    multi_tap_count = 0;
    multi_tap_last_time_ms = 0;
    multi_tap_first_axis = '?';
    multi_tap_first_sign = '?';

    if (n == 1) {
        /* Lone single tap.  Currently no action -- the surface-tap
         * re-engage path will hook here later once feature
         * extraction (Stage E) can confirm the tap is a surface-tap
         * (not a stray band-tap). */
        LOG_INF("Multi-tap commit: 1 (lone single, axis=%c sign=%c "
                "-- no action wired yet)",
                first_axis, first_sign);
    } else if (n == 2) {
        LOG_INF("Multi-tap commit: 2 (axis=%c sign=%c -> AIR_MOUSE "
                "entry via chip-double-tap path)",
                first_axis, first_sign);
        gesture_mode_on_chip_double_tap();
    } else { /* 3 or more */
        LOG_INF("Multi-tap commit: %d (axis=%c sign=%c -> SURFACE "
                "entry via chip-triple-tap path)",
                n, first_axis, first_sign);
        gesture_mode_on_chip_triple_tap();
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
 * high (~0.7+); tap -> ratio lower (~0.5 or less). */
#define FFT_BIN_LOW_START   12     /* ~19.5 Hz */
#define FFT_BIN_LOW_END     122    /* ~199 Hz, exclusive */
#define FFT_BIN_MID_START   122    /* ~199 Hz */
#define FFT_BIN_MID_END     246    /* ~400 Hz, exclusive */

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

/* Simple classifier rules.  Returns:
 *   'S' = snap
 *   'B' = band-tap
 *   '?' = unclassified (genuinely ambiguous)
 *
 * v3 (2026-06-10): spectral-feature classifier.  Previous version
 * (pre_event_energy) was rejected because it conflates "user
 * positioning motion" with "gesture intent" -- see
 * principle_real_world_grounding_before_engineering memory.
 *
 * Snap (bone-conducted via carpal bones):
 *   - Tissue is a natural low-pass filter
 *   - Energy concentrated 20-200 Hz
 *   - low_band_ratio expected HIGH (>0.7)
 *
 * Band-tap (direct mechanical impact on housing):
 *   - Housing/PCB resonance up to 1 kHz+
 *   - Energy spread across our 20-400 Hz band
 *   - low_band_ratio expected LOWER (~0.5 if energy is even-ish
 *     across our two bands)
 *
 * Initial thresholds are physics-motivated; tune from hardware
 * data.  MIN_PEAK_SUM keeps weak events from being classified
 * confidently. */
#define SNAP_LOW_RATIO_THRESH    0.70f  /* low_band fraction; above
                                            this is confidently snap */
#define TAP_LOW_RATIO_THRESH     0.55f  /* below this is confidently
                                            tap; in-between is unclassified */
#define MIN_PEAK_SUM             20000  /* |x|+|y|+|z| at peak;
                                            below this is too weak
                                            to be a real tap/snap */
static char _classify(const struct tap_features *f)
{
    uint32_t total_abs = (uint32_t)f->peak_x_abs +
                         (uint32_t)f->peak_y_abs +
                         (uint32_t)f->peak_z_abs;
    if (total_abs < MIN_PEAK_SUM) {
        return '?';  /* too weak to characterise */
    }

    /* Primary: spectral band-energy ratio. */
    if (f->low_band_ratio >= SNAP_LOW_RATIO_THRESH) {
        return 'S';  /* snap */
    }
    if (f->low_band_ratio <= TAP_LOW_RATIO_THRESH) {
        return 'B';  /* band-tap */
    }
    return '?';  /* boundary zone */
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
        char verdict = _classify(&f);

        uint32_t total_abs = (uint32_t)f.peak_x_abs +
                             (uint32_t)f.peak_y_abs +
                             (uint32_t)f.peak_z_abs;
        uint32_t z_ratio_pct = total_abs ?
                                (uint32_t)f.peak_z_abs * 100 / total_abs :
                                0;

        const char *verdict_str = (verdict == 'S') ? "SNAP" :
                                  (verdict == 'B') ? "BAND-TAP" :
                                  "UNCLASSIFIED";

        /* Log the full feature vector so empirical tuning of the
         * spectral thresholds is visible.  ratio is the headline
         * discriminator; low/mid absolute energies confirm we have
         * meaningful signal (vs noise floor). */
        LOG_INF("[BIO] verdict=%s peak[%d,%d,%d] dom=%c z_ratio=%u%% "
                "low_band=%.0f mid_band=%.0f low_ratio=%.2f idx=%u/%u",
                verdict_str,
                (int)f.peak_x, (int)f.peak_y, (int)f.peak_z,
                f.dominant_axis,
                (unsigned)z_ratio_pct,
                (double)f.low_band_energy,
                (double)f.mid_band_energy,
                (double)f.low_band_ratio,
                (unsigned)f.peak_sample_idx, (unsigned)n);

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
