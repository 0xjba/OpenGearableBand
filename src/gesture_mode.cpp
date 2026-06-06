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
 * Cooldown window after exiting AIR_MOUSE via orientation-drop.
 * During this window, raising the wrist back into AIR_MOUSE pose
 * auto-re-engages without requiring another double-tap.  Outside
 * this window, an explicit double-tap is required.
 *
 * 300 samples = 3 s at 100 Hz acquisition rate.  Long enough to
 * accommodate brief arm-down moments (sip of coffee, scratch nose);
 * short enough that "I'm done air-mousing" intent is respected.
 */
#define AIR_MOUSE_COOLDOWN_SAMPLES  300

/*
 * Orientation dwell required during cooldown to re-engage.  Shorter
 * than the cold-start raise dwell because the user already signalled
 * intent recently.  50 samples = 500 ms.
 */
#define COOLDOWN_REENGAGE_DWELL     50

/*
 * Dwell required for orientation-drop exit from AIR_MOUSE.  Pose
 * must NOT be UP_RAISED for this many consecutive samples before we
 * transition out.  50 samples = 500 ms tolerates brief unintentional
 * tilts without bouncing out of the mode.
 */
#define AIR_MOUSE_EXIT_DWELL        50

/*
 * Flat-to-surface trigger: keep the original orientation-driven path
 * since SURFACE mode wasn't part of the double-tap-only discussion.
 * Subject to revision once the user calibrates real gravity vectors
 * for "palm on desk" and the team decides if SURFACE should also be
 * double-tap-gated for consistency.
 *
 * 100 samples = 1 s.
 */
#define FLAT_DWELL_SAMPLES          100


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
static int raise_dwell = 0;          /* used during cooldown re-engage */
static int flat_dwell = 0;
static int exit_dwell = 0;           /* non-UP_RAISED dwell while in AIR_MOUSE */

/* AIR_MOUSE cooldown: after an orientation-drop exit, allow quick
 * re-engage via raise-pose alone for AIR_MOUSE_COOLDOWN_SAMPLES.
 * Decremented every accel sample. */
static int air_mouse_cooldown_remaining = 0;

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
     * NOTE on the X/Y/Z conventions:
     *   - These mappings depend on how the XIAO board sits on the
     *     wrist.  Default mapping below is a *starting hypothesis*;
     *     it needs empirical calibration against the user's actual
     *     wrist mounting.  See gesture-architecture.md TODOs.
     *   - The procedure is documented in the field: hold the band in
     *     each intended pose, read the filtered gravity vector from
     *     the orientation transition log line, and update this
     *     classifier with the empirical mapping. */
    switch (largest_axis) {
    case 0:  /* X-axis dominant */
        /* Positive X dominant -> palm-down flat on a desk */
        return (largest_signed > 0.0f) ? WRIST_DOWN_FLAT : WRIST_NEUTRAL;
    case 1:  /* Y-axis dominant */
        /* Negative Y dominant -> forearm raised in air-mouse pose */
        return (largest_signed < 0.0f) ? WRIST_UP_RAISED : WRIST_NEUTRAL;
    case 2:  /* Z-axis dominant */
        /* Forearm vertical / wrist hanging.  Not a cursor-bearing
         * pose -- treat as NEUTRAL for now. */
        return WRIST_NEUTRAL;
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
    raise_dwell = 0;
    flat_dwell = 0;
    exit_dwell = 0;
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
    raise_dwell = 0;
    flat_dwell = 0;
    exit_dwell = 0;
    air_mouse_cooldown_remaining = 0;
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
    if (air_mouse_cooldown_remaining > 0) {
        air_mouse_cooldown_remaining--;
    }

    /* AIR_MOUSE entry from IDLE:
     *   PRIMARY:   gesture_mode_on_chip_double_tap() (called from the
     *              LSM6DSL chip-event GPIO callback, or the serial 't'
     *              test command).  Pose-agnostic; explicit user intent.
     *   SECONDARY: orientation goes to UP_RAISED + short dwell, but
     *              ONLY when the cooldown window is open (i.e. we
     *              recently left AIR_MOUSE due to an orientation drop
     *              and the user is bringing the band back up).  This
     *              avoids requiring a double-tap for brief tactical
     *              "lowered for 1 s, raising again" sequences.
     */
    if (current_mode == MODE_IDLE &&
        air_mouse_cooldown_remaining > 0 &&
        orientation_current == WRIST_UP_RAISED) {
        if (raise_dwell < COOLDOWN_REENGAGE_DWELL) {
            raise_dwell++;
            if (raise_dwell == COOLDOWN_REENGAGE_DWELL) {
                LOG_INF("Cooldown re-engage: %d ms remaining when fired",
                        air_mouse_cooldown_remaining * 10);
                _transition_to(MODE_AIR_MOUSE);
                air_mouse_cooldown_remaining = 0;
            }
        }
    } else {
        raise_dwell = 0;
    }

    /* AIR_MOUSE exit via orientation drop:
     * Pose is no longer UP_RAISED for AIR_MOUSE_EXIT_DWELL samples
     * straight -> transition back to IDLE and start the cooldown
     * window. */
    if (current_mode == MODE_AIR_MOUSE &&
        orientation_current != WRIST_UP_RAISED) {
        if (exit_dwell < AIR_MOUSE_EXIT_DWELL) {
            exit_dwell++;
            if (exit_dwell == AIR_MOUSE_EXIT_DWELL) {
                LOG_INF("AIR_MOUSE exit: orientation dropped, "
                        "starting %d ms re-engage cooldown",
                        AIR_MOUSE_COOLDOWN_SAMPLES * 10);
                _transition_to(MODE_IDLE);
                air_mouse_cooldown_remaining = AIR_MOUSE_COOLDOWN_SAMPLES;
            }
        }
    } else {
        exit_dwell = 0;
    }

    /* SURFACE entry: keep the original DOWN_FLAT-dwell path for now.
     * Subject to revision once we decide whether SURFACE should also
     * require a double-tap for consistency with AIR_MOUSE. */
    if (current_mode == MODE_IDLE &&
        orientation_current == WRIST_DOWN_FLAT) {
        if (flat_dwell < FLAT_DWELL_SAMPLES) {
            flat_dwell++;
            if (flat_dwell == FLAT_DWELL_SAMPLES) {
                _transition_to(MODE_SURFACE);
            }
        }
    } else {
        flat_dwell = 0;
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
    /* Toggle semantics:
     *   IDLE -> AIR_MOUSE       (primary intentional entry)
     *   AIR_MOUSE -> IDLE       (intentional exit, clears cooldown)
     *   SURFACE -> IDLE         (same -- band returns to neutral)
     *   GESTURE_AMBIENT -> IDLE (future-proofing)
     *
     * The user picked double-tap as the deliberate trigger because
     * the air-mouse pose, while distinctive, can occur in normal
     * activities (drawing on a whiteboard, pointing at a screen,
     * etc.) and shouldn't accidentally hijack their device.
     */
    if (current_mode == MODE_IDLE) {
        LOG_INF("Chip double-tap from IDLE -- entering AIR_MOUSE");
        _transition_to(MODE_AIR_MOUSE);
        air_mouse_cooldown_remaining = 0;  /* explicit entry skips cooldown */
    } else {
        LOG_INF("Chip double-tap from %s -- exiting to IDLE (no cooldown)",
                _mode_str(current_mode));
        _transition_to(MODE_IDLE);
        /* Explicit exit -- no cooldown.  User wanted out, give them
         * out.  They can re-enter via a fresh double-tap. */
        air_mouse_cooldown_remaining = 0;
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

int gesture_mode_get_air_mouse_cooldown_remaining(void)
{
    return air_mouse_cooldown_remaining;
}
