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
 * Threshold for declaring an axis "dominant" in the gravity vector.
 * Gravity is ~9.81 m/s^2 total.  Requiring one axis to carry at
 * least 7.5 m/s^2 (~76 % of gravity) gives a clear "this axis points
 * up/down" classification while rejecting tilted positions.
 */
#define DOMINANT_AXIS_THRESHOLD_MS2 7.5f

/*
 * Raise-to-air trigger: orientation must transition NEUTRAL -> UP_RAISED
 * and dwell for RAISE_DWELL_SAMPLES before mode auto-changes.
 *
 * 50 samples = 500 ms.  Short enough to feel responsive, long enough
 * that a casual wrist movement does not accidentally enter AIR_MOUSE.
 */
#define RAISE_DWELL_SAMPLES         50

/*
 * Flat-to-surface trigger: same idea but longer dwell because we
 * don't want every "rest my hand on the desk" moment to enter
 * surface mode.  100 samples = 1 s.
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
static int raise_dwell = 0;
static int flat_dwell = 0;

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

    /* No axis dominant -> NEUTRAL. */
    if (ax < DOMINANT_AXIS_THRESHOLD_MS2 &&
        ay < DOMINANT_AXIS_THRESHOLD_MS2 &&
        az < DOMINANT_AXIS_THRESHOLD_MS2) {
        return WRIST_NEUTRAL;
    }

    /* Find the dominant axis. */
    if (ax >= ay && ax >= az) {
        /* X-axis dominant.  Positive X = palm-down flat (DOWN_FLAT). */
        return (gx > 0.0f) ? WRIST_DOWN_FLAT : WRIST_NEUTRAL;
    }
    if (ay >= az) {
        /* Y-axis dominant.  Negative Y = forearm raised (UP_RAISED). */
        return (gy < 0.0f) ? WRIST_UP_RAISED : WRIST_NEUTRAL;
    }
    /* Z-axis dominant -- forearm vertical with palm to side or
     * hanging.  Treat as NEUTRAL for now; future refinement may add
     * additional orientations. */
    return WRIST_NEUTRAL;
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

    /* --- Trigger gestures --- */

    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);

    /* Raise-to-air-mouse: must be in IDLE, see UP_RAISED, dwell. */
    if (current_mode == MODE_IDLE &&
        orientation_current == WRIST_UP_RAISED) {
        if (raise_dwell < RAISE_DWELL_SAMPLES) {
            raise_dwell++;
            if (raise_dwell == RAISE_DWELL_SAMPLES) {
                _transition_to(MODE_AIR_MOUSE);
            }
        }
    } else {
        raise_dwell = 0;
    }

    /* Flat-to-surface: must be in IDLE, see DOWN_FLAT, dwell. */
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
    /* Semantics: double-tap from IDLE does nothing (the orientation
     * triggers handle that).  Double-tap from any other mode cancels
     * back to IDLE -- the user's "get me out of this" panic button.
     * This is intentionally aggressive; users can re-enter modes
     * with the orientation triggers easily. */
    if (current_mode != MODE_IDLE) {
        LOG_INF("Chip double-tap -- cancelling current mode to IDLE");
        _transition_to(MODE_IDLE);
    } else {
        LOG_INF("Chip double-tap (in IDLE, no-op)");
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
