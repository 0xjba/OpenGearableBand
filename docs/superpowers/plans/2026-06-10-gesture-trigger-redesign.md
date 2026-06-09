# Gesture Trigger Redesign — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace single-event chip-tap trigger with a pose-gated two-stage trigger model (motion-into-pose + dwell + armed gesture window) so that incidental impulse events (skin taps near band, desk slaps, lap fidgets) do not falsely enter cursor modes.

**Architecture:** A new pose state machine wraps the existing chip-tap path. Pose detection (3 canonical poses with tolerance cones) runs continuously in the accel pipeline; when a motion-into-pose transition is detected, the chip-tap path is armed for 3 seconds. Mode entry requires both pose-armed state AND a matching gesture (single-tap for AIR_MOUSE/DICTATION; cadenced double-tap + spectral surface confirmation for SURFACE). Existing FFT pipeline is repurposed for desk-vs-lap surface check.

**Tech Stack:** Zephyr RTOS on nRF52840 (Xiao BLE Sense), Nordic Connect SDK, CMSIS-DSP for FFT, LSM6DSL accel/gyro (continuous FIFO at 833 Hz), MAX30102 PPG (off at trigger).

**Testing approach:** This codebase has no unit-test framework; testing has been hardware-in-the-loop throughout. Each task's "test" step is a specific hardware test procedure with expected log-line output. Setting up Zephyr ztest is a future improvement not blocking this work.

**Spec:** [`docs/superpowers/specs/2026-06-10-gesture-trigger-redesign-design.md`](../specs/2026-06-10-gesture-trigger-redesign-design.md)

---

## File Map

**Create:**
- `src/gesture_poses.h` — canonical pose definitions, tolerance cones, public API
- `src/gesture_poses.cpp` — `pose_score()` + `pose_classify_best()` implementations

**Modify:**
- `src/gesture_mode.h` — pose FSM public API (state enum, arm/disarm/query functions)
- `src/gesture_mode.cpp` — motion-into-pose detector, pose FSM state, integration with existing chip-tap path, cadenced double-tap detector, surface-spectral confirmation
- `src/main.cpp:1037-1078` — call into pose FSM for diagnostics; no behaviour change in dispatch (the gating moves into gesture_mode.cpp)
- `docs/research/gesture-architecture.md` — update architecture section to reflect pose-gated trigger

**Deprecate (remove or move):**
- The trigger-time classifier `_classify()` verdict path in `src/gesture_mode.cpp` — keep FFT and feature extraction (repurposed for surface check) but stop using `low_band_ratio` for snap-vs-tap.

---

## Task Order and Rationale

The tasks build bottom-up:
1. Pose math primitives (testable in isolation by log inspection)
2. Motion-into-pose detector (reads gravity history, no FSM yet)
3. Pose FSM with arm/disarm and timeout (no integration yet)
4. Wire pose FSM into the accel update loop (logs pose transitions)
5. Cadenced double-tap detection (modifies multi-tap counter)
6. Surface-spectral confirmation (uses existing FFT output)
7. Gate chip-tap path through pose-armed state
8. Trigger mode entry from armed-pose + matched gesture
9. Cleanup: deprecate snap-vs-tap trigger classifier
10. Update architecture doc + end-to-end hardware test

Each task ends with a hardware test and a commit.

---

## Task 1: Pose definitions and pose-score primitive

**Files:**
- Create: `src/gesture_poses.h`
- Create: `src/gesture_poses.cpp`
- Modify: `CMakeLists.txt` (add new .cpp to target_sources)

**Rationale:** Establish the canonical pose data structures and the continuous-score computation. Pure math, easiest to verify by visual inspection of test log values.

- [ ] **Step 1: Inspect CMakeLists.txt to see target_sources pattern**

Run: `grep -n "target_sources\|src/" CMakeLists.txt`
Expected: a list of existing src/*.cpp files added to the app target. Use the same pattern.

- [ ] **Step 2: Create `src/gesture_poses.h`**

```c
#ifndef GESTURE_POSES_H
#define GESTURE_POSES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pose identifiers.  POSE_NONE means no canonical pose currently
 * matched. */
typedef enum {
    POSE_NONE = 0,
    POSE_AIR_MOUSE,     /* forearm raised forward, band volar facing
                         * screen (away from user's face) */
    POSE_DICTATION,     /* forearm raised + rotated, band volar facing
                         * user's mouth */
    POSE_SURFACE,       /* wrist horizontal, band volar facing up
                         * (palm-down rest) */
    POSE_COUNT
} pose_id_t;

/* Canonical pose definition.  Gravity vector points DOWNWARD when
 * the wrist is in the canonical pose; we store the expected gravity
 * vector in band-frame g-units (±9.81 m/s^2 normalised to ±1.0).
 *
 * tolerance_cos is the minimum cosine of the angle between observed
 * gravity and canonical gravity to still count as a match.  Higher
 * value = tighter tolerance.  cos(30°) ≈ 0.866, cos(20°) ≈ 0.940.
 *
 * Numbers below are hand-tuned defaults for the developer's current
 * setup.  Productionization will replace these with per-user
 * calibration values from NVS. */
typedef struct {
    pose_id_t id;
    float gx;             /* canonical gravity x in band frame, g-units */
    float gy;             /* canonical gravity y */
    float gz;             /* canonical gravity z */
    float tolerance_cos;  /* min cos(angle) to canonical for match */
    const char *name;     /* for logging */
} canonical_pose_t;

/* Compute the match score for a given pose, given observed gravity
 * vector in band frame.  Returns continuous score in [0, 1]:
 *   - 1.0 = observed gravity is exactly the canonical
 *   - 0.0 = observed is outside the tolerance cone
 *   - intermediate = continuous falloff inside the cone
 *
 * Score formula: max(0, (cos_angle - tolerance_cos) /
 *                       (1.0 - tolerance_cos)).
 * This is 0 at the cone boundary and 1 at the canonical direction. */
float pose_score(const canonical_pose_t *p,
                 float gx, float gy, float gz);

/* Find the best-matching canonical pose for the observed gravity
 * vector.  Returns POSE_NONE if no pose has score above
 * pose_match_threshold (typically 0.5).  out_score (optional)
 * receives the winning score. */
pose_id_t pose_classify_best(float gx, float gy, float gz,
                              float pose_match_threshold,
                              float *out_score);

/* Access the canonical pose definitions (read-only).  Used by the
 * pose FSM. */
const canonical_pose_t *pose_get_canonical(pose_id_t id);
const char *pose_name(pose_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_POSES_H */
```

- [ ] **Step 3: Create `src/gesture_poses.cpp`**

```c
#include "gesture_poses.h"

#include <math.h>
#include <stddef.h>

/* Hand-tuned canonical pose definitions.
 *
 * IMPORTANT (productionization): these values are tuned for the
 * developer's current setup (open PCB, duct-tape mount).  They WILL
 * need recalibration when housing arrives and per-user when the
 * calibration ritual ships.  See
 * project_productionization_gesture_calibration memory. */
static const canonical_pose_t k_canonical_poses[POSE_COUNT] = {
    /* POSE_NONE: never matches; placeholder. */
    { POSE_NONE,      0.0f,  0.0f,  0.0f,  2.0f, "NONE" },

    /* POSE_AIR_MOUSE: forearm raised forward, band volar facing
     * screen (away from user's face).  Gravity vector is along the
     * band's NEGATIVE Y axis (pointing toward the floor through the
     * band's edge).  Tolerance ±30°. */
    { POSE_AIR_MOUSE, 0.0f, -1.0f,  0.0f,  0.866f, "AIR_MOUSE" },

    /* POSE_DICTATION: forearm raised + rotated, band volar facing
     * user's mouth.  Compared to AIR_MOUSE this is a roll about the
     * forearm axis -- band Y axis still points downward but band X
     * has rotated; gravity now has a non-trivial X component.
     * Approx (gx, gy, gz) = (+0.5, -0.85, 0).  Tolerance ±30°.
     *
     * Empirically verify and re-tune during hardware integration. */
    { POSE_DICTATION, 0.5f, -0.866f, 0.0f, 0.866f, "DICTATION" },

    /* POSE_SURFACE: wrist horizontal, band volar facing up (palm-
     * down rest position).  Gravity is along band POSITIVE Z (the
     * band is being "looked at from above").  Tolerance tighter
     * (±20°) to reduce lap false positives. */
    { POSE_SURFACE,   0.0f,  0.0f,  1.0f,  0.940f, "SURFACE" },
};

float pose_score(const canonical_pose_t *p,
                 float gx, float gy, float gz)
{
    if (p == NULL || p->id == POSE_NONE) return 0.0f;

    /* Normalise observed gravity. */
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (mag < 0.5f) {
        /* Free-fall or sensor problem; can't classify. */
        return 0.0f;
    }
    float ux = gx / mag;
    float uy = gy / mag;
    float uz = gz / mag;

    /* Canonical is already normalised (unit vector). */
    float cos_angle = ux * p->gx + uy * p->gy + uz * p->gz;

    if (cos_angle <= p->tolerance_cos) {
        return 0.0f;  /* outside tolerance cone */
    }
    return (cos_angle - p->tolerance_cos) /
           (1.0f - p->tolerance_cos);
}

pose_id_t pose_classify_best(float gx, float gy, float gz,
                              float pose_match_threshold,
                              float *out_score)
{
    pose_id_t best = POSE_NONE;
    float best_score = 0.0f;

    for (int i = 1; i < POSE_COUNT; i++) {  /* skip POSE_NONE */
        float s = pose_score(&k_canonical_poses[i], gx, gy, gz);
        if (s > best_score) {
            best_score = s;
            best = (pose_id_t)i;
        }
    }

    if (out_score) *out_score = best_score;
    if (best_score < pose_match_threshold) return POSE_NONE;
    return best;
}

const canonical_pose_t *pose_get_canonical(pose_id_t id)
{
    if (id <= POSE_NONE || id >= POSE_COUNT) return NULL;
    return &k_canonical_poses[id];
}

const char *pose_name(pose_id_t id)
{
    if (id <= POSE_NONE || id >= POSE_COUNT) return "?";
    return k_canonical_poses[id].name;
}
```

- [ ] **Step 4: Add `src/gesture_poses.cpp` to CMakeLists.txt**

Modify the existing `target_sources(app PRIVATE ...)` block (find it via Step 1) and add `src/gesture_poses.cpp` alongside the other source files. Preserve alphabetical / existing order.

- [ ] **Step 5: Build to confirm compilation**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`

Expected: build completes successfully. FLASH usage increases by < 1 KB (just pose data + small functions).

- [ ] **Step 6: Commit**

```bash
git add src/gesture_poses.h src/gesture_poses.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
poses: add canonical pose definitions + continuous score primitive

Defines three canonical poses (AIR_MOUSE, DICTATION, SURFACE) with
tolerance cones (±30° for raises, ±20° for SURFACE) and a continuous
pose_score() function that returns a soft match score rather than a
binary in/out-of-pose verdict.

Values are hand-tuned for the developer's current setup (open PCB,
duct-tape mount).  Per-user calibration via NVS is a productionization
item (project_productionization_gesture_calibration memory).

Pure math; no integration with the gesture FSM yet -- that lands in
later tasks.
EOF
)"
```

---

## Task 2: Motion-into-pose detector

**Files:**
- Modify: `src/gesture_mode.cpp` — add a short ring buffer of motion-residual samples and a `motion_into_pose_detected()` predicate

**Rationale:** The pose state machine arms only when the user MOVED into the pose recently. This task adds the detector but doesn't wire it to the FSM yet.

- [ ] **Step 1: Find existing motion-residual code in gesture_mode.cpp**

Run: `grep -n "motion residual\|ACTIVITY_GATE_THRESH\|samples_since_activity" src/gesture_mode.cpp | head`
Expected: existing per-sample motion-residual computation in `gesture_mode_update_accel()`. We'll reuse this same residual signal.

- [ ] **Step 2: Add motion-trajectory ring buffer state in gesture_mode.cpp**

Add near the existing activity gate state declarations (around line 327):

```c
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
```

- [ ] **Step 3: Add ring buffer push in `gesture_mode_update_accel()`**

Locate the motion-residual computation in `gesture_mode_update_accel()` (around line 770 where `motion_residual` is computed). Right after the residual is computed but before any other use:

```c
/* Push into motion history ring buffer for motion-into-pose
 * detection. */
motion_history_buf[motion_history_idx] = motion_residual;
motion_history_idx = (motion_history_idx + 1) % MOTION_HISTORY_SAMPLES;
```

(Confirm the existing variable name is `motion_residual` — if it's different in the actual code, use the right name.)

- [ ] **Step 4: Add `motion_into_pose_detected()` helper**

Below the ring buffer declarations and above any function that uses it:

```c
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
```

- [ ] **Step 5: Reset the ring buffer in `gesture_mode_init()`**

Find `gesture_mode_init()` (around line 660). Add inside the init block:

```c
for (int i = 0; i < MOTION_HISTORY_SAMPLES; i++) {
    motion_history_buf[i] = 0.0f;
}
motion_history_idx = 0;
```

- [ ] **Step 6: Build to confirm compilation**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build, RAM usage increases by ~200 bytes for the ring buffer.

- [ ] **Step 7: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: add motion-into-pose detector helper

Adds a 500 ms ring buffer of motion-residual samples and a
motion_into_pose_detected() predicate that returns true when older
samples show activity AND recent samples are still.

This is the "user just moved into a position and settled" signal that
the pose FSM (next task) will combine with pose-score matching to arm
the gesture detector.  Not wired to the FSM yet -- pure helper.
EOF
)"
```

---

## Task 3: Pose state machine (NONE / *_ARMED + timeout)

**Files:**
- Modify: `src/gesture_mode.h` — public API for pose state queries
- Modify: `src/gesture_mode.cpp` — pose FSM state + transitions

**Rationale:** Adds the state-machine layer above the pose primitives. Doesn't yet gate chip-tap path; just tracks state.

- [ ] **Step 1: Add pose FSM public API to `src/gesture_mode.h`**

Add near the other public declarations (around line 110):

```c
#include "gesture_poses.h"

/* Pose state machine query.  Returns the currently-armed pose, or
 * POSE_NONE if no pose is armed.  Used by the chip-tap path to
 * decide whether to accept a tap as a mode-entry trigger. */
pose_id_t gesture_mode_armed_pose(void);
```

- [ ] **Step 2: Add pose FSM state in `src/gesture_mode.cpp`**

Add near the multi-tap state declarations:

```c
#include "gesture_poses.h"

/* Pose FSM state.  Updated each accel sample from update_accel. */
static pose_id_t pose_armed_state = POSE_NONE;
static int64_t   pose_armed_time_ms = 0;

/* Pose-arm window: gesture must arrive within this many milliseconds
 * of pose-arm to count as a trigger. */
#define POSE_ARM_WINDOW_MS   3000

/* Minimum pose-match score to consider the user in a pose. */
#define POSE_MATCH_THRESH    0.5f
```

- [ ] **Step 3: Implement `gesture_mode_armed_pose()`**

Add as a public function:

```c
pose_id_t gesture_mode_armed_pose(void)
{
    if (pose_armed_state == POSE_NONE) return POSE_NONE;

    /* Auto-disarm if window expired. */
    int64_t now = k_uptime_get();
    if ((now - pose_armed_time_ms) > POSE_ARM_WINDOW_MS) {
        LOG_INF("Pose arm expired (%s, %d ms elapsed) -> NONE",
                pose_name(pose_armed_state),
                (int)(now - pose_armed_time_ms));
        pose_armed_state = POSE_NONE;
        return POSE_NONE;
    }
    return pose_armed_state;
}
```

- [ ] **Step 4: Add internal `pose_fsm_update()` helper**

Below `motion_into_pose_detected()`:

```c
/* Update pose FSM based on current gravity vector.  Called every
 * accel sample from gesture_mode_update_accel().  Transitions:
 *   - NONE -> *_ARMED when motion-into-pose detected AND gravity
 *     matches a canonical pose
 *   - *_ARMED -> NONE handled lazily in gesture_mode_armed_pose()
 *     via timeout, or via explicit gesture acceptance (next task) */
static void pose_fsm_update(float gx, float gy, float gz)
{
    /* Lazy timeout: re-query to flush expired arm. */
    (void)gesture_mode_armed_pose();

    if (pose_armed_state != POSE_NONE) {
        /* Already armed; nothing to do here. */
        return;
    }

    /* Check both conditions: motion-into-pose recently AND in a
     * canonical pose now.  motion_into_pose_detected() looks at the
     * ring buffer; pose_classify_best() looks at the current gravity
     * vector. */
    if (!motion_into_pose_detected()) {
        return;  /* not moving into anything */
    }

    float score = 0.0f;
    pose_id_t best = pose_classify_best(gx, gy, gz,
                                          POSE_MATCH_THRESH, &score);
    if (best == POSE_NONE) {
        return;  /* not in a canonical pose */
    }

    pose_armed_state = best;
    pose_armed_time_ms = k_uptime_get();
    LOG_INF("Pose ARMED: %s (score=%.2f, gravity=(%.2f, %.2f, %.2f))",
            pose_name(best), (double)score,
            (double)gx, (double)gy, (double)gz);
}
```

- [ ] **Step 5: Reset pose FSM state in `gesture_mode_init()`**

Add inside the init block:

```c
pose_armed_state = POSE_NONE;
pose_armed_time_ms = 0;
```

- [ ] **Step 6: Build to confirm compilation**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 7: Commit**

```bash
git add src/gesture_mode.h src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: add pose FSM (NONE / *_ARMED) with 3 second timeout

Adds the pose state machine that arms when motion-into-pose is
detected AND the current gravity vector matches a canonical pose.
The armed state lasts up to POSE_ARM_WINDOW_MS (3 sec); after that
gesture_mode_armed_pose() lazily disarms on next query.

Not wired to the accel update loop yet -- pose_fsm_update() exists
but no caller.  Next task wires it in.
EOF
)"
```

---

## Task 4: Wire pose FSM into the accel update loop

**Files:**
- Modify: `src/gesture_mode.cpp` — call `pose_fsm_update()` once per accel sample

**Rationale:** This is the first user-visible change. After flashing, the logs should show `Pose ARMED:` messages when the user moves into a canonical pose.

- [ ] **Step 1: Locate the place in `gesture_mode_update_accel()` where gravity-LPF is finalised**

Run: `grep -n "gravity_lpf_x\|gravity_lpf_y\|gravity_lpf_z" src/gesture_mode.cpp | head`
Expected: gravity_lpf_x/y/z are updated each call, and there's a place where they're "settled" enough to use.

- [ ] **Step 2: Insert the pose FSM update call**

After the gravity LPF has been updated for this sample, add:

```c
/* Update pose state machine.  Uses gravity_lpf for the canonical-
 * pose comparison and the motion-history ring buffer for the
 * motion-into-pose check. */
pose_fsm_update(gravity_lpf_x, gravity_lpf_y, gravity_lpf_z);
```

Place this AFTER the motion-history ring buffer push (from Task 2 Step 3) and AFTER any gravity-LPF settling logic. Exact line depends on existing code structure; the constraint is "called after both motion_residual AND gravity_lpf are valid for this sample."

- [ ] **Step 3: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 4: Hardware test — pose arming visible in logs**

Flash `build/zephyr/zephyr.uf2` to the band.

Test procedure (5 trials each):

1. **AIR_MOUSE pose**: from a resting position, deliberately raise the wrist forward with band volar facing the screen, hold ~1 second. Watch for log line:
   `Pose ARMED: AIR_MOUSE (score=X.XX, gravity=(...))`
   Expected: arms every time. Score should be > 0.5.

2. **DICTATION pose**: raise wrist and rotate toward mouth. Watch for:
   `Pose ARMED: DICTATION (score=X.XX, ...)`

3. **SURFACE pose**: lower wrist to desk surface, palm-down rest. Watch for:
   `Pose ARMED: SURFACE (score=X.XX, ...)`

4. **Negative test (no motion)**: hold each pose statically (don't move into it; have your wrist already in the pose before boot, or hold for >2 seconds before checking). Expected: NO arm-log fires because motion-into-pose wasn't detected.

5. **Negative test (random motion)**: wave arm around without settling into any canonical pose. Expected: NO arm-log fires because no canonical pose matched.

6. **Timeout test**: arm a pose, then wait 4 seconds without doing anything. Watch for:
   `Pose arm expired (...)`

Report findings: which poses armed reliably, which were unreliable, any tuning needed for canonical pose values or tolerance cones.

- [ ] **Step 5: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: wire pose FSM into accel update loop

Calls pose_fsm_update() each accel sample with the gravity LPF and
the motion-history ring buffer state.  Pose transitions are logged
so empirical tuning of tolerance/canonical values is visible.

Hardware testing required to confirm pose arming triggers reliably
and pose tolerances are right.  Canonical pose values in
gesture_poses.cpp are hand-tuned defaults; may need adjustment.
EOF
)"
```

---

## Task 5: Cadenced double-tap detector for SURFACE

**Files:**
- Modify: `src/gesture_mode.cpp` — add inter-tap timing constraint helper

**Rationale:** SURFACE mode entry requires a deliberate double-tap with 150-300 ms inter-tap spacing. Reuses existing multi-tap counter, adds a timing filter.

- [ ] **Step 1: Add cadence constants**

Near the existing `MULTI_TAP_WINDOW_MS` constant in gesture_mode.cpp:

```c
/* Cadenced double-tap: deliberate user-driven double-tap with inter-
 * tap interval in a specific window.  Two impacts within
 * MULTI_TAP_WINDOW_MS but with at least CADENCE_MIN_MS apart and at
 * most CADENCE_MAX_MS apart.  Apple VoiceOver convention is ~250 ms;
 * we accept ±100 ms around that. */
#define CADENCE_MIN_MS   150
#define CADENCE_MAX_MS   300
```

- [ ] **Step 2: Add `last_two_tap_interval_ms()` helper**

Add near the multi-tap counter logic:

```c
/* Time elapsed (ms) between the previous chip-tap event and the
 * one before it.  Used for cadenced-double-tap detection.  Returns
 * 0 if there aren't two recent events.  Set by
 * gesture_mode_on_chip_single_tap. */
static int64_t prev_chip_tap_time_ms = 0;

static int last_two_tap_interval_ms(void)
{
    int64_t now = k_uptime_get();
    /* The CURRENT tap arrives at "now"; the PREVIOUS one is at
     * prev_chip_tap_time_ms (set by the caller after this query if
     * the gesture is being processed). */
    if (prev_chip_tap_time_ms == 0) return 0;
    return (int)(now - prev_chip_tap_time_ms);
}

static bool is_cadenced_double_tap_window(int interval_ms)
{
    return (interval_ms >= CADENCE_MIN_MS) &&
           (interval_ms <= CADENCE_MAX_MS);
}
```

- [ ] **Step 3: Update `gesture_mode_on_chip_single_tap` to track previous tap time**

Find where the function updates `last_chip_tap_time_ms` (the ringing-rejection refractory state, from earlier work). After processing the current tap as a valid event, before incrementing the multi-tap counter, save the previous time:

```c
prev_chip_tap_time_ms = last_chip_tap_time_ms;  /* push current to "previous" */
/* (last_chip_tap_time_ms is then updated to `now` by existing logic) */
```

Place this BEFORE the existing `last_chip_tap_time_ms = now;` line.

- [ ] **Step 4: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 5: Reset `prev_chip_tap_time_ms = 0` in `gesture_mode_init()`**

Add the reset.

- [ ] **Step 6: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: add cadenced double-tap timing primitive

Adds inter-tap interval tracking and a is_cadenced_double_tap_window()
predicate.  SURFACE entry will require two consecutive chip-tap events
with 150-300 ms spacing (Apple VoiceOver-style cadence) -- this rules
out incidental single impacts on lap rest.

Helper only; not yet used by mode-entry path.
EOF
)"
```

---

## Task 6: Surface-spectral confirmation (desk-feedback ringing)

**Files:**
- Modify: `src/gesture_mode.cpp` — add a "last tap was on hard surface" predicate using the existing FFT band-energy features

**Rationale:** When the user taps on a hard surface (desk), the impact energy feeds back through the desk → hand → wrist → IMU, producing additional mid-band spectral content. On lap (soft tissue + fabric), this feedback is absorbed. We already compute `mid_band_energy` in the FFT pipeline; just need a threshold check.

- [ ] **Step 1: Add the threshold constant**

In the section where `SNAP_LOW_RATIO_THRESH` and friends are defined:

```c
/* Surface-spectral confirmation: when a tap occurs with the wrist
 * resting on a HARD surface (desk, glass table), the impact energy
 * reflects back through the surface → hand → wrist → IMU, producing
 * detectable mid-band content.  On a SOFT surface (lap, jeans), this
 * feedback is absorbed and mid-band stays low.
 *
 * Empirical threshold from desk-tap-vs-lap-tap data collection (to
 * be tuned during integration test).  Conservative initial value.
 *
 * IMPORTANT: this signal is housing-dependent.  Current value tuned
 * for open PCB; will need recalibration when housing arrives. */
#define SURFACE_RESONANCE_MID_BAND_THRESH   5e7f
```

- [ ] **Step 2: Add module state to record the most recent tap's mid_band energy**

Near the FFT-related declarations:

```c
/* Most recent tap's mid_band_energy, updated by bio_acoustic_worker
 * after each FFT.  Read by surface_spectral_confirms_hard_surface()
 * to gate SURFACE mode entry. */
static float last_tap_mid_band_energy = 0.0f;
```

- [ ] **Step 3: Save mid_band in the bio_acoustic_worker**

Find `bio_acoustic_worker` (already computes `f.mid_band_energy`). Right after the LOG_INF that prints the features:

```c
last_tap_mid_band_energy = f.mid_band_energy;
```

- [ ] **Step 4: Add the predicate**

```c
/* Returns true if the most recently captured tap shows desk-feedback
 * spectral content (mid_band above threshold).  Used to confirm
 * SURFACE entry. */
static bool surface_spectral_confirms_hard_surface(void)
{
    return last_tap_mid_band_energy >= SURFACE_RESONANCE_MID_BAND_THRESH;
}
```

- [ ] **Step 5: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: add surface-spectral confirmation predicate

Adds surface_spectral_confirms_hard_surface() using mid_band_energy
from the existing FFT pipeline.  Hard surfaces (desk, glass) reflect
tap energy back through the body, producing mid-band content; soft
surfaces (lap, jeans) absorb it.

Threshold (5e7) is conservative initial guess; needs hardware
calibration on desk-tap vs lap-tap.  Housing-dependent -- will need
recalibration when production hardware arrives.

Predicate only; mode-entry integration is next task.
EOF
)"
```

---

## Task 7: Gate chip-tap path through pose-armed state

**Files:**
- Modify: `src/gesture_mode.cpp` — modify `gesture_mode_on_chip_single_tap()` to require pose-armed state for mode entry

**Rationale:** This is the core safety gate. Chip-tap events without an armed pose should NOT trigger mode entry. They still fire (still logged), but they don't proceed to mode change.

- [ ] **Step 1: Inspect current `gesture_mode_on_chip_single_tap` structure**

Run: `grep -n "void gesture_mode_on_chip_single_tap\|Multi-tap commit:" src/gesture_mode.cpp | head -10`
Find the function entry and the existing multi-tap commit handler.

- [ ] **Step 2: Add pose-armed gate near the top of `gesture_mode_on_chip_single_tap`**

After the existing ringing-rejection and activity-gate checks, BEFORE the multi-tap accumulation:

```c
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
```

(Important: leave the multi-tap counter logic and existing ringing/activity checks untouched. Only the new pose gate is added.)

- [ ] **Step 3: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 4: Hardware test — chip-tap WITHOUT pose is ignored**

Flash and test:

1. **Without entering a pose**, tap the band directly. Expected log:
   `Chip single-tap IGNORED: no pose armed (...)`
   No multi-tap commit, no mode change.

2. **Tap skin near the band, slap desk, slap thigh** — same behaviour: IGNORED log lines, no mode change.

3. **Enter AIR_MOUSE pose (raise wrist deliberately), then within 3 sec, tap the band**. Expected logs in order:
   ```
   Pose ARMED: AIR_MOUSE (score=...)
   Chip single-tap: axis=... (sequence count=1, ...)
   Multi-tap commit: 1 (...)
   ```
   The tap proceeds through the existing path (no mode change yet — that wires up in next task).

4. **Wait 4 seconds (let pose arm expire), then tap**. Expected: `Pose arm expired ...` followed by `Chip single-tap IGNORED ...`.

This test confirms the gate works. No mode-entry is wired up yet, but the gate prevents false triggers.

- [ ] **Step 5: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: gate chip-tap path through pose-armed state

Adds a pose check at the top of gesture_mode_on_chip_single_tap().
Without an armed pose, the chip-tap is logged for diagnostics but
does NOT advance multi-tap state or trigger mode entry.

This is the core defence against incidental impulse events (skin
taps near band, desk slaps, thigh slaps, lap fidgets) -- they all
still fire the chip but get IGNORED at the firmware layer because
no pose was armed first.
EOF
)"
```

---

## Task 8: Wire mode entry from armed-pose + matched gesture

**Files:**
- Modify: `src/gesture_mode.cpp` — extend the multi-tap commit handler to trigger mode entry based on armed pose + gesture-shape match

**Rationale:** This is where mode entry actually happens. The multi-tap commit handler (existing `multi_tap_commit_handler`) checks the armed pose and the accumulated tap count, validates the cadence for SURFACE, and triggers mode entry.

- [ ] **Step 1: Find existing `multi_tap_commit_handler`**

Run: `grep -n "multi_tap_commit_handler" src/gesture_mode.cpp`
Find the function definition (the static C work-handler).

- [ ] **Step 2: Replace the handler body to add pose-based mode entry**

Locate the existing handler. Currently it logs `Multi-tap commit: N (...)`. Replace its action-decision logic with:

```c
static void multi_tap_commit_handler(struct k_work *work_arg)
{
    ARG_UNUSED(work_arg);

    int count = multi_tap_count;
    char axis = multi_tap_first_axis;
    char sign = multi_tap_first_sign;

    /* Snapshot pose at commit time. */
    pose_id_t armed = gesture_mode_armed_pose();

    /* Reset multi-tap counter regardless of outcome. */
    multi_tap_count = 0;
    multi_tap_first_axis = '?';
    multi_tap_first_sign = '?';

    if (armed == POSE_NONE) {
        /* Should not happen (the pose gate in on_chip_single_tap
         * should reject taps without a pose), but defensive. */
        LOG_INF("Multi-tap commit ABORT: no pose armed (count=%d)",
                count);
        return;
    }

    /* Apply per-pose gesture rules. */
    switch (armed) {
    case POSE_AIR_MOUSE:
        if (count >= 1) {
            LOG_INF("MODE ENTRY: AIR_MOUSE (pose + %d-tap gesture)",
                    count);
            /* TODO when integrating: trigger actual mode transition
             * here.  For now log only so the pose+gesture path is
             * verified before touching the power state machine.  See
             * existing 't'/'y' serial commands for the mode-entry
             * pattern. */
        }
        break;

    case POSE_DICTATION:
        if (count >= 1) {
            LOG_INF("MODE ENTRY: DICTATION (pose + %d-tap gesture)",
                    count);
            /* Dictation mode does not exist yet (its own spec).
             * Log only.  Will wire when dictation feature lands. */
        }
        break;

    case POSE_SURFACE:
        /* Require count >= 2 AND cadenced double-tap interval AND
         * spectral surface confirmation. */
        if (count < 2) {
            LOG_INF("SURFACE entry rejected: need double-tap "
                    "(got count=%d)", count);
            break;
        }
        {
            int interval = last_two_tap_interval_ms();
            if (!is_cadenced_double_tap_window(interval)) {
                LOG_INF("SURFACE entry rejected: inter-tap=%d ms "
                        "outside cadence window [%d, %d]",
                        interval, CADENCE_MIN_MS, CADENCE_MAX_MS);
                break;
            }
        }
        if (!surface_spectral_confirms_hard_surface()) {
            LOG_INF("SURFACE entry rejected: spectral signature "
                    "indicates soft surface (mid_band=%.0f < %.0f)",
                    (double)last_tap_mid_band_energy,
                    (double)SURFACE_RESONANCE_MID_BAND_THRESH);
            break;
        }
        LOG_INF("MODE ENTRY: SURFACE (pose + cadenced double-tap + "
                "hard-surface spectral confirmed)");
        /* TODO: trigger mode transition. */
        break;

    default:
        LOG_INF("Multi-tap commit ABORT: unknown armed pose %d",
                (int)armed);
        break;
    }

    /* Disarm pose after a gesture attempt (whether successful or
     * not) -- one shot per arm window. */
    pose_armed_state = POSE_NONE;
}
```

- [ ] **Step 3: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build.

- [ ] **Step 4: Hardware test — full pose-gated mode-entry flow**

Flash and test each mode:

**AIR_MOUSE entry path:**
1. Raise wrist deliberately (motion-into-pose), hold ~0.5 sec in raised forward position.
2. Within 3 sec, single-tap the band.
3. Expected logs in order:
   ```
   Pose ARMED: AIR_MOUSE (...)
   Chip single-tap: ...
   Multi-tap commit: 1 (...)
   MODE ENTRY: AIR_MOUSE (pose + 1-tap gesture)
   ```

**DICTATION pose path:**
1. Raise + rotate toward mouth.
2. Single tap.
3. Expected: `MODE ENTRY: DICTATION ...`

**SURFACE entry path (correct):**
1. Lower wrist to desk surface, dwell.
2. Within 3 sec, double-tap on the band with ~200 ms inter-tap spacing.
3. Expected:
   ```
   Pose ARMED: SURFACE (...)
   Chip single-tap: ... count=1 ...
   Chip single-tap: ... count=2 ...
   Multi-tap commit: 2 (...)
   MODE ENTRY: SURFACE (pose + cadenced double-tap + hard-surface spectral confirmed)
   ```

**SURFACE entry path (rejected — wrong gesture):**
1. Lower wrist to desk, single-tap.
2. Expected: `SURFACE entry rejected: need double-tap (got count=1)`.

**SURFACE entry path (rejected — wrong cadence):**
1. Lower wrist to desk, do two taps with > 400 ms between them.
2. Expected: `SURFACE entry rejected: inter-tap=... ms outside cadence window`.

**SURFACE entry path (rejected — soft surface):**
1. Rest wrist on lap (or pillow), do cadenced double-tap.
2. Expected: `SURFACE entry rejected: spectral signature indicates soft surface ...`.

This last one is the key defence test. If it fires correctly, the surface-spectral check is working. If lap-rest with cadenced double-tap STILL triggers SURFACE entry, we'll need to tune `SURFACE_RESONANCE_MID_BAND_THRESH` upward.

**Negative tests:**
- Skin tap near band without pose: still IGNORED (Task 7 behaviour preserved).
- Random ambient slap (desk hit while not wearing band on desk): IGNORED.

Report: which paths worked, which need threshold tuning, any surprises.

- [ ] **Step 5: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
poses: wire mode entry from armed pose + matched gesture

Replaces the multi-tap commit handler to trigger mode entry based on
which pose was armed plus the gesture shape:
  AIR_MOUSE  ← armed pose + single tap
  DICTATION  ← armed pose + single tap (no mode wired yet; log-only)
  SURFACE    ← armed pose + cadenced double-tap (150-300 ms) +
              hard-surface spectral confirmation (mid_band > thresh)

Pose is disarmed after each gesture attempt (success or fail), so
each pose-arm is one-shot.

Mode-transition calls are stubs (LOG_INF only) for now -- the
power state machine integration happens after the pose+gesture
flow is verified empirically.  See 't' / 'y' serial commands for
the existing mode-entry pattern when ready to wire up.
EOF
)"
```

---

## Task 9: Deprecate snap-vs-tap classifier at trigger

**Files:**
- Modify: `src/gesture_mode.cpp` — remove the snap-vs-tap classifier verdict path; keep FFT and feature extraction (still used by surface-spectral check)

**Rationale:** Once pose-gating is in place, snap-vs-tap discrimination at trigger time is no longer needed. The classifier function `_classify()` and its verdict-driven logging can be removed. FFT pipeline + `low_band_energy` / `mid_band_energy` / `low_band_ratio` features stay — `mid_band_energy` is now used by the surface-spectral check.

- [ ] **Step 1: Find the classifier function**

Run: `grep -n "_classify\|low_band_ratio\|SNAP_LOW_RATIO" src/gesture_mode.cpp`
Expected: `_classify()` function + its constants + its call site in `bio_acoustic_worker`.

- [ ] **Step 2: Remove `_classify()` function and its threshold constants**

Delete the entire `_classify()` function and the constants:
- `SNAP_LOW_RATIO_THRESH`
- `TAP_LOW_RATIO_THRESH`
- `MIN_PEAK_SUM` (only used by `_classify`)

Keep:
- The `tap_features` struct
- The `low_band_energy`, `mid_band_energy`, `low_band_ratio` fields
- The FFT computation
- The Hann window
- The `_compute_band_energies()` function

- [ ] **Step 3: Update `bio_acoustic_worker` log format**

The existing `LOG_INF("[BIO] verdict=...")` line currently reports the classifier verdict. Replace it with a feature-only log:

```c
/* Log the captured tap's features.  No more classifier verdict at
 * trigger time -- pose-gating + per-pose gesture rules handle that
 * now.  Features are still useful for diagnostics, surface-spectral
 * check, and future in-session snap-vs-tap discrimination (which
 * uses these features + PDM mic; see architecture roadmap Item 7). */
LOG_INF("[BIO] features peak[%d,%d,%d] dom=%c z_ratio=%u%% "
        "low_band=%.0f mid_band=%.0f low_ratio=%.2f idx=%u/%u",
        (int)f.peak_x, (int)f.peak_y, (int)f.peak_z,
        f.dominant_axis,
        (unsigned)z_ratio_pct,
        (double)f.low_band_energy,
        (double)f.mid_band_energy,
        (double)f.low_band_ratio,
        (unsigned)f.peak_sample_idx, (unsigned)n);
```

(Remove the `verdict_str` variable and its construction since there's no verdict.)

- [ ] **Step 4: Remove the now-unused `verdict` variable**

Inside `bio_acoustic_worker`, remove:

```c
char verdict = _classify(&f);
const char *verdict_str = (verdict == 'S') ? "SNAP" :
                          (verdict == 'B') ? "BAND-TAP" :
                          "UNCLASSIFIED";
```

- [ ] **Step 5: Build**

Run: `export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH; export ZEPHYR_BASE=/opt/nordic/ncs/zephyr; west build --build-dir build`
Expected: clean build, FLASH usage decreases slightly (function removal).

- [ ] **Step 6: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "$(cat <<'EOF'
classifier: remove deprecated snap-vs-tap verdict path at trigger

The pose-gated trigger architecture makes snap-vs-tap discrimination
unnecessary at trigger time -- pose carries the mode info (raise →
AIR_MOUSE, raise+rotate → Dictation, lower → SURFACE).

Removes:
- _classify() function and threshold constants (SNAP_LOW_RATIO_THRESH,
  TAP_LOW_RATIO_THRESH, MIN_PEAK_SUM)
- Verdict log path in bio_acoustic_worker

Keeps:
- FFT pipeline + Hann window + arm_rfft_fast_f32 setup
- Spectral band-energy features (low_band, mid_band, low_band_ratio)
- _compute_band_energies() function
- [BIO] features log (diagnostic only, no verdict)

The mid_band_energy feature is now consumed by the surface-spectral
confirmation check in the multi-tap commit handler (added in Task 6).

When in-session snap-vs-tap lands (Item 7+, mic + IMU fusion per
GestEar), it'll use these features + the PDM mic.  Re-introducing
a classifier function will mean wiring the fusion path; the spec
in 2026-06-10-gesture-trigger-redesign-design.md §5 covers the
direction.
EOF
)"
```

---

## Task 10: Architecture doc update + end-to-end hardware test

**Files:**
- Modify: `docs/research/gesture-architecture.md` — update to reflect pose-gated architecture
- No code changes; this is documentation + final validation.

- [ ] **Step 1: Locate the architecture section that describes the trigger model**

Run: `grep -n "Stage A\|Stage B\|Stage C\|Stage E\|chip tap engine\|multi-tap" docs/research/gesture-architecture.md | head -20`
Find the section that currently describes the chip-tap-based trigger.

- [ ] **Step 2: Add a new section "Trigger architecture (2026-06-10 redesign)"**

Insert after the existing Stage E section:

```markdown
### Trigger architecture (2026-06-10 redesign)

**Replaces**: previous single-event chip-tap → multi-tap commit →
mode entry path.

**Reason**: hardware testing revealed the chip-tap engine fires on
incidental impulse events (skin taps 1-2 cm from band, desk slaps,
thigh slaps, lap fidgets).  Per the engineering principle, this
is a fundamental property of bio-acoustic sensing on a wrist-worn
device; enumerating all possible accidental triggers is unbounded.
The disciplined approach is to design gestures with deliberate
signatures.

**Architecture**: pose-gated two-stage trigger.  Stage 1: continuous
pose monitoring (gravity vector + motion-history ring buffer); when
motion-into-pose transition is detected AND the gravity vector
matches a canonical pose within tolerance, the pose state machine
arms for 3 seconds.  Stage 2: within the armed window, the chip-tap
path is unmuted; mode entry requires the per-pose gesture
requirement (single tap for AIR_MOUSE / DICTATION; cadenced double-
tap + spectral surface confirmation for SURFACE).

**Three canonical poses**: AIR_MOUSE (raise forward), DICTATION
(raise + rotate toward mouth), SURFACE (horizontal palm-down rest).
Each pose has a tolerance cone (±30° / ±30° / ±20°) and uses a
continuous pose-score (not binary in/out-of-pose).

**Snap-vs-tap discrimination at trigger is deprecated.**  Pose
carries the mode info.  Snap-vs-tap moves to in-session use (Item
7+ timeframe, mic + IMU fusion).

**Production limits documented**: SURFACE/lap false positive is a
known edge case requiring additional sensors (PPG firmness,
altimetry, optical proximity) -- documented for productionization.

See `docs/superpowers/specs/2026-06-10-gesture-trigger-redesign-design.md`
for full design rationale.
```

- [ ] **Step 3: End-to-end hardware integration test**

The acceptance test for the whole feature.  Run all scenarios; report failures:

**Acceptance criteria (all must pass for success):**

1. **AIR_MOUSE entry works**: 5/5 trials of "raise forward + single tap" produce `MODE ENTRY: AIR_MOUSE` log.
2. **DICTATION entry works**: 5/5 trials of "raise + rotate + single tap" produce `MODE ENTRY: DICTATION` log.
3. **SURFACE entry works**: 5/5 trials of "lower to desk + cadenced double-tap" produce `MODE ENTRY: SURFACE` log.
4. **AIR_MOUSE rejected without pose**: 5/5 trials of single tap WITHOUT raising → `IGNORED: no pose armed`.
5. **SURFACE rejected on lap**: 5/5 trials of "lap rest + cadenced double-tap" → `rejected: spectral signature indicates soft surface`.
6. **SURFACE rejected with wrong cadence**: 5/5 trials of "desk rest + slow two-tap (>400 ms apart)" → `rejected: inter-tap=... outside cadence window`.
7. **Skin-tap-near-band suppressed**: 5/5 trials of tapping skin within 1-2 cm of the band without raising → `IGNORED`.
8. **Random desk slap suppressed**: 5/5 trials of slapping desk (band not on desk) → `IGNORED`.
9. **Random thigh slap suppressed**: 5/5 trials of slapping thigh → `IGNORED`.
10. **Pose-arm timeout works**: arm a pose, wait 4 sec, see `Pose arm expired`; subsequent tap → `IGNORED`.

If any scenario fails, document which thresholds need tuning:
- Canonical pose values (gx, gy, gz) → `src/gesture_poses.cpp`
- Pose tolerance cones → `src/gesture_poses.cpp` `tolerance_cos`
- Motion-into-pose thresholds → `src/gesture_mode.cpp` `MOTION_INTO_POSE_*`
- Surface-spectral threshold → `src/gesture_mode.cpp` `SURFACE_RESONANCE_MID_BAND_THRESH`
- Cadence window → `src/gesture_mode.cpp` `CADENCE_MIN_MS`/`CADENCE_MAX_MS`

- [ ] **Step 4: Commit architecture doc update**

```bash
git add docs/research/gesture-architecture.md
git commit -m "$(cat <<'EOF'
docs: update architecture for pose-gated trigger redesign

Adds a new "Trigger architecture (2026-06-10 redesign)" section
documenting the pose-gated two-stage model and the deprecation
of snap-vs-tap discrimination at trigger time.

Cross-references the design spec at
docs/superpowers/specs/2026-06-10-gesture-trigger-redesign-design.md.
EOF
)"
```

---

## Self-Review Checklist (run after writing)

Run mentally on the completed plan above:

**Spec coverage:**
- §3 architecture (two-stage) → Tasks 1-4, 7-8 ✓
- §4 soft-wired pose tolerance → Task 1 (continuous score) ✓
- §5 snap-vs-tap deprecation at trigger → Task 9 ✓
- §6 spectral repurposing → Task 6 ✓
- §7 components: gravity tracker → Tasks 2-3; pose FSM → Tasks 3-4; cadenced double-tap → Task 5; surface-spectral → Task 6; NVS calibration → DEFERRED to productionization (not in plan)
- §9 limits → documented in tests (lap FP test in Task 8 + Task 10)

**Placeholder scan:**
- No "TBD" or "implement later" in code blocks.
- Mode-transition triggers are LOG_INF stubs in Task 8 — explicitly explained why (verify pose path before touching power state machine).  This is a clear cutoff, not a placeholder.
- "Tune thresholds" in Task 10 has specific file paths and constant names to tune.

**Type consistency:**
- `pose_id_t` defined Task 1, used Tasks 3-9 consistently.
- `pose_score()`, `pose_classify_best()`, `pose_get_canonical()`, `pose_name()` signatures consistent throughout.
- `gesture_mode_armed_pose()` API consistent Task 3 + use sites.
- `surface_spectral_confirms_hard_surface()` matches Task 6 definition.

**Sensible gaps deferred (not in this plan):**
- NVS calibration storage (productionization, separate spec)
- Per-user calibration ritual (productionization)
- In-session snap-vs-tap via mic + IMU (Item 7+, separate spec)
- PPG firmness check (productionization, separate spec)
- Mode-transition wire-up (intentionally stub'd in this plan; lands when pose+gesture path is empirically validated)

Plan is internally consistent and covers spec scope minus explicitly-deferred items.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-10-gesture-trigger-redesign.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
