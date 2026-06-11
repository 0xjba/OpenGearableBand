# Pose-Trigger Realignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the roll-based AIR_MOUSE/DICTATION split (the false-DICTATION bug) and stop the orientation classifier flapping, so the raised hemisphere arms as AIR_MOUSE and a double-tap always enters AIR_MOUSE.

**Architecture:** The confirming gesture decides the mode, never the gravity pose (proven: a held max-right air-mouse is gravity-identical to dictation). So delete the roll split entirely; the raised hemisphere is one pose. Separately, add asymmetric-dwell hysteresis to the coarse orientation classifier so a brief `gx≈gy` excursion mid-sweep can't flip `UP_RAISED→NEUTRAL`.

**Tech Stack:** Zephyr RTOS / nRF52840 (XIAO Sense), C++. **No unit-test harness** — this is hardware-in-the-loop firmware. Per-task verification is (1) a clean `west build` and (2) hardware log observation against named expected output (the project's established loop). Build command, used in every build step:

```bash
export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH
export ZEPHYR_BASE=/opt/nordic/ncs/zephyr
west build --build-dir build
```

Spec: `docs/superpowers/specs/2026-06-11-pose-trigger-realignment-design.md`

---

### Task 1: Remove the roll-split (the bug)

**Files:**
- Modify: `src/gesture_mode.cpp` (`pose_fsm_update`, `multi_tap_commit_handler`)
- Modify: `src/gesture_thresholds.h` (remove `DICTATION_ROLL_THRESH_DEG`)

- [ ] **Step 1: Delete the roll reassignment in `pose_fsm_update`**

In `src/gesture_mode.cpp`, find this block (immediately after the `pose_classify_best(...)` call) and **delete it entirely** — the comment and the `if`:

```c
    /* Roll-based AIR_MOUSE/DICTATION split (validated 2026-06-10).
     * ... (full comment block) ...
     * entry).  DICTATION is log-only until its feature exists. */
    if (best == POSE_AIR_MOUSE) {
        float roll_deg = atan2f(gy, gz) * (180.0f / 3.14159265f);
        if (roll_deg >= DICTATION_ROLL_THRESH_DEG) {
            best = POSE_DICTATION;
        }
    }
```

Replace it with a one-line note so it isn't re-added:

```c
    /* NOTE: do NOT re-add a roll/gz-based AIR_MOUSE<->DICTATION split here.
     * A held max-right air-mouse is gravity-identical to dictation (measured
     * 2026-06-11); pose-only discrimination is impossible.  The confirming
     * GESTURE decides the mode (see 2026-06-11-pose-trigger-realignment spec
     * + decision_dictation_voice_gated_entry).  The raised hemisphere arms
     * as POSE_AIR_MOUSE; dictation entry is future (clench + voice).
     * Pose logic operates on the gravity-LPF components (gx/gy/gz) only --
     * NEVER orientation_get().roll_deg (the quaternion Euler roll lags ~10
     * deg in motion; it is for the cursor/logging, not pose decisions). */
```

- [ ] **Step 2: Delete the `case POSE_DICTATION` in `multi_tap_commit_handler`**

In `src/gesture_mode.cpp`, find and **delete** this case (POSE_DICTATION is no longer armed by gravity, so this is now dead + wrong per the voice-gated decision):

```c
    case POSE_DICTATION:
        LOG_INF("MODE ENTRY: DICTATION (pose + cadenced double-tap)");
        /* Dictation mode does not exist yet (its own spec).  Log
         * only; will wire when dictation feature lands. */
        break;
```

The `switch (armed)` now has `POSE_AIR_MOUSE`, `POSE_SURFACE`, and `default:` — all unchanged.

- [ ] **Step 3: Remove `DICTATION_ROLL_THRESH_DEG` from the threshold header**

In `src/gesture_thresholds.h`, find and **delete**:

```c
/* Roll (= atan2(gy,gz), deg) above which a raised pose is DICTATION
 * (palm supinated toward face) rather than AIR_MOUSE (palm to screen).
 * Validated from settled-pose data 2026-06-10: air-mouse ~30-64 deg,
 * dictation ~105-117 deg.  Drift-free (gravity-derived).  Tunable;
 * per-user calibration is a productionization item. */
#define DICTATION_ROLL_THRESH_DEG       85.0f
```

Replace with:

```c
/* (removed 2026-06-11) Roll-based AIR_MOUSE/DICTATION split deleted: a held
 * max-right air-mouse is gravity-identical to dictation, so pose-only
 * discrimination is impossible.  The confirming gesture decides the mode.
 * See 2026-06-11-pose-trigger-realignment spec. */
```

- [ ] **Step 4: Build**

Run the build command. Expected: clean compile, no `DICTATION_ROLL_THRESH_DEG` undefined errors (Step 1 removed its only use), FLASH ~46%.

- [ ] **Step 5: Commit**

```bash
git add src/gesture_mode.cpp src/gesture_thresholds.h
git commit -m "fix: remove roll-based AIR_MOUSE/DICTATION split (gesture decides mode)

Pose-only cannot discriminate (max-right air-mouse is gravity-identical to
dictation). Raised hemisphere arms as AIR_MOUSE; double-tap -> AIR_MOUSE.
Removed the roll reassignment, the DICTATION-on-tap case, and the roll
threshold. Dictation entry is future (clench + voice).
See 2026-06-11-pose-trigger-realignment spec."
```

---

### Task 2: Asymmetric-dwell hysteresis on the orientation classifier

**Files:**
- Modify: `src/gesture_thresholds.h` (replace `ORIENTATION_DWELL_SAMPLES`)
- Modify: `src/gesture_mode.cpp` (`gesture_mode_update_accel` dwell-commit block)

- [ ] **Step 1: Replace the single dwell constant with asymmetric enter/leave dwells**

In `src/gesture_thresholds.h`, find:

```c
/* Orientation classifier dwell (samples @100 Hz) before a NEUTRAL/UP_RAISED/
 * DOWN_FLAT reclassification commits.  [STRUCTURAL] */
#define ORIENTATION_DWELL_SAMPLES       25
```

Replace with:

```c
/* Orientation classifier dwell, asymmetric (hysteresis, samples @100 Hz).
 * Entering a DEFINITE pose (UP_RAISED/DOWN_FLAT) is responsive; LEAVING a
 * pose to NEUTRAL is sticky, so a brief gx~=gy excursion mid air-mouse sweep
 * cannot flip UP_RAISED->NEUTRAL (the 2026-06-11 flapping bug).  LEAVE is the
 * regression-tuned knob: raise it until the sweep log shows 0 spurious
 * transitions.  [USER] */
#define ORIENTATION_ENTER_DWELL         15
#define ORIENTATION_LEAVE_DWELL         80
```

- [ ] **Step 2: Rewrite the dwell-commit block to use the asymmetric dwells**

In `src/gesture_mode.cpp` (`gesture_mode_update_accel`), find this block:

```c
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
```

Replace the two `if` statements (keep the `LOG_INF` and everything after it) with:

```c
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
```

(The dwell counter caps at `ORIENTATION_LEAVE_DWELL` so the NEUTRAL case can always reach its threshold.)

- [ ] **Step 3: Build**

Run the build command. Expected: clean compile (no `ORIENTATION_DWELL_SAMPLES` references remain), FLASH ~46%.

- [ ] **Step 4: Commit**

```bash
git add src/gesture_mode.cpp src/gesture_thresholds.h
git commit -m "fix: asymmetric-dwell hysteresis on orientation classifier

Leaving a pose to NEUTRAL now needs a long sticky dwell (LEAVE_DWELL),
entering a definite pose stays responsive (ENTER_DWELL). Kills the
UP_RAISED<->NEUTRAL flapping during an air-mouse left-right sweep (a brief
gx~=gy dip no longer commits NEUTRAL). LEAVE_DWELL is regression-tuned.
See 2026-06-11-pose-trigger-realignment spec."
```

---

### Task 2b: Redefine the raised-zone (the real flapping fix)

Task 2's asymmetric dwell FAILED hardware verification (a wide air-mouse
sweep dwells ~0.8 s+ at each Y-dominant extreme — ~20 `UP_RAISED↔NEUTRAL`
transitions in 30 s). Root cause: `_classify_orientation` maps the
Y-dominant far-reach to NEUTRAL. Fix = redefine the raised-zone on the
unified gravity geometry (observability doc §3.5), ignoring the gy sweep
axis. The Task 2 dwell stays (harmless transient-rejection).

**Files:**
- Modify: `src/gesture_thresholds.h` (add `RAISED_ELEVATION_RATIO`)
- Modify: `src/gesture_mode.cpp` (`_classify_orientation` body)

- [ ] **Step 1: Add the elevation ratio constant**

In `src/gesture_thresholds.h`, right after `#define DOMINANCE_RATIO 1.3f`, add:

```c
/* Raised-zone elevation ratio: UP_RAISED when gx (forearm axis, positive)
 * exceeds this * |gz| -- "arm up, not flat", IGNORING the gy SWEEP axis so a
 * wide air-mouse reach (gy large at the extremes) stays raised instead of
 * flapping to NEUTRAL (the 2026-06-11 sweep bug).  Part of the unified
 * gravity-component geometry (observability doc §3.5).  EMPIRICAL: observed
 * min gx/|gz| across the raised sweep ~1.31, so 1.1 leaves margin; refine
 * against cross-session adversarial traces.  Regression test: 2026-06-11
 * sweep log -> 0 UP_RAISED<->NEUTRAL transitions.  [USER] */
#define RAISED_ELEVATION_RATIO          1.1f
```

- [ ] **Step 2: Replace the `_classify_orientation` body**

In `src/gesture_mode.cpp`, replace the ENTIRE body of `_classify_orientation`
(the `fabsf` magnitudes, the largest-axis sort, the `DOMINANCE_RATIO` neutral
gate, and the `switch (largest_axis)`) with:

```c
static WristOrientation _classify_orientation(float gx, float gy, float gz)
{
    /* Unified gravity-component geometry (observability-aware-pose-and-
     * cursor-design.md §3.5): gx = forearm elevation, gy = left-right SWEEP
     * axis, gz = volar-normal.  RAISED ignores gy so a wide air-mouse sweep
     * (gy large at the extremes) stays UP_RAISED instead of flapping to
     * NEUTRAL.  Thresholds are empirical (regression test: the 2026-06-11
     * left-right sweep must produce 0 UP_RAISED<->NEUTRAL transitions). */

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
```

Keep the function signature and everything outside the body unchanged. The
old per-axis comment block (the "Empirical mapping from field calibration"
narrative) is replaced by the new comment above.

- [ ] **Step 3: Build** — clean compile, FLASH ~46%. `RAISED_ELEVATION_RATIO`
  resolves; `DOMINANCE_RATIO` still used (FLAT); `fabsf` from `<math.h>` (already included).

- [ ] **Step 4: Commit**

```bash
git add src/gesture_mode.cpp src/gesture_thresholds.h
git commit -m "fix: redefine raised-zone on unified gravity geometry (kills sweep flapping)

UP_RAISED = gx>0 & gx>RAISED_ELEVATION_RATIO*|gz| (forearm up, not flat),
ignoring the gy SWEEP axis -- a wide air-mouse reach (gy-dominant at the
extreme) no longer flips to NEUTRAL. Replaces the failed asymmetric-dwell
approach. Same gx/gy/gz signals as the cone/gz-sign (one geometric model,
observability doc 3.5). RAISED_ELEVATION_RATIO is empirical; regression test
is the 2026-06-11 sweep -> 0 transitions.
See 2026-06-11-pose-trigger-realignment spec 3.2."
```

### Task 3: Hardware verification (the real test)

**Files:** none (flash + observe).

- [ ] **Step 1: Flash**

```bash
# copy build/zephyr/zephyr.uf2 to the XIAO in bootloader mode, or your flash path
```

- [ ] **Step 2: Regression — orientation flapping (the named test case)**

Press `v`, raise into air-mouse, and do a left↔right sweep across an imaginary screen for ~30 s (reproduce the 2026-06-11 sweep).
Expected: **zero** `Orientation: UP_RAISED -> NEUTRAL` / `NEUTRAL -> UP_RAISED` lines during the continuous sweep (the prior build showed ~6).
If any remain: raise `ORIENTATION_LEAVE_DWELL` (e.g., 80 → 110) in `gesture_thresholds.h`, rebuild, re-run, until 0. Commit the tuned value.

- [ ] **Step 3: Functional — no DICTATION on taps, ever**

Hold the **steep raised crane** that previously mislabeled, and do a cadenced double-tap.
Expected: `Pose ARMED: AIR_MOUSE` then `MODE ENTRY: AIR_MOUSE` — **never** `DICTATION`.

- [ ] **Step 4: Functional — max-right air-mouse + double-tap → AIR_MOUSE**

Reach to a held max-right air-mouse pose (gravity-identical to dictation, roll ~113) and double-tap.
Expected: `MODE ENTRY: AIR_MOUSE`, not DICTATION (proves the gesture, not the pose, decides).

- [ ] **Step 5: Functional — SURFACE unchanged**

Wrist on desk, cadenced double-tap on the desk.
Expected: `MODE ENTRY: SURFACE` when the hard-surface spectral check passes (unchanged behaviour).

- [ ] **Step 6: Confirm no regressions in arming**

Normal raises (forward, left) still `Pose ARMED: AIR_MOUSE`; `at_rest` logs clean 0/1. No `MODE ENTRY: DICTATION` appears in any tap test.
