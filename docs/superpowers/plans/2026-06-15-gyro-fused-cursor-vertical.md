# Gyro-fused cursor vertical signal — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the air-mouse vertical signal (`vert`) from the Mahony filter's fused gravity vector instead of the accel-only low-pass, killing the linear-accel jitter without adding lag.

**Architecture:** The Mahony filter (`orientation.cpp`) already computes a gyro-fused, accel-corrected unit gravity vector each update but only uses it internally. Expose it; compute `current_vert_deg()` from its roll-immune forearm-axis projection (same geometry as today, fused source); delete the now-dead accel-only LPF (`gx_cursor` + `s_cursor_alpha` + the `o`/`p` knob + `CURSOR_GRAVITY_ALPHA`). `current_vert_deg()` is the single switch point, so all four `vert` consumers (cursor servo, entry-snap top, rest-bottom capture, anchor-relative low-zone) move together and stay consistent. Rest-safe: fused gravity == measured accel at rest, so calibration is unchanged.

**Tech Stack:** C++ (C-linkage modules), Zephyr/NCS on XIAO nRF52840 Sense. Firmware verified via `./build.sh`; pure modules via host `g++`.

**Spec:** `docs/superpowers/specs/2026-06-15-gyro-fused-cursor-vertical-design.md`

**Verification note:** `orientation.cpp` / `gesture_mode.cpp` are not host-testable (Zephyr deps); per CLAUDE.md, per-change verification = clean `./build.sh` + serial-log reading on HW. The pure host suites (`cursor_track`, `cursor_calib`) are unaffected by this change and must keep passing as a regression guard.

---

## File structure

- `src/orientation.h` — add `float gravity[3]` to `orientation_state_t` (the fused gravity output).
- `src/orientation.cpp` — `orientation_get()` fills `gravity[3]` from the quaternion (the expression already used internally for the Mahony correction). No filter-behaviour change.
- `src/gesture_mode.cpp` — `current_vert_deg()` reads the fused gravity; remove the accel-only cursor LPF state, its reset/seed/update, the alpha, and `gesture_mode_adjust_cursor_alpha()`.
- `src/gesture_mode.h` — remove the `gesture_mode_adjust_cursor_alpha()` declaration.
- `src/main.cpp` — remove the `o`/`p` serial-console handlers (input parse + dispatch). They are not in the printed help list, so no help text changes.
- `src/gesture_thresholds.h` — remove `CURSOR_GRAVITY_ALPHA`.

---

## Task 1: Expose the fused gravity vector from `orientation`

**Files:**
- Modify: `src/orientation.h` (struct `orientation_state_t`, ~line 30-36)
- Modify: `src/orientation.cpp` (`orientation_get`, ~line 175-197)

- [ ] **Step 1: Add the `gravity` field to the state struct.** In `src/orientation.h`, the struct currently ends:

```c
    float yaw_deg;            /* gyro-only; drifts; re-zeroed at rest */
    bool  at_rest;            /* stillness flag (accel + gyro quiet) */
    float gyro_bias_dps[3];   /* estimated gyro bias, deg/s */
} orientation_state_t;
```

Add one field before the closing brace:

```c
    float yaw_deg;            /* gyro-only; drifts; re-zeroed at rest */
    bool  at_rest;            /* stillness flag (accel + gyro quiet) */
    float gyro_bias_dps[3];   /* estimated gyro bias, deg/s */
    float gravity[3];         /* fused gravity DIRECTION in band frame, unit vector
                               * (gyro-driven, accel-corrected) -- the roll-immune
                               * source for the cursor's vertical driver */
} orientation_state_t;
```

- [ ] **Step 2: Fill it in `orientation_get`.** In `src/orientation.cpp`, `orientation_get` currently ends:

```c
    out->at_rest = at_rest;
    out->gyro_bias_dps[0] = ifb_x * RAD2DEG;
    out->gyro_bias_dps[1] = ifb_y * RAD2DEG;
    out->gyro_bias_dps[2] = ifb_z * RAD2DEG;
}
```

Append the gravity output (same expression as the internal Mahony correction at lines 132-134):

```c
    out->at_rest = at_rest;
    out->gyro_bias_dps[0] = ifb_x * RAD2DEG;
    out->gyro_bias_dps[1] = ifb_y * RAD2DEG;
    out->gyro_bias_dps[2] = ifb_z * RAD2DEG;

    /* Fused gravity direction the quaternion predicts (unit vector, since q is
     * normalised).  Identical to the vector the Mahony step uses internally;
     * exposed for the cursor's roll-immune vertical driver. */
    out->gravity[0] = 2.0f * (q1 * q3 - q0 * q2);
    out->gravity[1] = 2.0f * (q0 * q1 + q2 * q3);
    out->gravity[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
```

- [ ] **Step 3: Build.** Run: `./build.sh`
Expected: clean build, `Flashable artifact: build/zephyr/zephyr.uf2`. (No consumer yet; this just compiles the new field.)

- [ ] **Step 4: Commit.**
```bash
git add src/orientation.h src/orientation.cpp
git commit -m "feat(orientation): expose fused gravity vector in orientation_state_t"
```

---

## Task 2: Drive `current_vert_deg()` from fused gravity; remove the dead accel-only LPF

**Files:**
- Modify: `src/gesture_mode.cpp` (decls ~50-61; `current_vert_deg` ~63-75; init reset ~744-746; seed ~833; FAST-IIR update ~842-847; `gesture_mode_adjust_cursor_alpha` ~1444-1453)
- Modify: `src/gesture_mode.h` (~280-288)
- Modify: `src/main.cpp` (input parse ~661-668; dispatch ~851-862)
- Modify: `src/gesture_thresholds.h` (~98-104)

- [ ] **Step 1: Rewrite `current_vert_deg()` to use the fused gravity.** In `src/gesture_mode.cpp`, replace the comment + function (the `gx_cursor`-based version):

```c
/* Angle-from-vertical (deg) for the cursor Y driver, from the FAST cursor
 * gravity filter.  Roll-immune (scalar projection of gravity onto the forearm
 * axis), unlike Euler pitch which saturates at high roll -- see cursor_track.h. */
static inline float current_vert_deg(void)
{
    float mag = sqrtf(gx_cursor * gx_cursor + gy_cursor * gy_cursor +
                      gz_cursor * gz_cursor);
    return (mag > 0.1f)
        ? acosf(fminf(1.0f, fabsf(gx_cursor) / mag)) * (180.0f / 3.14159265f)
        : 0.0f;
}
```

with the fused-gravity version:

```c
/* Angle-from-vertical (deg) for the cursor Y driver, from the Mahony filter's
 * FUSED gravity vector (gyro-driven, accel-corrected -- responsive AND immune to
 * linear-accel transients, unlike the old accel-only LPF).  Roll-immune: scalar
 * projection of gravity onto the forearm axis (X), NOT Euler pitch (which is
 * roll-contaminated near high roll -- see cursor_track.h).  Single source of
 * truth for `vert`: cursor servo, entry-snap top, rest-bottom capture, and the
 * anchor-relative low zone all read this. */
static inline float current_vert_deg(void)
{
    orientation_state_t ori;
    orientation_get(&ori);
    float gx = ori.gravity[0], gy = ori.gravity[1], gz = ori.gravity[2];
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    return (mag > 0.1f)
        ? acosf(fminf(1.0f, fabsf(gx) / mag)) * (180.0f / 3.14159265f)
        : 0.0f;
}
```

(`orientation.h` is already included — `gesture_mode.cpp` calls `orientation_update`/`orientation_get` elsewhere. If a compile error says `orientation_state_t` is undefined here, add `#include "orientation.h"` to the top includes.)

- [ ] **Step 2: Remove the cursor-LPF declarations + alpha.** Delete the `gx_cursor`/`gy_cursor`/`gz_cursor` statics and `s_cursor_alpha` (with their comments). The block currently reads:

```c
 *  - gx/gy/gz_cursor: FAST LPF (CURSOR_GRAVITY_ALPHA) for the cursor Y driver
 *    ONLY -- responsiveness is the feature (the slow LPF lagged the cursor ~18°
 *    = rate*tau).  INTERIM fix (a); the destination fix (b) is a gyro-fused
 *    inclination from the existing Mahony quaternion (rejects linear-accel
 *    transients on fast flicks, which this accel-only filter cannot). */
static float gx_filt = 0.0f;
static float gy_filt = 0.0f;
static float gz_filt = -9.81f;   /* assume face-up at boot */
static float gx_cursor = 0.0f;
static float gy_cursor = 0.0f;
static float gz_cursor = -9.81f;

/* Runtime-tunable smoothing coefficient for the FAST cursor filter, seeded
 * from the compile-time default.  Live-dialled via the serial console ('o'
 * smoother / 'p' sharper) so the user can trade responsiveness vs jitter to
 * taste this session without a rebuild.  Lower alpha = more smoothing + more
 * lag; higher = snappier + more accel noise.  Still INTERIM (a) -- the
 * destination fix (b) is the gyro-fused inclination, which is responsive AND
 * smooth without this knob. */
static float s_cursor_alpha = CURSOR_GRAVITY_ALPHA;
```

Replace it with (keep only the slow `gx_filt` trio, drop the cursor trio + alpha):

```c
 *  (The cursor's vertical driver no longer has its own accel LPF -- it now reads
 *  the Mahony filter's fused gravity via current_vert_deg(); see orientation.cpp.) */
static float gx_filt = 0.0f;
static float gy_filt = 0.0f;
static float gz_filt = -9.81f;   /* assume face-up at boot */
```

- [ ] **Step 3: Remove the cursor-LPF reset in `gesture_mode_init`.** It currently reads:

```c
    gx_filt = 0.0f;
    gy_filt = 0.0f;
    gz_filt = -9.81f;
    gx_cursor = 0.0f;
    gy_cursor = 0.0f;
    gz_cursor = -9.81f;
    filter_initialised = false;
```

Remove the three `gx_cursor`/`gy_cursor`/`gz_cursor` lines:

```c
    gx_filt = 0.0f;
    gy_filt = 0.0f;
    gz_filt = -9.81f;
    filter_initialised = false;
```

- [ ] **Step 4: Remove the cursor-LPF seed + update in `gesture_mode_update_accel`.** The seed block currently reads:

```c
    if (!filter_initialised) {
        gx_filt = ax;    gy_filt = ay;    gz_filt = az;
        gx_cursor = ax;  gy_cursor = ay;  gz_cursor = az;
        filter_initialised = true;
        return;
    }
```

Drop the `gx_cursor` seed line:

```c
    if (!filter_initialised) {
        gx_filt = ax;    gy_filt = ay;    gz_filt = az;
        filter_initialised = true;
        return;
    }
```

Then the FAST-IIR update block currently reads:

```c
    /* SLOW 1-pole IIR (pose / classifier / cone gate / shadow) -- keep slow. */
    gx_filt += GRAVITY_LP_ALPHA * (ax - gx_filt);
    gy_filt += GRAVITY_LP_ALPHA * (ay - gy_filt);
    gz_filt += GRAVITY_LP_ALPHA * (az - gz_filt);

    /* FAST 1-pole IIR (cursor Y driver ONLY) -- low lag.  INTERIM (a); the
     * destination is a gyro-fused inclination from the Mahony quaternion. */
    gx_cursor += s_cursor_alpha * (ax - gx_cursor);
    gy_cursor += s_cursor_alpha * (ay - gy_cursor);
    gz_cursor += s_cursor_alpha * (az - gz_cursor);
```

Remove the FAST block entirely (keep the slow one):

```c
    /* SLOW 1-pole IIR (pose / classifier / cone gate / shadow) -- keep slow. */
    gx_filt += GRAVITY_LP_ALPHA * (ax - gx_filt);
    gy_filt += GRAVITY_LP_ALPHA * (ay - gy_filt);
    gz_filt += GRAVITY_LP_ALPHA * (az - gz_filt);
```

- [ ] **Step 5: Delete `gesture_mode_adjust_cursor_alpha()`.** Remove the whole function from `src/gesture_mode.cpp`:

```c
float gesture_mode_adjust_cursor_alpha(float factor)
{
    /* Multiplicative step (so the perceived smoothness scales smoothly), then
     * clamp to a sane band.  Floor 0.03 (~0.3 s tau, very smooth but laggy);
     * ceil 0.50 (very snappy, lets through a lot of accel noise -- past this
     * there's no point, the raw signal dominates). */
    s_cursor_alpha *= factor;
    if (s_cursor_alpha < 0.03f) s_cursor_alpha = 0.03f;
    if (s_cursor_alpha > 0.50f) s_cursor_alpha = 0.50f;
    return s_cursor_alpha;
}
```

- [ ] **Step 6: Remove its declaration from `src/gesture_mode.h`.** Delete:

```c
/*
 * Live-tune the FAST cursor gravity-filter smoothing coefficient.
 * `factor` multiplies the current alpha (e.g. 0.8 = smoother/laggier,
 * 1.25 = sharper/snappier), clamped to [0.03, 0.50].  Returns the new
 * alpha so the caller can log it.  Wired to the serial console 'o'/'p'
 * keys -- lets the user dial cursor responsiveness vs jitter to taste
 * without a rebuild.  INTERIM: superseded by the gyro-fused inclination.
 */
float gesture_mode_adjust_cursor_alpha(float factor);
```

- [ ] **Step 7: Remove the `o`/`p` serial-console handlers in `src/main.cpp`.** First the input-parse arm:

```c
        } else if (c == 'o') {
            // Cursor SMOOTHER (lower the fast-filter alpha -> more smoothing,
            // slightly more lag).  Live knob for responsiveness vs jitter.
            pending_cmd = 'o';
        } else if (c == 'p') {
            // Cursor SHARPER (raise the fast-filter alpha -> snappier, more
            // accel noise).
            pending_cmd = 'p';
        }
```

becomes (drop the two `else if` arms, keep the closing `}` of the preceding `{` arm):

```c
        }
```

Then the dispatch arm:

```c
        } else if (cmd == 'o' || cmd == 'p') {
            pending_cmd = 0;
            /* Live-tune the FAST cursor gravity-filter alpha.  'o' = smoother
             * (lower alpha, more lag), 'p' = sharper (higher alpha, snappier
             * but more accel noise).  Multiplicative step, clamped in
             * gesture_mode_adjust_cursor_alpha.  tau ~= 10ms / alpha. */
            float factor = (cmd == 'p') ? 1.25f : 0.8f;
            float a = gesture_mode_adjust_cursor_alpha(factor);
            LOG_INF("Cursor smoothing %s -> alpha=%.3f (~%d ms tau)",
                    (cmd == 'p') ? "SHARPER" : "SMOOTHER",
                    (double)a, (int)(10.0f / a));
        } else if (cmd == 'y') {
```

becomes (remove the `o`/`p` branch; the `y` branch stays):

```c
        } else if (cmd == 'y') {
```

(The `o`/`p` keys are not in the printed help list, so no `LOG_INF` help text needs changing.)

- [ ] **Step 8: Remove `CURSOR_GRAVITY_ALPHA` from `src/gesture_thresholds.h`.** Delete:

```c
/* SEPARATE faster gravity LPF for the CURSOR Y driver only (gx/gy/gz_cursor in
 * gesture_mode.cpp).  ~0.15 -> tau ~67 ms @100 Hz, so the cursor lags the wrist
 * ~1-2 deg instead of the ~18 deg the slow filter caused.  INTERIM (a): an
 * accel-only filter still can't reject linear-accel transients on a fast flick;
 * the destination is a gyro-fused inclination from the Mahony quaternion.
 * Do NOT merge with GRAVITY_LP_ALPHA -- different consumers, different needs. [USER] */
#define CURSOR_GRAVITY_ALPHA            0.15f
```

(Leave `GRAVITY_LP_ALPHA` — the slow filter — untouched.)

- [ ] **Step 9: Confirm no orphans, then build.**
Run: `grep -rn "gx_cursor\|gy_cursor\|gz_cursor\|s_cursor_alpha\|CURSOR_GRAVITY_ALPHA\|adjust_cursor_alpha" src/`
Expected: **no matches** (all removed).
Run: `./build.sh`
Expected: clean build, artifact produced.

- [ ] **Step 10: Regression — host suites still pass.**
```bash
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc
```
Expected: both print `ALL PASS` (neither depends on the changed path).

- [ ] **Step 11: Commit.**
```bash
git add src/gesture_mode.cpp src/gesture_mode.h src/main.cpp src/gesture_thresholds.h
git commit -m "feat(cursor): drive vert from fused gravity; drop accel-only LPF + o/p knob"
```

---

## Task 3: HW acceptance, then strip the `[CURSOR]` diagnostic (post-flash)

**Files:**
- Modify (after acceptance): `src/gesture_mode.cpp` (the `[CURSOR]` telemetry block in `gesture_mode_update_gyro`, ~line 1118-1131)

- [ ] **Step 1: Flash.** `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/` (double-tap RESET first). Pointer acceleration OFF on the Mac (gotcha).

- [ ] **Step 2: HW acceptance (read the `[CURSOR]` log).** Confirm all three:
  1. **Spike immunity (headline):** tap the band / jolt the hand while in AIR_MOUSE → the logged `vert` does **not** spike, while `vinst` still does. The `vert`-vs-`vinst` gap on a tap is the proof the linear-accel leak is gone.
  2. **No lag:** slow and fast wrist tilts → cursor tracks immediately; at a stop `vert` and `vinst` agree.
  3. **Calibration intact:** entry logs `[CAL] … decision=SEED/ADOPT` with the bottom ≈ your rest; cursor reaches both screen edges.
  If any fails, STOP and report (do not strip the log) — the likely lever is `ORI_TWO_KP`, but only with a check that it does not regress pose/stillness (out of scope here; escalate).

- [ ] **Step 3: Strip the `[CURSOR]` telemetry.** Once acceptance passes, remove the `~50 Hz` diagnostic block in `gesture_mode_update_gyro` (the `static int cursor_tel_ctr` block that logs `[CURSOR] vert=… vinst=…`). Keep the `cursor_track_update` + `cursor_pipeline_inject_motion` calls; remove only the `LOG_INF("[CURSOR] …")` block and its `vinst`/`amag`/`cursor_tel_ctr` scaffolding.

- [ ] **Step 4: Build + commit.**
Run: `./build.sh` (expect clean) and `grep -n "\[CURSOR\]" src/gesture_mode.cpp` (expect none).
```bash
git add src/gesture_mode.cpp
git commit -m "chore(cursor): strip [CURSOR] diagnostic log (gyro-fused vert validated)"
```

---

## Self-review

- **Spec coverage:** core change (fused `current_vert_deg`) = Task 2 Step 1; expose gravity = Task 1; single-source-of-truth = inherent (one switch point); remove dead accel path + `o`/`p` + `CURSOR_GRAVITY_ALPHA` = Task 2 Steps 2-8; leave `ORI_TWO_KP` untouched = no task touches it (correct); verification + `[CURSOR]` strip = Task 3. All spec sections covered.
- **No placeholders:** every code step shows the exact before/after text.
- **Type consistency:** `orientation_state_t.gravity[3]` defined in Task 1 Step 1, read in Task 2 Step 1 as `ori.gravity[0..2]` — consistent.
