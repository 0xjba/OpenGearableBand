# Gyro-fused cursor vertical signal — design

**Date:** 2026-06-15
**Status:** approved design, not yet built.
**Branch:** `feature/gesture-foundation`.

## Problem (fact-based, from HW)

The air-mouse vertical driver `vert` is computed from an **accelerometer-only**
low-pass filter:

```c
/* gesture_mode.cpp */
gx_cursor += s_cursor_alpha * (ax - gx_cursor);   /* alpha = CURSOR_GRAVITY_ALPHA = 0.15, live o/p */
...
current_vert_deg() = acos(|gx_cursor| / |g_cursor|) * 180/pi   /* roll-immune forearm-axis projection */
```

An accel-only filter **cannot separate gravity-tilt from hand linear-acceleration**.
This is visible in the `[CURSOR]` HW log: during motion the instantaneous inclination
`vinst` spikes (e.g. `vert=34 vinst=28`, `vert=75 vinst=88`) while filtered `vert`
barely moves, and the gap drives a `dy` kick (e.g. `dy=-25`). The hand feels this as
both "twitch" (the spike) and "chop" (a step that should not be there). Lowering the
alpha only trades twitch for lag — accel-only has no escape from this tradeoff.

The servo (`cursor_track`, P-controller ±127/tick) is **not** the dominant cause: on a
clean raise its `dy` is steady (10–15/tick). The fault is the input signal.

## Key fact that makes the fix cheap

The Mahony orientation filter (`orientation.cpp`) **already computes a gyro-fused,
accel-corrected, unit-norm gravity direction** in the band frame every update, and
currently uses it only internally:

```c
/* orientation.cpp:132-134 — gravity direction the current quaternion predicts */
float vx = 2.0f * (q1 * q3 - q0 * q2);
float vy = 2.0f * (q0 * q1 + q2 * q3);
float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
```

This vector is **responsive** (the gyro integrates into the quaternion at full rate, so a
wrist tilt moves it with zero lag) and **linear-accel-immune** (the accelerometer only
nudges it slowly via `ORI_TWO_KP`; a tap spike barely moves it). It is exactly the signal
the drift-mitigation note (`docs/research/cursor-drift-mitigation.md` §4.1) specified for
the vertical axis: "take pitch from the gravity-referenced orientation filter… drift-free."

## Design

### Core change
Compute `vert` from the **fused gravity vector**, using the *same roll-immune
forearm-axis projection* already in use — only the source changes:

```c
current_vert_deg() = acos(|gravity_x| / |gravity|) * 180/pi   /* gravity = fused, not accel-LPF */
```

We deliberately do **not** use the filter's Euler `pitch_deg` — it is roll-contaminated
near high roll (per CLAUDE.md). The forearm-axis projection of the gravity *vector* is
roll-immune (roll about the forearm axis does not change `gravity_x`), so the existing
geometry is preserved.

### Single source of truth (scope decision — approved)
`current_vert_deg()` feeds four consumers that must agree or the anchors desync:
1. the cursor servo (`cursor_track_update`),
2. the entry-snap top capture (`top_now` at AIR_MOUSE entry),
3. the rest-bottom capture (`last_rest_vert`),
4. the anchor-relative low-zone (`air_in_rest_zone()`).

All four move to the fused source together — `current_vert_deg()` is the one switch point,
so they stay consistent automatically.

**This is safe for the just-validated calibration.** At rest the fused gravity converges
exactly to the measured accelerometer direction (that is what the Mahony correction does),
so `last_rest_vert`, the anchors, and the low-zone — all captured while still — are
**unchanged**. The fused-vs-accel difference exists only *during motion*, which is exactly
where the cursor should be smoother. Calibration stays put; only the moving cursor improves.

### Filter gains — leave shared filter untouched
Responsiveness is inherent (gyro). Smoothness/spike-immunity is governed by `ORI_TWO_KP`.
That filter is **shared** with pose detection, the dictation discriminator, and stillness
detection. Ship with the **existing `ORI_TWO_KP`/`ORI_TWO_KI`**; do not retune for cursor
feel (a good orientation estimate already serves all consumers). Revisit only if HW shows a
problem, and only without regressing pose/stillness.

### Removed (dead after the switch)
`gx_cursor`/`gy_cursor`/`gz_cursor`, `s_cursor_alpha`, `CURSOR_GRAVITY_ALPHA`,
`gesture_mode_adjust_cursor_alpha()`, and the `o`/`p` serial knob (the accel-LPF was always
labelled interim). If bring-up later shows we want to tune `ORI_TWO_KP` live, add a fresh
knob then — not pre-emptively (YAGNI).

## Components touched (small, well-bounded)

- **`orientation.h` / `orientation.cpp`** — add `float gravity[3]` to
  `orientation_state_t`; `orientation_get()` fills it from the quaternion (the same 3 lines
  above). No change to filter behaviour — pure copy-out. (Decision: use the struct field in
  the existing single accessor, not a separate getter — one accessor, one fetch.)
- **`gesture_mode.cpp`** — `current_vert_deg()` reads the fused gravity (via
  `orientation_get`) instead of `gx_cursor`. Delete the dead accel-LPF state + alpha knob +
  the `o`/`p` console handlers. The `gx_cursor` update block in `gesture_mode_update_accel`
  is removed.
- **`gesture_thresholds.h`** — remove `CURSOR_GRAVITY_ALPHA`.

### Threading / freshness
`orientation_update()` runs in `gesture_mode_update_gyro`. `current_vert_deg()` is called:
- in `update_gyro` *after* `orientation_update` (cursor servo) → fresh;
- in `update_accel` (rest tracker) → reads the gravity from the previous gyro sample → one
  sample (10 ms) stale. Benign — the rest tracker only acts when still, where the value is
  constant. Same benign-race class the file already documents.

## Verification

- **Build:** `./build.sh` clean.
- **Host tests:** `cursor_track` + `cursor_calib` still `ALL PASS` (neither depends on the
  changed path; the new math is a pure projection). `orientation.cpp` is not host-tested
  (pulls Zephyr logging) — verified on HW.
- **HW acceptance (via the kept `[CURSOR]` log — `vinst` is the control):**
  1. **Tap / jolt the band while pointing → fused `vert` does NOT spike** while `vinst`
     still does. The `vert`-vs-`vinst` gap is the direct proof of linear-accel immunity.
     This is the jitter/twitch fix, measured — the headline acceptance.
  2. **Slow and fast wrist tilt → cursor tracks with no perceptible lag** (fused `vert`
     follows the wrist immediately; at a stop `vert` and `vinst` agree).
  3. **Calibration unchanged** — entry still logs `[CAL] … SEED/ADOPT` with the same bottom
     at rest; cursor reaches both screen edges.
- **After acceptance:** strip the `[CURSOR]` diagnostic log (it was kept specifically for
  this tuning step; the last remaining debug-log item on the cursor branch).

## Out of scope (explicitly)

- Servo / quantization smoothing (output slew, sub-pixel) — only if stair-stepping remains
  *after* this lands; the data says the input filter is the cause, so this is held in
  reserve, not built now.
- Horizontal (X) axis — separate task; this is the vertical-Y signal only.
- Yaw-drift mitigation (`cursor-drift-mitigation.md` §4.2–4.4) — that is the X-axis future
  work; unrelated to this vertical-signal change.
