# Horizontal (X) Cursor Axis — Design Spec

**Date:** 2026-06-15
**Status:** approved (brainstorm complete)
**Branch:** feature/gesture-foundation
**Supersedes for X:** the current relative-roll X driver in `cursor_track.cpp`

---

## 1. Goal

Drive the air-mouse cursor's **horizontal (X)** axis from a comfortable
elbow-anchored forearm **sweep** (left ↔ right), mapped to screen position so a
given sweep angle always moves the cursor the same screen distance — an "arc"
feel matching the vertical axis, **not** a velocity/"nudge" feel.

## 2. The hard constraint (why X ≠ Y)

A left/right forearm sweep about the vertical line through the elbow is **yaw**
(rotation about the gravity axis). On this 6-axis IMU (LSM6DSL, **no
magnetometer**) gravity is invariant under yaw, so there is **no absolute heading
reference**: the sweep angle is only recoverable by **integrating the gyro**,
which drifts. This is information-theoretic, not a tuning gap — confirmed by the
literature and by the closest real product (Wii MotionPlus resets pitch/roll from
gravity "but not yaw," and needs the IR sensor bar to anchor absolute yaw; we have
no such anchor). See `docs/research/cursor-drift-mitigation.md`.

Consequence: vertical is **truly absolute** (gravity-locked, fixed top/bottom
angles). Horizontal is **pseudo-absolute** — angle-mapped within a sweep, with a
**floating origin** that is re-anchored at every natural pause to bound drift.

## 3. Mechanism — yaw-driven linear arc

**Driver:** the orientation filter's `yaw_deg` (heading about world-vertical),
which already carries the Mahony filter's live gyro-bias subtraction (ZARU).

**Mapping (linear angle → pixels):**
```
dx = gain_x · wrap180(yaw_deg − prev_yaw)      // per tick (~100 Hz)
```
clamped per tick to `CURSOR_MAX_DELTA_DEG`. Because the gain multiplies the
**angle delta** (not a rate with an acceleration curve), the cursor displacement
over a sweep is `gain_x · (yaw − yaw_at_entry)` — speed-independent: a 30° sweep
moves the same screen distance whether fast or slow. That linearity is the "arc"
feel and is the deliberate difference from the rejected rate/Gyration model.

Structurally this is a small change: the module already computes
`dx = gain_x · Δ(roll)`; we swap the angle source **roll → yaw** and keep the
linear gain. Roll is dropped as the X driver entirely (its comfortable range was
the original complaint).

## 4. Range — comfortable sweep span, no fixed angle anchors

There is no gravity-anchored "left-edge angle" (see §2), so range is set by a
**comfort span + gain**, not by fixed anchors:

- `CURSOR_YAW_HALF_SPAN_DEG` — comfortable elbow-anchored sweep to one side.
  Default **35°** (≈70° total arc): an easy seated sweep before the shoulder is
  recruited. Tagged `[USER]`; a tunable starting estimate, dialed on hardware.
- `gain_x ≈ (half-screen-width-counts) / CURSOR_YAW_HALF_SPAN_DEG`, seeded from
  the span and tuned live via the existing `]`/`[` serial knob (same workflow as
  the vertical gain).
- The user reaches the screen sides by sweeping ≈±span from the **current
  center**, which floats and re-syncs at pauses (§5). Over-sweeping past an edge
  pins the cursor at the OS edge (standard relative-mouse behaviour).

**Deferred (not v1):** a per-user X calibration ritual (sweep-left-then-right to
capture the user's comfortable extremes, the horizontal analogue of the vertical
natural calibration). v1 ships fixed span + live gain knob.

## 5. Drift control — three layers, all already in the codebase

1. **Cone gate near vertical.** When the forearm nears straight-up, yaw
   degenerates (gimbal). The existing `shadow = √(gy²+gz²)` from the gravity-LPF
   cleanly measures distance-from-vertical (→0 near vertical). Reuse its
   hysteresis (`CURSOR_ROLL_SHADOW_INVALIDATE` / `_REVALIDATE`) to suppress X in
   that zone — same gate as today, now justified by gimbal proximity.
2. **Freeze + re-sync at pauses (the ZUPT re-anchor).** When `at_rest` OR
   cone-invalid, emit `dx = 0` and keep `prev_yaw` synced to the live `yaw_deg`
   every tick. The filter's at-rest yaw re-zero then happens while the cursor is
   frozen (invisible), and resume produces no jump. Drift accumulated during the
   last sweep is dropped at the pause.
3. **Bias tracking between pauses.** The Mahony integral feedback subtracts gyro
   bias continuously (measured residual ≈ 0.05°/s on this unit), so within-sweep
   drift is small; the pause wipes the remainder.

**Honest failure mode:** a long, slow, unbroken sweep with no pause lets the
center creep until the user pauses or sweeps back — the unavoidable
no-magnetometer artifact.

## 6. Components / files

- **`src/cursor_track.{h,cpp}` (PURE, host-tested — main change):**
  - `cursor_track_update()` X-driver arg `roll_deg → yaw_deg`; Y args
    (`vert_deg`, `at_rest`, `shadow`) unchanged. Update the header doc comment.
  - Rewrite the X branch per §3/§5; rename `s_prev_roll → s_prev_yaw`,
    `s_roll_valid → s_x_valid`. Keep: per-tick clamp, slam-suppression of X,
    cone-gate hysteresis, freeze-at-rest with prev resync every tick.
  - Y axis, slam, anchors, natural-calibration hooks: **untouched.**
- **`src/orientation.{h,cpp}`:** no change. `yaw_deg` (bias-tracked, at-rest
  re-zeroed) + `at_rest` already provide what X needs. (A cleaner
  horizontal-heading extraction near the cone edge is a possible follow-up, not
  v1.)
- **`src/gesture_mode.cpp` (`gesture_mode_update_gyro`):** pass `ori.yaw_deg` to
  `cursor_track_update` in place of `ori.roll_deg`; keep `shadow` from the
  gravity-LPF for the cone gate. One-line wiring change.
- **`src/gesture_thresholds.h`:** add `CURSOR_YAW_HALF_SPAN_DEG` (≈35, `[USER]`);
  keep `CURSOR_GAIN_X`, the cone constants, and `CURSOR_MAX_DELTA_DEG`.

## 7. Testing

**Host unit tests** (extend `tests/test_cursor_track.cpp`):
1. **Arc-linearity / speed-independence:** a yaw ramp of N° → total dx ≈
   `gain_x·N`, and the **same** total whether fed as a few big steps or many
   small ones.
2. **Freeze + re-sync:** `at_rest=true` → `dx=0`; the tick after rest releases
   produces no jump (prev resynced across the rest).
3. **Cone-gate hysteresis:** `shadow` < invalidate → `dx=0`; must exceed
   revalidate to resume.
4. **Wrap:** yaw crossing ±180° → no ~360° spike.
5. **Slam suppression:** during a Y slam, `dx=0`.

**Hardware acceptance:** in AIR_MOUSE, a left/right forearm sweep traces a
horizontal arc reaching both screen sides at ≈±35° (tune `gain_x` with `]`/`[`);
pausing wipes drift with no jump; near-vertical does not garbage the cursor; the
only artifact is creep on a long slow unbroken sweep.

## 8. Out of scope (v1)

- Per-user X calibration ritual (§4, deferred).
- Cleaner near-gimbal heading extraction in `orientation` (§6, follow-up if HW
  shows the cone-edge yaw too noisy).
- Any change to the vertical axis, slam, natural calibration, or exit model.
