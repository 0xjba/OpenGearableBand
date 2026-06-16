# Sweep-Plane Coupling Compensation (Cursor X-arc fix) — Design Spec

**Date:** 2026-06-16
**Status:** approved-in-principle (Option 1); HARDWARE is the validation gate
**Branch:** feature/gesture-foundation
**Builds on:** the yaw-driven X axis (`docs/superpowers/specs/2026-06-15-horizontal-x-axis-design.md`)

---

## 1. Problem (measured)

A natural forearm sweep about a planted elbow swings on a *tilted* plane, so one
rotation projects onto **both** azimuth (the X we want) **and** elevation (the Y
driver `vert`). Measured on HW (`[XARC]` trace, 2026-06-16): during a horizontal
sweep `vert` rides a U-shape — ≈35° at center, rising to ≈46–50° at the extremes
(~15° swing) — so the cursor bows into an arch instead of tracking a straight
horizontal line. The X axis itself is correct; the arc is a coupling in Y.

Research verdict (Hillcrest/Freespace patents EP2337016, US7970571, US9958962):
the standard gravity-leveled spherical map (heading→X, elevation→Y) — which we
already do — faithfully reports this arc, because a tilted-cone sweep *is* an arc.
Straightening it is a deliberate ergonomic remap (align the pointing frame to the
user's swing plane), not a sensor fix. See `docs/research/cursor-drift-mitigation.md`.

## 2. Approach (Option 1: fixed, tunable swing-tilt — chosen)

Subtract the measured coupling from the Y driver before the servo:
```
corrected_vert = vert − k · sweep²
```
- `sweep` = horizontal displacement from the entry center, in **degrees** of yaw
  (gain-invariant; see §3).
- `k` = a single tunable curvature constant (the swing-plane tilt, lumped). It is
  the v1 approximation of the exact swing-cone shape; a symmetric quadratic fits
  the measured U-shape well over the usable sweep range.

Effect:
- **Pure horizontal sweep:** `vert` rises along ≈ this parabola → `corrected_vert`
  ≈ constant → Y servo holds → straight line.
- **Deliberate up/down (and diagonals):** real elevation moves `vert` *beyond* the
  parabola → `corrected_vert` moves → Y tracks normally. The correction removes
  only the sweep-coupled elevation, not intent.
- **`k = 0` disables it** (identical to today's behavior) — fully regressable.

Chosen over per-entry calibration (Option 2) and online fit (Option 3) for v1:
smallest change, no ritual, tuned live like the gains. Graduate to Option 2
(per-user capture of `k`) if a single fixed `k` proves posture-sensitive.

## 3. The `sweep` variable, center, and drift

`corrected_vert` is applied inside `cursor_track` (which owns the Y servo, the
gains, and — newly — the X position). The X axis gains a position accumulator
`s_cur_x` (integral of the emitted `dx`, in counts), mirroring the existing
`s_cur_y`. Then:
```
sweep_deg = s_cur_x / gain_x          (counts / (counts/deg) = deg)
correction = k · sweep_deg²
```
- **Center** = the entry pose: `s_cur_x = 0` at `cursor_track_start`, so `sweep=0`
  and `correction=0` at entry → the top-anchor snap, the bottom-rest capture, and
  the natural-Y calibration are all untouched (correction is exactly 0 there).
- **Gain-invariant:** dividing by `gain_x` means tuning the X gain does NOT change
  the arc correction (the correction tracks physical sweep *angle*, not pixels).
- **Cone-gate / at-rest:** when X is frozen (cone-invalid near vertical, or
  at_rest), `dx=0`, so `s_cur_x` holds and the correction holds — no jump.
- **Drift caveat (known, v1):** `s_cur_x` accumulates from the floating yaw origin,
  so on a long drifting session the parabola's center can wander, degrading the
  straightening until a pause/re-entry. Same class of limit as the X axis itself;
  re-entry resets `s_cur_x=0`. Acceptable for v1.

## 4. Components / files

- **`src/cursor_track.{h,cpp}` (PURE, host-tested — main change):**
  - Add `s_cur_x` (accumulate emitted `dx`; reset to 0 in `cursor_track_start`).
  - Add a runtime `s_swing_comp_k` (seeded from `CURSOR_SWING_COMP_K`) + a setter
    `cursor_track_set_swing_comp(float k)` and getter (for the serial knob).
  - In `cursor_track_update`, compute `corrected_vert = vert_deg − s_swing_comp_k *
    sweep_deg²` (with `sweep_deg = s_cur_x / s_gain_x`; guard `s_gain_x != 0`) and
    feed `corrected_vert` to `target_counts()` in place of raw `vert_deg`. The slam
    path, anchors, and X output are unchanged.
  - Introspection getter for `sweep_deg`/`corrected_vert` for tests + telemetry.
- **`src/gesture_thresholds.h`:** add `CURSOR_SWING_COMP_K` (default `0.003`
  deg/deg², documented `[USER][HOUSING]`). First-guess from the trace: ~15° of
  coupling at ~70° sweep → k ≈ 15/70² ≈ 0.003. Ships a visible partial correction
  to tune from; `k=0` (dial all the way down) is the regressable no-op.
- **`src/main.cpp`:** a live tuning knob for `k` on the freed `'o'` / `'p'` keys
  (up/down, additive step), printing the new `k`; add to the serial help.
- **`src/gesture_mode.cpp`:** no logic change. Keep the temporary `[XARC]` log
  (now also useful to watch `vert` flatten as `k` is tuned); strip it once the
  arc is validated fixed.

## 5. Testing

**Host unit tests** (extend `tests/test_cursor_track.cpp`):
1. **`k=0` is identity:** with `k=0`, Y behavior is byte-identical to today (a
   regression guard).
2. **Parabola subtraction:** with a known `k`, drive `s_cur_x` to a known sweep and
   confirm the Y target uses `vert − k·sweep²` (servo target shifts by the expected
   counts).
3. **Center is unaffected:** at `s_cur_x=0` (entry/center), correction is 0 → Y
   target equals the raw-`vert` target.
4. **Deliberate vertical survives:** at a fixed non-zero sweep, increasing `vert`
   beyond the parabola still increases the Y target (intent preserved).

**Hardware acceptance (the real gate):** flash; in AIR_MOUSE, sweep left↔right and
turn `k` up via `'o'`/`'p'` until the bow flattens to a straight line; confirm a
deliberate up/down still moves Y and diagonal targeting still works; watch `[XARC]`
`vert` go flat across a sweep as `k` converges. Record the tuned `k` into
`CURSOR_SWING_COMP_K`.

## 6. Out of scope (v1)

- Per-user / per-entry capture of `k` (Option 2) — graduation path if posture-sensitive.
- Asymmetric or exact swing-cone (cosine) correction — quadratic approximation first.
- Online running estimation (Option 3).
- Any change to the X output, slam, natural-Y calibration, or exit model.
