# Spec: Air-Mouse Cursor — pointing (movement) v1

**Date:** 2026-06-11
**Status:** Design for review.
**Scope:** v1 = **pointing only** (no clicks). Wire AIR_MOUSE entry to the
cursor, and turn the drift-free orientation into relative mouse motion.
SURFACE cursor is a later, separate pass.
**Grounding:** `docs/research/cursor-drift-mitigation.md` (drift spec),
`docs/research/observability-aware-pose-and-cursor-design.md` §3.5 (the
gx/gy/gz geometry + the cone), `orientation.{h,cpp}` (the foundation),
`cursor_pipeline.{h,cpp}` (the output path, already built).

---

## 1. Architecture — three units, clean seams

```
gesture_mode (FSM)  ──▶  cursor_track (NEW)  ──inject_motion──▶  cursor_pipeline (EXISTS)  ──▶ BLE HID
 mode + entry/exit       orientation → dx,dy      125 Hz publish, dead-zone, int8
```

- **cursor_pipeline** — *unchanged*. Already gates on `MODE_AIR_MOUSE`,
  applies sensitivity + 1 px dead-zone + int8 quantization, calls
  `ble_hid_send_report`. Entry point: `cursor_pipeline_inject_motion(dx,dy)`.
- **cursor_track** (NEW, `src/cursor_track.{h,cpp}`) — the only new pointing
  logic. Pure function of its inputs; no global reads. API:
  - `cursor_track_start(float pitch0, float roll0)` — on AIR_MOUSE entry;
    captures the reference (`prev_pitch/prev_roll`) so there's no entry jump,
    and inits `roll_valid = false` so X stays gated until the shadow clearly
    clears the cone (no first-frame garbage-roll move).
  - `cursor_track_update(float pitch, float roll, bool at_rest, float shadow)`
    — per tick; computes `dx,dy` and calls `cursor_pipeline_inject_motion`.
  - `cursor_track_stop(void)` — on exit; clears state.
- **gesture_mode** (MODIFY) — wires entry/exit and feeds `cursor_track`.

## 2. Mode wire-up (F2)

- `multi_tap_commit_handler` `case POSE_AIR_MOUSE`: replace the `LOG_INF`
  stub with `_transition_to(MODE_AIR_MOUSE)` (already exists; the `t` serial
  command uses it). That opens the `cursor_pipeline` mode gate.
- In `_transition_to`: entering `MODE_AIR_MOUSE` → `cursor_track_start(pitch,
  roll)` (current fused angles). Leaving it → `cursor_track_stop()`.
- **Exit reuses the existing FSM logic** — orientation leaves UP_RAISED /
  flick-to-cancel → `_transition_to(MODE_IDLE)`. No new exit path.
- `cursor_track_update` is called from `gesture_mode_update_gyro` (right after
  `orientation_update`, so the fused orientation + gravity-LPF are current)
  **only when** `gesture_mode_get() == MODE_AIR_MOUSE`.

## 3. The pointing math (cursor_track_update, ~100 Hz)

Inputs: `pitch, roll` from `orientation_get()` (fused quaternion — gyro-
responsive AND gravity-drift-free; NOT the slow 1 s gravity-LPF). `at_rest`
from `orientation_get()`. `shadow = √(gy²+gz²)` from the gravity-LPF
(gx/gy/gz_filt — the §3.5 cone signal). Relative / rate model.

```
dpitch = wrap180(pitch - prev_pitch)
droll  = wrap180(roll  - prev_roll)

// (3c) Euler-wrap / glitch guard: a real wrist move is << this per 10 ms.
if (|dpitch| > CURSOR_MAX_DELTA_DEG) dpitch = 0
if (|droll|  > CURSOR_MAX_DELTA_DEG) droll  = 0

// (3b) Asymmetric freeze: dwell-engage (at_rest), immediate sensitive release.
ang_speed = |dpitch| + |droll|
frozen = at_rest && (ang_speed < CURSOR_FREEZE_RELEASE_DELTA)

// (3a) Cone-gate the X axis WITH HYSTERESIS (latched valid/invalid).
if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE) roll_valid = true
else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) roll_valid = false
// between the two: roll_valid holds

if (frozen) { dx = 0; dy = 0 }
else {
    dy = CURSOR_GAIN_Y * dpitch                 // vertical: pitch always observable
    dx = roll_valid ? CURSOR_GAIN_X * droll : 0 // horizontal: only when roll trustworthy
}
cursor_pipeline_inject_motion(dx, dy)
prev_pitch = pitch; prev_roll = roll            // ALWAYS re-sync -> no jump on un-freeze/un-gate
```

Why each amendment:
- **(3a) Cone-gate hysteresis** — a single shadow threshold would flap `dx`
  on/off when hovering near it (the bug class we just fixed in the
  orientation classifier). Two thresholds (invalidate < 3.5, revalidate >
  4.5) latch the state; the every-tick `prev` re-sync guarantees no jump on
  re-entry. `BUILD_ASSERT(REVALIDATE >= INVALIDATE)`.
- **(3b) Asymmetric freeze** — `at_rest` engages slowly (its dwell, fine for
  parking) and releases immediately, but only above 5.7 °/s gyro; the per-
  tick `ang_speed` release (a smaller angle/tick) un-freezes on *slow*
  deliberate moves so the cursor never feels sticky.
- **(3c) Δ clamp** — `wrap180` handles the roll (atan2) ±180 discontinuity;
  the `MAX_DELTA` guard discards any remaining glitch so a wrap can't
  teleport the cursor. The operating envelope (pitch −34..−67, roll 17..94)
  sits away from the wraps, so this is belt-and-suspenders, but permanent.
- **Re-sync `prev` every tick** — deltas are always tick-to-tick, so a frozen
  or cone-gated tick discards that motion instead of accumulating a jump.

`CURSOR_GAIN_X/Y` are **signed**; on-hardware we confirm "up = up / right =
right" and flip a sign if inverted.

## 4. Constants (`gesture_thresholds.h`, all empirical `[USER]`)

| Constant | Initial | Meaning |
|---|---|---|
| `CURSOR_GAIN_Y` | `8.0f` | pitch Δdeg → px (signed; flip on HW if inverted) |
| `CURSOR_GAIN_X` | `8.0f` | roll Δdeg → px (signed; flip on HW if inverted) |
| `CURSOR_ROLL_SHADOW_INVALIDATE` | `3.5f` | gate X off below this shadow |
| `CURSOR_ROLL_SHADOW_REVALIDATE` | `4.5f` | re-enable X above this (hysteresis) |
| `CURSOR_FREEZE_RELEASE_DELTA` | `0.15f` | per-tick \|Δangle\| (deg) that un-freezes |
| `CURSOR_MAX_DELTA_DEG` | `30.0f` | discard Δ above this (wrap/glitch guard) |

Initial values are starting points, tuned on hardware (per-mount). `GAIN`
8.0 px/deg gives a brisk response (a ~1–3°/tick twist → ~8–24 px/tick at
125 Hz); `FREEZE_RELEASE_DELTA` 0.15°/tick sits above the angle-noise floor
(~0.05°/tick) so noise can't un-freeze, but any real move does.

(Shadow thresholds bracket the measured cone: vert 15 → shadow 2.4 invalid,
vert 31 → shadow 5.0 valid. Gains/freeze tuned on feel; per-mount.)

## 5. Telemetry / hygiene
- Verify EVERY `at_rest` log field is `(int)`-cast (pose-trace fixed
  `f371c34`; check the pose-arm ORI line) — the freeze now depends on
  `at_rest`, so it must be observable while tuning.
- An optional throttled `[CURSOR] dpitch/droll dx/dy frozen rollvalid` debug
  line (rate-limited) helps tune gains without spamming.

## 6. Testing (on-host, BLE)
Pair the band as a BLE mouse, enter AIR_MOUSE (pose + double-tap), then:
- Tilt wrist up/down → cursor moves vertical; twist wrist → cursor
  horizontal (flip a gain sign if inverted).
- Hold still → cursor parks (freeze); start a *slow* move → it responds
  immediately (no stickiness).
- Raise toward vertical → horizontal output freezes (cone gate) while
  vertical still tracks; lower back → horizontal resumes with no jump.
- Exit (lower out of UP_RAISED / flick) → cursor stops.
Tune `GAIN_X/Y`, the shadow pair, and `FREEZE_RELEASE_DELTA` on feel.

## 7. Future (v2+, explicitly NOT now)
- **Lag:** v1 deltas inherit the orientation filter's Mahony lag (~10° seen
  during the fast sweep). If fast moves feel swimmy, the fix is **gyro-rate
  deltas corrected by gravity (complementary)** — NOT cranking the gains
  against a lag problem. Do not tune gains to mask lag.
- Clicking (the in-session tap-while-moving problem,
  `finding_tap_while_moving_2026_06_11`), scroll/right-click, SURFACE cursor,
  absolute/laser-pointer mode, per-user gain calibration.

## 8. Out of scope (v1)
Any clicks/buttons, SURFACE cursor, the lag fix, calibration UX.
