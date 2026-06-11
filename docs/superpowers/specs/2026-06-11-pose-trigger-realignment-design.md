# Spec: Pose-Trigger Realignment — gesture-led entry + orientation hysteresis

**Date:** 2026-06-11
**Status:** Design for approval (no code yet).
**Supersedes:** the roll-split dictation discriminator (commit `60feb91`).
**Grounding:** `docs/research/observability-aware-pose-and-cursor-design.md`
(measurement campaign), memory `decision_dictation_voice_gated_entry`.

---

## 1. Problem

Two live bugs, both confirmed on hardware:

1. **Roll-split mislabels air-mouse as DICTATION.** Commit `60feb91` split
   AIR_MOUSE↔DICTATION by `roll = atan2(gy,gz)` (threshold 85°) and entered
   DICTATION on a double-tap. It fails because:
   - Near a vertical forearm, roll is in the gravity blind spot — a steady
     air-mouse crane read roll 87° one session, 145° another (same intent).
   - A held **max-right** air-mouse reach is **gravity-identical to
     dictation** (measured: gz −2.5 / roll 113 vs dictation −2.1 / 108 — a
     *full overlap*, not a thin margin).
   - **Conclusion: gravity/roll cannot discriminate air-mouse from
     dictation. Pose-only discrimination is impossible.**

2. **Coarse orientation classifier flaps.** During a normal left-right
   air-mouse sweep, `_classify_orientation` flipped `UP_RAISED↔NEUTRAL` six
   times in 30 s. The 25-sample dwell is too weak.

## 2. Principle (settled, now proven by data)

**The confirming GESTURE decides the mode; the pose only arms.** Each
mode's gesture is deliberately different, and that difference — not the
gravity pose — is the discriminator.

| Mode | Pose (arms) | Confirming gesture (decides) | Status |
|---|---|---|---|
| AIR_MOUSE | raised hemisphere | cadenced double-tap | active |
| SURFACE | wrist on desk | cadenced double-tap + hard-surface spectral | active |
| DICTATION | bring-to-face | **clench → mic → voice** | **deferred (PPG phase)** |

The max-right overlap is the proof of *why* this must be gesture-led: even
a perfect gravity classifier can't separate a max-right air-mouse from
dictation, but the gestures (taps vs clench+voice) trivially do.

## 3. Changes (this spec)

### 3.1 Remove the roll-split
- `pose_fsm_update()`: delete the `roll ≥ DICTATION_ROLL_THRESH_DEG →
  POSE_DICTATION` reassignment. The raised hemisphere arms as
  `POSE_AIR_MOUSE`, full stop.
- `multi_tap_commit_handler()`: delete `case POSE_DICTATION:` (the
  double-tap → DICTATION path). A cadenced double-tap in the raised
  hemisphere → **AIR_MOUSE**. SURFACE path unchanged.
- `gesture_thresholds.h`: remove `DICTATION_ROLL_THRESH_DEG` (no longer
  used). Leave a one-line note pointing to this spec / §8b so it isn't
  re-added.
- `POSE_DICTATION` canonical stays disabled (tol 2.0) — it is never armed
  by gravity; dictation entry is future via clench+voice, not a pose cone.

### 3.2 Redefine the raised-zone on the unified gravity geometry (fix the flapping)

**Superseded approach:** asymmetric-dwell hysteresis was tried and FAILED
hardware verification (2026-06-11) — a wide air-mouse sweep dwells ~0.8 s+
at each Y-dominant extreme, exceeding any reasonable dwell. Root cause is
classifier *semantics*, not dwell length.

**Fix:** redefine `_classify_orientation`'s raised-zone using the **unified
gravity-component geometry** (see `observability-aware-pose-and-cursor-
design.md` §3.5 — same `gx/gy/gz` signals as the cone and gz-sign; do NOT
create a second pose-geometry definition):
- `UP_RAISED` = `gx > 0` AND `gx > RAISED_ELEVATION_RATIO · |gz|`. The **gy
  (sweep) axis is ignored**, so a wide reach (gy large) stays raised.
- `DOWN_FLAT` = `gz > 0` AND `gz` dominates both `|gx|` and `|gy|`.
- else `NEUTRAL`.
- Keep a short dwell for transient rejection (the existing
  ENTER/LEAVE dwell from the prior attempt is fine and harmless — the
  redefinition means NEUTRAL is no longer proposed mid-sweep, so the dwell
  rarely fires; do not over-engineer it further).

**`RAISED_ELEVATION_RATIO` is an empirical threshold** (not designed):
initial ~1.1 (observed min `gx/|gz|` across the raised sweep ≈1.31 → margin),
calibrated/refined against the cross-session adversarial traces.

- **Regression test (named):** the 2026-06-11 `POSE-TRACE` left-right sweep
  log must produce **zero** `UP_RAISED↔NEUTRAL` transitions. (The
  asymmetric-dwell build still showed ~20 in 30 s — that is the baseline to
  beat with the redefinition.)

### 3.3 Signal hygiene (so the bug class can't return)
- Any gravity-orientation decision operates on the **gravity-LPF
  components** (`gx_filt/gy_filt/gz_filt`) only — **never**
  `orientation_get().roll_deg` (the quaternion Euler roll, which lags ~10°
  during motion; the trace showed filter-roll 94° while gravity-roll was
  83°). State this explicitly in a code comment at the decision site.

## 4. Reserved for the future (documented here, NOT built now)

- **Cone** (forearm-from-vertical, ≈vert 20–22° / shadow ≈4, **per-mount
  calibrated, not frozen**): reserved for the **cursor's roll→X axis**
  reliability and near-vertical handling. NOT used for pose discrimination.
- **Clench-gated dictation** (PPG-gestures phase): power ladder
  bring-to-face POSE (IMU, IDLE) → wake PPG → clench (white-knuckle
  perfusion; IMU-alone can't, isometric) → wake mic → voice. Preserves
  PPG-off-in-IDLE. Dictation stays **log-only** until then.
- **Max-right overlap** stays documented as the reason pose-only can't
  discriminate — guardrail against re-introducing a roll-split.

## 5. Calibration (per-mount, per friend's C1)
No air-mouse/dictation thresholds are frozen — none are needed, because the
gesture decides. The cone angle (future cursor) is a per-mount calibrated
parameter with a measurement procedure, never a hardcoded default. DIY tape
mount shifts the roll-zero between sessions, so re-measure per session.

## 6. Test plan
- **Regression:** sweep log → 0 `UP_RAISED↔NEUTRAL` flaps.
- **Hardware:**
  - Raised + cadenced double-tap → `MODE ENTRY: AIR_MOUSE`, including the
    steep crane that previously mislabeled — **no DICTATION on taps, ever**.
  - Held max-right air-mouse + double-tap → AIR_MOUSE (not DICTATION),
    despite its dictation-identical gravity.
  - SURFACE entry unchanged (double-tap + hard-surface spectral).
  - `at_rest` logs clean 0/1 (already fixed, `f371c34`).

## 7. Out of scope (explicit)
Cursor pointing (roll→X), clench detection, voice detection, F2
mode-transition wire-up (entry stays LOG_INF), snap-vs-tap discrimination.
