# Observability-Aware Pose Classification & Cursor — Design Notes

**Date:** 2026-06-11
**Status:** Design *direction* settled. The cone threshold and dictation
signature are **pending measurement** (A/B/C pose traces on today's mount).
No classifier code written yet — measure first, then spec, then implement.

**Why this doc exists:** these decisions currently live in conversation +
volatile memory. Persisting them so a context compaction can't lose them.
Supersedes the roll-split conclusion in
[`orientation-and-pose-discrimination-findings.md`](orientation-and-pose-discrimination-findings.md).
Builds on [`cursor-drift-mitigation.md`](cursor-drift-mitigation.md) and the
memories `decision_dictation_voice_gated_entry`, `project_orientation_foundation`.

---

## 1. The mistake being corrected (the roll-split)

This session, post-compaction, a **roll/twist-based AIR_MOUSE↔DICTATION
split** was built (commit `60feb91`: `roll = atan2(gy,gz)`, threshold 85°).
That is the *blind-spot* approach the settled decision explicitly rejected.

- **Symptom:** a steady air-mouse pose with the forearm near-vertical
  mislabeled as DICTATION. Roll there is noise-dominated — when the forearm
  aligns with gravity, the Y-Z "shadow" `sqrt(gy²+gz²)` collapses (~1.8 of
  9.8), so `atan2(gy,gz)` random-walks across the threshold with no movement.
- **Also:** the entry handler entered DICTATION on a cadenced double-tap,
  which contradicts the voice-gated decision.
- **Action:** remove the roll-split; replace with the observability-aware
  classifier below.

## 2. The settled dictation model (restated, do not re-derive)

From `decision_dictation_voice_gated_entry` (the user's call, long settled):

- **DICTATION = wrist brought to the mouth, volar to the face** — a
  *bring-to-face GEOMETRY* (different forearm angle), **not** a palm twist.
- **Confirming gesture = VOICE** (near-field loud/clear mic), not taps.
  AIR_MOUSE / SURFACE confirm by cadenced taps. The differing gesture is
  itself the disambiguator.
- The bring-to-face pose **escapes the gravity blind spot** by changing the
  forearm angle (you cannot reach your mouth with a vertical forearm). Voice
  is the backstop for any residual overlap.
- Dictation mode does not exist yet → **log-only in v0** until voice lands.

## 3. The keystone — one observability cone for the whole product

Forearm pronation/supination ("which way the volar faces") **is** detectable
from gravity — but **only when the forearm is not near vertical.** When the
forearm aligns with gravity, the twist becomes rotation about the gravity
axis = **yaw**, which a 6-axis IMU (no magnetometer) cannot observe.
(Search-verified — see refs.)

Define the **observability cone** by the forearm-from-vertical angle,
equivalently the Y-Z shadow magnitude `sqrt(gy²+gz²)`:

- **Inside the cone** (forearm near vertical, shadow small) → **roll INVALID.**
- **Outside the cone** (forearm angled, shadow large) → **roll VALID.**

This single geometric parameter serves **both** subsystems:

| Subsystem | Inside cone (roll invalid) | Outside cone (roll valid) |
|---|---|---|
| Pose classifier | hard-block DICTATION | roll usable to classify |
| Cursor (roll→X mode) | horizontal axis falls back | roll drives horizontal |

One concept, one threshold (lives in `gesture_thresholds.h`), whole product.

## 4. Pose classifier plan (observability-aware) — to implement after measuring

1. **Roll-validity mask (A1).** Forearm within the cone of vertical → roll
   invalid → **hard-block entry into dictation**, regardless of entry path.
   Physically safe: dictation cannot have a vertical forearm. The crane lives
   inside the cone; dictation outside it. Kills most false positives alone.
2. **Dictation = conjunction (A2)**, not a region: elevation-in-band AND
   roll-in-volar-range AND roll-valid AND short stillness dwell. Each weak
   alone; the AND is strong.
3. **Sticky states / hysteresis (A3).** Require sustained high-confidence
   evidence to leave crane for dictation — no single frame flips it.
4. **Voice = commit gate (A4).** Nothing happens until near-field speech.
   (Future; dictation log-only until then.)
- **Gyro supination burst** during a crane→dictation transition = a
  confirming *tiebreaker vote*, not load-bearing (raw gyro integral was shown
  unreliable as a primary).

## 5. Cursor reframe — record for the (deferred) cursor work

1. **roll→X, pitch→Y (B1) — the big idea.** Both are gravity-observable, so
   **both axes become drift-free and yaw is removed entirely** — the
   `cursor-drift-mitigation.md` drift problem dissolves. From gravity,
   `pitch ≈ asin(−gx/g)` (X only) and `roll ≈ atan2(gy,gz)` (Y,Z only) — near-
   independent components, so no shared-state coupling in the *sensing*. The
   same observability cone is the only fallback (near-vertical forearm).
   **Open:** motor ergonomics — can you twist for X without tilting, and hold
   a twist for fine targeting? Supination range (~120°) is coarser than a yaw
   sweep. **Ship as a mode, A/B on yourself before committing.**
2. **Temperature-indexed bias table (B2) — DEMOTED.** Measured cold↔warm bias
   swing is negligible (≤0.05 dps, §6), so it's unneeded for correctness.
   Keep only if a long yaw sweep ever needs it; moot if B1 wins.
3. **Click-as-rezero (B3).** A click certifies cursor position → treat as a
   re-zero event; feed pre-click overshoot-nudge into the bias estimator.
   Host-side, no model. Demotes under B1.

If B1 holds ergonomically, the cursor doc's architecture inverts: gravity
carries both axes; ZUPT/bias-tracking demote to the near-vertical cone case —
the *same* cone as the pose classifier.

## 6. Measured facts (today's mount, 2026-06-11)

- **Gyro bias (this unit):** `[+0.67, −1.64, −0.56] dps`, stable to **≤0.05
  dps** across a cold→warm change AND a re-taped mount on a different day.
  Noise floor ~0.15 dps (Y/Z). Cold-start swing negligible → B2 unneeded.
- **PENDING (A/B/C pose traces via `v`):**
  - the cone angle (where roll destabilizes vs forearm-from-vertical);
  - dictation's bring-to-face gravity + roll signature;
  - separation of air-mouse-up (A) vs air-mouse-forward (C) vs dictation (B).
- **Caveat — fresh DIY tape mount each session:** re-measure pose canonicals,
  cone, etc. per session; band-X may not perfectly align with the forearm.
  Assume nothing from prior days.

## 7. Measurement tooling (serial commands)

- `z` — stationary gyro-bias trace (60 s; bias/noise per axis).
- `v` — pose-observability trace (30 s @5 Hz: gravity, shadow, forearm-from-
  vertical, pitch, roll, at_rest). Runs in IDLE; just hold the pose.
- `g` — single live gravity-vector dump (pose calibration).

## 8. References (added this session)

**6-axis yaw / heading unobservable without a magnetometer**
- IMU guide, JOUAV. <https://www.jouav.com/blog/inertial-measurement-unit.html>
- How many axes? Ceva. <https://www.ceva-ip.com/blog/motion-sensors-how-many-axes-do-you-need/>

**Pronation/supination IS detectable from gravity orientation**
- IMU wrist-rotation control (transradial prosthesis), PMC10734105. <https://pmc.ncbi.nlm.nih.gov/articles/PMC10734105/>
- Single-IMU arm-motion classification, BMC. <https://biomedical-engineering-online.biomedcentral.com/articles/10.1186/s12938-019-0677-7>

**Gyro captures supination during movement (roll axis), <4° RMS**
- MediaPipe+IMU pronation/supination estimation, MDPI. <https://www.mdpi.com/2076-3417/15/19/10527>
- IMU→joint-angle systematic review, PMC10386307. <https://pmc.ncbi.nlm.nih.gov/articles/PMC10386307/>

**Mic / always-on power**
- TDK T5838 AAD PDM mic ~20 µA. <https://invensense.tdk.com/news-media/tdk-low-power-t5838-mems-microphones-edge-ai-applications/>
- XIAO nRF52840 Sense onboard mic = MSM261D3526H1CPM (plain PDM, no AAD). <https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html>

(Madgwick/Mahony, ZUPT/ZARU refs in `cursor-drift-mitigation.md`.)

## 9. Status / next

Measure A/B/C → derive cone + dictation signature on today's mount → write
the classifier spec off those numbers → implement (remove roll-split, add the
observability-aware classifier). Cursor B1 recorded here for the later cursor
work. Voice gating (A4) remains future.
