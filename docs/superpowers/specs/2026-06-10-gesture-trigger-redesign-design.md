# Gesture Trigger Redesign — Pose-Gated Two-Stage Architecture

**Date**: 2026-06-10
**Status**: Brainstormed; awaiting user spec review before plan writing
**Supersedes**: Stage E snap-vs-tap classifier work at trigger time

## 1. Background and motivation

### What we built so far

Through Items 0 and Stage E of the firmware foundation we put in place:

- Chip tap engine (LSM6DSL) firing single-tap events on impulse detection
- Firmware multi-tap counter with `k_work_delayable` commit-on-timeout
- Activity gate (motion residual filter) to suppress taps near recent motion
- Bio-acoustic capture pipeline: continuous FIFO at 833 Hz, async worker
  thread, peak extraction, CMSIS-DSP FFT, spectral band-energy features
- Spectral classifier (snap vs band-tap) using `low_band_ratio`

### What hardware testing revealed (2026-06-10)

The classifier sometimes worked, but the chip-tap engine itself is firing
on incidental events that are not user gestures:

- Skin tap 1-2 cm AROUND the band (vibration travels through tissue;
  ViBand documents 20 Hz - 1 kHz transmits well through arm)
- Slap on thigh while wearing band (energy couples body → arm → wrist)
- Slap on desk while wearing band (vibration through hand → wrist)

This is not a tuning bug.  It's a fundamental property of bio-acoustic
sensing on a wrist-worn device: ANY impulse on the user's body near the
band registers as a chip-tap event.

### The framing shift

Trying to enumerate all possible false-positive events is unbounded.
The right approach (per user 2026-06-10) is to **design the gesture to
have a deliberate signature that's hard to replicate accidentally**, not
chase every possible incidental event.

This is the model production wearables actually use:

- Apple Watch Double Tap uses index-thumb pinch (PPG + IMU + ML) -- a
  very specific muscle/blood-flow signature that incidental impacts
  cannot replicate.
- Apple Watch Raise-to-Wake uses a dual-threshold counter on smoothed
  gravity-vector transition + lookup tables of canonical poses.
- Samsung Bixby Lift-to-Talk uses the same two-stage pattern.

Reference: [Apple Patent US10095186](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10095186)
explicitly says "raising of the user's wrist AND subsequent movement
toward the smartwatch improves the reliability of detection."

## 2. Design principle

Two principles drive this redesign:

1. **Deliberate signatures, not exhaustive filters.**  The trigger
   gesture's required physical features should be unlikely to occur by
   accident.  Don't try to enumerate all possible accidental triggers.

2. **Multi-modal sensor fusion for defense in depth.**  When a single
   signal is ambiguous (e.g. desk-vs-lap pose has identical gravity
   vector), use additional independent signals (tap spectral feedback)
   to disambiguate.  Each signal needn't be perfect; the combination
   filters most accidental events.

## 3. Architecture: two-stage trigger

Replaces the single-event chip-tap-fires-gesture model with a two-stage
pose-gated model:

```
                    ┌────────────────────────────────────┐
                    │  IDLE: pose monitoring + chip tap  │
                    └────────────┬───────────────────────┘
                                 │
                  Motion-into-canonical-pose detected
                                 │
                                 ▼
                    ┌────────────────────────────────────┐
                    │  POSE_ARMED: gesture window 3 sec  │
                    └────────────┬───────────────────────┘
                                 │
                    Gesture matches mode's requirement
                                 │
                                 ▼
                    ┌────────────────────────────────────┐
                    │  Mode entered (AIR_MOUSE/DICT/SURF) │
                    └────────────────────────────────────┘
```

### Stage 1: pose detection (always running)

Three canonical poses recognised, each with a tolerance cone:

| Pose | Gravity vector signature | Tolerance | Mode entered |
|------|--------------------------|-----------|--------------|
| AIR_MOUSE-raise | volar facing screen, forearm raised forward | ±30° | AIR_MOUSE |
| DICTATION-raise | volar facing user's mouth, forearm raised+rotated | ±30° | Dictation |
| SURFACE-rest | volar facing up, wrist horizontal | ±20° (tighter to reduce lap FP) | SURFACE |

Pose match requires both:

- Gravity vector within tolerance cone of the canonical pose, AND
- A **motion-into-pose transition** detected within the last ~500 ms
  (Apple-style: smooth motion towards pose + dwell still in pose)

The motion-into-pose check is critical.  Simply being in a pose without
having transitioned into it doesn't arm anything.  This prevents
wrist-already-on-desk states from auto-arming, and matches the user's
intent ("user assumes or is almost in the pose to assume").

### Stage 2: armed gesture window

Once a pose is matched, gesture detection is armed for 3 seconds.  The
required gesture depends on the mode:

| Mode | Required gesture | Why this gesture |
|------|------------------|------------------|
| AIR_MOUSE | single tap (or snap, indistinguishable) | Raise pose is rare in incidental activity; single tap acceptable. |
| DICTATION | single tap (or snap, indistinguishable) | Raise+rotate pose is highly distinctive (band volar flips to face user); single tap acceptable. |
| SURFACE | cadenced double-tap (150-300 ms inter-tap) **AND** tap-spectral surface check pass | Horizontal pose has lap/desk ambiguity; double-tap cadence rejects most incidental impacts; tap spectral signature confirms hard-surface (desk feedback ringing in mid-band). |

If the armed window expires without a matching gesture, return to IDLE
silently (no false trigger).

## 4. Soft-wired pose detection

Hardware testing of the current AIR_MOUSE pose detection revealed the
existing pose check is "hard-wired" -- requires nearly-exact canonical
angle.  Real users hold their wrist with ±20-30° natural variation in
tilt and roll.

This redesign builds in tolerance from the start:

- **Continuous score**, not binary match.  `score = max(0, 1 -
  angle_to_canonical / tolerance_cone_angle)`.
- **Pose matched** when score > 0.5 AND held still for 300-500 ms.
- **Per-user calibration** stored in NVS, anchored to project
  productionization plan.

This matches Apple's documented "lookup tables of canonical starting
poses" approach.

## 5. Snap-vs-tap discrimination — moved to in-session

The earlier work assumed snap-vs-tap discrimination at trigger time was
necessary (different gestures → different modes).  In the pose-gated
architecture, **pose carries the mode info**, not the gesture.

Snap-vs-tap discrimination is therefore **deprecated at trigger time**
and **deferred as an in-session capability** for richer in-mode
gestures (e.g. snap = left-click vs tap = right-click in AIR_MOUSE).

When in-session snap-vs-tap is implemented (architecture-doc Item 7
timeframe), it uses **mic + IMU fusion (GestEar-class)**:

- PDM mic with bandpass at 2 kHz to detect snap's distinctive acoustic
  signature (1.5-3.5 kHz peak, well-documented)
- IMU spectral features (existing FFT pipeline) as secondary signal
- Fusion decision improves robustness to noise and edge cases
- Mic acceptable in-session (user explicitly entered a mode; power and
  privacy profile permissive)

Reference: [GestEar (UbiComp 2019)](https://www.researchgate.net/publication/335647801_GestEar_combining_audio_and_motion_sensing_for_gesture_recognition_on_smartwatches)
does exactly this.

## 6. Repurposing existing work

The Stage E firmware investment isn't deleted; it gets repurposed:

| Existing component | New use |
|--------------------|---------|
| LSM6DSL FIFO continuous capture at 833 Hz | Same |
| Bio-acoustic worker thread + 512-sample buffer | Same |
| CMSIS-DSP arm_rfft_fast_f32 FFT + Hann window | Same |
| Spectral band-energy features (low/mid) | Repurposed: desk-vs-lap surface discrimination (mid_band high = desk-feedback ringing). |
| Spectral classifier (low_band_ratio thresholding) | DEPRECATED at trigger.  Move to in-session use later. |
| Activity gate (motion residual) | Same |
| Firmware multi-tap counter | Modified to detect cadenced double-tap with timing constraints. |
| Firmware ringing-rejection refractory | Same |
| Chip ODR at 833 Hz | Same (works for both surface discrimination and future snap-vs-tap). |

The work to delete is small (the trigger-time spectral classifier
verdict path).  The work to add is the pose detection layer above the
existing tap detector.

## 7. Components to build

### 7.1 Gravity-vector + motion tracker

A continuous background task that maintains:

- Current gravity-LPF vector (already done in existing code as
  `gravity_lpf_x/y/z`).
- Recent motion history: short ring buffer of motion-residual samples,
  ~500 ms worth.
- "In motion-into-pose" flag: triggered when motion-residual was high
  recently then dropped quickly to near zero AND the gravity vector is
  close to a canonical pose.

### 7.2 Pose state machine

- States: `NONE`, `AIR_MOUSE_ARMED`, `DICTATION_ARMED`, `SURFACE_ARMED`.
- Transitions: NONE → *_ARMED when motion-into-pose + canonical match.
- *_ARMED → NONE after 3 seconds without matching gesture (timeout).
- *_ARMED → mode entry when matching gesture arrives.

### 7.3 Cadenced double-tap detector (for SURFACE)

- Reuses existing multi-tap counter infrastructure.
- Adds inter-tap timing constraint: 150-300 ms between two taps to count
  as a deliberate cadenced double-tap.

### 7.4 Surface-spectral confirmation (for SURFACE)

- Reuses existing FFT pipeline.
- New threshold check: `mid_band_energy > SURFACE_DESK_RESONANCE_THRESH`
  indicates desk-feedback ringing (hard surface confirmed).

### 7.5 Per-pose calibration storage

- Stored in NVS (Zephyr settings subsystem) so per-user pose centers and
  tolerances survive reboot.
- Defaults shipped in code; user calibration overrides.

## 8. What stays the same

The existing IDLE / SNAPSHOT / WORKOUT / AIR_MOUSE / SURFACE power state
machine, BLE HID + HRS + BAS services, gesture cooldown, all remain
unchanged.  The trigger-detection LOGIC inside IDLE changes; everything
else is untouched.

## 9. Documented limits + productionization escalation

### v0 known limits

- **SURFACE/lap false positive**: a user with hand on lap who happens
  to do a cadenced double-tap that produces the right spectral
  signature will incorrectly enter SURFACE.  Real but rare;
  cadence + spectral combination makes it unlikely.
- **Pose tolerance hand-tuning**: defaults are tuned for one user (the
  developer).  Other users may need different tolerance cones until
  calibration ritual ships.

### Productionization improvements

These all strengthen the trigger.  Implement in any order based on
hardware availability:

1. **PPG firmness check** (housed + properly strapped device).  Briefly
   wake MAX30102 during pose-arm window to read DC contact level.
   Confirmed by literature (`WF-PPG dataset 2025`, force-sensing patent
   US10874348).  Requires stable strap baseline.
2. **Altimetry / barometer** for absolute height (desk vs lap).
   Documented as `project_productionization_altimetry_evaluation`.
3. **Optical proximity sensor** for direct surface contact detection.
   Documented same place.
4. **Per-user calibration ritual** via companion app.  Documented in
   `project_productionization_gesture_calibration_2026_06_09`.
5. **Re-tune all thresholds** against housed-device data once
   production hardware revision ships.
6. **In-session snap-vs-tap discrimination** via mic + IMU fusion
   (GestEar-class) for richer in-mode gestures.  Architecture roadmap
   Item 7+ timeframe.

## 10. Open implementation questions (for the plan stage)

These don't change the design but need empirical work:

- **Motion-into-pose threshold values**: how high must motion-residual
  be before it counts as "user is moving into pose"?
- **Dwell duration**: 300 ms? 500 ms? Tuning empirically.
- **Pose tolerance cone angles**: ±30° / ±20° are guesses; verify with
  hardware.
- **Cadenced double-tap timing window**: 150-300 ms inter-tap is from
  Apple VoiceOver convention; may need tuning for this user.
- **Surface-resonance spectral threshold**: needs desk-tap + lap-tap
  data collection.

## 11. Risks

- **Spectral surface check unreliable without housing**: the user's
  current band is duct-tape-mounted on an open PCB.  The desk-feedback
  ringing signature depends on mechanical coupling.  Empirical test
  may need calibration before this signal is reliable.  Mitigation:
  threshold tuning is recalibratable when housing arrives (per user
  2026-06-10 decision).
- **Pose tolerance too loose or too tight**: real users have wide
  natural variation.  Mitigation: ship conservative defaults, calibrate
  per-user during ritual.
- **SURFACE/lap FP edge cases**: real possibility for some users
  habitually fidgeting while resting hand on lap.  Mitigation:
  productionization sensor additions (PPG / altimetry / proximity).

## 12. Success criteria

This design will be considered successful when:

1. AIR_MOUSE / DICTATION / SURFACE modes each have a documented
   trigger gesture that the user can perform reliably.
2. Incidental events (skin tap near band, desk slap, thigh slap, lap
   rest with random tap) do NOT trigger any mode entry except in
   genuinely-ambiguous edge cases.
3. The pose detector accepts a natural range of wrist orientations
   (not requiring exact-canonical posing).
4. Existing power state machine, BLE services, and other foundation
   work continue to function unchanged.

## 13. Cross-references

- Memory: [decision_ppg_fusion_session_only_2026_06_10](file:///Users/0xjba/.claude/projects/-Users-0xjba-Projects-gestureband/memory/decision_ppg_fusion_session_only_2026_06_10.md)
- Memory: [principle_real_world_grounding_before_engineering](file:///Users/0xjba/.claude/projects/-Users-0xjba-Projects-gestureband/memory/principle_real_world_grounding_before_engineering.md)
- Memory: [project_productionization_gesture_calibration_2026_06_09](file:///Users/0xjba/.claude/projects/-Users-0xjba-Projects-gestureband/memory/project_productionization_gesture_calibration_2026_06_09.md)
- Memory: [project_productionization_altimetry_evaluation](file:///Users/0xjba/.claude/projects/-Users-0xjba-Projects-gestureband/memory/project_productionization_altimetry_evaluation.md)
- Doc: `docs/research/gesture-architecture.md`

## 14. References

- [Apple raise gesture detection patent (US10303239)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10303239)
- [Apple UI activation with raise + movement patent (US10095186)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10095186)
- [Apple Watch Double Tap (watchOS 10.1)](https://www.apple.com/newsroom/2023/10/apple-watch-double-tap-gesture-now-available-with-watchos-10-1/)
- [ViBand: High-Fidelity Bio-Acoustic Sensing (UIST 2016)](https://www.robertxiao.ca/research/viband/)
- [GestEar: combining audio and motion sensing](https://www.researchgate.net/publication/335647801_GestEar_combining_audio_and_motion_sensing_for_gesture_recognition_on_smartwatches)
- [WF-PPG contact pressure dataset (Nature Scientific Data 2025)](https://www.nature.com/articles/s41597-025-04453-7)
- [Force-sensing PPG patent (US10874348)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10874348)
- [False triggering prevention patent (US12236042)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12236042)
