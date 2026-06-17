# Dictation Entry A.1 — POSE_EAR + Voiced-Onset Entry FSM — Design Spec

**Date:** 2026-06-17
**Status:** design for review
**Branch:** feature/gesture-foundation
**Sub-project:** A.1 of the dictation umbrella (`docs/superpowers/specs/2026-06-07-gesture-triggered-dictation-design.md`).
**Predecessor:** A.0 (`docs/superpowers/specs/2026-06-17-dictation-entry-firmware-design.md`),
BUILT + FLASHED + MEASURED — gate PASSED. This spec builds the entry FSM on A.0's `mic_vad` module.

---

## 1. Scope

Decide *"the user wants to dictate"* and expose it as a mode — **detect + log only**.
Two-factor entry: **POSE_EAR held (IMU) AND voiced-onset (mic)**. No audio streaming, no HID,
no Mac app (those are sub-projects B–E). The architecture is Apple Watch "Raise to Speak":
the IMU pose gates *when* the mic runs; an in-window spectral voice check decides onset.

## 2. What A.0 settled (the empirical basis)

- **Mic works at the ear:** speak-at-ear RMS syllable peaks 1000–2700 (sustained 600–1900);
  speak-hand-down ~300–370 (≈4–5× ear advantage, the near-field win); still-ambient (fan) ~90–200
  (≈10× speech margin).
- **Pure RMS energy is NOT a sufficient discriminator:** mechanical handling transients while
  *moving* the arm hit 1000–2400 — overlapping speech energy. (Matches the research: no modern
  product uses bare energy VAD; it false-fires on keystrokes/footsteps/motion.)
- **Live ambient floor creeps up mid-speech** (EMA went 100→1000+), so `ratio` self-erodes the
  longer you talk → the floor must be **latched**, not live.
- **POSE_EAR canonical (20 s `v`-trace, rock-steady):** gravity ≈ `(8.2, −4.6, 2.6)` m/s²,
  `vert ≈ 32–33°` from vertical, `pitch ≈ −57`, `roll ≈ −60`, `at_rest=1`. Dominant `gx ≈ 8.2`
  separates it from `AIR_MOUSE` (`gx ≈ 5–6`).

## 3. Discriminator decision (locked)

In-pose voice-vs-noise = **voiced-band spectral energy, adaptive latched-floor, M-of-N
count-in-window onset** (the consecutive-dwell of the original design failed — speech `veM` is
bursty; see §6), against a **latched ambient floor**, while the ear pose is present. Reuses the
CMSIS-DSP FFT already running for `bio_acoustic`. Rationale: pure RMS proven insufficient (§2); the
300–3000 Hz voiced-band energy separates speech from ambient (~3–20× on the prototype mount); the
pose-gate + M-of-N handle the rest.
**Tier-2 escalation (NOT built in v0, documented for later):** Picovoice **Cobra** neural VAD —
the only production neural VAD confirmed on nRF52840 (same chip, 98.9% TPR @ 5% FPR), closed-source
/ beta. Adopt only if field noise (TV, café, nearby speech — untested in A.0) erodes the spectral
margin. Silero/TEN have no Cortex-M4 port; RNNoise is CPU-marginal at 64 MHz.

## 4. POSE_EAR + pose-model cleanup

The current pose enum is vestigial from the air-mouse extraction and must be reconciled as part of
this work (A.1 touches `gesture_poses` + `gesture_mode` anyway):
- **Remove `POSE_AIR_MOUSE`.** Despite the name it is just a broad "raised-arm hemisphere" cone and
  is the *only* pose that currently arms — a leftover from the extracted feature. Generic "raised"
  is already represented by the orientation classifier (`WRIST_UP_RAISED`); it does not need an
  armed pose. Removing it also stops the broad cone from arming during the ear-pose *raise*.
- **`POSE_DICTATION` → `POSE_EAR`.** Rename the existing (currently DISABLED, tolerance 2.0) entry,
  give it the measured canonical from §2 (`(8.2, −4.6, 2.6)`, normalized; tight tolerance), and
  re-enable it. Pose = posture; `MODE_DICTATION` = the mode it enters. The old DICTATION was disabled
  because the *volar-to-mouth* posture was gravity-identical to an air-mouse raise (2026-06-11); the
  **ear posture is genuinely separable** (`gx≈8.2` vs raise `gx≈5–6`), which is why a real tight EAR
  pose works where the old one didn't.
- **`POSE_SURFACE`** stays dormant scaffolding (canonical kept, not armed) — unchanged.
- **Keep the tap machinery; only unbind its purpose.** The multi-tap (single/double/triple) counter,
  cadence logic, and `multi_tap_commit_handler` are **kept intact in the code** as ready, working
  scaffolding — a purpose will be assigned later, after the ear/dictation work settles (user
  decision). The ONLY tap-related change here is removing the dead `POSE_AIR_MOUSE` pose and its
  `case` in the commit switch (it references a removed enum). Do NOT delete the counter/cadence/handler.
- **Net result: only `POSE_EAR` arms; the tap-commit path is inactive at runtime but fully present.**
  `POSE_EAR` is voice-gated (not tap-bound). Chip taps are still detected + logged at the hardware
  level. To keep the retained handler clean while no pose is tap-bound: a stray double-tap while
  `POSE_EAR` is armed must log a graceful "no tap-bound mode" line, NOT the alarming
  `Mode entry ABORT: unknown armed pose`. (Either skip feeding the tap counter when the armed pose is
  `POSE_EAR`, or make the commit switch's default a benign log — plan-level choice.)

**POSE_EAR condition:** pose score above threshold — **gravity match alone, NO stillness/`at_rest`
requirement.** A person cannot hold an arm dead-still at their ear, and the band may be in motion
(walking / running / in a car), so gating on stillness is wrong (it caused intermittent multi-second
start delays — 2026-06-17). The mic starts the instant the ear pose is present; the floor latches
over the first ~500 ms of whatever ambient is there (motion-tolerant, since onset is relative to it).

**Build-time HW verification:** confirm `POSE_EAR` matches across comfortable leans (the cone is
widened, 2026-06-17) and that the mic starts promptly on raise (no stillness gate). A brief mic-on
during a non-dictation raise (e.g. scratching) is harmless — entry still requires voice-onset, and
the mic turns off when the pose leaves.

## 5. Mic power gate

`POSE_EAR` present (gravity match, no stillness gate) → `mic_vad_start()`. Pose drops → `mic_vad_stop()`.
The mic capture (PDM clock + DMIC acquisition) only runs while the ear pose is present — naturally duty-cycled.
**Out of scope for v0:** toggling the P1.10 *regulator* (deeper analog power-off). `mic_vad_start/stop`
already gates the expensive part (capture); the regulator toggle is a noted power-optimization follow-up.

## 6. VAD inside the window (mic_vad extension)

- **Spectral feature:** per ~20 ms PCM block, Hann-window + zero-pad the 320 samples to a 512-pt
  real FFT (CMSIS `arm_rfft`); sum |X|² in **300–3000 Hz** (the voiced band) → `voiced_energy`,
  reported scaled to millions as **`veM`** (raw |X|² overflows an int log). Bin width 16000/512 ≈
  31.25 Hz, so the band ≈ bins 10–96.
- **Measured discriminator (T2, 2026-06-17 — supersedes the original plan):** on the prototype
  skin-mount, `veM` separates speech (syllable peaks 3000–21000) from ambient (≤ ~1030) by ~3–20×.
  The voiced/total **`frac`** was ALSO measured and **rejected**: the face-down-in-skin mic low-passes
  ambient so the fan is itself voiced-band-dominated → `frac` ambient (400–800) overlaps speech.
  `frac` is still logged for diagnostics but is NOT the discriminator.
- **Adaptive latched floor (universality):** ambient level varies hugely per environment (fan / silent
  AC / traffic / outdoor — user requirement). So do NOT use a fixed absolute threshold. When POSE_EAR
  first matches (mic start), sample `veM` over a short window (`VAD_FLOOR_SAMPLE_MS` ≈
  500 ms) and freeze it as `veM_floor`. The threshold is `max(VAD_VEM_ABS_MIN, VAD_K × veM_floor)` —
  the ratio term re-adapts to wherever the user is, each pose entry. Do NOT update the floor live while
  a candidate voice segment is active (avoids floor-creep).
- **Onset rule = M-of-N count-in-window, NOT consecutive-dwell.** Speech `veM` is *bursty* (spikes per
  syllable, drops to ambient between syllables), so a consecutive-block dwell falls in the gaps. A
  block is "hot" if `veM ≥ threshold`; onset fires when **≥ `VAD_ONSET_HITS` hot blocks occur within
  `VAD_ONSET_WINDOW_MS`** (≈ 3 hits / 700 ms). This catches bursty speech and rejects the isolated
  single-block ambient spike.
- **Genuine-SNR-limit honesty:** if ambient is so loud/broadband that speech cannot clear
  `VAD_K × floor` (heavy traffic/outdoor), energy VAD physically cannot separate — that is the
  documented **Cobra Tier-2** trigger (§3), not a reason to inflate the constants.
- **Voice-continuity hold (posture robustness):** also expose `bool mic_vad_voice_active(void)` (true
  while a hot block occurred within `VAD_VOICE_HOLD_MS` ≈ 1500 ms). The FSM uses it to HOLD a session
  through a comfortable lean that pushes gravity out of the `POSE_EAR` cone — as long as the user keeps
  speaking at the near-field level. Self-limiting by physics: lowering the arm makes the voice
  far-field/quiet → not hot → releases, so it can't hold a session when the wrist isn't near the mouth.
- **Interface:** add `bool mic_vad_voice_onset(void)` (latches true on onset, cleared on read) and
  `bool mic_vad_voice_active(void)` (non-clearing). Keep `mic_vad_block_rms`. The `[MIC]` log keeps
  `veM` + `frac` for diagnostics.
- Thresholds `VAD_VEM_ABS_MIN`, `VAD_K`, `VAD_ONSET_HITS`, `VAD_ONSET_WINDOW_MS`, `VAD_FLOOR_SAMPLE_MS`,
  `VAD_VOICE_HOLD_MS` — all in `gesture_thresholds.h`, **tagged `[HOUSING]`** (mount-dependent;
  provisional values from the skin-mount T2 measurement, expected to move when the housing is built).

## 7. The FSM (gesture_mode)

```
IDLE ── POSE_EAR present (gravity match, NO stillness) → mic ON; latch floor over first window
     ── voice-onset (while POSE_EAR present) ──────────→ MODE_DICTATION   [A.1: detect + LOG only]
MODE_DICTATION ── POSE_EAR out of cone > EXIT_DWELL_MS  AND  no near-field voice
                  for VAD_VOICE_HOLD_MS ──────────────→ IDLE + mic OFF
               (pose out of cone but still speaking → HOLD, lean tolerance)
```
- Add `MODE_DICTATION` to the `GestureMode` enum (joining `MODE_IDLE` / `MODE_GESTURE_AMBIENT`).
- Two-factor (pose AND voice) → low false-positive. **No stillness anywhere** — start and hold both
  key on the gravity pose match alone (people can't hold still at the ear, and the band may be moving).
  Exit requires pose-gone AND voice-stopped (voice-continuity hold above). Logs
  `POSE_EAR present -> mic ON`, `DICTATION entry: ear-pose + voice`, the lean-hold, and
  `DICTATION exit: pose dropped + voice stopped`. No audio stream, no HID (sub-project B).
- Negative checks (must NOT enter): silent hand-at-ear (pose, no voice); speech with hand down
  (no pose). A raise that passes *through* the ear orientation briefly turns the mic on but does
  NOT enter (no voice-onset) and turns off when the pose leaves — harmless, not a false entry.

## 8. Components / files

- `src/gesture_poses.{h,cpp}` — remove `POSE_AIR_MOUSE`; rename `POSE_DICTATION` → `POSE_EAR` with
  the measured canonical + re-enabled tolerance; `POSE_SURFACE` unchanged (dormant).
- `src/mic_vad.{h,cpp}` — FFT voiced-band energy, latched floor, `mic_vad_voice_onset()`, extended
  `[MIC]` spectral log. (`mic_vad_rms.cpp` pure helper unchanged.)
- `src/gesture_mode.{cpp,h}` — remove the `POSE_AIR_MOUSE` arming + its commit-switch `case` only;
  KEEP the multi-tap counter / cadence / `multi_tap_commit_handler` intact (unbound scaffolding) with
  a benign default log. Add `MODE_DICTATION`; `POSE_EAR` gating of `mic_vad`, entry on onset, exit on
  pose drop (detect + log).
- `src/gesture_thresholds.h` — POSE_EAR canonical + VAD thresholds/dwells (`[USER]`/`[HOUSING]`).
- `src/main.cpp` — no new wiring beyond the mode log; serial `m` probe stays (used for step-1 measurement).

## 9. Build order (measure → gate, mirroring A.0)

1. **Extend `mic_vad` to compute + LOG voiced-band energy** (FFT + 300–3000 Hz sum) and re-run the
   A.0 three captures → get the spectral separability numbers (speak-at-ear vs still-ambient vs
   hand-down vs mechanical-transient). Set `ABS_MIN`/`K`/`ONSET_DWELL_MS` from this.
2. **Pose-model cleanup + `POSE_EAR`** (§4): remove `POSE_AIR_MOUSE`, rename `POSE_DICTATION` →
   `POSE_EAR` with the measured canonical + re-enable; confirm on HW that `POSE_EAR` arms only when
   settled in the ear posture (not during the raise, not for a generic raised arm). Keep the tap
   counter/cadence/commit handler intact but unbound (raw chip taps still log).
3. **Wire the FSM:** pose-gate the mic, latch the floor, sustained-onset → `MODE_DICTATION`
   (detect + log), exit on pose drop. Thresholds from step 1.

## 10. Testing / verification

- Host: the FFT voiced-band helper factored pure (like `mic_vad_block_rms`) gets a small host test
  (known tone in-band → high energy; out-of-band tone / DC → low). Pose canonical scoring is the
  existing path (no new host test).
- HW (hardware-in-the-loop, per CLAUDE.md): build clean; `POSE_EAR` arms only in the ear pose
  (and not in AIR_MOUSE); speaking while held logs `DICTATION entry`; dropping the pose logs exit;
  the three negative checks (§7) do NOT trigger.

## 11. Risks / open

- **Field noise untested** — A.0's ambient was only a fan. Loud real-world noise will compress the
  margin; spectral+dwell is the v0 bet, Cobra (§3) the documented Tier-2.
- **POSE_EAR false-arming on a generic raised arm** — the ear cone must be tight enough that
  raising the arm (forward, or in transit to the ear) does not arm it; §4 HW verification gates this.
  (The old broad `AIR_MOUSE` cone is removed precisely to avoid this.)
- **Latched-floor staleness** — if ambient changes while the pose is held a long time, the frozen
  floor drifts. v0 accepts this (pose holds are short); a re-latch-on-long-silence is a noted refinement.
- **Co-existence with a future KWS / §6.5** (umbrella N3) — both use the PDM mic; mutual exclusion at
  the FSM level when that ships. Out of scope here.

## 12. Out of scope (this sub-project)

Audio streaming (PDM→Opus→GATT), HID, Mac app, STT, backend (sub-projects B–E). The P1.10 regulator
power-gate, KWS, and Cobra integration are all deferred (noted above).
