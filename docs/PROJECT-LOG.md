# gestureband — Project Log

**One living doc** = the project history (changelog), what worked / what didn't, and the
product-stage TODO. This **replaces per-session status docs** (don't create new ones — update
this). For build/flash/file-map see [CLAUDE.md](../CLAUDE.md); durable references are listed at
the bottom.

**Hardware:** Seeed XIAO nRF52840 Sense (nRF52840 Cortex-M4F) + LSM6DSL IMU + MAX30102 PPG +
onboard PDM mic + MAX98357A I2S speaker. Right wrist, volar, thumb-side. Host link: BLE.
**Current branch:** `beta` (gesture-detect + HR foundation + dictation + voice loop).
Air-mouse cursor lives on `feature/air-mouse`.

---

## 1. Changelog (what we built, in order)

### Stage 0 — HR / PPG baseline (May 27–28)
Initial wrist HR monitor. MAX30102 PPG + power state machine (`IDLE → SNAPSHOT → WORKOUT`),
LSM6DSL significant-motion → INT1 wake, System-ON sleep. Motion-artifact removal (chained NLMS,
spectral stride exclusion), dual-method cross-validation, physiological slew limiter.

### Stage 1 — HR v0.4 "production-track" (Jun 6)
Stripped the fragile motion-path HR DSP; **stationary HR only**, signals "stay steady" during
motion (rather than reporting a bad moving estimate). → `docs/research/hr-algorithm-decisions.md`.

### Stage 2 — Gesture foundation (Jun 7–9)
Mode detector + orientation classifier; pose FSM (`NONE / *_ARMED`, 3s timeout); chip tap engine;
snap-vs-tap classifier (settled on **IMU spectral band-energy ratio**, Path A); bio-acoustic
(high-ODR FIFO) path; fixed sig-motion → WORKOUT_VERIFY eating taps (recent-gesture guard).

### Stage 3 — Orientation / pose maturity (Jun 10–11)
**Mahony complementary filter** (pitch/roll gravity-locked, gyro-bias/ZARU, stillness). Pose
canonicals re-tuned for the final mount. Centralized tuned constants → `gesture_thresholds.h`.
Asymmetric-dwell orientation hysteresis; raised-zone redefined on unified gravity geometry.

### Stage 4 — Air-mouse cursor (Jun 11–15) — *now on `feature/air-mouse`*
Cursor pointing (gyro-fused vertical, yaw-driven pseudo-absolute X), calibration at placement,
deliberate-double-tap exit + raise-to-resume.

### Stage 5 — Branch split (Jun 16)
Extracted air-mouse → `feature/air-mouse`. `beta` = clean gesture-detect + HR foundation.

### Stage 6 — Dictation entry A.0 / A.1 (Jun 17–18)
Enabled onboard PDM mic. `mic_vad` module: 16 kHz DMIC + CMSIS rFFT voiced-band (300–3000 Hz)
energy, **continuously-adaptive noise floor**, **M-of-N voice-onset**. `POSE_EAR` gate →
`MODE_DICTATION` (detect + log), voice-continuity hold through a lean.

### Stage 7 — Dictation audio stream B (Jun 18)
BLE LC3 uplink stream off the capture thread (producer/consumer + dedicated audio thread).

### Stage 8 — Speaker downlink (Jun 20–21)
`audio_out` (I2S → MAX98357A) + BLE LC3 **downlink** (phone/Mac → speaker) + **clock recovery**
(drift-free, setpoint 140 ms). Full-duplex verified. *The bug:* `lc3_decode` stack overflow →
silent halt; fixed by `DL_THREAD_STACK=4096`. → `docs/device-ble-contract.md`.

### Stage 9 — Mac voice loop (Jun 21–23)
`ts32` mic-sample timestamp on the uplink frame. Backend-agnostic **voiceio** core
(`tools/voiceio/`): Gemini Live backend, real-time full-duplex orchestrator, **DTLN neural AEC**,
**online render→capture delay estimator** (no hardcoded acoustic constant), startup calibration
chirp, conversation session lifecycle (raise = open AI socket, lower = close).

### Stage 10 — Barge-in saga (Jun 28 – Jul 1) — **the hard part; see §2**
Many detector attempts (energy VAD, abs floor, mic-mute, AEC3-style delay hysteresis, onset grace,
convergence gate) → **all failed**. Switched to **"clean signal + backend VAD"** (forward the
AEC-cleaned mic; Gemini's server VAD decides). Added an **AEC instability clamp** (`clean ≤ mic`).
Then **root-caused the residual self-barge to MECHANICAL echo coupling** and **fixed it by
repositioning the speaker.**

---

## 2. What worked / what didn't (the learnings)

### HR
- ✅ Stationary dual-method HR is reliable. ❌ Motion-path HR — deferred (crude; commercial-grade
  auto-workout detect is a future TODO).

### Gesture
- ✅ Pose + voice entry (`POSE_EAR` + voice-onset) for dictation. ✅ IMU spectral features for
  snap-vs-tap. ❌ The chip's **binary tap engine is the wrong tool** for surface/bio-acoustic taps
  — high-ODR + feature extraction (ViBand-style) is the right path. ❌ Motionless finger-pinch
  needs PPG/EMG (out of scope). Constants are mount/user-specific → need housing recalibration.

### Voice loop
- ✅ Full-duplex band↔Gemini works; downlink is **drift-free** (clock recovery, overflow 10→0).
- ✅ DTLN AEC cancels **linear** echo well (ERLE +20–27 dB in good conditions).

### Barge-in — the big one
- ❌ **Every detector/energy tuning failed.** Energy gates, absolute floors, onset grace, Gemini-VAD
  detuning — all hit a ceiling, because in the bad cases the residual and the user's voice **overlap
  in loudness** (no fixed amplitude threshold separates them).
- ⚠️ **AEC instability clamp** (`clean ≤ mic`) — *does* help: caught DTLN amplification spikes
  (up to ~5× over mic). Cheap, structural (an AEC can't add energy). **Keep.** But it only catches
  `clean > mic`, not `clean ≈ mic`.
- ❌ **Cupping on the dev rig = 5/7 self-barges, volume-INDEPENDENT** (same at 70% and 40%). ERLE
  collapsed to ~0 — the AEC was physically defeated.
- 🔑 **ROOT CAUSE = mechanical.** The dev rig's mic + speaker are co-located on an open PCB, so
  loud/cupped audio drives **nonlinear echo** (clipping at the 3.3V amp + sealed-cavity coupling)
  that DTLN cannot cancel → residual → self-barge.
- ✅ **FIX = reposition the speaker** (toward the production geometry: mic on the outer face facing
  the mouth, speaker perpendicular firing into the palm). Result: echo coupling **~10× lower**
  (mic p90 0.045 → 0.004–0.007), clamp catches 83 → 5–16, **ERLE healthy +23/24 dB**, **0
  self-barge**. The remaining loud spikes don't self-barge because the echo is now *linear* and the
  AEC cancels it. **Physics fixed it, not ML.**
- 📉 **Personal VAD / neural voice-ID = fragile, rejected** as the primary fix: our measured
  speaker-ID margin was only ~0.23, which a head-cold / whisper would erode → a "namesake" solution.
  Free neural options (RNNoise/DeepFilterNet) handle *noise* (traffic) but not speaker separation;
  turnkey speaker isolation (Krisp/Koala) is paid and inherits the same voice-drift fragility.
- 📌 Condition matrix (dev rig, single mic): clean = 0 self-barge; cupping = 5/7; traffic = 1/2
  (loud noise crossing the gate); rain w/ loud passages = 6/7 (nonlinear echo); **repositioned = 0.**
- Stutter (separate audio-quality issue): fixed via thread priorities (LC3 encode/decode prio 7→8,
  below the I2S feeder) + `CONFIG_TIMESLICING`. A few residual underruns under peak load remain.
- 🌀 **Fan / ambient noise is a NON-issue (2026-07-01 A/B).** Fan-ON gave **0** self-barges despite a
  *higher* mic floor + more clamps; fan-OFF (with the intermittent volume boost) gave 5. The backend's
  speech-VAD ignores broadband noise — only *speech* (incl. the AI's own residual) fires it. So the
  earlier "traffic 1/2" was the boost, not the noise per se.
- 🎚️ **Self-barges = AI residual during the volume boost** (nonlinear echo). Mechanism (confirmed by
  the clamp logs): boost → DTLN diverges → clamp caps `clean` to *mic*, but mic is loud (>0.008) → the
  clamped value still crosses the forward gate → opens the 0.5 s hangover → forwards the AI's own
  residual → VAD fires. The latch A/B was confounded by this the whole time.
- ❌ **TRIED & REVERTED (host, 2026-07-01): don't open the forward hangover on a `diverged` (clamp-fired)
  block.** Premise was "clean>mic = AI residual, never a real barge." **FALSE — it MISSED REAL BARGES.**
  During a real barge (double-talk) DTLN *also* diverges (clean>mic), so the clamp fired on the user's
  own voice and the gate dropped the barge (divfix1/2: 89–134 clamps incl. mic up to 0.039–0.047, very
  few barges through). So `clean>mic` is **NOT** unambiguous — it's *both* AI-residual boost *and* user
  double-talk, the **same energy ceiling** as `clean≈mic`. **Lesson: no energy/AEC-state signal separates
  self-barge from real barge** (clamp ratio, clean/mic, ERLE all fail). Only HW (5V amp + decoupling, stop
  the boost) or identity (Personal VAD, who's speaking) can. The clamp itself (amplitude cap) is kept;
  only the forward-gate coupling was reverted.

### Wearable acoustic design (researched — our geometry matches the leaders)
- Smart speakers separate mic↔speaker by **10–15 cm**; a wrist device can't, so it trades distance
  for **orientation + directivity + decoupling**. **Apple Watch** = mic/speaker on **opposite
  edges**. **Humane AI Pin** = **dual mic + directional (HRTF) speaker** aimed away from its mics.
  Our planned geometry does **both**. Targets: AEC works from **6 dB ERL**; **25–30 dB ERLE** is
  industry-good — our rough prototype already hit **+23 dB**.

---

## 3. Product-stage TODO

### Audio hardware (highest leverage for the voice loop)
- **Speaker amp on its own 5V rail** (off the shared 3V3) — more headroom, removes digital/analog
  cross-coupling. The 3.3V dev rail clips early → nonlinear echo (literature confirms amp/speaker
  nonlinearity is the #1 residual-echo source).
- **Standard supply decoupling** — `10µF bulk + 0.1µF bypass` at the amp VDD (datasheet spec). These
  are **tiny flat MLCC ceramics** (0201 = 0.6×0.3mm, <0.5mm tall; 10µF exists in 0201) — they fit a
  thin band trivially; battery + speaker dominate the Z-height. Size up extra bulk **only if the real
  PCB measures rail sag** under max-volume playback (scope it: target tens of mV dip, not hundreds).
- **Acoustic isolation** — gasket/baffle/foam between speaker and mic, no shared resonant cavity,
  aim the speaker away from the mic port, star-ground the amp.
- **Transducers** — low-THD speaker at the ear-worn SPL (we need *clean*, not loud); class-D amp
  with good PSRR not driven into clip; PDM mic with high **AOP** so close speaker SPL doesn't clip it.

### Geometry (the confirmed barge-in fix)
- Mic on the **outer/top face facing the mouth**; speaker on the **side, perpendicular, firing into
  the palm**. Validated to drop echo coupling ~10× and eliminate self-barge.

### 2nd mic (design in, decide to use later)
- **Lay out the footprint** on the product PCB: nRF52840 PDM is **stereo over one CLK + DATA**, so a
  2nd mic shares those lines (L/R-select to a rail) — no extra signal pins; pads D6/D7 are free; mic
  <$1. Asymmetric cost (free now, PCB respin if needed later; software-reversible).
- **Populate/stream it only if** the real wrist form shows an edge case (very loud env, imperfect
  cupping, placement variance). Streaming stereo ≈ 2× uplink power. At a ~2–4 cm baseline it helps via
  echo/noise **reference filtering on the phone**, not classic beamforming.

### Validation that needs the real PCB / wrist form
- **Intended-pose cupping test** (palm over ear, speaker into palm, mic facing mouth) — can't be done
  on the current non-wearable prototype; it's a product-stage milestone.
- Scope the 5V rail under load; compare ERLE vs the dev board.
- **Re-evaluate the AEC convergence latch** (the `AEC_CONV_*` gate that blocks forwarding until the AEC
  proves it's cancelling). Its necessity is **UNPROVEN** — every removal A/B on the dev rig was
  confounded by the intermittent volume boost, so we kept it conservatively. Once the boost is gone
  (5V amp + decoupling), re-run the removal A/B on clean hardware: if reply-1 onset stays clean without
  it, drop the latch + its 4 `[USER]` constants (simplification). Do NOT remove before that clean re-test.

### Other deferred
- HR: commercial-grade auto-workout detection (activity recognition + sustained HR elevation).
- Gesture: housing-aware threshold recalibration + per-user calibration ritual; eventual ML model.
- Productionization: NVS-stored per-user calibration; evaluate barometer/proximity for desk-plane lock.

---

## 4. Current uncommitted state (host-only unless noted)
- `tools/voiceio/orchestrator.py` — AEC instability clamp (chunk-aligned `clean ≤ mic`); `[stat]`
  shows `erle` not the misleading `res`.
- `tools/voiceio/gemini_backend.py` — server-VAD detune (START/END `_SENSITIVITY_LOW`).
- `tools/voiceio/ble_link.py` — `underrun_count()`/`buffer_fill()` telemetry (diagnostic).
- Firmware: speaker volume default 70% (`audio_out.cpp`); LC3 encode/decode prio 8 + timeslicing
  (stutter fix); help text.
- **Nothing sealed** — keep observing across sessions/conditions before committing (standing rule).
- The only truly-redundant code to trim once sealed = **diagnostic scaffolding** (verbose
  `[aec-clamp]` prints, ble telemetry). The functional gates are lean — leave them.

---

## 5. Reference docs (kept — not redundant with this log)
- [CLAUDE.md](../CLAUDE.md) — build/flash/test, file map, threading, serial console, gotchas.
- [device-ble-contract.md](device-ble-contract.md) — durable BLE wire format (audio UUIDs, uplink/
  downlink/status, FLUSH, clock model).
- [research/gesture-architecture.md](research/gesture-architecture.md) — gesture design/roadmap/
  feasibility (consult §12 before speccing any verb).
- [research/hr-algorithm-decisions.md](research/hr-algorithm-decisions.md) — HR algorithm + roadmap.
- [research/barge-in-aec-2026-research.md](research/barge-in-aec-2026-research.md) — cross-domain AEC
  research (front-end/nonlinear-reference is the field's fix; we achieved the same end mechanically).
- `docs/superpowers/specs/` + `plans/` — per-feature brainstorm→spec→plan design archive (history).
