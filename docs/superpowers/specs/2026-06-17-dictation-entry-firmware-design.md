# Dictation Entry — Firmware (Sub-project A) — Design Spec

**Date:** 2026-06-17
**Status:** design for review
**Branch:** feature/gesture-foundation
**Umbrella:** `docs/superpowers/specs/2026-06-07-gesture-triggered-dictation-design.md`
(the full dictation product: A firmware entry · B BLE audio · C Mac app · D STT · E backend).
This spec is **Sub-project A only**: the firmware voice-gated entry. B–E follow.

---

## 1. Scope & supersedes

Build the firmware that decides *"the user wants to dictate"* and exposes it as a
mode — nothing downstream (no audio streaming, no HID, no Mac app). Two stages:
- **A.0 — PDM-mic feasibility measurement** (build first; de-risks SNR).
- **A.1 — voice-gated entry FSM** (built using A.0's measured numbers).

**Supersedes the umbrella spec's entry model:** the `2026-06-07` activation was
**snap + wrist-at-face**. This replaces it with **raise-to-ear (phone-call pose) +
voice-onset** — no snap, no temple tap (user: temple-tapping is uncomfortable).
Also: the umbrella assumed extending the cursor's HOGP HID + reusing
`MODE_AIR_MOUSE`/`MODE_SURFACE` + the cursor cooldown — all extracted to
`feature/air-mouse`, so the firmware integration here is from a clean foundation
(`GestureMode` = `MODE_IDLE` + `MODE_GESTURE_AMBIENT`).

## 2. Hardware facts (verified 2026-06-17, corrects the umbrella spec)

- Mic = **MSM261D3526H1CPM** PDM (NOT MP34DT05 — that's the Arduino Nano 33 Sense).
- The board DTS already defines `&pdm0` (`nordic,nrf-pdm`, pinctrl PDM_CLK P1.00 /
  PDM_DIN P0.16); it is **disabled by default** — we enable it in `app.overlay`.
- The mic is powered through a **GPIO-controlled regulator on P1.10** (mic-enable).
  This is both a bring-up requirement AND the natural **power gate**: powering the
  mic only while the ear-pose is held = toggling this regulator.
- Known-good config (Seeed wiki + Zephyr `dmic` sample): **16 kHz mono, PDM clock
  1.28 MHz, clock source PCLK32M**, via Zephyr's `DMIC` API (`dmic_configure` /
  `dmic_read`). Gotcha: arbitrary PCM rates raise "Cannot find suitable PDM clock
  configuration" — stick to the known-good 16 kHz / 1.28 MHz pairing.

## 3. A.0 — PDM-mic feasibility measurement (build first)

**Goal:** answer the make-or-break question — *can the wrist mic detect voice-onset
with the hand at the ear, distinct from ambient?* (Wearable-mic SNR is the known
limiter; measure before building thresholds/FSM.)

**Build (measurement-only — no FSM, no mode):**
- `app.overlay`: `&pdm0 { status = "okay"; }` (reuse board pinctrl); ensure the
  P1.10 mic-enable regulator is modeled/asserted.
- `prj.conf`: `CONFIG_AUDIO=y`, `CONFIG_AUDIO_DMIC=y`, `CONFIG_AUDIO_DMIC_NRFX_PDM=y`.
- New module **`src/mic_vad.{h,cpp}`** (its own module, mirroring how `bio_acoustic`
  is isolated): `dmic_configure` 16 kHz mono → a read thread that computes
  **short-term RMS energy** per ~20 ms block and logs a rate-limited
  `[MIC] rms=… floor=… ratio=…` line. No Opus, no BLE. Asserts/deasserts the P1.10
  enable around capture.
- A serial command to start/stop the probe (so the mic isn't always-on during traces).

**Measurement protocol (capture + paste):**
1. **Voice-at-ear vs ambient:** ear pose, speak normally → `[MIC] rms`; then silent →
   `rms`. Does speech sit clearly above the ambient floor (target a clear ratio gap)?
2. **Ear vs hand-down** while speaking: does the ear pose (~15–20 cm from mouth)
   give materially better SNR than hand-at-side?
3. **Ear-pose gravity signature:** run the existing `v` pose trace in the ear pose →
   capture gravity/pitch/roll to define `POSE_EAR` in A.1.

**Gate:** if speech-at-ear doesn't separate from ambient at this mic/position, that
finding reshapes the trigger (e.g., add a confirming gesture back) **before** any FSM
is built. If it separates, A.0's numbers set the VAD threshold + the pose canonical.

## 4. A.1 — voice-gated entry FSM (built from A.0 data)

**`POSE_EAR` (held):** new class in the existing gravity/pose classifier; canonical
**set from A.0's measured signature**. Reuses the existing pose-dwell/arm machinery.

**Mic power gate (resolves the chicken-and-egg + the power concern):** `POSE_EAR`
held → assert P1.10 + start `mic_vad` (listening). Pose drops → stop + deassert.
Mic only runs while the hand is at the ear (naturally duty-cycled).

**VAD:** energy-based (RMS over an adaptive ambient floor, held for a short dwell) =
voice-onset. Threshold **from A.0**. The pose-gate covers energy-VAD's main weakness
(noise false-alarms only matter while the pose is held). **Upgrade path if A.0/field
shows ambient bleed-through:** WebRTC VAD (lightweight sub-band) or a tiny ML VAD
(AtomicVAD-class, proven on this MCU tier) — NOT built in v0 (YAGNI).

**FSM:**
```
IDLE ── POSE_EAR held → mic ON (listening)
     ── voice-onset (while POSE_EAR held) → MODE_DICTATION   [A: detect + LOG only]
MODE_DICTATION ── POSE_EAR drops > exit-dwell → IDLE + mic OFF
```
Two-factor (pose AND voice) → low false-positive. **Sub-project A is detect-and-log
only**: logs `DICTATION entry: ear-pose + voice` / `exit`. No audio stream, no HID
(those are B). Pre-roll buffering (so the first words aren't clipped when B streams)
is noted for B, not built here.

## 5. Components / files

- `app.overlay` — enable `&pdm0` + mic-enable regulator (P1.10).
- `prj.conf` — DMIC Kconfig.
- **`src/mic_vad.{h,cpp}`** (new module): `mic_vad_start()` / `mic_vad_stop()` /
  `mic_vad_voice_onset()` + (A.0) the `[MIC]` energy log. Owns the PDM/DMIC + the
  P1.10 enable. No dependency into the gesture FSM (clean boundary, like `bio_acoustic`).
- `src/gesture_mode.{cpp,h}` — add `MODE_DICTATION` to the enum + `POSE_EAR` class +
  the entry/exit logic that gates `mic_vad` on the pose and enters on voice-onset
  (detect + log).
- `src/gesture_thresholds.h` — `POSE_EAR` canonical + VAD threshold + dwells, **all
  seeded from A.0's measured numbers** (tagged `[USER]`/`[HOUSING]`).
- `src/main.cpp` — A.0 serial probe command; A.1 wires nothing new beyond the mode log.

## 6. Testing / verification

- A.0: `./build.sh` clean with DMIC enabled; flash; the `[MIC]` log prints sane RMS;
  the 3-capture protocol (§3) produces the separability numbers.
- A.1: pose-trace-derived `POSE_EAR` arms in the ear pose; speaking while held logs
  `DICTATION entry`; dropping the pose logs exit; silent-hand-at-ear and
  speech-with-hand-down do NOT trigger (two-factor check).
- No host-test module here (PDM is hardware-in-the-loop); `mic_vad`'s RMS math could
  get a small host test if it's factored pure.

## 7. Risks / open (resolved by A.0 or noted)

- **Wrist-mic SNR (make-or-break)** — A.0 measures it directly. Mitigations if weak:
  ear pose already shortens mic→mouth distance; could add a confirming gesture back.
- **Energy-VAD false alarms** — pose-gate mitigates; WebRTC/tiny-VAD is the documented
  upgrade.
- **PDM clock config** — use the known-good 16 kHz / 1.28 MHz pairing (gotcha §2).
- **Co-existence with §6.5 KWS** (umbrella N3) — both use the PDM mic; mutual exclusion
  at the FSM level when that feature ships. Out of scope for A.

## 8. Out of scope (this sub-project)

Sub-projects B (BLE audio: PDM→Opus→GATT, borrow Omi), C (Mac app), D (STT/Deepgram),
E (subscription backend) — all per the `2026-06-07` umbrella spec. No audio streaming,
no HID, no Mac-side work here.
