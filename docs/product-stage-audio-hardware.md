# Product-stage audio hardware recommendations (for early PCBs)

> **Status:** forward-looking notes for the early-PCB / product-stage prototype. **None of this
> is actionable on the current open dev board** (Seeed XIAO nRF52840 Sense + duct-taped speaker/amp).
> Captured 2026-06-30 while the dev board's nonlinear self-echo is blocking robust barge-in.

## Why this matters
Robust full-duplex **barge-in** depends on the acoustic echo canceller (AEC) being able to *predict*
the echo of our own speaker in the mic. The single thing that defeats an AEC is **nonlinearity** in the
echo path — clipping, rail-sag-induced distortion, and over-strong speaker↔mic coupling. The dev board
has all three. Fixing them in the PCB makes *every* software layer (AEC, double-talk detection,
voice-isolation, turn-detection) work better at once. This is the highest-leverage hardware investment
for the voice loop.

## 1. Speaker power rail (the dev-board's measured failure)
- **Measured problem:** on the dev board the speaker amp (MAX98357A class-D) shares the **3V3** rail;
  loud/bass transients pull large current, the rail **sags**, and the speaker output **distorts** →
  nonlinear echo the AEC can't model → residual → false barge-in. (See memory:
  `project_speaker_i2s_blocked_2026_06_19`, `project_barge_in_2026_research_2026_06_29`.)
- **Planned fix:** the product gives the speaker amp its **own 5V rail** (decided). Moving off the
  *shared* rail is the bigger win — it removes the digital/analog cross-coupling.
- **Bulk cap — STILL recommended even on 5V.** A class-D amp draws large *transient* currents on
  loud transients regardless of nominal rail voltage; without a local charge reservoir the rail dips
  during the transient and the output distorts. The MAX98357A datasheet calls for it.
  - Add a **bulk electrolytic/ceramic at the amp VDD** (start ~**100–470 µF**; size to the chosen
    speaker impedance + max SPL) **plus 0.1 µF ceramic** right at the supply pin.
  - Keep the amp's ground return short; **star-ground** the amp so its return current doesn't modulate
    the mic/analog ground.
  - Consider a small **series ferrite / LC** on the amp 5V feed to keep its switching noise off other rails.
- **Validation on first PCB:** scope the 5V rail under max-volume playback — confirm dip is small
  (target a few tens of mV, not hundreds). Compare residual-echo level / ERLE vs the dev board.

## 2. Acoustic mic↔speaker isolation (the biggest single lever)
- **Maximize physical separation** of the PDM mic port and the speaker on the PCB/enclosure.
- Add an **acoustic gasket / baffle / foam** between speaker and mic; avoid a shared resonant cavity;
  **aim the speaker away** from the mic port.
- Goal: cut the direct coupling so the echo reaching the mic is *weaker and more linear* → smaller
  residual for the AEC to leave behind. (The dev board's "hand cupped over device" test — all false
  barges — is the worst-case coupling; the enclosure should engineer the *opposite*.)

## 3. Transducer & amp choices
- **Speaker:** pick one with **low THD at the target SPL** for the ear-worn distance; we don't need
  loud, we need *clean*.
- **Amp:** class-D with good **PSRR**; verify it isn't driven into clipping at our volume setpoint.
- **Mic:** ensure the PDM mic's **AOP (acoustic overload point)** is high enough that close-range
  speaker SPL doesn't **clip** the mic — mic clipping is pure nonlinearity and an AEC killer.

## 4. Optional / future (bigger changes, not required for v1)
- **Second mic → beamforming.** Smart speakers use mic arrays + beamforming for near/far spatial
  discrimination; a second mic would let us spatially separate the wearer (near) from the speaker
  (a few cm away) and is the strongest possible echo/double-talk discriminator. Flag as a future option.
- **Higher speaker-path sample rate** *only* if we ever want an inaudible (near-ultrasonic) watermark —
  infeasible at the current 16 kHz pipeline (8 kHz Nyquist + LC3 strips masked content). Not recommended
  for v1; documented for completeness.

## 5. What stays in software regardless of hardware
The AEC **reference** is the digital downlink we already have — keep the playback path low-latency and
deterministic so the render→capture delay stays stable (the delay estimator depends on it). Hardware
reduces the *residual*; software (AEC + reference-aware double-talk detection / turn model) makes the
*decision*. Both are needed; this doc covers only the hardware half.
