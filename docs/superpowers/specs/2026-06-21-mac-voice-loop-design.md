# Mac Voice-Loop (band ↔ Gemini Live, with AEC + barge-in) — Design

**Date:** 2026-06-21 (validated-architecture update 2026-06-22)
**Branch:** beta
**Status:** Approved (design) + hardware-validated; ready for implementation plan
**Builds on:** the BLE downlink + clock-recovery + full-duplex foundation (committed
`397bc49`→`218e187`). The device firmware is feature-complete for this loop except
one small addition (a mic-capture timestamp, below).

---

## ⚑ VALIDATED ARCHITECTURE (2026-06-22) — supersedes the AEC3/Speex, drift-alignment, and barge-in sections below

Three weeks of investigation + two deep-research passes + direct hardware tests on real
recordings settled the three open design points. **Where this section conflicts with the
older text below, this section wins.** The module list, the device BLE contract, the
backend interface, and the conversation-gating sections below are all still correct.

**1. AEC engine = a mask-based NEURAL residual-echo SUPPRESSOR (DTLN-aec), NOT Speex/AEC3.**
Our cheap class-D amp + tiny speaker produces a *nonlinear, weak, spectrally-mangled* echo
(measured ref↔mic waveform corr ~0.1). Linear subtractive AEC (Speex AND WebRTC AEC3) gave
only **2–4 dB ERLE** — they need a linearly-predictive reference, which a nonlinear speaker
can't provide (this is the *documented* failure mode for cheap loudspeakers; a feedback mic
is the hardware fix but is NOT required). A neural *suppressor* (mask-based, "suppress don't
subtract") does not need a predictive reference. **Tested pretrained DTLN-aec (Google liblc3-
era TF-Lite 512 model, `github.com/breizhn/DTLN-aec`) on our real recording: it drops the
residual-echo floor ~29 dB, giving a 33.6 dB voice-over-echo margin** while preserving the
user's voice — confirmed by ear and by a controlled silent-AI-floor test. Inputs = mic + the
drift-corrected far-end reference (same two-signal contract Speex/AEC3 wanted). Runs real-time,
phone-capable (1.8–10.4 M params, TF-Lite). Fine-tuning on our own speaker is a future quality
lever, not needed for v1. Tooling already built: `tools/dtln_test.py`, `tools/DTLN-aec`,
`tools/dtln-venv`.

**2. Drift compensation = resample the reference by a FIXED constant ratio (synchronous same-SoC
clocks), NOT a live-tracked estimate.** The echo is PLAYED on the speaker's I2S clock (15873 Hz =
32 MHz/21/96; the nRF I2S can't hit 16000 exactly) and CAPTURED on the mic's PDM clock (16000 Hz).
DTLN has no internal drift tracker, so the reference must be resampled by I2S/PDM = **15873/16000 =
−0.794%** to land the echo on the mic timeline; uncorrected, this fixed rate difference *accumulates*
(~870 ms over 110 s) and the echo audibly leaks late (the symptom that *looked* like drift).
CRITICAL CLASSIFICATION (corrected 2026-06-22 after a wrong first pass): the speaker and mic are
**both on the nRF52840, both divided from the same HFCLK/PCLK32M oscillator → SYNCHRONOUS**. So
their ratio is a FIXED divider ratio, invariant to temperature and unit-to-unit crystal variation
(temp scales HFCLK → scales both equally → ratio unchanged). This is the "same-SoC shared master
clock" case: production AEC uses a fixed ratio, NOT continuous SRO estimation. (Continuous SRO
estimation — DXCP/ts32 — is only for ASYNCHRONOUS multi-device setups with SEPARATE crystals, e.g.
a BT speaker vs a phone; we are not that.) The only way drift sneaks in on a shared clock is using
the NOMINAL rate (16000) instead of the actual (15873) — which is exactly why the constant is the
*actual* I2S/PDM ratio. **Implementation: `voiceio.clocks.AEC_DRIFT_RATIO = 15873/16000` →
`voiceio.resample.resample_ref` → DTLN.** No live estimator, no ts32-for-drift. Verified on HW
(drift1): the fixed constant holds the echo removed start-to-end (early −71 / late −80 dB), and an
independent blind sweep found best alignment at −0.80%, matching. (Sources: nRF52840 I2S/PDM share
PCLK32M [Nordic clock docs]; same-SoC = synchronous = fixed ratio [forasoft]; nominal-vs-actual
caveat [US7120259B1]. The ts32 firmware stays in the wire format for jitter/gap-detection use, just
not as the AEC drift source. The host-vs-device drift that DOES vary, ±0.2% temp, is handled by the
downlink clock-recovery and only sets the constant bulk delay, not the echo-alignment rate.)

**3. Barge-in = ENERGY-GATED VAD on the AEC'd stream, NOT a plain speech-VAD.** The residual AI
echo is *itself speech*, so webrtcvad/Gemini-VAD fire on it → false barge-ins. The discriminator
is LEVEL: after AEC the user's voice is ~33 dB above the residual-echo floor. The detector =
**`speech (webrtcvad) AND frame-RMS > (learned echo-floor + ~12 dB)`**, with the floor learned
online from the AEC'd stream's quiet (AI-only) frames + a short onset debounce (~150 ms).
Controlled test: 0 echo-induced false barge-ins, every spoken barge-in caught. Crucially, AEC
only needs to *detect* the interruption — once detected the backend stops, the echo vanishes, and
the rest of the user's turn is echo-free. Tooling: `tools/barge_test.py`. This becomes the
shared client-side VAD in the `turn` module (works for ALL backends, including ones with no
server VAD).

**Net:** full-duplex barge-in is viable software-only, no hardware/feedback-mic change. Pipeline:
`mic → BLE → DTLN suppressor (with ts32-ASRC drift-corrected reference) → energy-gated VAD →
barge-in → backend FLUSH/stop`. The clean DTLN output is also the reusable STT-grade mic stream.

---

## Goal

A Mac program that closes the conversational voice loop: speak to the band, the
band's mic streams up, an AI's spoken reply plays back on the band's speaker, and you
can **talk over it (barge-in)**. Built in Python on the Mac (matching `audio_tx.py`/
`audio_rx.py`), and architected so the **phone companion app later reuses the same
device contract, the same AEC library, and the same backend interface** — the Mac
build is the phone's blueprint.

**Primary deliverable = a backend-agnostic, full-duplex voice I/O layer with echo
cancellation.** The core product is NOT "a Gemini client" — it is: *clean
(echo-cancelled) mic PCM out + an audio-play sink in + a barge-in/turn signal*, exposed
behind a clean interface. **The AI backend is pluggable.** Gemini Live is the first
backend we wire in, but the same clean-mic stream must be routable to anything else
later — a STT API, a transcription model, a different realtime API, or a
STT→LLM→TTS pipeline — without touching the BLE/codec/AEC core. Echo cancellation that
produces a *clean, reusable* mic stream is the heart of the project; the conversation is
the first consumer of it.

## Why this shape (research-backed, 2026)

- **AEC is the client's job, not the API's.** Both OpenAI Realtime (ships client-side
  AEC via Agora) and Gemini Live (browsers rely on `getUserMedia` echo cancellation)
  assume the client sends *echo-cancelled* audio. Browsers hide this; we're a native
  client with a *remote* speaker, so we must do AEC explicitly — otherwise Gemini
  hears its own output and self-interrupts. (Confirmed: Google forum "Gemini Live API:
  acoustic echo cancellation needed for SIP telephony deployments".)
- **Host-side AEC is the apt approach for our hardware.** Premium hearables (AirPods
  H2) do AEC *on-device* because they have a dedicated audio DSP; our nRF52840 (64 MHz
  M4F, no DSP, already busy) cannot. Wearables without a DSP use "application-level AEC
  on the connected phone" — exactly our path. So host-side AEC is industry-recognized,
  not a workaround.
- **Lightweight software AEC behind a swappable interface, not ML.** The `aec` module is
  an interface (near+far streams in → clean out); the library behind it can differ per
  platform. **Phase-1 Mac uses `speexdsp-python`** — the maintained Python AEC with exactly
  the separate near(mic)+far(reference) API we need (pure-C Speex, runs Mac/iOS/Android/even
  ESP32, so still light + portable). **WebRTC AEC3** is the documented quality/phone upgrade
  (native lib on phone, or libwebrtc-from-source on Mac) — chosen as the *ideal* but its
  Python bindings are stale/incomplete, so not worth yak-shaving for the prototype. ML
  (DeepFilterNet/DTLN-aec) is the last-resort fallback (heavy; would burden the phone). We
  measure Speex on the real echo and upgrade only if its quality is insufficient.
- **Bluetooth is "the hardest case" for AEC delay alignment** (large, moving delay).
  Our **clock-recovery loop pins the device buffer at a constant 140 ms**, stabilising
  the delay — which de-fangs the worst part and makes host-side AEC viable.

Sources: OpenAI Realtime docs; Gemini Live capabilities; Forasoft "Echo Cancellation
on Speakerphones, Bluetooth, AirPods"; Switchboard "How WebRTC AEC3 Works"; arXiv
2508.07561 "Small-footprint AEC for Mobile Full-Duplex".

## Architecture

```
            ┌──────────────────────────── Mac voice-loop (one process) ───────────────────────┐
 band ⇄ BLE │  ble_link ──uplink [seq][ts][LC3]──► codec(dec) ──mic PCM 16k──► aec ──clean──► gemini
            │     ▲                                                              ▲(ref)        │ (PCM 16k up)
            │     │                                                              │             ▼
            │  downlink LC3 ◄── codec(enc) + clock_recovery ◄── resample 24→16 ◄── AI PCM 24k ◄─ gemini
            │     │                                                                            │
            │   FLUSH ◄──────────────────── orchestrator ◄──── 'interrupted' / turn events ────┘
            └─────────────────────────────────────────────────────────────────────────────────┘
```

One Mac process; independently-testable modules. The **device-side BLE contract**
(uplink `[seq16][ts32][LC3]`, status feedback, downlink, FLUSH) is the durable,
portable interface the phone app will also speak. The **voice-I/O core** (ble_link +
codec + clock_recovery + aec) is backend-agnostic — it produces clean mic PCM and
consumes play-PCM; the AI **`backend` plugs into that core via an interface**, so Gemini
Live can be swapped for any other API without touching the core.

## Components

Each has one responsibility and a clean interface, so it's testable in isolation and
re-creatable natively on the phone.

1. **`ble_link`** — one bidirectional BLE client (merges today's `audio_tx.py` +
   `audio_rx.py`): subscribe uplink audio (`47A10002`) + status (`47A10005`), write
   downlink audio (`47A10003`) + control/FLUSH (`47A10004`). Exposes: inbound mic
   frames `(seq, ts32, lc3)`, inbound status, `send_downlink(lc3)`, `flush()`.
2. **`codec`** — LC3 encode/decode (16 kHz/10 ms, our params) + sample-rate convert
   16 k↔24 k (Gemini out is 24 k, in is 16 k).
3. **`clock_recovery`** — the adaptive-playout PI loop (DONE; ported verbatim from
   `audio_tx.py`'s `clock_recovery_step`), driving downlink send pacing from the status
   feedback. Setpoint 140 ms.
4. **`aec`** — echo-canceller behind a swappable interface (`cancel(near_pcm, far_pcm) ->
   clean_pcm`). Phase-1 impl = `speexdsp-python`'s `EchoCanceller` (separate near/far
   streams; filter tail sized to the residual echo delay after pre-alignment). Inputs: the
   **reference** (downlink PCM we send) and the **timestamp-aligned mic** PCM. Output: clean
   near-end (user) PCM. Owns the reference ring + the delay estimate (below). Note: AEC must
   be the *first* thing to touch the mic and the *last* to see the speaker signal (no NS/AGC
   before it).
5. **`backend`** — a small **pluggable interface** (`VoiceBackend`), NOT a Gemini-specific
   module. The contract it exposes to the orchestrator: `feed_mic(clean_pcm16)`,
   `on_response_audio(cb)`, `on_turn_event(cb)` (turn-end / `interrupted`), `close()`.
   - `GeminiLiveBackend` (v1 impl) — Gemini Live realtime session (`google-genai`,
     WebSocket): streams clean PCM up (`audio/pcm;rate=16000`), receives AI PCM (24 k) +
     events (`interrupted`, `turn_complete`). It provides turn/interrupt server-side.
   - Future impls drop in unchanged against the same core: `SttBackend` (route clean mic
     to a transcription API, no playback), `OpenAiRealtimeBackend`, `PipelineBackend`
     (STT→LLM→TTS), etc. The **clean-mic PCM is a first-class output** the orchestrator can
     also just tee off (record / transcribe) independent of any backend.
6. **`turn`** (pluggable, optional) — backend-agnostic turn/barge-in detection. When the
   backend supplies turn/interrupt events (Gemini), the orchestrator uses those. When it
   doesn't (e.g. a plain STT), a shared **client-side VAD running on the clean (AEC'd)
   audio** supplies them. So "is the user talking / barging in" is decoupled from any one
   API. v1 wires Gemini's server events; the client-VAD path is the fallback that keeps
   the system API-independent.
7. **`orchestrator`** — wires the voice-I/O core (ble_link + codec + clock_recovery + aec)
   to whichever `backend` is configured; owns the turn/barge-in state machine and the
   barge-in → FLUSH action. The core never imports a backend directly — only the interface.

## Data flow

**Up (user → AI):** band mic → LC3 → BLE uplink `[seq16][ts32][LC3]` → `codec` decode →
PCM 16 k *(user + speaker echo)* → **`aec` removes the echo** → clean PCM → `gemini` up.

**Down (AI → user):** `gemini` streams PCM 24 k → `codec` resample to 16 k → the same
16 k PCM is handed to **`aec` as the reference** → `codec` LC3-encode → `clock_recovery`
paces the BLE downlink writes → band decodes → I2S speaker.

**Barge-in:** Gemini's server VAD — running on the *clean* (AEC'd) audio we send it —
detects the user talking over the AI → emits `interrupted` → `orchestrator` fires
**FLUSH** + purges the encode queue + resets `clock_recovery` + tells `aec` the
reference stopped → near-instant silence.

## The AEC reference↔mic alignment (the crux)

AEC3 subtracts the reference from the mic — only if they're time-aligned, and ours
arrive by different paths separated by ~300–500 ms. Mechanism:

1. `aec` keeps a timestamped ring of the **reference** (the downlink PCM we generated).
2. It computes the **bulk echo delay** from three knowns: the device's per-frame
   **mic-capture timestamp** `ts32` (new), the device's **buffer-fill status** (already
   reported — gives the playout latency), and the fixed BLE/codec latencies → a
   per-frame round-trip delay that tracks drift.
3. It **pre-delays the reference** by that estimate to line up with the mic, then feeds
   both to **AEC3**, whose continuously-adapting delay estimator + adaptive filter
   cancel the residual.
4. The **clock-recovery loop keeps the bulk delay stable** (constant device buffer), so
   AEC3 converges and stays converged.

The `ts32` is the de-risking knob: it gives AEC3 a good starting delay and lets us
*measure* the true echo delay (a HW click-test calibrates the fixed-latency constants).
If measurement shows AEC3's own estimator suffices given the stabilised delay, the
timestamp remains cheap insurance + a validation aid.

## Barge-in / turn state machine (in `orchestrator`)

```
LISTENING  ──Gemini VAD: end-of-user-turn──►  AI_SPEAKING
AI_SPEAKING ──Gemini 'interrupted' (user over)──►  BARGE_IN ──►  LISTENING
AI_SPEAKING ──'turn_complete' (AI done, ring drains)──►  LISTENING
```
- During **AI_SPEAKING** the mic keeps streaming (clean, AEC'd) so barge-in is
  detectable.
- **BARGE_IN** (on `interrupted`): FLUSH downlink + purge encode queue + reset
  clock-recovery + signal `aec` the reference stopped → instant silence → LISTENING.
- Gemini owns turn detection (server VAD on our clean stream); the orchestrator shuttles
  audio and reacts to events. **v1 barge-in is host-driven** (latency = one round-trip,
  ~300–500 ms — acceptable; measured on HW). An **on-device double-talk detector** for
  sub-100 ms local barge-in is a documented future fast-path, added only if the measured
  latency feels laggy (it's a clean, separable addition; the device already holds the
  downlink PCM it's playing, so an echo-aware energy/band detector is feasible without
  full AEC).

## Conversation gating (uses the existing pose/voice gate — no new firmware)

A conversation runs **while `POSE_EAR` is held** (the natural "band at the ear" posture).
The existing gate already supports this: raising to ear turns the mic on; the first user
utterance enters `MODE_DICTATION` (which, with a subscribed host, streams the uplink); and
crucially **`DICTATION` exits only on pose-gone *and* voice-stopped** — so while the pose is
held, the uplink keeps streaming *even when the user is silent during AI playback*, which is
exactly what barge-in needs. Lowering the band (pose released) ends the uplink and the
conversation turn. So no new gating firmware is required; the orchestrator simply streams
whatever uplink arrives and treats "uplink active" as "conversation live."

## The one device-side change (firmware)

The uplink notification today is `[seq16][LC3-80B]` (82 B). Add a **per-frame mic-capture
timestamp** → `[seq16][ts32 LE][LC3-80B]` (86 B, well within MTU 247). `ts32` = the
cumulative mic sample index at the first sample of the block (16 kHz PDM clock; wraps at
2^32 ≈ 74 h). Touches `ble_audio` (notify packing) + `audio_stream` (stamp the block) +
the Mac/`audio_rx` parser. Trivial on the band; load-bearing for AEC alignment. This is
the only firmware change; everything else is Mac-side.

## Error handling

- **BLE drop mid-conversation** → device re-advertises (existing); `ble_link` reconnects;
  `orchestrator` resumes or restarts the Gemini session.
- **Gemini 24 k output is bursty** (faster than real-time) → the encode queue +
  `clock_recovery` pace it to the device's real-time consumption.
- **AEC divergence on a delay jump** → AEC3 residual-echo suppression catches it;
  clock-recovery keeps the delay stable to prevent it; a hard jump (reconnect) re-converges
  the filter.
- **Barge-in race** (FLUSH in flight as new downlink arrives) → reset clock-recovery on
  barge-in; the device's existing benign auto-restart handles stray in-flight frames.
- **No API key / Gemini unreachable** → fail fast with a clear message (prerequisite).

## Testing

- **Component (host-unit, no hardware):**
  - `codec`: LC3 encode→decode round-trip is near-identity; 24↔16 resample correctness.
  - `clock_recovery`: already unit-tested (`tools/test_audio_tx.py`).
  - `aec`: feed the **real echo recording captured in the full-duplex test** (reference +
    mic-with-echo) and assert measurable echo reduction (ERLE) + that clean output tracks
    the near-end voice.
  - `orchestrator`: drive the turn/barge-in state machine with mocked Gemini events
    (`interrupted`, `turn_complete`) and assert the FLUSH/reset actions fire.
- **HW integration (the acceptance test):** talk to the band, hear the AI, interrupt it.
  Acceptance = **with AEC on, Gemini does NOT self-interrupt** (the no-AEC failure mode is
  the control); barge-in cuts playback within a few hundred ms; the conversation is
  intelligible. Measure: ERLE, barge-in latency, end-to-end conversational latency.

## Phone portability (built in by construction)

- The **device BLE contract** (`[seq16][ts32][LC3]` uplink, status, downlink, FLUSH) is the
  stable interface; the phone app speaks the same protocol unchanged.
- The **host modules** (codec, clock_recovery, aec, gemini_client, orchestrator) are the
  reference implementation the phone re-creates natively (Swift/Kotlin) against the *same*
  AEC3 library, the *same* Gemini Live API, and the *same* algorithms.
- AEC3 and the Gemini API are cross-platform; the phone's built-in platform AEC is *useless*
  here (it assumes a local speaker), so software AEC is required on both — same code path.

## Scope (v1)

- **In:** the backend-agnostic voice-I/O core (ble_link, codec, clock_recovery, aec)
  exposing a clean mic stream + play sink + barge-in; the `VoiceBackend` interface with a
  `GeminiLiveBackend` impl + the client-VAD turn fallback; the orchestrator; the
  `[seq16][ts32][LC3]` device-timestamp firmware change; conversational-assistant mode.
- **Out:** the phone companion app (this design is its blueprint); translation mode (a
  Gemini prompt/config variant — easy follow-on); ML AEC (fallback only); the on-device
  double-talk detector (future barge-in fast-path); multi-session memory beyond what a
  Gemini Live session provides.

## Prerequisite

Gemini Live API access (key + `google-genai` Python SDK + a Live-capable model). This is
the one external dependency to have in place before the build.
