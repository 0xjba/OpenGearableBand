# Gesture-triggered live voice dictation — design

**Status:** brainstorm complete, awaiting user spec review
**Date:** 2026-06-07
**Related:**
- Memory: `project_idea_gesture_dictation_2026_06_07.md` (original idea capture)
- Architecture: `docs/research/gesture-architecture.md` §6.5 (voice-activated context state machine — related, different purpose)
- Architecture: `docs/research/gesture-architecture.md` §3 (hardware capabilities)

---

## 1. Background

The user proposed turning the gestureband into a Wispr-Flow / Superwhisper-class dictation
device: trigger a gesture, speak into the band's PDM mic, see text appear at the cursor on
their Mac in near-realtime. This is conceptually different from the closed-vocabulary
keyword-spotter already specified in architecture §6.5 — that work recognizes a small set
of pre-registered context words on-device; this work transcribes open-vocabulary speech
of arbitrary length via streaming Mac-side ASR.

The wearable-dictation product category is established (Omi, Friend, Plaud, Wispr Flow,
Superwhisper), and the open-source landscape covers every layer we need. The design below
maximizes leverage of existing open-source code and minimizes net-new engineering to the
truly novel pieces (the gesture and FSM).

## 2. Goals and non-goals

### Goals

- **G1.** Wearable-driven dictation: a deliberate compound gesture engages a hot mic;
  speech captured by the band is transcribed in near-realtime; transcribed text is
  injected at the user's text-input cursor on macOS.
- **G2.** Single integrated product: one Mac install, pair the band, done. No requirement
  for the user to install separate dictation apps, virtual audio drivers, or other tools.
- **G3.** Maximum leverage of open-source: borrow the audio pipeline (firmware + Mac BLE
  central + Opus decode) from Omi; borrow the text-injection layer from one of
  Voquill / FreeFlow / OpenWhispr / VoiceInk / Handy (chosen during the implementation
  plan based on license, language, and code-shape fit); use a streaming STT API
  (Deepgram Nova-3 as the default vendor) for transcription.
- **G4.** Compatible with a subscription business model in v1 while shipping a fully
  functional BYOK (bring-your-own-key) mode in v0.
- **G5.** Compose cleanly with the existing gesture FSM (Item 0): the new mode is added
  alongside MODE_IDLE / MODE_SURFACE / MODE_AIR_MOUSE; existing patterns
  (`_mode_needs_continuous_imu`, acq-request callback, cooldown re-engage) are reused.

### Non-goals

- **N1.** On-device ASR. The band does not run Whisper, KWS for transcription, or any
  ML model for STT. The band's only job is to capture and stream audio and emit
  control signals (HID key, mode transitions).
- **N2.** Cross-platform support. v0–v2 target macOS exclusively. iOS / Windows / Linux
  are explicit non-goals through v2.
- **N3.** Replacing the §6.5 voice-context state machine. That work uses the mic for a
  fundamentally different purpose (closed-vocabulary KWS) and ships separately. The two
  features will eventually share the PDM driver and the BLE audio service but use the
  mic non-concurrently.
- **N4.** A virtual audio device (BlackHole-style). Considered and rejected: not needed
  if our app does ASR directly in-process.

## 3. User experience

### Activation gesture (v0)

A compound AND-gate that's deliberately hard to false-trigger:

1. **Finger snap** — discrete, unmistakable transient event. Detected by the LSM6DSL
   chip-embedded tap engine, retuned for the snap impulse signature (sharper rise time
   and shorter duration than a finger-tap-on-band), with optional PDM mic acoustic
   verification of the ~2-4 kHz crack.
2. **Wrist raised, palm facing the user's face** — held condition. A new orientation
   class in the existing gravity-vector classifier, distinct from `WRIST_UP_RAISED`
   (which is palm-toward-display at shoulder height) by the palm direction (gravity
   vector signature).

Both conditions must be true for the dictation session to engage. Snap-only (hand at side)
or wrist-at-face-only (scratching nose, drinking water) is ignored.

### Flow

```
            user snaps fingers
                    │
                    ▼
       is wrist in raised palm-to-face pose?
            │                    │
           NO                   YES
            │                    │
        ignored      MODE_DICTATION engages:
                     • BLE HID key DOWN sent (configurable code,
                       defaults to right-Option)
                     • Opus audio stream starts over BLE custom GATT
                     • Mac app opens STT WebSocket; partials begin
                       streaming into text field
                            │
                            ▼
              while wrist stays in pose, session holds
                            │
                     wrist drops out of pose
                            │
                            ▼
                 MODE_DICTATION exits:
                 • BLE HID key UP sent
                 • Opus stream stops
                 • Final STT transcript committed at cursor
                 • Cooldown opens (re-engage by snap+pose during window)
```

### Edge cases

- **User snaps multiple times while pose is held**: ignored. Snap is the *engage*
  signal only; once engaged, the pose is the held signal.
- **Wrist briefly leaves pose during dictation** (e.g., natural hand sway while speaking):
  exit dwell (~250-500 ms, tuned during implementation) tolerates micro-motion. Longer
  out-of-pose triggers exit + cooldown re-engage window — same pattern as cursor modes.
- **Snap detected while in another cursor mode (SURFACE / AIR_MOUSE)**: snap is ignored
  in non-IDLE modes for v0 — only IDLE can transition to MODE_DICTATION. Avoids
  cross-mode false-triggers.
- **Cooldown re-engage**: re-engaging via snap+pose during the cooldown window skips the
  HID key debounce. Same pattern as the cursor-mode cooldown we just shipped.

## 4. Architecture

```
┌────────────────────────────────────────────────────────────┐
│  XIAO band firmware                                        │
│   ┌──────────────────────────────────────────────────┐     │
│   │ NEW (ours):                                      │     │
│   │  • Snap detector (LSM6DSL tap engine retuned     │     │
│   │    + PDM acoustic verification)                  │     │
│   │  • New pose class: wrist-raised-palm-to-face     │     │
│   │  • MODE_DICTATION in existing gesture FSM        │     │
│   │  • BLE HID keyboard report (extends HOGP we      │     │
│   │    already ship for cursor)                      │     │
│   └──────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────┐     │
│   │ BORROWED from Omi (BasedHardware/omi):           │     │
│   │  • PDM driver (Zephyr, nRF52840)                 │     │
│   │  • Opus encoder (16 kHz mono, ~24 kbps voice)    │     │
│   │  • Custom BLE GATT audio service                 │     │
│   │  • broadcast_audio_packets() pattern             │     │
│   └──────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────┘
                │                       │
                │ Opus audio (BLE GATT  │ HID key down/up
                │ notify, ~10 ms frames)│ (existing HOGP profile,
                ▼                       │  extend with keyboard
                                        │  report)
┌────────────────────────────────────────────────────────────┐
│  Our Mac app — single install                              │
│   ┌──────────────────────────────────────────────────┐     │
│   │ BORROWED from Omi macOS Swift app:               │     │
│   │  • BLE central (CoreBluetooth)                   │     │
│   │  • Opus decode → PCM 16 kHz mono                 │     │
│   └──────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────┐     │
│   │ NEW (ours):                                      │     │
│   │  • STT vendor adapter                            │     │
│   │     - Deepgram Nova-3 (default)                  │     │
│   │     - OpenAI Realtime / gpt-4o-transcribe (v1)   │     │
│   │     - AssemblyAI Universal-Streaming (v1)        │     │
│   │  • PCM → WebSocket framing                       │     │
│   │  • Streaming partial + final transcript handler  │     │
│   │  • BYOK key storage (Keychain)                   │     │
│   └──────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────┐     │
│   │ BORROWED from one of Voquill/FreeFlow/OpenWhispr/│     │
│   │ VoiceInk/Handy (pick chosen during impl. plan):  │     │
│   │  • Accessibility API focused-field detection     │     │
│   │  • Text injection at cursor (CGEvent or AX-      │     │
│   │    based; partial updates + final commit)        │     │
│   └──────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────┐     │
│   │ NEW (ours, minimal):                             │     │
│   │  • Menu-bar UI: band pairing status, mode (BYOK  │     │
│   │    vs subscription), STT vendor picker, key      │     │
│   │    entry, accessibility-permission prompt        │     │
│   └──────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────┘
                                 │
                                 ▼ types at cursor
            User's focused text-input field
            (any macOS app: Slack, Notes, browser, etc.)
```

### Component sourcing summary

| Component | Source |
|---|---|
| PDM driver | Borrow from Omi |
| Opus encoder on nRF52840 | Borrow from Omi |
| Custom BLE GATT audio service | Borrow from Omi |
| Mac BLE central + Opus decode | Borrow from Omi macOS Swift app |
| Streaming STT vendor adapter | Ours (thin wrapper around Deepgram SDK) |
| Mac app shell + text injection + AI cleanup | **Lead candidate: Voquill** (AGPLv3) — exact architectural fit (BYOK + cloud-provider abstraction + at-cursor injection + Wispr-Flow-style AI cleanup of verbal cues + personal glossary). Fallback if AGPL is rejected: Handy (MIT) with custom build-out of the cloud abstraction. |
| Snap detection (firmware) | Ours |
| Wrist-at-face pose (firmware) | Ours — extends existing orientation classifier |
| MODE_DICTATION FSM (firmware) | Ours — extends existing gesture FSM |
| BLE HID keyboard report | Ours — incremental on existing HOGP |
| Menu-bar UI | Ours (minimal) |

## 5. Activation flow detail

### Snap detection

Two-stage detector:

**Stage 1 — IMU impulse (always-on, low-power).** Configure the LSM6DSL chip-embedded
tap engine with:
- Higher threshold than current tap config (snap impulse is larger than finger-on-band tap)
- Shorter quiet time before re-arm (snap rings briefly then stops)
- Shorter shock window (snap is < 50 ms; band-tap can be 100+ ms)

The tap-engine interrupt fires the same callback path as the cursor-mode chip events
(`gesture_mode_on_chip_*`). New entry point: `gesture_mode_on_chip_snap()`.

**Stage 2 — Acoustic verification (only fires after Stage 1).** When the IMU stage
detects a candidate, briefly enable the PDM mic (~200 ms window) and run a lightweight
spectral check for the snap signature (sharp transient with energy concentrated in 2-4 kHz).
If the acoustic check passes, fire the snap confirmation.

For v0 the acoustic verification can be optional — IMU-only snap detection is acceptable
if calibrated well. Add Stage 2 if false-positive rate is too high in field testing.

### Wrist-at-face pose

Add a fourth class to the existing orientation classifier: `WRIST_AT_FACE`.

Empirical capture (during implementation): hold the band in the dictation pose (wrist
near face, palm toward face, forearm vertical) and capture the filtered gravity vector
triplet via the existing `g` serial command. Distinguish from `WRIST_UP_RAISED`
(palm-toward-display) by sign of the dominant axis.

Reuse the existing pose-dwell, entry-grace, and exit-dwell patterns from cursor modes.

### MODE_DICTATION FSM integration

```
IDLE  ── snap (while wrist already in WRIST_AT_FACE) ──→  MODE_DICTATION
IDLE  ── snap (wrist not in WRIST_AT_FACE)        ──→  ignored
ANY non-IDLE ── snap ──→  ignored (v0)

MODE_DICTATION  ── pose leaves WRIST_AT_FACE > exit_dwell ──→  IDLE + cooldown
MODE_DICTATION cooldown:
   IDLE  ── snap + WRIST_AT_FACE during cooldown ──→  MODE_DICTATION
   IDLE  ── cooldown expires (~20 s, tunable) ─────→  cleanly closed
```

The HID key down/up and audio stream start/stop are wired to the
`MODE_DICTATION ↔ IDLE` transitions inside the FSM. Audio gating happens at the source:
the BLE audio service only emits notifications while `mode == MODE_DICTATION`.

## 6. Audio pipeline

### Firmware side (borrowed from Omi)

- PDM mic at 16 kHz mono, 16-bit PCM
- 10 ms frames (160 samples)
- Opus encoder, ~24 kbps voice mode
- Custom BLE GATT audio service, notify per frame
- `broadcast_audio_packets(data, len)` pattern in `transport.c`

Power: PDM + Opus + BLE notify combined draws on the order of 1-3 mA while streaming
(per architecture §3 estimates for the §6.5 KWS work, similar ballpark). Streaming is
gated by `MODE_DICTATION`, so the duty cycle is "while user holds the pose" — typically
short bursts of 10-60 seconds.

### Mac side

- BLE central subscribes to the band's audio characteristic
- Opus decode to 16 kHz mono PCM (Apple `AudioToolbox` or libopus)
- PCM piped frame-by-frame to the STT WebSocket

End-to-end latency budget (BLE Opus → decode → WebSocket → STT partial → text injection):
target < 500 ms for the first partial to appear at the cursor. Deepgram Nova-3 typical
streaming latency is ~250-300 ms; BLE adds ~30-100 ms; decode is negligible; injection
adds < 20 ms. Total ~300-450 ms — within target.

## 7. STT integration

### v0: BYOK only

User enters their own Deepgram API key in the Mac app menu-bar settings. Stored in
macOS Keychain. Mac app opens a direct WebSocket to Deepgram's streaming endpoint per
session, streams audio, receives partial + final transcripts.

**Why BYOK first:** ships the entire pipeline without us needing to build a backend,
auth, billing, or vendor-account management. Users with Deepgram free-tier credit can
test the product. We validate the gesture + audio + injection pipeline end-to-end before
committing to backend infrastructure.

### v1: subscription proxy

A backend service (small, ~one service) that:
- Authenticates the user (Apple Sign-In or similar)
- Maintains the user's subscription state
- WebSocket-proxies audio from the Mac app to whichever STT vendor we contract with
- Returns transcripts to the Mac app

The subscription path adds a billing layer and an audio-passes-through-our-infra
privacy story. Both should be honest in marketing: "we forward, we don't store" is
defensible only if the backend is implemented stateless on audio (transcripts may be
logged briefly for debugging if user opts in).

BYOK remains supported in v1 for users who prefer the privacy or cost profile.

### Vendor adapter design

Even in v0, the Mac app's STT integration is structured as an `STTVendorAdapter`
protocol so v1 can swap between Deepgram / OpenAI Realtime / AssemblyAI / our backend
proxy without touching audio or injection code:

```swift
protocol STTVendorAdapter {
    func openSession(language: String) async throws
    func sendAudio(_ pcm: Data)
    var partialTranscripts: AsyncStream<String> { get }
    var finalTranscripts: AsyncStream<String> { get }
    func closeSession() async
}
```

## 8. Text injection and Mac app shell

### Lead candidate: Voquill (AGPLv3)

[Voquill](https://github.com/josiahsrc/voquill) is the architectural first-best fit:

| Property | Voquill | Why it matters |
|---|---|---|
| Activation model | Hold-PTT (Wispr-Flow-style hotkey) | Exact match to our BLE HID held-modifier design |
| ASR backend | Local Whisper *or* cloud STT provider (BYOK from day one) | Exact match to our BYOK-first v0 + multi-vendor v1 design |
| Text injection | At-cursor with AI cleanup ("verbal cues → formatted text") | Beyond just injection — also gets us Wispr-Flow's signature command vocabulary ("new paragraph", "comma", etc.) for free |
| Personal glossary | Built-in | A v2 feature we'd otherwise design ourselves |
| Cross-platform | Mac + Windows + Linux | Easier to port forward later |
| Project framing | Built as the OSS Wispr Flow alternative | The exact UX shape we're after |

### License trade-off

Voquill is **AGPLv3** — viral in both code distribution and network distribution
contexts. Concrete implications:

- If we fork or substantially copy Voquill, **our Mac app must be AGPL** (source
  released).
- The band firmware (separate artifact) and the v1 subscription backend (separate
  artifact, communicates only over network protocols) can remain proprietary.
- AGPL does not prevent commercialization — MongoDB, Elastic, Grafana, etc. all
  ship AGPL clients with proprietary backends.
- A source-available Mac app is a net trust positive for a privacy-adjacent
  product.

### Fallback candidates if AGPL is rejected

- **Handy** (MIT, Rust + tauri) — license-clean but local-Whisper-first; we'd
  build the cloud-provider abstraction ourselves
- **OpenWhispr** — closer to our shape (BYOK + local), no AI-cleanup of the
  Voquill kind
- **VoiceInk** (GPL-3) — same viral-license issue, less feature overlap than
  Voquill
- **FreeFlow** — lighter project, fewer features

### Implementation-plan decision

Voquill is the lead unless the AGPL license is a project-policy blocker. Final
selection is locked in during the implementation plan after a code-level read of
the Voquill repo to confirm the audio-source / text-injection / cleanup boundaries
are clean enough to swap in our BLE-central path as the audio source.

The core requirement (independent of which donor we pick): given a stream of
partial transcripts and a final transcript, inject text at the focused field such
that partials update in place and the final commits cleanly. Every Wispr-Flow-
class product already does this.

## 9. Privacy and data handling

### v0 (BYOK)

- Audio: travels from band → Mac app → user's chosen STT vendor (Deepgram by default).
  Never touches our infrastructure.
- Transcripts: returned to the Mac app, injected at cursor, not persisted by our app
  (a session log might be kept in a local file for debugging, opt-in only).
- API key: stored in macOS Keychain.

Strong privacy story: "we never see your audio or transcripts."

### v1 (subscription)

- Audio: travels from band → Mac app → our backend → STT vendor → backend → Mac app.
- We commit to: audio not stored (forwarded only), transcripts not stored (returned
  only), session metadata (start/end timestamps for billing) stored.
- Marketing line: "we forward, we don't store." Must be operationally true.

BYOK remains available in v1 for users who reject the subscription model on privacy or
cost grounds.

## 10. Power budget

(Estimates; firm up during implementation.)

| State | Estimated draw |
|---|---|
| IDLE (existing) | < 50 µA average |
| MODE_DICTATION active (PDM + Opus + BLE notify continuous) | ~1.5-3 mA |
| Snap detector (LSM6DSL tap engine always-on) | < 5 µA additional |
| PDM acoustic snap verification (200 ms windows) | negligible amortized |

Typical user pattern: 5-10 short dictation sessions per day, 30 s each = ~7 min/day
mic-on time. Battery cost: ~0.4 mAh/day at 3 mA. Well within budget.

The high-power state (MODE_DICTATION) is naturally duty-cycled by the user holding the
pose. No always-on listening, no continuous streaming.

## 11. Risks and open questions

### Validated assumptions

- nRF52840 can run Opus encoder at 16 kHz mono ~24 kbps — confirmed by Omi shipping
  this on the same chip.
- BLE custom GATT audio at 10 ms frame cadence works on Xiao BLE Sense — confirmed
  by Omi.
- Deepgram Nova-3 streaming latency < 300 ms — published vendor spec.
- macOS app can run BLE central + Opus decode + WebSocket + text injection in a single
  process — confirmed by Omi's Mac Swift app architecture.

### Open questions (resolved during implementation)

- **Snap detection reliability.** Apple Watch's "Double Clench" assistive feature uses
  similar hardware tier — proves feasibility, but we need to characterize our LSM6DSL
  tap engine's behavior on snaps specifically. Risk if false-positive rate is too high:
  fall back to Stage 2 (acoustic) verification mandatory rather than optional.
- **Wrist-at-face pose distinguishability from AIR_MOUSE raised pose.** Both involve
  the forearm being elevated. Distinguishing axis (palm direction) needs empirical
  validation — capture gravity vectors in both poses during implementation calibration
  and confirm they're separable.
- **Voquill AGPLv3 license acceptance.** Lead donor candidate is AGPL — viral in both
  code and network distribution. Mac app becomes AGPL; band firmware and v1 backend
  stay proprietary. Decision required from product owner before locking the donor in
  the implementation plan. Mitigations if rejected: pick Handy (MIT) and build the
  cloud-provider abstraction ourselves, or clean-room rewrite the injection layer.
- **Co-existence with the §6.5 KWS feature.** Both use the PDM mic. Solution: mutual
  exclusion at the mode-FSM level — `MODE_DICTATION` and the KWS active state cannot
  both be true. Define interaction explicitly when §6.5 ships.

### Known unknowns (acceptable for v0)

- Deepgram Nova-3 latency in real-world wireless conditions (vs. published spec).
- BLE throughput stability on Xiao BLE Sense at 24 kbps sustained Opus + concurrent
  HID reports. (Both fit easily in BLE 2M PHY budget; observed jitter is the unknown.)
- User comfort with snap-as-trigger vs. preference for some other discrete gesture.

## 12. Roadmap

| Version | Scope | What it proves |
|---|---|---|
| **v0** | Snap + wrist-at-face compound trigger. Omi-borrowed firmware audio. BYOK Deepgram streaming. Borrowed text-injection. One-click Mac install. | End-to-end dictation works. Validates gesture, audio, transcription, injection. |
| **v1** | Subscription backend (Apple Sign-In, billing, WebSocket proxy). Multi-vendor STT (OpenAI Realtime + AssemblyAI added). Fist-clench detection added as a third AND-condition for false-positive hardening. Per-user calibration ritual. | Business model lives. False-positive rate of compound gesture approaches zero. |
| **v2** | Polished UI (model picker, language select, custom vocab). Power optimization. App Store / Notarization for distribution. Optional companion-app features (battery indicator, session history). | Productized. |

## 13. Cross-references

- **Architecture-doc roadmap placement: Item 5** (between custom macro gestures and
  voice-context KWS). Sequenced before items 6-8 ("advanced gesture features": KWS,
  PPG-fused pinch, multi-gesture vocabulary) because it completes the "band as input
  device" story (cursor + clicks + keyboard typing replacement) before heavier
  Edge-Impulse-and-training-data work begins.
- §6.5 voice-context state machine (architecture-doc item 6): separate feature using
  the same PDM mic; mutual exclusion required at the FSM level (cannot both be
  active). Coexistence will be implemented when item 6 ships.
- Cursor-mode cooldown bug (project memory 2026-06-07): the cooldown-keeps-acq-alive
  pattern shipped on `feature/gesture-foundation` is reused for `MODE_DICTATION`
  cooldown — same problem, same fix, no new design.
- Hardware wear position (memory): volar-side wrist mount means raising the wrist to
  the face puts the band toward the user's mouth — favorable PDM SNR. The wrist-at-face
  gravity-vector signature also follows from the volar mount.

## 14. Appendix: rejected alternatives

- **On-device Whisper inference** (WhisperKit on Mac side). Rejected because:
  business-model viability requires a recurring-cost component; streaming API gives
  better transcription quality (Deepgram Nova-3 / OpenAI Realtime beat OSS Whisper);
  avoids 1+ GB model bundle / download.
- **BlackHole virtual audio device + user's choice of any dictation app**. Rejected
  because: violates "single install" goal; forces user to install + configure three
  things (BlackHole, our app, dictation app); doesn't capture a subscription business.
- **iOS as the host platform**. Rejected for v0–v2 because: iOS text injection is
  fundamentally harder (custom-keyboard sandbox, no global PTT key, no Accessibility
  API for arbitrary apps). Revisit post-v2 if user demand is there.
- **Wrist-at-face pose alone as trigger** (no snap). Rejected because: geometrically
  too similar to `WRIST_UP_RAISED` (AIR_MOUSE pose) and not deliberate enough — user
  can accidentally trigger by scratching nose, drinking water, etc. The snap fixes
  this.
- **HID held-modifier + virtual mic + external dictation app**. Earlier iteration of
  this design; rejected in favor of the single-install architecture above.
