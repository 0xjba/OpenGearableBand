# OLED-variant voice→text bridge (host) — design

**Goal:** A host app that closes the OLED-variant loop: you speak into the band,
and a text reply appears on its OLED. v1 uses **Gemini all-in-one** (STT+LLM in
one call) to prove the full round-trip fast; the "brain" is behind a single seam
so the real staged pipeline (local STT → gateway → agent) drops in later without
touching the harness.

**Runs from the user's Terminal** (`tools/dtln-venv/bin/python tools/oled_bridge.py`)
— macOS grants Bluetooth to Terminal, not to sandboxed processes.

## Product architecture (target — for context, NOT built in v1)
```
band (raise to POSE_READ, audio over BLE)
  → phone/PC app → STT → gateway (our service: intent route + memory lookup)
  → LLM or agent (API / Hermes / OpenClaw / MCP tool) → text over BLE → display
```
v1 collapses STT+LLM+gateway into one `GeminiBrain`; the harness around it is the
same one the product will use.

## Architecture
A stable **harness** (BLE I/O, LC3 decode, AGC, turn-bounding, pagination) wrapping
a single swappable **Brain**:

```
band (POSE_READ+voice → LC3 uplink)
  → [BLE]        subscribe uplink audio char (47A10002); hold display-text char (e9a10002)
  → [decode+AGC] LC3 -> int16 -> float32 16 kHz; normalize toward ~-12 dBFS
  → [turn-bound] band stops uplinking when dictation ends -> ~500 ms frame gap = utterance done
  → BRAIN.process(utterance_pcm) -> reply_text
  → [paginate]   word-wrap reply to 2-line pages; send page 1, then ~2.5 s/page (host scroll)
  → OLED
```

## Components (one file, small functions)

### Brain (the one seam)
```python
class Brain:
    def process(self, pcm_f32_16k) -> str: ...   # utterance audio in -> reply text out
```
- **v1 `GeminiBrain`** — one `genai` `generateContent` call: the utterance audio
  (16 kHz PCM, sent as an inline audio part) + a concise system instruction
  ("Answer in one short, glanceable sentence; prefer <= ~40 chars."). Gemini does
  STT + LLM together. Model: a current Gemini model with audio input
  (e.g. `gemini-2.5-flash`); confirm the exact id at build time against the
  installed `google-genai`. Key from `GEMINI_API_KEY` (env / git-ignored `.env`).
- **Future `PipelineBrain`** (NOT in v1) — same `process()`, internally
  STT (local: parakeet/whisper/moonshine, escalate to Deepgram) → gateway
  (intent + memory) → agent (API / Hermes / OpenClaw / MCP). No harness change.

### BLE (bleak)
- Scan by name substring `gband` (matches `gband-oled`); connect.
- `start_notify` on the uplink audio char; keep the display-text char handle for writes.
- Reconnect on disconnect.

### Decode + AGC
- Each notification `[seq16][ts32][80B LC3]` → `Lc3Codec.decode` (2×40 B → 2×160
  samples) → int16 → float32 / 32768.
- AGC: compute the utterance's peak/RMS, apply a gain toward a target (~-12 dBFS),
  clamped to a sane max (e.g. ≤ 40×) so silence isn't amplified into noise. Applied
  once per utterance before the brain (adaptive — no hardcoded gain).

### Turn boundary
- The band only uplinks while its dictation gate is open (POSE_READ + voice). When
  the user finishes and the gate closes, uplink frames stop. The host treats a gap
  of > ~500 ms with no frames (while it holds buffered audio) as end-of-utterance.
- Ignore utterances shorter than a floor (e.g. < 300 ms) as accidental.

### Display / scroll
- Word-wrap the reply into 2-line pages (reuse the device's ~12-char/line wrap
  logic on the host side; the device renders whatever 1–2 lines it receives).
- Send page 1 immediately; subsequent pages every ~2.5 s. Short reply = 1 page.
- Also show status lines ("thinking…") between utterance-end and the reply.

## Data flow (one turn)
1. Frames arrive → decode → append PCM to the current-utterance buffer.
2. Frame gap > 500 ms → utterance complete.
3. AGC the buffer → `brain.process()` → reply text.
4. Paginate → write pages to the display char (scrolled) → OLED.
5. Clear buffer, wait for the next turn.

## Error handling
- Missing `GEMINI_API_KEY` → clear message, exit.
- BLE disconnect → attempt reconnect; log.
- Brain error / empty reply → show a short "…" / "no reply" on the OLED, continue.
- Gemini rate-limit / network error → catch, show "error", continue.

## Testing
- `--dump` prints each utterance's transcript-equivalent (the brain's reply) + timing
  to the terminal, so you can verify without watching the tiny screen.
- Manual loop: run bridge → trigger dictation (`d` force or POSE_READ) → speak a
  question → see the reply paginate on the OLED (and in `--dump`).
- `GeminiBrain` unit-testable offline by feeding a known WAV (bypassing BLE) via a
  `--wav <file>` input mode.

## Out of scope (v1)
- The staged pipeline (separate STT / gateway / agent) — future, behind the same seam.
- Barge-in / interruption (no speaker; turn-based glance model).
- On-device changes (firmware already exposes both chars; harness is host-only).
