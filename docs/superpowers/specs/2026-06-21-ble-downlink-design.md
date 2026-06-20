# BLE Audio Downlink (phone → speaker) — Design

**Date:** 2026-06-21
**Branch:** feature/gesture-foundation
**Status:** Approved (design), pending implementation plan
**Builds on:** `audio_out` (speaker engine, committed af6d374), the uplink
(`ble_audio`/`audio_stream`/`lc3_codec`, committed cd38b3f).

## Goal

Close the two-way voice loop: play the AI's spoken response on the speaker. The
phone bridges the device to a real-time AI API (Gemini Live etc.); this is the
**device-side downlink** — receive LC3 audio over BLE, decode, and play it through
`audio_out`, plus an interruption (barge-in) flush. Mirror image of the uplink.

## Design principle (load-bearing)

**Bare minimum on device; heavy lifting on the phone** (battery MCU). The device
only: receives LC3, does a light decode (<1 ms/frame), plays, and flushes on
command. The phone does everything heavy: the AI API, codec/rate bridging, VAD,
turn management, and barge-in detection. LC3 stays on-device because the trivial
codec cost is far cheaper than the ~8x heavier radio of raw PCM (32 vs 256 kbps).

## Why LC3 16 kHz both directions (verified against the AI API, 2026-06-21)

Gemini Live (the demanding real-time case; translation behaves the same):
- **Input:** raw 16-bit PCM, **16 kHz**, mono, LE (`send_realtime_input`,
  `audio/pcm;rate=16000`); compressed codecs only for non-realtime uploads.
- **Output:** raw 16-bit PCM, **24 kHz**, LE, streamed in chunks.
- **Turn signals:** `generation_complete` then `turn_complete`.
- **Barge-in:** an **`interrupted`** signal; the client MUST stop + flush queued
  audio immediately or the user hears "ghost audio."

So the device stays on **LC3 16 kHz** both ways; the phone translates to/from the
AI's PCM rates. Our existing 16 kHz LC3 uplink already matches the AI's 16 kHz
input rate — no uplink change needed.

## Architecture

```
phone ──audio frames──► ble_audio downlink char (Write-No-Resp) ─► audio_downlink
                                                                     (k_msgq + decode thread)
                                                                        │ lc3 decode
                                                                        ▼
phone ──FLUSH (barge-in)─► ble_audio control char (Write) ─► audio_downlink_flush ─► audio_out_*
                                                                        │
                                                              audio_out_write ─► [audio_out ring → feeder → I2S → MAX98357A → speaker]
```

## Components

### 1. `lc3_codec` — add a decoder
Add to the existing encoder module (same liblc3, `CONFIG_LIBLC3`):
- `int lc3_codec_decode_frame(const uint8_t *in40, int16_t *pcm160_out);` — one
  10 ms frame: 40 B LC3 → 160 int16 PCM @ 16 kHz.
- Static `lc3_decoder_mem_16k_t` (~2 KB), no heap. Init alongside the encoder.
- Mirrors the encoder's params (16 kHz / 10 ms / 32 kbps). Plays bad/lost frames
  as liblc3's PLC (pass a null/zero-length frame) — optional for v1.

### 2. `ble_audio` — add two characteristics
Extend the existing service (it already owns the uplink notify char):
- **Downlink audio** `47A10003-…` — **Write / Write-Without-Response**. Phone
  writes `[seq16 LE][LC3 payload]` (payload = one or more 40 B frames, typically
  an 80 B / 20 ms block). Write callback validates length and calls
  `audio_downlink_feed(payload, len)`. WWR = unacked/high-throughput, like notify.
- **Control** `47A10004-…` — **Write**. 1-byte command; v1 defines `0x01 = FLUSH`
  (barge-in). Write callback calls `audio_downlink_flush()`. The byte leaves room
  for future commands (e.g. explicit stop, volume) without a new characteristic.
- Reuse `ble_audio_set_fast_conn(true)` while a downlink stream is active (~50
  writes/s needs a short conn interval); relax on stop.
- `ble_audio_downlink_subscribed()` is not needed (writes are central-initiated);
  the device just accepts writes when connected.

### 3. `audio_downlink` — new bridge module (mirror of `audio_stream`)
Decouples the BLE RX context from decode+play (the discipline that fixed sub-
project B's slab starvation):
- `int audio_downlink_init(void);` — boot init (ensures decoder ready).
- `void audio_downlink_feed(const uint8_t *lc3, size_t len);` — called from the
  BLE write callback; **copies the raw LC3 payload into a `k_msgq`** (non-blocking,
  drop-on-full + drop counter). Does NO decode in the BLE context.
- **Decode thread** (prio 7, ~2 KB stack): drains the msgq → `lc3_codec_decode_frame`
  per 40 B frame → `audio_out_write(pcm, 160)`. On the first frame after idle, it
  `audio_out_start(16000)`. The `audio_out` silence auto-stop ends a normal turn
  (phone stops sending at `turn_complete` → ring drains → amp mutes).
- `void audio_downlink_flush(void);` — barge-in: purge the msgq AND
  `audio_out_stop()` (drops queued I2S + the ring's buffered audio) → instant
  silence, no ghost audio. (Add `audio_out_flush()` = ring-reset + stop if needed
  to guarantee the ring is cleared immediately.)

### 4. Integration
`main.cpp` calls `audio_downlink_init()` at boot; `ble_audio` registers the two
new chars (auto via `BT_GATT_SERVICE_DEFINE`). No change to the uplink, gesture,
or HR code. CMakeLists adds `src/audio_downlink.cpp`.

## Data flow

- **Normal turn:** frames arrive → decode thread → `audio_out` plays; phone stops
  at `turn_complete` → silence auto-stop mutes the amp. No control traffic.
- **Barge-in:** phone sees `interrupted` → writes `FLUSH` → device purges decode
  queue + stops `audio_out` → silent within a BLE round-trip; buffered audio
  discarded.

## Phone-side contract (documented for the phone app; out of scope here)
- **Uplink:** LC3-decode the device's notify stream (our params: 16 kHz/10 ms/
  32 kbps/40 B) → 16 kHz PCM → `send_realtime_input` (`audio/pcm;rate=16000`).
- **Downlink:** take the AI's 24 kHz PCM output → resample to 16 kHz → LC3-encode
  (our params) → write `[seq16][LC3]` to the downlink char in ~20 ms blocks.
- **Barge-in:** on the AI's `interrupted` signal, immediately write `FLUSH` and
  stop sending downlink frames.

## Tunables (housing/production retune)
- **Jitter buffer / latency:** the downlink wants LOWER latency than the SD test
  (conversation). `audio_out`'s current `RING_BYTES`/`PREBUF_BYTES` are SD-tuned
  (generous). Make the prebuffer depth a parameter (e.g. `audio_out_start(rate,
  prebuf_ms)`) or a downlink-specific smaller value (~60-120 ms). `[STRUCTURAL]`,
  tuned on hardware (latency vs underrun).
- Decode msgq depth, decode-thread priority. `[STRUCTURAL]`.

## Testing (hardware-in-the-loop + host sender)
- **`tools/audio_tx.py`** — host BLE central (bleak + liblc3 via ctypes, mirror of
  `tools/audio_rx.py`): connect to the device, LC3-encode a WAV (16 kHz), write
  `[seq16][LC3]` blocks to the downlink char, and optionally fire a `FLUSH` to
  test barge-in. Verifies the full path without a phone app.
- HW: run `audio_tx.py` from the Mac → hear the WAV on the speaker; trigger FLUSH
  → playback stops instantly; confirm uplink (mic notify) + downlink coexist.

## Scope
- **In:** the device-side downlink (decoder, 2 BLE chars, bridge, audio_out
  integration) + the host test sender.
- **Out:** the phone app (separate project; contract documented above), and any
  on-device AI/VAD/turn logic (that's the phone's job per the design principle).
