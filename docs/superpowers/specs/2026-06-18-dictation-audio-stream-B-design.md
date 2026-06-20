# Sub-project B — Dictation Audio Stream (BLE, LC3) — Design

**Date:** 2026-06-18
**Branch:** `beta`
**Status:** Design — approved in brainstorm, pending spec review
**Depends on:** A.1 (POSE_EAR + voice-onset → `MODE_DICTATION`, detect+log) — DONE.
**Source of borrowed parts:** prior same-chipset project `oneDiary`
(`/Users/0xjba/Projects/personal-projects/oneDiary`).

---

## 1. Goal

When the gesture FSM is in `MODE_DICTATION`, stream the live microphone audio off
the band over BLE as **LC3-encoded frames**, so a host can decode it (and later
transcribe it). B is **firmware + a throwaway Mac test receiver only** — no
production host app, no on-device STT.

This is the first sub-project that produces real off-band output. A.1 only
detected and logged the dictation gesture; B carries the audio.

## 2. Context & key decision: borrow LC3 from oneDiary

oneDiary is an **always-listening** recorder (PDM mic → LC3 → SD card +
optional BLE streaming) built on this exact chip. It is a *different product*;
we port only the apt elements, not the architecture.

The important find: oneDiary encodes with **LC3** (Google `liblc3`), which ships
**native in NCS** (`CONFIG_LIBLC3=y`) — not Opus. LC3 is the BLE LE-Audio
standard codec: low-complexity, low-RAM, FPU-friendly, and already proven
working on this chip. This supersedes the umbrella dictation spec's "borrow Omi
→ Opus" plan. **B uses LC3.**

oneDiary's LC3 parameters already match our mic exactly: **16 kHz mono, 10 ms
frames, 32 kbps, 40 encoded bytes/frame, 160 PCM samples/frame.** Our `mic_vad`
captures 16 kHz mono in 20 ms blocks (320 samples) = exactly two LC3 frames per
block.

## 3. Scope

### In scope (B)
- Port `lc3_codec.{c,h}` (~verbatim; params already match).
- A custom BLE GATT **audio service** (128-bit UUID), data NOTIFY characteristic,
  oneDiary-compatible frame framing.
- A new `audio_stream` glue module: PCM block → LC3 encode → dispatch to the
  enabled **sink(s)** (B has one sink: BLE notify).
- A one-line hook in the `mic_vad` capture loop.
- BLE service registration + advertising update in `main.cpp`.
- `prj.conf` deltas (LC3, FPU, BLE MTU/DLE/buffers).
- A throwaway **Mac receiver script** (Python: `bleak` + `liblc3`) that
  subscribes, decodes frames → WAV (+ optional live playback) to verify
  throughput and that the audio is real/intelligible.

### Out of scope (B), explicitly deferred
- **SD card path** (oneDiary `sd_storage.{c,h}`, `sd_writer` thread, FIFO/slab,
  background mode, bulk transfer). The SD module is on the near-term roadmap, so
  B is **designed for** it (see §9) but does **not** build it. oneDiary's SD
  files remain the reference for that future sub-project.
- oneDiary's **host command protocol** (control char with start/stop/mode/SD
  commands). B is gesture-gated; the device decides when it dictates. No control
  char.
- The oneDiary **Flutter app**. Not needed — a Mac test script suffices; a real
  host app is a later sub-project.
- On-device STT / transcription.

## 4. Architecture & data flow

```
 PDM mic (DMIC, 16 kHz)
        │  one DMIC owner, one capture thread (already runs during MODE_DICTATION)
        ▼
 mic_vad capture loop  (src/mic_vad.cpp : mic_thread)
   dmic_read → block (pcm, nsamp=320)
        ├── VAD: mic_spectral() → onset / voice_active        (existing A.1 path)
        └── audio_stream_feed(pcm, nsamp)   ← NEW hook, before k_mem_slab_free
                 │
                 ▼
         audio_stream module  (src/audio_stream.{h,cpp})  ← NEW
           if !audio_stream_active(): return            (gating, §6)
           lc3_codec_encode_pcm_buffer(pcm) → 2 × 40 B LC3
           for each frame: dispatch to enabled sinks
                 │
                 ▼  (B: exactly one sink)
         BLE audio sink  (src/ble_audio.{h,cpp})  ← NEW (ported from oneDiary ble_service)
           build [type|seq16|40B] → bt_gatt_notify (non-blocking, drop-on-ENOMEM, seq++)
```

**REVISED 2026-06-18 (post bring-up + deep research):** the LC3 encode + BLE
notify do **NOT** run on the capture thread. `audio_stream_feed()` copies the PCM
block into a `k_msgq` and returns immediately (so `mic_vad` frees the slab at
once); a **dedicated audio thread** (priority 7, below the capture thread's 6)
drains the queue and does encode + notify. See §7 for the root cause and the
research that drove this.

## 5. Modules & wire format

### 5.1 `lc3_codec.{c,h}` — ported ~verbatim
- API: `lc3_codec_init()`, `lc3_codec_encode_frame(pcm160, out40)`,
  `lc3_codec_encode_pcm_buffer(pcm320, out80, *out_len)`, `lc3_codec_is_ready()`.
- Static `lc3_encoder_mem_16k_t` (~2 KB), no heap. liblc3 calls:
  `lc3_setup_encoder(10000, 16000, 0, &mem)`, `lc3_encode(enc, LC3_PCM_FORMAT_S16,
  pcm, 1, 40, out)`.
- Requires `CONFIG_LIBLC3=y`, `CONFIG_REQUIRES_FULL_LIBC=y`, FPU.

### 5.2 `audio_stream.{h,cpp}` — new glue (the SD-ready seam)
- `void audio_stream_init(void);`
- `void audio_stream_feed(const int16_t *pcm, size_t nsamp);`
  — encodes one 320-sample block to two LC3 frames, dispatches each to enabled
  sinks. No-op when not active.
- `bool audio_stream_active(void);` — gating predicate (§6).
- Internally holds the list of sinks. **B registers exactly one: the BLE sink.**
  A future SD sink registers here without touching the encode stage (§9).

### 5.3 `ble_audio.{h,cpp}` — new BLE audio service (ported from oneDiary `ble_service`)
- **128-bit UUID base (gestureband-specific, distinct from oneDiary):**
  - Audio Service: `47A10001-9B70-4C2E-8A1D-2F6B9E4A77C1`
  - Audio Data char (NOTIFY + CCC): `47A10002-9B70-4C2E-8A1D-2F6B9E4A77C1`
- **Frame format (gestureband-owned; intentionally NOT oneDiary-compatible —
  separate products):** `bytes 0–1 = seq16 little-endian`, `bytes 2..81 = one
  20 ms encoded block = 80 bytes (two 40-byte LC3 frames)` ⇒ **82 bytes /
  notification**, ~50 notifications/s. No "type" byte (that was an oneDiary
  multiplexing artifact we don't need). The 2-byte sequence number is the only
  header — kept solely so the test receiver can detect dropped/reordered
  notifications. Host splits the 80-byte payload into 2×40 B and LC3-decodes
  each.
- `ble_audio_notify(const uint8_t *lc3_40, ...)`: builds the 43-byte packet,
  `bt_gatt_notify(conn, &svc.attrs[<data value>], pkt, 43)`, increments seq
  (wraps at 0xFFFF). On `-ENOMEM` (TX queue full): **drop the frame**, bump a
  `ble_audio_drops` counter, never block.
- `bool ble_audio_subscribed(void);` — true when a host has enabled the data CCC
  and a connection exists.
- No control char, no status char in B (YAGNI for a throwaway host; codec params
  are fixed and documented). An optional READ "info" char is a possible future
  add, not built now.

### 5.4 `mic_vad.cpp` — one-line hook
Insert `audio_stream_feed(pcm, nsamp);` immediately **before** the
`k_mem_slab_free(&mic_slab, buf);` at `src/mic_vad.cpp:163`, after `mic_spectral`.
`mic_vad` stays otherwise isolated; the feed is a no-op unless streaming is active.

### 5.5 `main.cpp` — registration & advertising
- Register the audio service (static `BT_GATT_SERVICE_DEFINE`) — coexists with
  HRS+BAS (16-bit UUIDs; **no collision** — verified, see §8).
- Call `audio_stream_init()` / `lc3_codec_init()` at boot.
- Advertising: HRS+BAS UUID16 list stays in `ad[]`. The 128-bit audio UUID does
  **not** need to be advertised — the test host connects by name (`"Xiao_Pulse"`)
  and discovers the service. (Optionally add the 128-bit UUID to the scan
  response if a host needs to filter on it; not required for B.)

## 6. Gating — gesture-driven, no host command

`audio_stream_active()` is true **iff all hold**:
1. `gesture_mode_get() == MODE_DICTATION`, **and**
2. `ble_audio_subscribed()` (host connected + data CCC enabled).

The mic is already turned on during `MODE_DICTATION` by the A.1 ear-gate, so PCM
is already flowing; B only decides whether each block is also encoded+sent.
Entry/exit ride the **existing** ear-gate FSM (raise-to-ear + voice-onset in,
pose-gone + voice-stopped / session-silence out) — B adds no new entry/exit
logic. The `'m'` bench probe can run the mic without streaming (gate condition 1
is false outside `MODE_DICTATION`).

## 7. Threading — dedicated audio thread (FINAL, 2026-06-18)

**Decision: encode + notify run on a DEDICATED audio thread, NOT on the capture
thread.** The first build did it inline and HW-tested it — that produced
intermittent `dmic_nrfx_pdm: Failed to allocate buffer: -12` / read `-EAGAIN`,
capture restarts, and ~1 s audio dropouts. A deep-research pass
(`docs` / 2026-06-18) confirmed the root cause at the driver-source level and the
fix:

- Zephyr's `dmic_nrfx_pdm` allocates its next DMA buffer with `K_NO_WAIT` and
  **fail-STOPS** (`stop=true → nrfx_pdm_stop()`) the instant the slab returns
  `-ENOMEM`. So the slab must be **drained faster than it fills** — it is
  fail-fast pool exhaustion, *not* blocking back-pressure.
- Holding a slab buffer through the variable-latency `bt_gatt_notify` on the
  capture thread lowers the drain rate → starvation. Encode CPU was never the
  problem (~5 ms wall-clock incl. preemption vs the 20 ms budget); **latency
  coupling** between capture and BLE was.
- The consensus fix (and Nordic's own LE-Audio reference: a FIFO runs the audio
  datapath in its own thread, block-based FIFO between stages) is a
  producer/consumer split.

**Implemented design:**
- Capture thread (`mic_vad` `mic_thread`, prio 6, stack stays 2048): `dmic_read`
  → `audio_stream_feed()` copies the 640 B PCM into a `k_msgq` (`K_NO_WAIT`,
  drop-if-full) → frees the slab immediately → runs the FFT VAD. Slab held only
  for a memcpy.
- Audio thread (`audio_stream.cpp` `audio_thread`, **prio 7 — below capture so it
  can never delay the slab drain**, stack 8192 for LC3 FPU frames): drains the
  `k_msgq` (depth 6 ≈ 120 ms slack), LC3-encodes, `ble_audio_notify`.
- Instrumentation kept: `[AUD] encode+notify=… us, ble_drops=… q_drops=…` every
  ~1 s, for the HW bring-up CPU/headroom read.

**Connection interval (research's free win):** on the first PCM block of a burst
the audio thread calls `ble_audio_set_fast_conn(true)` (request ~15–30 ms so the
host can sustain ~50 notif/s); after `AUDIO_STREAM_IDLE_MS` (500 ms) of no audio
it calls `ble_audio_set_fast_conn(false)` (relax to ~30–50 ms for power). The
host may honour/ignore it (esp. iOS) — the actual negotiated interval is logged
via `le_param_updated` (`conn params: interval=…`), the open item to verify on HW.

## 8. BLE coexistence & `prj.conf` deltas

### Verified clean (data, 2026-06-18)
`beta` has **no HID** anywhere: `prj.conf` BLE is only
`BT_HRS`, `BT_BAS`, `BT_SMP`, `BT_SIGNING`, `BT_SETTINGS`, `BT_MAX_PAIRED=4`. No
`CONFIG_BT_HIDS`, no HID/mouse code in `src/` (the only "air-mouse/HID" hits are
history comments). The HID-mouse GATT lives on `feature/air-mouse`. So our new
128-bit audio service collides with nothing.

### Deltas to add (reconciled with our existing config)
```
# LC3 codec
CONFIG_LIBLC3=y
CONFIG_REQUIRES_FULL_LIBC=y

# FPU — verify already on (CMSIS FFT needs it); add if absent
CONFIG_FPU=y
CONFIG_FPU_SHARING=y

# BLE throughput for 43-byte audio notifications
CONFIG_BT_L2CAP_TX_MTU=247
CONFIG_BT_BUF_ACL_TX_SIZE=251
CONFIG_BT_BUF_ACL_RX_SIZE=251
CONFIG_BT_CTLR_DATA_LENGTH_MAX=251   # DLE: 43-byte frame fits in 1 LL PDU
CONFIG_BT_L2CAP_TX_BUF_COUNT=12      # queue headroom (oneDiary used 20 for SD chunking; RT needs less)
```

Notes for the plan to reconcile against oneDiary's set:
- We keep `CONFIG_BT_SIGNING=y` and `CONFIG_BT_DEVICE_NAME="Xiao_Pulse"` (ours);
  oneDiary used different values.
- oneDiary pins `CONFIG_BT_MAX_CONN=1` and a 7.5 ms connection interval. Our
  audio load is tiny (~100 frames/s × 43 B ≈ 4.3 KB/s) and multiple
  notifications can fit per connection event, so **we do not need to force
  7.5 ms** — leave interval to negotiation and only tighten if the bring-up
  measurement shows starvation. Keep `MAX_CONN` at its current value.
- Confirm HRS/BAS notifications still behave under the MTU/DLE changes (they
  will — these only raise ceilings).

## 9. SD-readiness seam (deferred, design-for)

The SD card module is coming. B keeps SD an **additive** change, not a refactor,
via the `audio_stream` sink seam: the encode stage produces LC3 frames and
dispatches to enabled sinks. B registers the BLE sink only. When SD lands, an SD
sink (ported from oneDiary `sd_storage` + the `sd_writer`/FIFO backpressure
pattern) registers alongside — same LC3 frames, no change to encode or capture.
This mirrors oneDiary's background/realtime duality, with the fan-out made
explicit instead of an `if/else` on a mode flag. **B does not build the SD sink
or copy the SD files** — it only leaves the seam and names the reference.

## 10. Test plan — Mac receiver (throwaway)

Hardware-in-the-loop, plus the existing host unit-test path for any pure helpers.

1. **Build/flash** clean (`./build.sh`, copy UF2).
2. **Bring-up instrumentation** (§7): serial log of per-block encode cycles +
   block period. Confirm headroom with real numbers before trusting inline.
3. **Mac receiver script** (`tools/audio_rx.py`, ~30–50 lines):
   - `bleak`: scan for `"Xiao_Pulse"`, connect, request MTU, discover the audio
     service, `start_notify` on the data char.
   - Parse each notification: `type, seq = b[0], b[1] | b[2]<<8`; track seq for
     gap/drop detection; collect the 40-byte LC3 payload.
   - Decode via `liblc3` (ctypes binding or `pylc3`): `lc3_setup_decoder(10000,
     16000, 0)`, `lc3_decode` → 160×int16 PCM/frame.
   - Write PCM → WAV (16 kHz mono); optional live playback (`sounddevice`).
4. **Validation criteria:**
   - Notifications arrive only while in `MODE_DICTATION` (raise-to-ear + speak),
     and stop on dictation exit.
   - Sustained ~100 frames/s with low `ble_audio_drops`.
   - Decoded WAV is intelligible speech (the real proof the pipeline works).
   - HRS/BAS still notify normally during streaming.

## 11. Risks / open items — RESOLVED WITH HW DATA (2026-06-18)
All measured on HW with the thread analyzer (since reverted) + a force-stream
serial command (since reverted), streaming concurrently with an HR snapshot:
- **Capture-thread timing / `-12` starvation** — FIXED by the dedicated audio
  thread. Zero `dmic_nrfx_pdm -12` / `-EAGAIN` / capture restarts across many
  minutes of capture (inline build threw them within ~12 s).
- **Concurrent CPU budget** — `audio_thread` (LC3 encode + notify) **15–17%**,
  `mic_thread` (capture + FFT VAD) 6–7%, `acq` (IMU/Mahony/gesture) 3%, HR DSP
  ~0–1%. **Worst case (audio + HR snapshot + IMU) ≈ 30% total, ~70% idle** —
  ample headroom for the future workout feature.
- **`audio_thread` stack** — measured high-water 3520 B → set to **6144** (was a
  provisional 8192). **`mic_thread`** high-water 576 B → **2048 kept** (safe).
- **Connection interval** — the fast-conn request is honoured: macOS moved
  45 ms → **30 ms** on stream start and also raised the supervision timeout from
  a flaky 420 ms to 4 s (which stabilised the link).
- **TX buffer count (12)** — `ble_audio_drops = 0` throughout. `q_drops`: 7
  blocks dropped only at stream start (a one-off ~89 ms stall during the conn-param
  renegotiation), then **0 for the rest** — steady-state is clean. Queue depth 6
  is sufficient; revisit only if startup clipping matters.
- **liblc3 in our NCS tree** — confirmed: `CONFIG_LIBLC3=y` builds and links
  (`modules/liblc3/libliblc3.a`). Audio decoded to an intelligible WAV on Mac.
- **Mac `liblc3` decode** — built a dylib from the NCS liblc3 sources (no
  `-ffast-math`), ctypes-wrapped in `tools/audio_rx.py`. Works.

### New finding (separate, for A.1 — not a B issue)
The VAD **floor-latch assumes silence during its first ~0.5 s window**. If the
user is talking/noisy at the moment of raise, the floor latches high (observed
veM≈15638 vs ~150 normal), the voice threshold inflates ~8×, and onset never
fires → `MODE_DICTATION` never enters. Worth an A.1 robustness pass (e.g. cap the
floor, or re-latch on a quiet sample). Tracked separately.

## 12. References
- Borrowed-from: `oneDiary/src/{lc3_codec,ble_service,pdm_audio}.{c,h}`,
  `oneDiary/prj.conf`. SD reference for the future: `oneDiary/src/sd_storage.*`,
  `sd_writer` thread in `oneDiary/src/main.c`.
- Ours: `src/mic_vad.cpp` (capture hook at line 163), `src/main.cpp`
  (BLE init/adv ~line 188+), `src/gesture_mode.*` (`MODE_DICTATION`, gating).
- Umbrella: `docs/superpowers/specs/2026-06-07-gesture-triggered-dictation-design.md`.
- Entry FSM: `docs/superpowers/specs/2026-06-17-dictation-entry-A1-voice-onset-fsm-design.md`.
