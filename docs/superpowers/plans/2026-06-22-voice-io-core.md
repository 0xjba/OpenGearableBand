# Backend-Agnostic Voice-I/O Core — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a backend-agnostic, full-duplex voice I/O layer on the Mac — `mic → BLE → DTLN neural AEC (ts32-drift-corrected reference) → energy-gated barge-in VAD → clean-mic-out + audio-play-sink-in + barge-in signal` — with **no hardcoded drift constant** and **no AI backend dependency**.

**Architecture:** A Python package `tools/voiceio/` of small single-responsibility modules (pure DSP helpers unit-tested with pytest; BLE + real-time integration verified on-device and on recorded sessions). One firmware change: the uplink frame gains a `ts32` mic-sample-count timestamp so the host can estimate the device clock rate live and resample the AEC reference to it. The validated DSP — DTLN suppressor + ts32-driven ASRC + energy-gated VAD — is already proven offline (`tools/dtln_test.py`, `tools/barge_test.py`); this plan productizes it into reusable modules behind a clean core interface, with a pluggable `VoiceBackend` (an echo-loopback test backend ships here; Gemini lands later with no core changes).

**Tech Stack:** Python 3.11 (`tools/dtln-venv`: tensorflow/TF-Lite, soundfile, numpy, webrtcvad; add `soxr`), `bleak` (BLE), liblc3 (ctypes), DTLN-aec TF-Lite model. Firmware: Zephyr/C on nRF52840 (`./build.sh`).

**Source of truth:** `docs/superpowers/specs/2026-06-21-mac-voice-loop-design.md` — see the "⚑ VALIDATED ARCHITECTURE (2026-06-22)" section (DTLN suppressor, ts32-ASRC drift, energy-gated barge-in).

---

## Phase roadmap (each phase = working, testable software on its own)

- **Phase A — Drift compensation. ✅ DONE (2026-06-22), but RESOLVED DIFFERENTLY than first specced.** The empirical A6 gate revealed the speaker (I2S 15873) and mic (PDM 16000) are SYNCHRONOUS (both off the nRF52840's one HFCLK), so the echo-alignment is a FIXED ratio `15873/16000 = −0.794%`, NOT a live-tracked drift. Final design: `voiceio.clocks.AEC_DRIFT_RATIO` → `voiceio.resample` → DTLN (no live estimator). The `ts32` firmware (A1) + `audio_tx` parse/sidecar (A4) stay (jitter/gap use); the live `ts32` estimator (was A3) was deleted. Verified on HW: fixed constant holds echo removed end-to-end (early −71 / late −80 dB). See the spec's "⚑ VALIDATED ARCHITECTURE §2" for the full reasoning. *(Tasks A1/A2/A4 stand as written; A3 deleted, A5 stands, A6 = the constant-verifies-end-to-end check, done.)*
- **Phase B — `voiceio` package: pure modules + offline pipeline.** `codec`, `frame` (wire format), `drift` (from A), `aec` (DTLN streaming wrapper), `vad` (energy-gated barge-in). One offline `pipeline.run_session(recording)` ties them together on a recorded session. Pure helpers TDD'd; pipeline checked against `barge1`. *(Tasks listed; detail finalized at execution start.)*
- **Phase C — Real-time orchestrator + backend interface.** `ble_link` (bidirectional, merges `audio_tx`/`audio_rx`), the async real-time loop, the `VoiceBackend` interface + an `EchoLoopbackBackend` (plays a canned WAV, needs no API), exposing `clean_mic_out`, `audio_sink_in`, `barge_in` events. HW-tested live.
- **Phase D — Hardening + the device contract doc.** Underrun/jitter behavior, the durable `[seq16][ts32][LC3]` contract documented for the phone app.

This document fully specifies **Phase A**. Phases B–D have task skeletons at the end; expand them at execution time once A is hardware-verified (the BLE/real-time code is iterated on hardware, per `CLAUDE.md`).

---

## Phase A — Live drift compensation (ts32 + ASRC)

### Task A1: Firmware — add `ts32` (cumulative mic-sample count) to the uplink frame

**Files:**
- Modify: `src/audio_stream.cpp` — count PCM samples fed; pass the count to the notify.
- Modify: `src/ble_audio.cpp` (`ble_audio_notify`) — pack `ts32` after `seq16`, before the LC3 payload.
- Modify: `src/ble_audio.h` — bump the documented wire format + any size constant.
- Verify: hardware (serial log + host parse) — no pytest (firmware is HW-in-the-loop per `CLAUDE.md`).

**Wire format change:** uplink notify goes from `[seq16 LE][LC3 80B]` (82 B) to `[seq16 LE][ts32 LE][LC3 80B]` (86 B, within MTU 247). `ts32` = cumulative count of 16 kHz mic samples captured since stream start (wraps at 2³²; ~74 h, irrelevant). Each uplink frame carries 2 LC3 frames = **320 samples**, so `ts32` increments by exactly 320 per uplink notify.

- [ ] **Step 1: Add a sample counter in the uplink producer.** In `src/audio_stream.cpp`, find where the capture block is enqueued / where `ble_audio_notify(encoded, enc_len)` is called (the audio thread consumer). Maintain `static uint32_t mic_sample_count;` incremented by the block's sample count (160 samples/LC3 frame × frames-per-block) each time a block is produced, reset to 0 in the session-start path. Pass the *pre-increment* value (the timestamp of the first sample in this notify) into the notify call.

- [ ] **Step 2: Thread `ts32` through `ble_audio_notify`.** Change `ble_audio_notify(const uint8_t *payload, uint16_t len)` to `ble_audio_notify(uint32_t ts32, const uint8_t *payload, uint16_t len)`. In the body, build the packet as `[seq16][ts32 LE 4B][payload]`:
  ```c
  pkt[0] = seq_num & 0xFF;
  pkt[1] = (seq_num >> 8) & 0xFF;
  pkt[2] = ts32 & 0xFF;
  pkt[3] = (ts32 >> 8) & 0xFF;
  pkt[4] = (ts32 >> 16) & 0xFF;
  pkt[5] = (ts32 >> 24) & 0xFF;
  memcpy(&pkt[6], payload, len);
  uint16_t pkt_len = 6 + len;  // BLE_AUDIO_SEQ_HDR_SIZE(2) + 4 + len
  ```
  Update `BLE_AUDIO_SEQ_HDR_SIZE` usage / the `pkt` buffer size in `ble_audio.cpp` to account for the extra 4 bytes, and update the comment in `ble_audio.h` documenting the wire format.

- [ ] **Step 3: Update the call site.** In `src/audio_stream.cpp`, pass the sample counter: `ble_audio_notify(ts32_for_this_block, encoded, enc_len);`.

- [ ] **Step 4: Build.** Run `./build.sh`. Expected: links clean, `zephyr.uf2` produced, FLASH/RAM within budget.

- [ ] **Step 5: Hardware verify.** Flash; run a `--duplex --record` capture (Task A4 will parse it). In the serial/host parse, confirm: (a) each uplink frame is 86 B, (b) `ts32` increases by exactly 320 per frame with no gaps (matching `seq16` continuity), (c) `Δts32 / Δhost_time ≈ 15881` (the device rate). No pytest — this is the HW gate.

- [ ] **Step 6: Commit.**
  ```bash
  git add src/audio_stream.cpp src/ble_audio.cpp src/ble_audio.h
  git commit -m "feat(ble_audio): add ts32 mic-sample-count timestamp to uplink frame"
  ```

### Task A2: Host parse — `voiceio.frame` decodes `[seq16][ts32][LC3]`

**Files:**
- Create: `tools/voiceio/__init__.py` (empty package marker)
- Create: `tools/voiceio/frame.py`
- Test: `tools/voiceio/tests/test_frame.py`

- [ ] **Step 1: Write the failing test.**
  ```python
  # tools/voiceio/tests/test_frame.py
  import struct
  from voiceio.frame import parse_uplink, UplinkFrame

  def test_parse_uplink_extracts_seq_ts32_and_lc3():
      lc3 = bytes(range(80))
      pkt = struct.pack("<HI", 1234, 320) + lc3   # seq16, ts32, then 80B
      f = parse_uplink(pkt)
      assert isinstance(f, UplinkFrame)
      assert f.seq == 1234
      assert f.ts32 == 320
      assert f.lc3 == lc3

  def test_parse_uplink_rejects_short_packet():
      assert parse_uplink(b"\x00\x00\x00") is None   # < 6-byte header
  ```

- [ ] **Step 2: Run it, expect failure.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_frame.py -v`. Expected: FAIL (`ModuleNotFoundError: voiceio.frame`).

- [ ] **Step 3: Implement.**
  ```python
  # tools/voiceio/frame.py
  """Uplink wire format: [seq16 LE][ts32 LE][LC3 payload]. ts32 = cumulative 16 kHz
  mic-sample count (increments 320 per frame = 2 LC3 frames)."""
  import struct
  from dataclasses import dataclass

  HDR = 6  # seq16 (2) + ts32 (4)

  @dataclass
  class UplinkFrame:
      seq: int
      ts32: int
      lc3: bytes

  def parse_uplink(pkt: bytes):
      if len(pkt) < HDR:
          return None
      seq, ts32 = struct.unpack_from("<HI", pkt, 0)
      return UplinkFrame(seq=seq, ts32=ts32, lc3=pkt[HDR:])
  ```

- [ ] **Step 4: Run it, expect pass.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_frame.py -v`. Expected: 2 passed.

- [ ] **Step 5: Commit.**
  ```bash
  git add tools/voiceio/__init__.py tools/voiceio/frame.py tools/voiceio/tests/test_frame.py
  git commit -m "feat(voiceio): uplink frame parser for [seq16][ts32][lc3]"
  ```

### Task A3: Host — `voiceio.drift` estimates the live device rate from `ts32`

**Files:**
- Create: `tools/voiceio/drift.py`
- Test: `tools/voiceio/tests/test_drift.py`

**Design (UPDATED 2026-06-22 after review caught a bug):** `DriftEstimator.update(ts32, host_time) -> rate_hz` estimates the device sample rate via a **windowed linear regression of `host_time` on the exact `ts32`** (slope = sec/sample → rate = 1/slope). A per-frame `Δts32/Δhost_time` ratio + EMA (the original draft) is BIASED (errors-in-variables attenuation + reciprocal-of-jittery-interval) and wobbles ±1.9% under ±8 ms BLE arrival jitter; regressing on the noise-free `ts32` axis is unbiased and holds <1 Hz. Window (default `window_s=12.0`) averages jitter yet tracks slow temperature drift. Wrap-safe on the 32-bit counter. Exposes `.rate_hz` and `.ratio` (`rate_hz/nominal_hz`, the ASRC ratio). Production timestamp method (patent US11477328): no hardcoded constant. **The implemented `tools/voiceio/drift.py` (regression form) is the source of truth — it supersedes the EMA code block below; tests include a `test_robust_to_arrival_jitter` regression guard.**

- [ ] **Step 1: Write the failing test.**
  ```python
  # tools/voiceio/tests/test_drift.py
  from voiceio.drift import DriftEstimator

  def test_estimates_true_device_rate_from_ts32_stream():
      # Device runs at 15881 Hz; host clock is true. Feed 320-sample frames.
      true_rate = 15881.0
      est = DriftEstimator(nominal_hz=16000.0, tau_s=2.0)
      ts = 0
      t = 0.0
      rate = None
      for _ in range(2000):                  # ~40 s of 20 ms frames
          ts += 320
          t += 320 / true_rate               # host time advances at the device rate
          rate = est.update(ts, t)
      assert abs(rate - true_rate) < 5.0     # converges within a few Hz
      assert abs(est.ratio - true_rate / 16000.0) < 5e-4

  def test_first_update_returns_nominal_no_divide_by_zero():
      est = DriftEstimator(nominal_hz=16000.0, tau_s=2.0)
      assert est.update(0, 0.0) == 16000.0   # no prior sample yet

  def test_handles_uint32_wrap():
      est = DriftEstimator(nominal_hz=16000.0, tau_s=2.0)
      est.update(2**32 - 320, 0.0)
      r = est.update(0, 320 / 16000.0)       # wrapped; delta should be +320
      assert abs(r - 16000.0) < 50.0
  ```

- [ ] **Step 2: Run it, expect failure.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_drift.py -v`. Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement.**
  ```python
  # tools/voiceio/drift.py
  """Live device-clock-rate estimate from ts32 timestamps (production timestamp method,
  patent US11477328). No hardcoded offset -- the rate emerges from Δts32/Δhost_time,
  smoothed with a time-constant EMA. Drives the reference resampler (voiceio.resample)."""
  import math

  class DriftEstimator:
      def __init__(self, nominal_hz=16000.0, tau_s=2.0):
          self.nominal_hz = nominal_hz
          self.tau_s = tau_s
          self.rate_hz = nominal_hz
          self._prev_ts = None
          self._prev_t = None

      def update(self, ts32: int, host_time: float) -> float:
          if self._prev_ts is None:
              self._prev_ts, self._prev_t = ts32, host_time
              return self.rate_hz
          dts = (ts32 - self._prev_ts) & 0xFFFFFFFF       # wrap-safe
          dt = host_time - self._prev_t
          self._prev_ts, self._prev_t = ts32, host_time
          if dt <= 0 or dts <= 0:
              return self.rate_hz
          inst = dts / dt
          alpha = 1.0 - math.exp(-dt / self.tau_s)        # EMA weight from dt + tau
          self.rate_hz += alpha * (inst - self.rate_hz)
          return self.rate_hz

      @property
      def ratio(self) -> float:
          return self.rate_hz / self.nominal_hz
  ```

- [ ] **Step 4: Run it, expect pass.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_drift.py -v`. Expected: 3 passed.

- [ ] **Step 5: Commit.**
  ```bash
  git add tools/voiceio/drift.py tools/voiceio/tests/test_drift.py
  git commit -m "feat(voiceio): live device-rate drift estimator from ts32"
  ```

### Task A4: `audio_tx.py --record` saves `ts32` alongside the recording

**Files:**
- Modify: `tools/audio_tx.py` — parse the 6-byte uplink header (was 2), capture `ts32` + host arrival time per frame, write a `<base>_ts.csv` sidecar (`frame_index,ts32,host_time`).
- Verify: hardware capture produces a monotonic `ts32` stream stepping by 320.

- [ ] **Step 1: Update `on_uplink` to parse the new header.** Where `audio_tx.py` currently reads `seq = data[0] | (data[1] << 8)` and slices `payload = data[2:]`, change to parse `ts32` too: `seq, ts32 = struct.unpack_from("<HI", data, 0)`, `payload = bytes(data[6:])`. Record `(seq, ts32, time.monotonic())` into a list when `--record` is on.

- [ ] **Step 2: Write the `<base>_ts.csv` sidecar at save time.** Alongside `<base>_ref.wav` / `<base>_mic.wav`, write `<base>_ts.csv` with a header line `frame_index,ts32,host_time` and one row per uplink frame.

- [ ] **Step 3: Hardware capture.** Reboot, run `python3 tools/audio_tx.py aecspeech16k.wav --lib tools/lib/liblc3.dylib --duplex --record drifttest` (use `j` to force-stream + speak so it triggers). Confirm `drifttest_ts.csv` exists, `ts32` is monotonic and steps by 320, and `N>0` frames. (This is the HW gate; no pytest.)

- [ ] **Step 4: Commit.**
  ```bash
  git add tools/audio_tx.py
  git commit -m "feat(audio_tx): parse + record ts32 sidecar from uplink frames"
  ```

### Task A5: `voiceio.resample` — drift-correct the reference with `soxr`, driven by the live ratio

**Files:**
- Create: `tools/voiceio/resample.py`
- Test: `tools/voiceio/tests/test_resample.py`
- Modify: install `soxr` into the venv (`tools/dtln-venv/bin/pip install soxr`)

**Design (UPDATED 2026-06-22):** `resample_ref(ref, ratio, n)` returns length `n` with `out[i]=ref[ratio·i]` (linear interp) — the *validated* offline drift-resampler semantics. NOTE: the original draft below used `soxr` with the resample direction **inverted** (it compressed; we must stretch since the device plays slow). At a ~0.75% rate change linear interp's error is negligible and it's exactly what produced the proven 33 dB margin. A polyphase `soxr` upgrade + a streaming variable-rate version are Phase-C quality levers, not needed for v1. **The implemented `tools/voiceio/resample.py` is the source of truth; the soxr code block below is superseded.**

- [ ] **Step 1: Write the failing test.**
  ```python
  # tools/voiceio/tests/test_resample.py
  import numpy as np
  from voiceio.resample import resample_ref

  def test_resample_changes_length_by_ratio():
      x = np.zeros(16000, dtype=np.float32)
      y = resample_ref(x, ratio=15881/16000.0)
      assert abs(len(y) - int(16000 * 15881/16000.0)) <= 2

  def test_resample_preserves_a_tone_frequency():
      sr = 16000
      t = np.arange(sr) / sr
      x = np.sin(2*np.pi*1000*t).astype(np.float32)
      y = resample_ref(x, ratio=0.9925)           # ~-0.75%
      # dominant bin still ~1000 Hz after resample (within a few Hz)
      f = np.fft.rfftfreq(len(y), 1/ (sr*0.9925))
      peak = f[np.argmax(np.abs(np.fft.rfft(y)))]
      assert abs(peak - 1000) < 10
  ```

- [ ] **Step 2: Run it, expect failure.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_resample.py -v`. Expected: FAIL.

- [ ] **Step 3: Implement.**
  ```python
  # tools/voiceio/resample.py
  """Resample the far-end AEC reference to the device (mic) clock. Polyphase ASRC via
  soxr -- the production resampler choice. ratio = device_rate / 16000 (from
  voiceio.drift). v1: one ratio per session; Phase C makes it streaming/variable."""
  import numpy as np
  import soxr

  SR = 16000

  def resample_ref(ref: np.ndarray, ratio: float) -> np.ndarray:
      # Stretch the reference onto the device timeline: device plays at SR*ratio, so a
      # reference of N samples occupies N/ratio device-samples; resample to SR*ratio
      # then index at SR keeps it 1:1 with the mic stream.
      out = soxr.resample(ref.astype(np.float32), SR, SR * ratio)
      return out.astype(np.float32)
  ```

- [ ] **Step 4: Run it, expect pass.** Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest voiceio/tests/test_resample.py -v`. Expected: 2 passed.

- [ ] **Step 5: Commit.**
  ```bash
  git add tools/voiceio/resample.py tools/voiceio/tests/test_resample.py
  git commit -m "feat(voiceio): soxr reference resampler driven by drift ratio"
  ```

### Task A6: Integration — `barge_test`/`dtln_test` use the LIVE ts32 drift, not the hardcoded −0.75%

**Files:**
- Modify: `tools/dtln_test.py` and `tools/barge_test.py` — when a `<base>_ts.csv` exists, build the reference resample ratio from `DriftEstimator` over the recorded `ts32` stream (via `voiceio.drift` + `voiceio.resample`) instead of the `DRIFTS`/`-0.0075` constant.
- Verify: hardware — the live-estimated ratio reproduces ≈ −0.75% AND the echo stays removed start-to-end (margin ≈ 33 dB across the whole clip, including the end that previously leaked).

- [ ] **Step 1: Wire the live drift into the cleaner.** In `tools/barge_test.py`'s `dtln_clean`, if `<base>_ts.csv` is present: feed its `(ts32, host_time)` rows through `DriftEstimator`, take the converged `.ratio`, and build the reference via `voiceio.resample.resample_ref(ref, ratio)` instead of the hand-rolled `np.interp(..., 1+drift, ...)`. Keep the constant path as a fallback when no sidecar exists.

- [ ] **Step 2: Re-run the barge de-risk on the new recording.** Run: `tools/dtln-venv/bin/python tools/barge_test.py drifttest 15`. Expected: the report prints the live-estimated ratio ≈ 0.9925 (−0.75%), the AEC'd margin ≈ 33 dB, **and an added early-vs-late echo-floor check shows both ≈ −70 dB** (no end-of-clip leak — the failure the user heard with a fixed/wrong constant).

- [ ] **Step 3: Add the early-vs-late hold assertion to the report.** Extend `barge_test.py` to print echo-removal in the first 12 s vs last 12 s of the silent/AI-only regions; both should be within a few dB. This is the regression guard for drift tracking.

- [ ] **Step 4: Commit.**
  ```bash
  git add tools/dtln_test.py tools/barge_test.py
  git commit -m "feat(voiceio): drive AEC reference resample from live ts32 drift (no hardcoded offset)"
  ```

**Phase A done = the hardcoded −0.75% is gone; the reference is drift-corrected from live `ts32`, verified on hardware to hold the echo removed end-to-end across units/temperature.**

---

## Phase B–D — task skeletons (expand at execution time, after A is HW-verified)

> These touch BLE + real-time asyncio + the TF-Lite model — iterated on hardware per `CLAUDE.md`. Pure helpers get pytest TDD; integration is verified on recordings + live. Full code is written at execution start so it matches the real module APIs Phase A establishes.

### Phase B — `voiceio` package: pure modules + offline pipeline
- **B1 `voiceio.codec`** — LC3 encode/decode (port the ctypes wrappers from `audio_tx.py`/`audio_rx.py`) + 16k↔24k resample (soxr). TDD: encode→decode round-trip RMS error bound; resample length/tone tests.
- **B2 `voiceio.aec`** — streaming DTLN wrapper around the TF-Lite two-stage model (lift the loop from `tools/DTLN-aec/run_aec.py` into a `DtlnAec.process(mic_block, ref_block) -> clean_block` with persistent LSTM state). TDD: on a bundled DTLN sample, far-end-single-talk ERLE > 30 dB (proves the wrapper matches the reference impl).
- **B3 `voiceio.vad`** — energy-gated barge-in detector (lift from `barge_test.py`): learns the echo floor online, fires on `speech AND rms > floor+MARGIN_DB` with onset debounce. TDD: synthetic floor + a loud burst → exactly one onset; pure echo floor → zero onsets.
- **B4 `voiceio.pipeline`** — `run_session(ref, mic, ts)` ties codec+drift+resample+aec+vad over a recorded session → `(clean_mic, barge_events)`. Verify on `barge1`/`drifttest`: margin ≈ 33 dB, barge events ≈ the spoken count, zero echo false-fires.

### Phase C — real-time orchestrator + backend interface
- **C1 `voiceio.ble_link`** — bidirectional bleak client merging `audio_tx` (downlink + clock-recovery) and `audio_rx` (uplink): exposes `async frames()` yielding `UplinkFrame`, `send_downlink(lc3)`, `flush()`, inbound status. HW-tested live.
- **C2 `VoiceBackend` interface + `EchoLoopbackBackend`** — `feed_mic(clean_pcm)`, `audio_out()` async iterator, `on_barge_in()`. The loopback backend plays a canned WAV (needs no API) so the full loop is testable end-to-end without Gemini.
- **C3 `voiceio.orchestrator`** — the async real-time loop: BLE up → codec → drift+resample → DTLN → VAD → backend; backend audio → codec → clock-recovery → BLE down; barge-in → backend FLUSH/stop. Exposes the core API (`clean_mic_out`, `audio_sink_in`, `barge_in` events). HW-tested live with the loopback backend.

### Phase D — hardening + device contract doc
- **D1** Jitter/underrun behavior under the real-time load (reuse Phase-A HR-defer findings).
- **D2** Document the durable device BLE contract (`[seq16][ts32][LC3]`, status, downlink, FLUSH) in `docs/` for the future phone app.

---

## Self-Review

**Spec coverage:** AEC=DTLN (B2 ✓), ts32-drift ASRC (A1–A6 ✓), energy-gated barge-in (B3 ✓), backend-agnostic interface + non-Gemini test backend (C2 ✓), clean-mic-out/audio-sink-in/barge-in core API (C3 ✓), codec + clock-recovery (B1/C ✓), conversation gating reuses existing pose/voice gate (no new task needed — unchanged firmware). No spec requirement left unmapped.

**Placeholder scan:** Phase A has complete code + exact pytest/HW commands per step. Phases B–D are explicitly marked as skeletons to expand at execution start (the BLE/real-time code must match Phase-A module APIs and is HW-iterated) — this is a deliberate, declared boundary, not a hidden TODO.

**Type consistency:** `UplinkFrame(seq, ts32, lc3)` (A2) is consumed by A4/A6/C1; `DriftEstimator.update()->rate_hz` + `.ratio` (A3) feeds `resample_ref(ref, ratio)` (A5) used in A6/B4. Names consistent across tasks.
