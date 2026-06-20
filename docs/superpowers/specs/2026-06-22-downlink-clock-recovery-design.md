# Downlink Clock Recovery (adaptive playout) — Design

**Date:** 2026-06-22
**Branch:** beta
**Status:** Approved (design), pending implementation plan
**Builds on:** the BLE audio downlink (`audio_downlink` / `ble_audio` / `audio_out`,
committed `397bc49`).

## Problem

The downlink sink (nRF52840 I2S) and the source (Mac/phone host) run on
**independent clocks** that differ by <1 %. On a continuous (gapless) stream the
`audio_out` ring slowly fills or drains until it **overflows (drops audio) or
underruns (silence-pads)** — both audible as crackle. Measured on HW (2026-06-21):
a gapless 60 s clip produced **39 ring-overflow drops**; the effective I2S rate
read ~15.9 kHz vs the host's exact 16 kHz. Real speech has inter-phrase gaps that
reset the buffer (the device silence-auto-stops), so this mainly bites long,
gapless audio — but it must be solved for smooth playback in general.

## Why this approach (validated against established practice)

This is the textbook **asynchronous-sink** problem. The canonical solution is the
**USB Audio Class asynchronous explicit-feedback** model: the sink reports its
desired data rate / buffer state, and the **source adjusts how fast it sends** so
the buffer neither overflows nor underflows. The same shape appears in VoIP/IPTV
adaptive playout and WebRTC. Two consequences, both research-backed:

- **Host-side recovery, device just reports** — matches "bare minimum on device,
  heavy lifting on the phone." The device adds only a status report; the host runs
  the control loop. (We have only first-party companion apps as real senders, so
  requiring host logic is acceptable.)
- **Push, not pull** — async-feedback systems *continuously push* the sink's state
  so the controller has a steady signal; polling is not how these loops are built.
- **Correct by sub-audible resampling nudges**, not chunk drop/insert — drift
  compensation should be "split into many small steps" so phase changes stay
  inaudible. This is the host's job (it already resamples 24 k→16 k from the AI).
- **Buffer-fill-level control, not explicit rate feedback** — the device reports
  only its buffer fullness (it stays dumb); the host's PI integral converges to the
  drift on its own. (USB's explicit-rate variant converges faster but needs the
  device to measure its own clock — more on-device than warranted here. Buffer-level
  PI for audio clock recovery is well-established and its correction is sub-audible.)

References: USB Audio 2.0 async feedback (usb.org); Zephyr `uac2_explicit_feedback`
sample; adaptive playout buffer literature (Springer); drift-correction methods
(resample / frame drop / insert).

## Architecture

```
[device]  audio_out ring ──fill level──► status NOTIFY (47A10005, ~100 ms)
                                                  │
[host]   adaptive-playout controller ◄────────────┘
              │ nudges resample ratio (real app) / send pace (test sender)
              ▼
         downlink audio writes (47A10003) ──► ring held near setpoint → no over/underflow
```

While a downlink session is active the device notifies its ring fill every ~100 ms.
The host keeps the fill at a setpoint via small continuous rate corrections. The
device performs no control math and no resampling.

## Components

### 1. Device: downlink status characteristic (the only on-device change)
- **New characteristic** `47A10005-9B70-4C2E-8A1D-2F6B9E4A77C1` "downlink status",
  **NOTIFY** + CCC, appended to the existing `ble_audio` GATT service (uplink notify
  + downlink audio/control chars unchanged).
- **Payload — 5 bytes, little-endian:**
  `[used_bytes u16][capacity_bytes u16][flags u8]`
  - `used_bytes` = `audio_out_ring_used()` at sample time.
  - `capacity_bytes` = ring capacity (new `audio_out_ring_capacity()` returning
    `RING_BYTES`). Sent so the host needs no hard-coded constant and survives a
    future ring-size retune.
  - `flags`: bit0 = session active (`audio_out_active()`); bit1 = overflowed since
    last report; bit2 = underran since last report. Bits 1–2 are **sticky,
    read-and-clear** — a hard event signal so the host reacts immediately, not only
    by inferring from `used` hitting the rails.
- **Reporter — lives in `audio_downlink`, not `ble_audio`** (keeps the new
  `audio_out` dependency in the module that already bridges `audio_out`↔`ble_audio`;
  `ble_audio` stays GATT-only). `audio_downlink` already owns session start/stop
  (it calls `audio_out_start` on the first frame), so it drives a periodic
  `k_work_delayable` re-armed every `DL_STATUS_PERIOD_MS = 100` `[STRUCTURAL]` **only
  while a session is active** — no work, no notifies while idle (saves power; the
  host doesn't need feedback when nothing is playing). Each tick: if
  `ble_audio_status_subscribed()`, read used/capacity/flags from `audio_out` and
  call `ble_audio_notify_status()`.
- **`ble_audio` additions:** the status char + CCC; `bool ble_audio_status_subscribed(void)`
  (set by the status CCC-changed cb); `int ble_audio_notify_status(const uint8_t *p, uint16_t len)`.
- **Supporting `audio_out` additions (small, public):**
  - `size_t audio_out_ring_capacity(void);` → `RING_BYTES`.
  - `uint8_t audio_out_take_event_flags(void);` → returns bits {overflow, underrun}
    accumulated since the last call and clears them (feeds `flags` bits 1–2). The
    overflow path (`audio_out_write`) and the partial/empty-underrun path (feeder)
    set these latches.

### 2. Host: adaptive-playout controller (contract + reference)
**Contract (normative for the real Mac/phone companion apps):**
- Subscribe to the status NOTIFY (`47A10005`).
- Maintain a **setpoint** = a target buffered duration of **~120 ms** `[STRUCTURAL]`
  (latency vs jitter-absorption; under the ~150 ms ceiling for interactive audio).
  `setpoint_bytes = setpoint_ms × rate_hz × 2 / 1000` (the host knows the 16 kHz
  rate and 2 B/sample); the reported `capacity_bytes` is used for the fill *fraction*
  and to bound the setpoint below capacity.
- Run a **low-bandwidth (slow) proportional-integral loop** on
  `error = used_bytes − setpoint_bytes`. Output = a small multiplicative correction
  to the **24 k→16 k resample ratio**. The loop MUST be slow — the drift is small
  and near-constant, and an aggressive loop *introduces* buffer oscillation / audible
  modulation (the real risk, more than per-step audibility).
- **Correction authority = ±1.5 %** `[STRUCTURAL]`. This must *exceed worst-case
  drift*: the nRF I2S cannot hit exactly 16 kHz (nearest achievable ≈ 15873 Hz ⇒
  ~0.8 % low), so a ±0.5 % clamp could not cancel it. ±1.5 % gives ~2× margin and,
  as a *continuous* resample ratio, stays sub-audible (a steady fraction-of-a-percent
  offset, not a step). The PI integral converges to this drift offset.
- On `flags` overflow bit → step toward *sending slower*; underrun bit → *faster*
  (fast recovery from a rail event).
- **Reset the controller** (zero the integral, recentre) whenever the buffer is
  discontinuous — on a barge-in `FLUSH` or a session restart — since the prior loop
  state is stale after the ring is cleared.

**Reference implementation — `tools/audio_tx.py` (adaptive by default):**
- The device only ever streams to our first-party companion app, which *always*
  runs the loop — so the test sender does too. Adaptive playout is the **default
  behaviour**: it subscribes to `47A10005`, parses the 5-byte payload, runs the PI
  loop, and logs fill % and applied correction.
- The test sender has no resampler (it streams a fixed-rate 16 k WAV), so it
  actuates the *same control law* by **nudging its inter-block send pacing**
  (the `BLOCK_MS` sleep) within ≤ ±1.5 % (matching the contract's correction
  authority). This proves the device feedback loop end-to-end on hardware and is the
  documented blueprint the real apps mirror with a resample-ratio actuator instead.
- A **`--no-adaptive`** flag disables the loop (don't subscribe, fixed pacing) — its
  *only* purpose is to reproduce the drift for the before/after acceptance baseline.

### 3. Integration
`ble_audio` registers the status char (auto via `BT_GATT_SERVICE_DEFINE`) and tracks
its subscription state, exposing `ble_audio_status_subscribed()` +
`ble_audio_notify_status()`. `audio_downlink` owns the reporter work and reads the
new `audio_out` getters. The decoder, gesture, and HR code are unchanged. No
`prj.conf` change expected.

### 4. Reconcile the startup prebuffer with the setpoint
`audio_out` currently prebuffers to `RING_BYTES/2` (≈256 ms) before playback — far
above the ~120 ms control setpoint, so today every session would start high and the
loop would just drain it. Lower the **downlink** prebuffer target to ≈ the setpoint
(~120 ms) so initial fill and steady-state agree and startup latency matches the
target. This is the same `[STRUCTURAL]` prebuffer item the original downlink spec
flagged as SD-tuned; expose it as a parameter (e.g. `audio_out_start(rate, prebuf_ms)`)
or a downlink-specific value rather than the shared `PREBUF_BYTES`. Keep a small
floor so a slow first burst can't underrun the opening.

## Data flow
- **Session active:** every 100 ms → device notifies `[used, capacity, flags]` →
  host computes `error` → nudges rate → ring converges to the setpoint → no
  over/underflow.
- **Rail event:** overflow/underrun latch set → reported next tick → host steps the
  correction hard in the right direction → ring recovers.

## Error handling
- **The companion app (and the test sender) always run the loop** — there is no
  supported "dumb sender." So smooth playback is the normal path, not an opt-in.
- The device's existing **overflow-drop / underrun-silence-pad is the inherent
  buffer-rail behaviour**, not a designed fallback mode: it's simply what the ring
  does at its limits. With the loop active the buffer never reaches those rails; if
  a sender somehow doesn't subscribe (only `--no-adaptive`, used for the baseline),
  it drifts into them exactly as today. Nothing on the device special-cases this.
- The reporter only notifies when the status CCC is subscribed (mirrors the uplink
  notify guard), so a not-yet-subscribed or idle host costs nothing.
- A lost status notification is self-correcting: the next tick carries the current
  absolute `used_bytes`, so the loop needs no per-packet reliability.

## Testing
- **Host unit-testable:** the PI control law in `audio_tx.py` is a pure function of
  (used, capacity, setpoint) → correction; exercise it with synthetic fill
  trajectories (converges, clamps at ±1.5 %, cancels a 0.8 % drift without
  overflow, reacts to flag bits, resets on flush).
- **HW acceptance (the headline test):** the **60 s gapless clip** via
  `audio_tx.py` (adaptive by default) plays with **~0 overflow and ~0
  partial/audible underrun** (vs 39 overflow today), confirmed by the
  `audio_out session:` stats line. The same clip run with **`--no-adaptive`**
  reproduces the drift as the before/after control.
- Confirm the status notify does not perturb the audio path (still `0 write-err`,
  mic+speaker coexistence intact).

## Tunables (housing/production retune)
- `DL_STATUS_PERIOD_MS` (100) — feedback cadence. `[STRUCTURAL]`.
- Host setpoint (~120 ms buffered) — latency vs jitter. `[STRUCTURAL]`.
- Downlink prebuffer (~120 ms, ≈ setpoint) — startup latency vs opening underrun.
  `[STRUCTURAL]`.
- Host PI gains (low-bandwidth) + the ±1.5 % correction clamp — convergence vs
  oscillation/audibility; the clamp must exceed worst-case I2S drift (~0.8 %).
  `[STRUCTURAL]`, tuned on hardware.

## Scope
- **In:** the device status NOTIFY char + the `audio_downlink` reporter, the two
  `audio_out` getters, the downlink-prebuffer retune, the reference adaptive loop in
  `audio_tx.py`, and the host-side contract (this doc).
- **Out:** the production Mac/phone companion apps (separate projects; they
  implement the contract above with a resample-ratio actuator). No change to the
  uplink, gesture, or HR paths.
