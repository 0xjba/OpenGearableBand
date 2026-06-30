# Gestureband — Device BLE Contract

The durable Bluetooth-LE interface the band exposes. **This is the seam the phone app
(and the Mac `tools/voiceio` reference client) implements against.** It is intentionally
host/language-agnostic: anything that speaks these GATT characteristics and respects the
wire formats below can drive the band — no firmware change required to add a new client.

Source of truth in firmware: `src/ble_audio.h` (custom audio service), `src/lc3_codec.h`
(codec framing), `src/main.cpp` (advertising + HRS/BAS). Reference host implementation:
`tools/voiceio/ble_link.py`, `tools/voiceio/frame.py`, `tools/voiceio/clock_recovery.py`.

---

## 1. Advertising & standard services

- **Device name:** `Xiao_Pulse` (advertised local name).
- **One connection at a time.** The firmware stops advertising while connected and
  re-advertises on disconnect (self-heals supervision-timeout drops).
- **Standard GATT services** (use off-the-shelf phone APIs):
  - **HRS** — Heart Rate Service (0x180D): heart-rate measurement notifications.
  - **BAS** — Battery Service (0x180F): battery level.

The rest of this doc is the **custom audio service**, which carries the full-duplex voice path.

---

## 2. Custom audio GATT service

128-bit UUID base: `47A1xxxx-9B70-4C2E-8A1D-2F6B9E4A77C1`.

| Char | UUID (short) | Properties | Direction | Purpose |
|---|---|---|---|---|
| Service | `47A10001` | — | — | Container |
| Uplink audio | `47A10002` | NOTIFY (+CCC) | device → host | Mic LC3 stream (dictation / voice turn) |
| Downlink audio | `47A10003` | WRITE / WRITE_NR | host → device | AI/playback LC3 stream |
| Downlink control | `47A10004` | WRITE | host → device | 1-byte command (barge-in FLUSH) |
| Downlink status | `47A10005` | NOTIFY (+CCC) | device → host | Playout buffer feedback (clock recovery) |

To start a voice session a client subscribes (writes the CCC) to **uplink** (`02`) and
**status** (`05`); the band streams uplink only while the mic gate is open (see §7).

---

## 3. Codec framing (LC3)

All audio on this service is **LC3, 16 kHz mono, 10 ms frames, 32 kbps → 40 bytes/frame**
(`LC3_FRAME_BYTES = 40`). The firmware packs **2 frames per packet = 80-byte block = 20 ms**
(`LC3_FRAMES_PER_PDM_BUF = 2`). One decoded frame = 160 PCM samples; one block = 320 samples.

A client needs an LC3 codec (e.g. Google `liblc3`) configured for 16 kHz / 10 ms / 40 B.

---

## 4. Uplink audio (`47A10002`, device → host, NOTIFY)

Each notification:

```
[ seq16 LE ][ ts32 LE ][ LC3 payload ]
   2 bytes    4 bytes     N bytes        (B firmware: N = 80 → 86-byte notification)
```

- **`seq16`** — little-endian frame counter; lets the receiver detect dropped/reordered
  notifications. Increments only on a successful send.
- **`ts32`** — little-endian **cumulative 16 kHz mic-sample count** of this frame's first
  sample (increments **320 per notification** = 2 LC3 frames). Cumulative from boot; wraps
  at 2³² (~74 h). Use it to (a) detect gaps and (b) estimate the device mic clock rate live.
  *(See §8 for how the AEC reference uses the clocks.)*
- **Payload** — the LC3 block (decode each 40-byte frame in order).

Header size = `BLE_AUDIO_HDR_SIZE = 6`. Parse: `seq, ts32 = struct.unpack_from("<HI", pkt, 0)`,
payload = `pkt[6:]`. (Reference: `tools/voiceio/frame.py`.)

**Backpressure:** uplink notifies are **non-blocking on the device** — if the BLE TX queue
is full the frame is **dropped** (a drop counter is bumped), never blocking the mic thread.
So the host must tolerate gaps (`seq`/`ts32` discontinuities), not assume a lossless stream.

---

## 5. Downlink audio (`47A10003`, host → device, WRITE / WRITE_NR)

Each write:

```
[ seq16 LE ][ LC3 payload ]
   2 bytes      N bytes      (host reference sends N = 80 → 82-byte packet)
```

- **`seq16`** — host-assigned LE counter (device-side ordering/diagnostics).
- **Payload** — LC3 block(s); the device decodes every 40-byte frame in the packet
  (up to `DL_MAX_PAYLOAD = 240` B = 6 frames).
- Use **Write-Without-Response** for the steady stream (throughput); the reference client
  does a small escalating **Write-With-Response** probe at connect to confirm the path /
  let DLE settle before streaming.

**Pacing is the host's job.** The device plays at its own rate; the host must pace sends to
real time and use the status feedback (§6) to hold the device's jitter buffer at a setpoint
— do **not** dump a whole reply at once (it overflows the device ring). See §6/§8.

---

## 6. Downlink status (`47A10005`, device → host, NOTIFY) + clock recovery

Each notification is `BLE_AUDIO_STATUS_LEN = 5` bytes:

```
[ ring_used u16 LE ][ ring_capacity u16 LE ][ flags u8 ]
```

- **`ring_used` / `ring_capacity`** — device playout buffer fill / size, in bytes
  (16 kHz mono 16-bit). The host holds `ring_used` near a setpoint (reference: **140 ms**
  = 4480 B) with a slow PI loop that nudges the **send pace** (and, in the AEC client, the
  reference-resample ratio). This absorbs the fixed device/host clock-rate difference and
  jitter. (Reference: `step()` in `voiceio/clock_recovery.py`.)
- **`flags`** byte bits:
  - `0x01` `ACTIVE` — a playback session is active on the device.
  - `0x02` `OVERFLOW` — the ring overflowed since the last report (host sending too fast).
  - `0x04` `UNDERRUN` — the ring underran since the last report (host too slow / stalled).

The host should react to OVERFLOW/UNDERRUN with an integral kick (bias slower/faster).

---

## 7. Downlink control (`47A10004`, host → device, WRITE) — barge-in

1-byte command:

- **`0x01` `FLUSH`** — barge-in: the device **stops playback and clears its buffered audio
  immediately.** The host sends this when the user talks over the AI, and must **also stop
  sending** downlink audio (otherwise the next frame auto-restarts playback). After a flush
  the host typically resets its clock-recovery integral.

---

## 8. Clock model (for the echo canceller)

The speaker (I2S DAC) and the mic (PDM) are **both clocked from the band's single on-chip
HFCLK**, so they are **synchronous**: their sample-rate ratio is a **fixed constant**,
temperature- and unit-invariant.

- Speaker LRCK = 32 MHz / 21 / 96 = **15873 Hz** (integer-divider limited; can't hit 16000).
- Mic PCM rate = **16000 Hz**.
- AEC reference drift ratio = `15873 / 16000 = 0.99206` (−0.794 %), applied by the host to
  put the far-end reference on the mic clock. **Fixed — not estimated.** (`voiceio/clocks.py`.)

The render→capture **loop delay** (jitter buffer + BLE scheduling + pipeline), by contrast,
is **not** a constant: the host estimates it online by cross-correlating the played
reference against the mic (`voiceio/delay_estimator.py`). `ts32` + the status buffer level
feed that estimate; nothing acoustic is hardcoded.

---

## 9. Connection interval

`ble_audio_set_fast_conn(fast)` requests:
- **fast** (~15–30 ms) to sustain the ~50 notif/s audio rate during a voice burst,
- **relaxed** (~30–50 ms) to save power when idle.

The central may honor, modify, or ignore the request (e.g. iOS clamps). The host should not
assume a specific interval; the negotiated value is logged device-side via `le_param_updated`.

---

## 10. Session lifecycle (host responsibility)

The band streams uplink only while its **mic gate** is open (raise-to-ear `POSE_EAR` + voice
→ `MODE_DICTATION`). The **host** infers conversation start/end from **uplink presence**:

- **Uplink frames begin** → user started a conversation → open the AI session, (re)acquire
  the echo-delay lock (a brief calibration tone is the reference client's trick), converse.
- **Uplink silent for ~2.5 s** (arm lowered, mic gate closed) → end the conversation → close
  the AI session, `FLUSH` the downlink, reset.

No dedicated "session" characteristic is needed — uplink presence is the signal.
(Reference: `Orchestrator._session_loop` in `voiceio/orchestrator.py`.)

---

## 11. Minimal phone-app checklist

1. Scan/connect to `Xiao_Pulse` (one connection; handle re-advertise on drop).
2. Subscribe HRS + BAS for heart-rate + battery (standard APIs).
3. LC3 codec @ 16 kHz / 10 ms / 40 B (e.g. liblc3, or Android LE-Audio LC3).
4. Subscribe uplink (`02`) + status (`05`); parse the 6-byte uplink header (§4).
5. Downlink: pace sends, prepend `seq16` (§5), run the status-driven clock-recovery loop (§6).
6. Barge-in: `FLUSH` (`0x01`) on `04` **and** stop sending (§7).
7. Echo cancellation: fixed `15873/16000` reference resample (§8) + online delay estimate.
8. Drive session start/end off uplink presence (§10).
9. Tolerate uplink gaps (drops are expected, §4).
