# Audio / Storage / BLE Architecture Review (2026 best-practice check)

**Date:** 2026-06-20
**Method:** deep-research harness — 5 search angles, 22 sources fetched, 80 claims
extracted, adversarially verified (3-vote), 7 confirmed findings.
**Question:** Is our embedded audio + storage + BLE stack on the XIAO nRF52840
(battery voice-assistant wristband, Zephyr/NCS, phone bridges to cloud AI) the
right 2026 approach?

## Verdict

**The stack is well-chosen for what the hardware actually is.** Two big
architectural choices are validated; three things are worth acting on; one
research claim was wrong (and in our favour — see the version note).

---

## Validated as correct 2026 practice

1. **Custom GATT-NOTIFY + software-LC3 is the RIGHT transport for the nRF52840.**
   The nRF52840 **cannot** do true Bluetooth LE Audio (LC3 over CIS/BIS
   isochronous channels) through the Nordic/Zephyr stack — that requires the
   dual-core **nRF5340** (network core runs the LE Audio controller). On a
   single-core nRF52840, custom-GATT + software-LC3 is the established, pragmatic
   pattern (GATT notifications = low overhead, async — ideal for streaming
   audio; LC3 is a software codec that runs on a general core even on the 5340).
   **Decision (2026-06-20): we have NO plans for standard LE-Audio interop**
   (off-the-shelf earbuds / native phone LE Audio), so this is FINAL — no
   nRF5340 migration. The phone-bridges-to-AI-API model is the product.
   *Sources:* Nordic DevZone "does nRF52840 support LE Audio"; nRF5340 audio
   firmware-architecture docs; TI BLE5 voice-over-GATT guide.

2. **The producer/consumer I2S design IS the standard pattern.** Ring buffer →
   dedicated feeder thread → fixed-size DMA blocks (memory slab) → prebuffer +
   backpressure. This maps directly onto Zephyr's native I2S primitives, and the
   "cracking from just-in-time feeding" is the documented underrun failure mode
   (TX underrun → `I2S_STATE_ERROR`). What we built (`audio_out`) is best
   practice. *Sources:* Zephyr I2S docs/API; i2s_nrfx driver.

---

## Act on these (ranked by impact)

1. **[RESOLVED for us] The i2s_nrfx write-vs-ISR race (Zephyr PR #63790).** Real
   historical bug — an unsynchronized `next_tx_buffer_needed` flag could make
   later audio transmit before earlier in exactly our TX feeder pattern. Fixed
   upstream in **Zephyr v3.5.0**. **CORRECTION to the research:** it assumed our
   tree was "Zephyr ~3.2.x" (from the "NCS 3.2.3" label), but `zephyr/VERSION`
   and the boot banner show **Zephyr v4.2.99** — so the fix is ALREADY present.
   Not our bug. (Lesson: NCS x.y.z != Zephyr x.y.z; NCS 3.x ships Zephyr 4.x.)

2. **Recover from I2S underrun, don't just end the session.** Underrun is a
   recoverable fault: `I2S_STATE_ERROR` → `I2S_TRIGGER_PREPARE` → resume. Our
   `audio_out` currently ENDS the session on a write error. For the real BLE
   downlink, add the PREPARE recovery path so a transient underrun self-heals.
   *(Low urgency — with prebuffer+backpressure we measured 0 mid-stream
   underruns and 0 write-errors; this is belt-and-suspenders.)*

3. **SD-over-SPI + FatFs is the throughput-weakest, most dated link — and SD's
   PRIMARY role is WRITING (recording).** Measured Zephyr SPI+FatFs ~27 KB/s
   write, bottlenecked by single-block writes (CMD24) + per-call `fs_sync`. Fine
   for our read-only test; for **continuous audio recording** use batched/
   multi-block writes, a higher SPI clock, or a pre-allocated contiguous append
   file. Address when we build the recorder sub-project. *Source:* jblopen FAT
   SD performance benchmark; Zephyr #22906.

---

## Inconclusive

- **MAX98357A amp:** the research surfaced no strong 2026-specific guidance
  for/against it (amp datasheets didn't yield verified claims). It's a standard,
  fine choice for voice playback; no reason to change.

## How this validated the live debugging (2026-06-20)

The SD-WAV playback cracked in the real firmware but not in the standalone
`spkr_test`. Root-caused (with per-session counters) to **ring underrun → silence
injection**: 131/648 blocks underran because the producer fed just-in-time and
the 0.37 s ring sat near-empty. Fix = keep the ring FULL (prebuffer at start +
backpressure in the producer). Result: **0 mid-stream underruns** (the music is
clean; the residual ~5 are benign end-drain tail silence). The research's I2S
pattern + underrun findings independently corroborated both the mechanism and
the fix.

## PRODUCTION POWER REQUIREMENT (decided 2026-06-20) — `[HOUSING]`

The MAX98357A speaker amp **must have an appropriate power supply in production.**
On the bare XIAO dev board the amp shares the **3V3 LDO rail** with the PPG LEDs,
BLE radio, and sensors; at high volume the amp's peak current sags that rail and
the audio crackles (confirmed: crackle scales directly with volume — clean ≤½,
onset at ¾, worse at full; `audio_out` itself is clean, 0 underruns). This is the
same root cause as the earlier "volume fluctuation."

**Production decision:** power the amp from a **proper rail — a boost converter
(e.g. 3.7V LiPo → 5V) or a dedicated/stouter supply — with local decoupling**, so
it can run **100% volume** cleanly. The production PCB should also add the `[HOUSING]`
ESD/short protection noted elsewhere.

**Dev decision (for now):** we **skip the bulk cap** and run the dev board at
**75% volume** (firmware default in `audio_out.cpp`, `volume_pct = 75`), which is
clean enough for development. Adjust live with the `'0'-'9'` serial keys
(0–90%); `audio_out_set_volume(0-100)` is the firmware lever (set to 100 in
production once the amp has its proper rail).

## Key sources

- https://devzone.nordicsemi.com/f/nordic-q-a/77227/does-or-will-the-nrf52840-support-le-audio
- https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/applications/nrf5340_audio/doc/firmware_architecture.html
- https://docs.zephyrproject.org/latest/hardware/peripherals/audio/i2s.html
- https://github.com/zephyrproject-rtos/zephyr/issues/63730  (i2s_nrfx race; fixed v3.5.0 / PR #63790)
- https://www.jblopen.com/fat-sd-card-performance/  (SPI+FatFs throughput)
- https://software-dl.ti.com/.../ble_voice.html  (GATT-notify voice transport pattern)
