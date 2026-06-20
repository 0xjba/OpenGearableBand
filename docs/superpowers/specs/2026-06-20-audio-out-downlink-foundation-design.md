# Audio-Out (Speaker Downlink Foundation) — Design

**Date:** 2026-06-20
**Branch:** beta
**Status:** Approved (design), pending implementation plan

## Goal

Fold the speaker (I2S → MAX98357A) into the **real firmware** as a reusable
audio-output engine. This is **step A** of the assistant downlink: the component
that actually turns PCM into sound. The BLE downlink that feeds it AI-response
audio (**step B**) lands on top later via one API call.

## Context

The device is a two-way voice assistant:
- **Uplink (built):** mic → `mic_vad` (PCM) → `audio_stream` (LC3) → `ble_audio`
  (BLE notify) → phone → AI API (Gemini Live / translate / …).
- **Downlink (this + B):** AI audio → phone → BLE → **decode → `audio_out` →
  I2S → MAX98357A → speaker.**

The speaker hardware is fully validated in the throwaway `spkr_test/` app (stock
`i2s_nrfx` driver, original pins, intelligible playback). This spec moves that
capability into the real firmware as a clean module.

`audio_out` mirrors the established uplink module boundaries (`mic_vad` /
`audio_stream` / `ble_audio`): one module, one clear responsibility, a small
well-defined API.

## Scope

**In scope (A):**
- `src/audio_out.{h,cpp}` — I2S output engine + feeder thread + amp power control.
- I2S/amp pins added to `app.overlay`; `CONFIG_I2S=y` in `prj.conf`.
- A serial `'p'` test command in `main.cpp` that plays a generated tone through
  `audio_out`, to verify the speaker works inside the full firmware.

**Out of scope (B, later):** the BLE downlink receive characteristic, LC3 decode,
and the phone-side app. B will simply call `audio_out_write()`.

## Architecture

```
[B, later]  BLE recv char -> LC3 decode --\
                                           +--> audio_out_write(pcm) --> [k_msgq]
[now]       'p' test command --> tone -----/                               |
                                                          audio_out feeder thread
                                                          (mono->stereo, i2s_write)
                                                                           |
                                                                  I2S0 -> MAX98357A -> speaker
```

`audio_out` is a self-contained module: a producer API (`audio_out_write`) feeds
a `k_msgq`; a dedicated consumer thread drains it to I2S. Producers (the test
hook now, BLE decode later) never touch I2S directly.

## Module API (`src/audio_out.h`)

```c
/* One-time boot init: configure I2S TX (stock i2s_nrfx), set amp-SD GPIO muted,
 * spawn the (idle) feeder thread. Returns 0 / negative errno. */
int audio_out_init(void);

/* Begin a playback session at the given sample rate (Hz). Enables the amp,
 * (re)configures I2S if the rate changed, starts clocking. Returns 0 / errno. */
int audio_out_start(uint32_t sample_rate);

/* Enqueue mono 16-bit PCM for playback. Non-blocking; drops on queue overflow
 * (logged). No-op if no session is active. This is the seam B feeds. */
void audio_out_write(const int16_t *mono_pcm, size_t nsamp);

/* End the session: stop I2S, mute the amp (SD low). */
void audio_out_stop(void);
```

Rationale: mono input (AI/TTS audio is mono); the feeder duplicates to L/R for
the stereo I2S frame. `start(rate)` keeps the rate flexible (Gemini Live ≈ 24 kHz,
our infra ≈ 16 kHz) without hard-coding.

## Threading

- A dedicated **`audio_out` feeder thread**, priority 7 (same tier as the uplink
  audio thread), stack ~2 KB.
- Loop: drain one PCM block from the `k_msgq` → expand mono→stereo into an I2S
  mem-slab block → `i2s_write`. Prime two blocks then `I2S_TRIGGER_START`.
- **Mid-session underrun:** if the queue is empty when a block is due, feed a
  block of **silence** (keeps I2S clocked so it doesn't underrun-stop). After
  `AUDIO_OUT_IDLE_STOP_MS` of continuous silence, auto-`stop()`.
- Idle (no session): thread blocks on the queue / a session flag; zero CPU.

**Coexistence:** PDM (mic) and I2S (speaker) are **separate nRF52840 peripherals
with independent EasyDMA → true full-duplex**. The dedicated thread + producer/
consumer split is the same discipline that fixed the sub-project-B slab
starvation; output must not stall mic capture.

## I2S / amp / overlay

`app.overlay` (add alongside existing i2c1 + pdm0):
- `i2s0` pinctrl: `I2S_SCK_M`→P0.03 (D1/BCLK), `I2S_LRCK_M`→P0.02 (D0/LRCK),
  `I2S_SDOUT`→P0.28 (D2/DIN). Stock driver, **no MCK** routed.
- Amp **SD** on P1.12 (D7) as a plain GPIO, driven by `audio_out`.

`prj.conf`: `CONFIG_I2S=y`.

Format: 16-bit, stereo framing, master clock from I2S; mono source duplicated to
both channels. Pins confirmed free in the real firmware (D0/D1/D2/D7 unused).

## Power

Amp **SD held low (MAX98357A shutdown, ~µA)** whenever no session is active;
raised only between `audio_out_start()` and `audio_out_stop()`. Important for the
battery wristband — the amp must not idle-drain.

## Test hook

Serial command **`'p'`** in the `main.cpp` console:
`audio_out_start(16000)` → `audio_out_write()` a generated ~1 s 440 Hz tone
(computed in a small loop, **no embedded PCM** — zero flash cost) → `audio_out_stop()`.
Coexists with the standing `'b'`/`'r'` commands.

## Integration

- `main.cpp`: call `audio_out_init()` at boot (after the other subsystem inits);
  add the `'p'` command to the console handler.
- `CMakeLists.txt`: add `src/audio_out.cpp`.
- No change to `mic_vad`, `audio_stream`, `ble_audio`, gesture, or HR code.

## Constants (tagged for housing/production retune)

- `AUDIO_OUT_FRAMES_PER_BLOCK` — I2S block size (latency vs. SD/BLE jitter). `[STRUCTURAL]`
- `AUDIO_OUT_NUM_BLOCKS` — mem-slab depth (buffering). `[STRUCTURAL]`
- `AUDIO_OUT_IDLE_STOP_MS` — silence-before-auto-stop. `[STRUCTURAL]`
- `AUDIO_OUT_TEST_TONE_HZ` / duration — test hook only. `[UNIT]`
- Default/test sample rate (16 kHz). `[UNIT]`

## Testing

Hardware-in-the-loop (no host unit test — same as the rest of the firmware):
1. Flash → press `'p'` → hear a clean 440 Hz tone from the speaker.
2. Press `'p'` **during `MODE_DICTATION`** (mic capturing) → tone plays AND mic
   uplink/HR keep working → confirms full-duplex coexistence, no slab starvation.
3. Idle current sanity: with no session, amp SD is low (multimeter optional).

## Future (B — not built here)

BLE downlink: a new receive characteristic in (or beside) `ble_audio`, LC3
**decode** via the existing liblc3 (`CONFIG_LIBLC3`), feeding `audio_out_write()`.
No change to `audio_out`'s API.
