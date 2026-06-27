# gestureband — project orientation

Firmware for a gesture + heart-rate wristband. **Read this instead of
re-discovering the basics** (build command, where Zephyr lives, file map,
known gotchas). Keep it current when these stable facts change.

> **Branch split (2026-06-16):** the **air-mouse cursor** feature was extracted
> to branch **`feature/air-mouse`** (cursor_track / cursor_calib / cursor_pipeline
> / ble_hid mouse + the AIR_MOUSE mode). **`beta`** (this
> branch) is the clean **gesture-detection + HR foundation**: IMU orientation,
> pose detection, the chip-tap / multi-tap counter, and the power/HR pipeline.
> **Dictation entry (sub-project A.1) is built here**: raise-to-ear pose
> (`POSE_EAR`) + voice-onset on the PDM mic → `MODE_DICTATION` (**detect + log
> only**; no audio stream/HID yet — those are sub-projects B–E). The multi-tap
> counter stays as unbound scaffolding for a future tap mode.
> Resume air-mouse work on `feature/air-mouse` (its design specs are under
> `docs/superpowers/specs/2026-06-1{5,6}-*`).

## Build / flash / test
- **Build:** `./build.sh` (wraps `west build --no-sysbuild`, board
  `xiao_ble/nrf52840/sense`) → artifact `build/zephyr/zephyr.uf2`. `-p` =
  pristine.
- **Flash:** double-tap RESET on the Xiao, then
  `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`.
- **Host unit tests:** `mic_vad`'s pure helpers (`mic_vad_block_rms`,
  `mic_vad_band_sum` in `src/mic_vad_rms.cpp`) — build + run directly:
  `g++ -std=c++11 -Isrc tests/test_mic_vad.cpp src/mic_vad_rms.cpp -lm -o /tmp/mv && /tmp/mv`.
  Everything else is hardware-in-the-loop: clean `./build.sh` + read the serial
  log against expected output.

## Toolchain / hardware (so nobody greps for "where is Zephyr")
- NCS root `/opt/nordic/ncs`; `ZEPHYR_BASE=/opt/nordic/ncs/zephyr`; toolchain
  `/opt/nordic/ncs/toolchains/185bb0e3b6` (Zephyr SDK under it). `build.sh`
  exports all of this — just run `./build.sh`, don't set it up by hand.
- Board: Seeed **XIAO nRF52840 Sense** (nRF52840, Cortex-M4F). C++.
- IMU: **LSM6DSL** (accel+gyro, 6-axis, NO magnetometer), I2C; on-chip tap
  engine + significant-motion engine; INT1 on P0.11.
- PPG: MAX30102. Mic: PDM (onboard). Host link: BLE **HRS + BAS** (heart-rate +
  battery) to a phone/Mac. (The HID-mouse output is on `feature/air-mouse`.)

## File map
- `src/main.cpp` — power state machine (`run_idle` / `run_snapshot` /
  `run_workout_verify` / `run_workout`, dispatched on `power_state`);
  `service_chip_int1()` (INT1 tap + sig-motion demux, called from ALL states);
  `wait_servicing_taps()`; BLE init + advertising (HRS + BAS, + `conn_recycled`
  re-advertise on disconnect); the **serial console** (single-letter cmds);
  INT1 ISR `lsm6dsl_int1_isr` → `motion_wake_sem`; acq + DSP threads.
- `src/gesture_mode.cpp` — gesture FSM hub (runs on the **acq thread**): pose
  detect/arm, orientation classifier `_classify_orientation`, the **`POSE_EAR`
  mic gate → `MODE_DICTATION`** (`ear_gate_update`: ear pose present → `mic_vad`
  on; voice-onset → enter; pose-gone + voice-stopped → exit; voice-continuity
  holds through a lean — detect + log only), the firmware multi-tap counter +
  commit handler (unbound scaffolding, benign log), chip-tap handlers,
  `gesture_mode_recent_activity` (HR workout-suppression guard). `GestureMode` =
  `MODE_IDLE` + `MODE_DICTATION`.
- `src/mic_vad.{h,cpp}` + `src/mic_vad_rms.cpp` — PDM mic (16 kHz mono, DMIC) on
  its own capture thread; per-block voiced-band (300–3000 Hz) spectral energy via
  CMSIS rFFT (`veM`); **continuously-adaptive noise floor** (min-seeded over a
  short warm-up, then instant-down / slow-up tracker — robust to a non-silent
  raise; 2026-06-18) + **M-of-N voice-onset** (`mic_vad_voice_onset`, read-and-clear)
  + voice-continuity (`mic_vad_voice_active`); `[MIC]` serial log. Pure helpers
  (`mic_vad_block_rms`, `mic_vad_band_sum`) in `mic_vad_rms.cpp` are host-tested.
  Isolated — no dependency into the FSM; the FSM gates it on `POSE_EAR`. The capture
  loop also hands each block to `audio_stream_feed()` for the dictation stream.
- **Dictation audio stream (sub-project B, 2026-06-18)** — streams `MODE_DICTATION`
  mic audio over BLE as LC3, off the capture thread:
  - `src/lc3_codec.{c,h}` — LC3 encoder wrapper (Google liblc3, `CONFIG_LIBLC3`);
    16 kHz / 10 ms / 32 kbps → 40 B/frame; one 20 ms block → 80 B.
  - `src/audio_stream.{h,cpp}` — bridge: capture thread copies the PCM block into a
    `k_msgq`; a dedicated **audio thread** (prio 7) drains it, LC3-encodes, notifies.
    Gate = `MODE_DICTATION && ble_audio_subscribed()`. Requests a fast BLE conn
    interval on stream start. (Producer/consumer split is load-bearing — inline
    encode+notify starved the PDM slab; see the B spec.)
  - `src/ble_audio.{h,cpp}` — minimal custom 128-bit GATT service, one NOTIFY char,
    own connection tracking + conn-param control. No control/status char.
  - Borrowed from the prior `oneDiary` project; host test receiver `tools/audio_rx.py`.
- `src/orientation.{h,cpp}` — Mahony complementary filter → pitch/roll
  (gravity-locked, drift-free), yaw (gyro-only, drifts), `at_rest` (ZUPT),
  gyro-bias (ZARU). Shared IMU foundation for pose detection. (Auto yaw re-zero on
  stillness was removed 2026-06-18 — `orientation_rezero_yaw()` kept as manual API;
  yaw currently has no consumer.)
- `src/gesture_poses.{h,cpp}` — pose canonicals (gravity direction per pose);
  current set = `POSE_EAR` (`POSE_NONE` sentinel only otherwise).
- `src/WearableDSP.{h,cpp}` — HR/PPG DSP. `src/power_ctrl.{h,cpp}` — MAX30102 +
  power helpers.
- `src/gesture_thresholds.h` — central catalog of empirical constants, each
  tagged `[HOUSING]` / `[USER]` / `[UNIT]` / `[STRUCTURAL]`.

## Threading
acq thread (100 Hz IMU → `gesture_mode_update_accel/gyro`) · power thread (state
machine + chip-tap servicing) · DSP thread (HR) · `mic_vad` capture thread (prio 6;
PDM 16 kHz DMIC + FFT; runs only while `POSE_EAR` gates it on) · **audio thread**
(prio 7, below capture so it can't starve the PDM slab; drains the PCM `k_msgq` →
LC3 encode → BLE notify during `MODE_DICTATION`). Cross-thread state uses `atomic_t`
(e.g. `mic_onset`, `mic_voice_active`) or a documented benign race (e.g. reading
`gx_filt` for a threshold). Measured concurrent load (audio + HR + IMU) ≈ 30% CPU,
~70% idle (2026-06-18).

## Serial console (single letters)
`r` reboot · `b` UF2 bootloader · `g` dump gravity · `t` sim double-tap
(unbound) · `y` sim triple-tap (unbound) · `c` tap-cal logging · `+`/`-` TAP_THS ·
`q` PPG probe · `z` gyro-bias trace · `v` pose trace · `u` clear BLE bonds ·
`m` PDM mic bench probe (`[MIC]` rms/veM/frac log; toggle OFF for the auto gate) ·
`k` toggle chip tap engine + tap-event log (speaker-vs-tap false-trigger test) ·
`0`-`9` speaker volume 0–90% (boot default 100%).
(The `j` force-mic / `p` test-tone / `w` SD-WAV test commands were removed 2026-06-24;
the mic now gates ONLY on the real `POSE_EAR` + voice path.)

**STANDING RULE:** every firmware AND throwaway test app must include the `r`
(reboot, `sys_reboot` COLD) and `b` (UF2 bootloader via `NRF_POWER->GPREGRET=0x57`
+ reboot) console commands by default. Needs `CONFIG_REBOOT=y` +
`CONFIG_UART_INTERRUPT_DRIVEN=y` + a `uart_rx_cb` on `DT_CHOSEN(zephyr_console)`.

## Gotchas (do not relearn these every session)
- **Chip taps are serviced in ALL power states** (`service_chip_int1` from every
  state loop, not IDLE-only) — so pose/tap/dictation keep working during the
  HR SNAPSHOT, and the recent-gesture guard (`gesture_mode_recent_activity`)
  stops a sig-motion → WORKOUT_VERIFY from eating taps. **This HR-independence is
  a retained must-keep.**
- **Stale BLE bond** → host drops with `reason 0x13` shortly after connect. Fix:
  serial `u` (clear bonds) + Forget on the host + re-pair. `reason 0x08` =
  supervision timeout (RF/idle); the device re-advertises on disconnect, so it
  self-heals.
- Geometry: `gx`=forearm elevation, `gy`=left-right sweep, `gz`=volar-normal.
- Process: gesture design/roadmap/feasibility in `docs/research/gesture-architecture.md`
  (the consolidated gesture doc); HR algorithm + roadmap in
  `docs/research/hr-algorithm-decisions.md`; specs + plans under
  `docs/superpowers/`. We use brainstorm → spec → plan → subagent build.
- **Device BLE contract** (the durable wire format for any host/phone client —
  audio service UUIDs, uplink `[seq16][ts32][LC3]`, downlink, status, FLUSH,
  clock model): `docs/device-ble-contract.md`.
- (Air-mouse only, on `feature/air-mouse`): macOS pointer-accel must be OFF;
  `CURSOR_SENSITIVITY` stays 1.0; HID chars are `PERM_READ_ENCRYPT`.

## For subagents
When dispatching implementer/review subagents, point them here for build/test
commands and the file map rather than re-explaining — e.g. "build: `./build.sh`;
file layout in `CLAUDE.md`."
