# gestureband — project orientation

Firmware for a gesture + heart-rate wristband. **Read this instead of
re-discovering the basics** (build command, where Zephyr lives, file map,
known gotchas). Keep it current when these stable facts change.

> **Branch split (2026-06-16):** the **air-mouse cursor** feature was extracted
> to branch **`feature/air-mouse`** (cursor_track / cursor_calib / cursor_pipeline
> / ble_hid mouse + the AIR_MOUSE mode). **`feature/gesture-foundation`** (this
> branch) is the clean **gesture-detection + HR foundation**: IMU orientation,
> pose detection, the chip-tap / multi-tap / bio-acoustic engine, and the
> power/HR pipeline. Gesture detection currently **detects + logs** ("no mode
> bound") — it routes to no mode, kept open for future modes (dictation, etc.).
> Resume air-mouse work on `feature/air-mouse` (its design specs are under
> `docs/superpowers/specs/2026-06-1{5,6}-*`).

## Build / flash / test
- **Build:** `./build.sh` (wraps `west build --no-sysbuild`, board
  `xiao_ble/nrf52840/sense`) → artifact `build/zephyr/zephyr.uf2`. `-p` =
  pristine.
- **Flash:** double-tap RESET on the Xiao, then
  `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`.
- **Host unit tests:** none on this branch — the only pure/host-testable modules
  were the cursor ones (`cursor_track`, `cursor_calib`), which left with
  air-mouse. Per-change verification = clean `./build.sh` + reading the serial
  log against expected output (hardware-in-the-loop).

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
  detect/arm, orientation classifier `_classify_orientation`, the firmware
  multi-tap counter + commit handler (single/double/triple → **detect + log,
  no mode bound**), chip-tap handlers, `gesture_mode_recent_activity` (HR
  workout-suppression guard), and the bio-acoustic tap path. `GestureMode` =
  `MODE_IDLE` + `MODE_GESTURE_AMBIENT` (reserved). FSM skeleton stays so a future
  mode is a small add.
- `src/orientation.{h,cpp}` — Mahony complementary filter → pitch/roll
  (gravity-locked, drift-free), yaw (gyro-only, drifts), `at_rest` (ZUPT),
  gyro-bias (ZARU), fused gravity vector. Shared IMU foundation (pose detection,
  future dictation discriminator).
- `src/gesture_poses.{h,cpp}` — pose canonicals (gravity direction per pose).
- `src/WearableDSP.{h,cpp}` — HR/PPG DSP. `src/power_ctrl.{h,cpp}` — MAX30102 +
  power helpers.
- `src/gesture_thresholds.h` — central catalog of empirical constants, each
  tagged `[HOUSING]` / `[USER]` / `[UNIT]` / `[STRUCTURAL]`.

## Threading
acq thread (100 Hz IMU → `gesture_mode_update_accel/gyro`) · power thread (state
machine + chip-tap servicing) · DSP thread (HR) · BIO worker (FFT on tap).
Cross-thread state uses `atomic_t` or a documented benign race (e.g. reading
`gx_filt` for a threshold).

## Serial console (single letters)
`r` reboot · `b` UF2 bootloader · `g` dump gravity · `t` sim double-tap
(gesture detect, no mode bound) · `y` sim triple-tap (unbound) · `c` tap-cal
logging · `+`/`-` TAP_THS · `q` PPG probe · `z` gyro-bias trace · `v` pose trace ·
`u` clear BLE bonds.

## Gotchas (do not relearn these every session)
- **Chip taps are serviced in ALL power states** (`service_chip_int1` from every
  state loop, not IDLE-only) — so pose/tap/bio-acoustic keep working during the
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
- (Air-mouse only, on `feature/air-mouse`): macOS pointer-accel must be OFF;
  `CURSOR_SENSITIVITY` stays 1.0; HID chars are `PERM_READ_ENCRYPT`.

## For subagents
When dispatching implementer/review subagents, point them here for build/test
commands and the file map rather than re-explaining — e.g. "build: `./build.sh`;
file layout in `CLAUDE.md`."
