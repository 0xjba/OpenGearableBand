# gestureband — project orientation

Firmware for a gesture + heart-rate wristband. **Read this instead of
re-discovering the basics** (build command, where Zephyr lives, file map,
known gotchas). Keep it current when these stable facts change.

## Build / flash / test
- **Build:** `./build.sh` (wraps `west build --no-sysbuild`, board
  `xiao_ble/nrf52840/sense`) → artifact `build/zephyr/zephyr.uf2`. `-p` =
  pristine.
- **Flash:** double-tap RESET on the Xiao, then
  `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`.
- **Host unit test** (the one pure module, `cursor_track`):
  `g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct` → expect `ALL PASS`.
- **No on-target unit harness** — hardware-in-the-loop firmware. Per-change
  verification = clean `./build.sh` + reading the serial log against expected
  output. `cursor_track` is the only host-testable module (pure, no Zephyr).

## Toolchain / hardware (so nobody greps for "where is Zephyr")
- NCS root `/opt/nordic/ncs`; `ZEPHYR_BASE=/opt/nordic/ncs/zephyr`; toolchain
  `/opt/nordic/ncs/toolchains/185bb0e3b6` (Zephyr SDK under it). `build.sh`
  exports all of this — just run `./build.sh`, don't set it up by hand.
- Board: Seeed **XIAO nRF52840 Sense** (nRF52840, Cortex-M4F). C++.
- IMU: **LSM6DSL** (accel+gyro, 6-axis, NO magnetometer), I2C; on-chip tap
  engine + significant-motion engine; INT1 on P0.11.
- PPG: MAX30102. Mic: PDM (onboard). Host link: BLE HID mouse (HOGP) to a Mac.

## File map
- `src/main.cpp` — power state machine (`run_idle` / `run_snapshot` /
  `run_workout_verify` / `run_workout`, dispatched on `power_state`);
  `service_chip_int1()` (INT1 tap + sig-motion demux, called from ALL states);
  `wait_servicing_taps()`; BLE init + advertising (+ `conn_recycled`
  re-advertise on disconnect); the **serial console** (single-letter cmds);
  INT1 ISR `lsm6dsl_int1_isr` → `motion_wake_sem`; acq + DSP threads.
- `src/gesture_mode.cpp` — gesture FSM: pose detect/arm, multi-tap → mode
  entry, mode transitions + cursor-mode exit/cooldown, orientation classifier
  `_classify_orientation`, cursor wiring (`cursor_track_start/update` from
  `gesture_mode_update_gyro`), `[CURSOR]`/`[DESK]` telemetry, bio-acoustic tap
  path. Runs on the **acq thread**. (Large file — the FSM hub.)
- `src/cursor_track.{h,cpp}` — **PURE** absolute-Y cursor: fixed top anchor →
  servo toward `GAIN_Y*(vert-vert_top)` → slam-to-edge + latched top re-pin.
  Host-unit-tested. No Zephyr deps.
- `src/cursor_calib.{h,cpp}` — **PURE** entry-time cursor calibration: extracts
  the resting-pose bottom anchor from a `vert` history + scores the air-mouse
  entry ritual → adoption verdict (`cursor_calib_decide`). Host-unit-tested
  (`tests/test_cursor_calib.cpp`). `gesture_mode` runs it at AIR_MOUSE entry
  (decide→set_anchors→start) and logs `[CAL]`. No Zephyr deps.
- `src/cursor_pipeline.{h,cpp}` — 125 Hz publish thread; sensitivity (MUST stay
  1.0) + 1px dead-zone + int8; `cursor_pipeline_inject_motion`; calls
  `ble_hid_send_report`.
- `src/ble_hid.{h,cpp}` — HOGP mouse: report map (declares Report ID 1, but the
  notification **payload carries NO ID prefix** — 4 bytes buttons/x/y/wheel);
  `ble_hid_send_report`; security/bond; conn + `security_changed` callbacks.
- `src/orientation.{h,cpp}` — Mahony complementary filter → pitch/roll
  (gravity-locked, drift-free), yaw (gyro-only, drifts), `at_rest` (ZUPT),
  gravity vector. NOTE Euler pitch is roll-contaminated near high roll; the
  cursor's Y signal is `vert = acos(|gx|/|g|)` instead (0°=vertical, 90°=flat).
- `src/gesture_thresholds.h` — central catalog of empirical constants, each
  tagged `[HOUSING]` / `[USER]` / `[UNIT]` / `[STRUCTURAL]`.

## Threading
acq thread (100 Hz IMU → `gesture_mode_update_accel/gyro` + cursor) · power
thread (state machine + chip-tap servicing) · DSP thread (HR) · cursor publish
thread (125 Hz BLE HID) · BIO worker (FFT on tap). Cross-thread state uses
`atomic_t` or a documented benign race (e.g. reading `gx_filt` for a threshold).

## Serial console (single letters)
`r` reboot · `b` UF2 bootloader · `g` dump gravity · `t` sim double-tap
(AIR_MOUSE entry) · `y` sim triple-tap (SURFACE) · `m` mouse test mode · `c`
tap-cal logging · `+`/`-` TAP_THS · `q` PPG probe · `z` gyro-bias trace · `v`
pose trace · `u` clear BLE bonds · `]`/`[` horizontal-X gain · `}`/`{`
vertical-Y gain.

## Gotchas (do not relearn these every session)
- **Cursor needs macOS pointer acceleration OFF** (System Settings → Mouse →
  Advanced → Pointer acceleration). With it ON, the absolute-Y servo's
  max-velocity catch-up bursts get amplified and the cursor flies off-target.
  `CURSOR_SENSITIVITY` must stay 1.0 (compile-asserted in cursor_pipeline.cpp).
- **Stale BLE bond** → host drops with `reason 0x13` shortly after connect (HID
  chars are `PERM_READ_ENCRYPT`). Fix: serial `u` (clear bonds) + Forget on the
  Mac + re-pair. `reason 0x08` = supervision timeout (RF/idle); the device
  re-advertises on disconnect now, so it self-heals.
- **Chip taps are serviced in ALL power states** now (`service_chip_int1` from
  every state loop) — was IDLE-only, which dropped taps during the HR SNAPSHOT.
- Geometry: `gx`=forearm elevation, `gy`=left-right sweep, `gz`=volar-normal.
- Process: design/roadmap in `docs/research/gesture-architecture.md`;
  HR roadmap in `docs/research/software-optimization-roadmap.md`; specs + plans
  under `docs/superpowers/`. We use brainstorm → spec → plan → subagent build.

## For subagents
When dispatching implementer/review subagents, point them here for build/test
commands and the file map rather than re-explaining — e.g. "build: `./build.sh`;
host test + file layout in `CLAUDE.md`."
