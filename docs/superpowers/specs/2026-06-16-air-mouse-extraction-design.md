# Air-Mouse Extraction → Clean Gesture+HR Foundation — Design Spec

**Date:** 2026-06-16
**Status:** approved-in-principle (user requested true extraction); spec for review
**Branch:** feature/gesture-foundation (extract here; air-mouse preserved on feature/air-mouse @ 09b9eac)

---

## 1. Goal

Remove the air-mouse cursor feature from `feature/gesture-foundation`, leaving a
clean, buildable **gesture + HR foundation** that future modes (dictation, etc.)
hang off. Air-mouse is fully preserved on `feature/air-mouse`; this is a one-way
removal on the foundation line. The foundation must still **boot and run**: IDLE +
pose detection + chip-tap/multi-tap + bio-acoustic + HR pipeline, with no cursor,
no HID mouse, no AIR_MOUSE mode.

Dependency direction is favorable — the foundation does NOT call into cursor; cursor
calls into the foundation — so removal cannot break foundation logic; the only risk
is dangling references / build breakage, caught by `./build.sh`.

## 2. MUST RETAIN (explicit — do not remove while stripping air-mouse)

- **HR independence from gesture detection.** Chip taps + pose + bio-acoustic are
  serviced in ALL power states via `service_chip_int1()` (called from every state
  loop), and the recent-gesture guard (`gesture_mode_recent_activity()`) that stops
  a sig-motion → `WORKOUT_VERIFY` transition from eating taps during an HR snapshot.
  KEEP `service_chip_int1` (both `handle_sigmotion` paths), `transition_to_workout_verify`,
  and the recent-activity guard intact.
- **Power state machine** (IDLE / SNAPSHOT / WORKOUT_VERIFY / WORKOUT) + the full
  HR/PPG/DSP pipeline (`WearableDSP`, MAX30102, `power_ctrl`).
- **Gesture detection, kept live + open for future modes:** pose arming/classifier,
  the chip-tap engine, the firmware multi-tap counter (single/double/triple), and the
  bio-acoustic FIFO/FFT path. After extraction these **detect + log** (a "gesture
  detected" hook) but route to NO mode — ready to wire a future mode.
- **Shared IMU orientation filter** (`orientation.{h,cpp}` — Mahony + gyro-bias/ZUPT).
- BLE infra: HRS + BAS, connection/security callbacks, re-advertise-on-disconnect.
- Serial console (minus the cursor/mouse commands), INT1 ISR, acq + DSP threads.

## 3. DELETE (files — air-mouse only)

- `src/cursor_track.{h,cpp}`, `src/cursor_calib.{h,cpp}`, `src/cursor_pipeline.{h,cpp}`
- `src/ble_hid.{h,cpp}` (the HID **mouse** output — no other mode uses it)
- `tests/test_cursor_track.cpp`, `tests/test_cursor_calib.cpp`
- Remove the four sources from `CMakeLists.txt` (lines 14–17: ble_hid, cursor_pipeline,
  cursor_track, cursor_calib).

## 4. STRIP (air-mouse woven into shared files)

**`src/main.cpp`:**
- Includes `ble_hid.h`, `cursor_pipeline.h`, `cursor_track.h` (21–23).
- HID init/register (`ble_hid_init()`, ~258–265) and the HID/mouse mention in BLE
  advertising (keep HRS + BAS; drop the mouse HID service).
- The entire **mouse-test mode** block (`mouse_test_active`, the `'m'` command, the
  WASD/click/scroll injects, `MOUSE_TEST_STEP`, `gesture_mode_set(MODE_AIR_MOUSE)`).
- Cursor gain serial knobs `']' '[' '}' '{'` and `'o' '/' 'p'` + their help lines.
- The `cursor_pipeline` thread start + the acq-keep-alive comments referencing cursor.
- KEEP everything in §2 (power state machine, `service_chip_int1`, HR, threads, BLE HRS/BAS).

**`src/gesture_mode.{cpp,h}`:** remove the AIR_MOUSE regions —
- cursor includes (`cursor_track.h`, `cursor_calib.h`, `cursor_pipeline.h`).
- `cursor_calib_run_on_entry()`; the AIR_MOUSE branch in `_transition_to`
  (cursor_track_start/stop, calib); the rest-tracker + cooldown re-engage + AIR_MOUSE
  exit-detection block in `update_accel`; the cursor wiring + `[XARC]`/diagnostics in
  `update_gyro`; the AIR_MOUSE tap-consumption in `gesture_mode_on_chip_single_tap`;
  the cursor-related state (`air_desk_tap`, `air_exit_tap_*`, `last_rest_vert`,
  `rest_dwell`, `reengage_*`, `air_in_rest_zone`, cursor anchors).
- KEEP: orientation classifier, `pose_fsm_update` + arming, `gesture_mode_armed_pose`,
  `gesture_mode_recent_activity`, multi-tap counter + commit handler, the chip-tap
  handlers, the bio-acoustic worker, `update_accel`/`update_gyro` entry points
  (minus cursor), pose-trace.
- **Multi-tap routing:** the double-tap that entered AIR_MOUSE now becomes a
  **log-only "gesture detected" hook** (no mode entry). Pose + tap detection stays
  fully alive for future wiring.

**`src/gesture_mode.h`:** drop cursor-related declarations + acq-request-cb if
cursor-only (verify; keep if the power machine uses it for HR).

**`src/gesture_thresholds.h`:** remove the `CURSOR_*` constants block (gains, anchors,
calib seeds, cone, swing-comp, slam, pin). KEEP pose/tap/orientation/HR constants.

## 5. Mode enum (decision)

`GestureMode`: keep `MODE_IDLE` and `MODE_GESTURE_AMBIENT` (reserved for future
modes). Remove `MODE_SURFACE` (already retired) and `MODE_AIR_MOUSE`. Update
`gesture_mode_str` + any switch/log accordingly. The FSM skeleton + transition
plumbing stays so a future mode is a small addition.

## 6. Verification

- `./build.sh` clean (no dangling refs, no missing symbols), artifact produced.
- `grep` shows zero remaining `cursor_`, `ble_hid`, `MODE_AIR_MOUSE`, `CURSOR_`,
  `mouse_test` references in `src/`.
- No host cursor tests remain; the host build line for `cursor_track`/`cursor_calib`
  is gone (those tests deleted).
- **Boot smoke (HW, user):** boots to IDLE; pose ARM + chip-tap + multi-tap still log;
  HR SNAPSHOT runs; a tap during SNAPSHOT is still serviced; sig-motion → WORKOUT_VERIFY
  still guarded by recent-activity. (Confirms §2 retained.)

## 7. Out of scope

- Re-modularizing the remaining foundation (no `air_mouse_mode` module needed now —
  air-mouse is gone). Future air-mouse work happens on `feature/air-mouse`.
- Any behavior change to the retained foundation beyond removing cursor coupling.
