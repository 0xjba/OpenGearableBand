# Air-Mouse Extraction Implementation Plan

> **For agentic workers:** execute as ONE atomic task — the tree will not build until ALL air-mouse is removed, so do every step, then build once at the end. Spec (the authoritative delete/strip/retain lists): `docs/superpowers/specs/2026-06-16-air-mouse-extraction-design.md`. Build: `./build.sh`. File map/commands: `CLAUDE.md`.

**Goal:** Remove the air-mouse cursor feature from `feature/gesture-foundation`, leaving a clean, buildable gesture+HR foundation (IDLE + pose/tap/bio-acoustic + HR), with HR-independence retained. Air-mouse is preserved on `feature/air-mouse`.

**Architecture:** One-way removal. The foundation does NOT depend on cursor, so removal can't break foundation logic — the only risk is dangling references, caught by `./build.sh` + a `grep` sweep.

---

### Task 1: Extract air-mouse (atomic)

**Read first:** `docs/superpowers/specs/2026-06-16-air-mouse-extraction-design.md` §2 (MUST RETAIN), §3 (DELETE), §4 (STRIP), §5 (mode enum). Those lists are authoritative; the steps below are the execution order.

- [ ] **Step 1: Delete air-mouse-only files**

```bash
git rm src/cursor_track.cpp src/cursor_track.h \
       src/cursor_calib.cpp src/cursor_calib.h \
       src/cursor_pipeline.cpp src/cursor_pipeline.h \
       src/ble_hid.cpp src/ble_hid.h \
       tests/test_cursor_track.cpp tests/test_cursor_calib.cpp
```

- [ ] **Step 2: CMakeLists.txt** — remove the four source lines (ble_hid.cpp, cursor_pipeline.cpp, cursor_track.cpp, cursor_calib.cpp). Leave main/WearableDSP/power_ctrl/gesture_mode/gesture_poses/orientation.

- [ ] **Step 3: `src/gesture_thresholds.h`** — remove the `CURSOR_*` constants block (gains, anchors, calib seeds, cone gate, swing-comp, slam, pin, yaw-half-span). KEEP all pose / tap / orientation / HR constants. (Grep `CURSOR_` after to confirm none remain.)

- [ ] **Step 4: `src/gesture_mode.{cpp,h}`** — strip the AIR_MOUSE regions per spec §4:
  - Remove cursor includes (`cursor_track.h`, `cursor_calib.h`, `cursor_pipeline.h`).
  - Remove `cursor_calib_run_on_entry()`; the AIR_MOUSE branch in `_transition_to` (cursor_track_start/stop + calib call); the rest-tracker + cooldown-re-engage + AIR_MOUSE exit-detection block in `gesture_mode_update_accel`; the cursor wiring + `[XARC]`/diagnostic in `gesture_mode_update_gyro`; the AIR_MOUSE tap-consumption branch in `gesture_mode_on_chip_single_tap`; cursor state (`air_desk_tap`, `air_exit_tap_*`, `last_rest_vert`, `rest_dwell`, `reengage_*`, `air_in_rest_zone`, cursor anchors, `s_transition_via_cooldown_reengage` if cursor-only) and the acq-request-cb if cursor-only (KEEP if the power machine still needs it).
  - **Multi-tap routing:** where a committed double-tap entered AIR_MOUSE, replace with a log-only "gesture detected (no mode bound)" line — keep the detection, drop the mode entry.
  - KEEP: orientation classifier, `pose_fsm_update` + arming, `gesture_mode_armed_pose`, `gesture_mode_recent_activity`, multi-tap counter + commit handler, chip-tap handlers, bio-acoustic worker, `update_accel`/`update_gyro` entry points (minus cursor), pose-trace.

- [ ] **Step 5: Mode enum (spec §5)** — in `gesture_mode.h`, reduce `GestureMode` to `MODE_IDLE` + `MODE_GESTURE_AMBIENT`; remove `MODE_SURFACE` and `MODE_AIR_MOUSE`. Update `gesture_mode_str()` and any `switch`/log over the enum. Remove cursor-related declarations from the header.

- [ ] **Step 6: `src/main.cpp`** — strip per spec §4:
  - Remove includes `ble_hid.h`, `cursor_pipeline.h`, `cursor_track.h`.
  - Remove `ble_hid_init()` + the HID/mouse service from BLE setup/advertising (keep HRS + BAS + the advertising of those).
  - Remove the entire mouse-test mode: `mouse_test_active`, the `'m'` handler, WASD/click/scroll injects, `MOUSE_TEST_STEP`, `gesture_mode_set(MODE_AIR_MOUSE)`.
  - Remove cursor gain serial commands `]` `[` `}` `{` `o` `p` (input-arm + dispatch + help lines).
  - Remove the cursor_pipeline thread start + any cursor-only acq-keep-alive comments/wiring.
  - KEEP: power state machine, `service_chip_int1()` (BOTH `handle_sigmotion` paths) + `transition_to_workout_verify` + the recent-gesture guard, HR/MAX pipeline, acq + DSP threads, INT1 ISR, serial console (other commands), BLE HRS/BAS + reconnect.

- [ ] **Step 7: Build + grep sweep (the gate)**

```bash
./build.sh 2>&1 | tail -5
grep -rEn "cursor_|ble_hid|MODE_AIR_MOUSE|MODE_SURFACE|CURSOR_|mouse_test|XARC|cursor_pipeline" src/ || echo "CLEAN: no air-mouse refs remain"
```
Expected: build prints `==> Flashable artifact:` with NO errors; the grep prints `CLEAN`. If the build fails on a dangling symbol, that's a missed strip — fix it (do NOT stub air-mouse back in). If grep finds a ref, remove it (unless it's an unrelated false match, e.g. a `mouse`-substring in unrelated text — judge each).

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "$(printf 'refactor: extract air-mouse cursor -> clean gesture+HR foundation\n\nRemove the air-mouse cursor feature (preserved on feature/air-mouse): delete\ncursor_track/cursor_calib/cursor_pipeline/ble_hid + cursor host tests; strip the\nAIR_MOUSE mode, cursor wiring, mouse-test, cursor serial knobs, and CURSOR_*\nconstants from gesture_mode/main/gesture_thresholds; trim GestureMode to\nIDLE + GESTURE_AMBIENT.\n\nRetained: power state machine + HR/PPG/DSP, Mahony orientation filter, pose\ndetection, chip-tap + multi-tap + bio-acoustic engine (now detect+log, open for\nfuture modes), and HR-independence (service_chip_int1 in all states + the\nrecent-gesture guard). Foundation builds clean; gesture detection logs but routes\nto no mode.\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

## Done when
- `./build.sh` clean; grep sweep prints CLEAN.
- Foundation retains §2 (verified by reading: `service_chip_int1`, recent-gesture guard, pose/tap/bio-acoustic, HR pipeline all intact).
- HW boot-smoke (user, after): IDLE + pose ARM + tap + HR snapshot + tap-during-snapshot + guarded sig-motion all still work.
