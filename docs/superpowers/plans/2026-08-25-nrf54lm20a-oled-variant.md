# nRF54LM20A OLED-Output Variant — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement task-by-task. Steps use
> checkbox (`- [ ]`) syntax. **This is hardware-in-the-loop firmware on PRELIMINARY
> silicon** — most tasks verify by *clean build + flash + read serial log*, not host
> unit tests (matches the project's existing test model in `CLAUDE.md`). Expect to
> iterate on hardware; the user's standing rule is "build/flash/measure/adjust," not
> line-by-line spec review.

**Goal:** Bring up a second product variant on the **XIAO nRF54LM20A Sense** whose
output is a **0.91" SSD1306 OLED** (text) instead of a speaker — eliminating the
entire audio-downlink + AEC/barge-in subsystem — while carrying over the input side
(IMU orientation/pose, `POSE_EAR`→dictation, PDM mic + LC3 **uplink**). HR/PPG is
**out for v1**, gated behind Kconfig for a future rev.

**Architecture:** Out-of-tree Zephyr board definition (`boards/seeed/xiao_nrf54lm20a/`,
seeded from the in-tree `nrf54lm20dk` SoC nodes + `xiao_nrf54l15` connector shape).
Firmware reuses the existing `src/` modules, newly guarded so the audio-downlink and
HR stacks compile out on this board and a new OLED output module compiles in.
De-risked in 4 milestones, each independently flashable: **M1** board+boot+OLED
"hello" (no PMIC/sensor code), **M2** BLE display-text, **M3** sensors (PMIC LDO →
mic + IMU → dictation), **M4** host wiring (Gemini *text* → OLED) + strip audio.

**Tech Stack:** Zephyr 4.2.99 / NCS 3.2.x, C++, nRF54LM20A (Cortex-M33), SSD1306 +
Zephyr CFB, custom 128-bit GATT, DTS/pinctrl/Kconfig board port.

**Board reference:** `docs/nrf54lm20a-board.md` (pins, power topology, open HW items).
Read it before any DTS work. Build/flash/file-map basics in `CLAUDE.md`.

---

## File Structure

**New (board definition, out-of-tree in repo):**
- `boards/seeed/xiao_nrf54lm20a/board.yml` — board/SoC/variant declaration
- `boards/seeed/xiao_nrf54lm20a/Kconfig.xiao_nrf54lm20a` — SoC select
- `boards/seeed/xiao_nrf54lm20a/board.cmake` — J-Link runner (`nRF54LM20A_M33`)
- `boards/seeed/xiao_nrf54lm20a/xiao_nrf54lm20a-pinctrl.dtsi` — UART/I2C/PDM psels (real pins)
- `boards/seeed/xiao_nrf54lm20a/xiao_nrf54lm20a_common.dtsi` — LEDs/buttons/aliases/power hogs
- `boards/seeed/xiao_nrf54lm20a/xiao_nrf54lm20a_nrf54lm20a_cpuapp.dts` — top-level app DTS
- `boards/seeed/xiao_nrf54lm20a/xiao_nrf54lm20a_nrf54lm20a_cpuapp.yaml` — capabilities
- `boards/seeed/xiao_nrf54lm20a/xiao_nrf54lm20a_nrf54lm20a_cpuapp_defconfig` — base defconfig
- `boards/seeed/xiao_nrf54lm20a/Kconfig.defconfig` — board defconfig hooks

**New (firmware):**
- `build_nrf54.sh` — build wrapper targeting the out-of-tree board (J-Link/nrfutil flash, no UF2)
- `src/display_oled.{h,cpp}` — SSD1306+CFB text output module (the speaker's replacement)
- `boards/xiao_nrf54lm20a.overlay` (app-level) — SSD1306 node on the header I2C, chosen display
- `Kconfig` additions — `CONFIG_GESTUREBAND_HR` (default n here), `CONFIG_GESTUREBAND_OLED`, `CONFIG_GESTUREBAND_AUDIO_DOWNLINK`

**Modified (guarded, no behaviour change on `beta`/nRF52840):**
- `src/main.cpp` — board-conditional peripheral init; HR/audio-downlink behind Kconfig
- `src/mic_vad.cpp` / pinctrl — mic pin source from DT (already DT-driven; verify)
- `prj.conf` / new `prj_nrf54.conf` — per-board Kconfig
- `CLAUDE.md` — add the board variant + build command to the orientation doc

---

## Milestone 1 — Board definition + boot + OLED "hello"

**Goal:** `build_nrf54.sh` produces a flashable image for a custom board; on flash,
the serial console prints a banner and responds to `r`/`b`, and the OLED shows text.
**No PMIC code, no sensors, no BLE.** This proves the riskiest layer (new SoC + board
def + display) in isolation.

**HW pre-check (do first — see board ref open-item #1):** confirm whether the header
`3V3_OUT` rail that powers the OLED is always-on or gated by a GPIO (candidate
`P1.12/DCDC_EN`). If gated, M1 must assert it (add a `regulator-fixed`/`gpio-hog`).

### Task 1.1: Scaffold the out-of-tree board definition

**Files:** all `boards/seeed/xiao_nrf54lm20a/*` above. Seed SoC-level nodes from
`/opt/nordic/ncs/zephyr/boards/nordic/nrf54lm20dk/` and the XIAO connector shape from
`/opt/nordic/ncs/zephyr/boards/seeed/xiao_nrf54l15/`.

- [ ] **Step 1:** Create `board.yml` (name `xiao_nrf54lm20a`, soc `nrf54lm20a`,
  cpuapp variant). Mirror `nrf54lm20dk/board.yml` structure; drop the `ns`/`xip`
  variants for now (single `cpuapp`).
- [ ] **Step 2:** Create `Kconfig.xiao_nrf54lm20a` selecting
  `SOC_NRF54LM20A_ENGA_CPUAPP` for the cpuapp board, and `board.cmake` with
  `board_runner_args(jlink "--device=nRF54LM20A_M33" "--speed=4000")` +
  `include(.../jlink.board.cmake)` + `nrfutil.board.cmake`.
- [ ] **Step 3:** Write `xiao_nrf54lm20a-pinctrl.dtsi` with the **real** pins from
  the board ref: console UART TX `P1.11`/RX `P1.10` (uart20); header I2C
  SDA `P1.03`/SCL `P1.07` on a TWIM instance (call it the OLED bus); PDM
  CLK `P1.13`/DIN `P1.14`; IMU I2C SCL `P0.07`/SDA `P0.08`. (Sensors defined but
  their nodes left `disabled` until M3.)
- [ ] **Step 4:** Write `_common.dtsi` — RGB LED nodes (`P1.22/23/24`), user button
  `P0.09`, `chosen { zephyr,console = &uart20; }`, aliases, LFXO on `P1.20/21`,
  `vregmain` DCDC mode, partitions via `#include <nordic/nrf54lm20a_partition.dtsi>`.
  If M1 HW pre-check found the OLED rail is GPIO-gated, add the enable hog here.
- [ ] **Step 5:** Write the top `_cpuapp.dts` (`#include` SoC cpuapp dtsi +
  connector + common), `_cpuapp.yaml` (supported: gpio, i2c, pwm, serial),
  `_cpuapp_defconfig` (SERIAL/CONSOLE/GPIO/REGULATOR, cf. the nrf54l15 defconfig),
  and `Kconfig.defconfig`.
- [ ] **Step 6 (verify):** `west build -b xiao_nrf54lm20a/nrf54lm20a/cpuapp
  --board-root . <minimal app>` configures + compiles clean. Fix DT/Kconfig errors.
  Expected: build succeeds, no unresolved `chosen`/pinctrl refs.

### Task 1.2: `build_nrf54.sh` + minimal boot app

- [ ] **Step 1:** Create `build_nrf54.sh` (copy `build.sh`, change `BOARD` to
  `xiao_nrf54lm20a/nrf54lm20a/cpuapp`, add `--board-root ${PWD}`, replace the UF2
  hint with the flash command for this board — `west flash` via the SAMD11
  CMSIS-DAP/J-Link, **verify actual flash method on HW**: J-Link runner vs. a UF2
  volume). Keep `-p` pristine passthrough.
- [ ] **Step 2:** Ensure the console + `r`/`b` commands build for this board
  (`CONFIG_REBOOT=y`, `CONFIG_UART_INTERRUPT_DRIVEN=y`, the `uart_rx_cb` on
  `DT_CHOSEN(zephyr_console)`) — standing rule. On nRF54L, `b` (UF2 bootloader via
  `GPREGRET=0x57`) may differ; if there is no UF2 bootloader, make `b` a documented
  no-op/`sys_reboot` and note it. `r` (`sys_reboot COLD`) must work.
- [ ] **Step 3 (verify, HW):** flash; serial prints the boot banner; `r` reboots.

### Task 1.3: OLED "hello" via SSD1306 + CFB

**Files:** `src/display_oled.{h,cpp}`, app overlay adding the SSD1306 node.

- [ ] **Step 1:** Add the SSD1306 node on the header I2C bus in the board/app
  overlay: `compatible = "solomon,ssd1306fb"`, `reg = <0x3c>`, width/height for the
  0.91" (128×32), `chosen { zephyr,display = &ssd1306; }`. Enable
  `CONFIG_DISPLAY=y`, `CONFIG_SSD1306=y`, `CONFIG_CHARACTER_FRAMEBUFFER=y`.
- [ ] **Step 2:** Write `display_oled.{h,cpp}`: `display_oled_init()` (get
  `DEVICE_DT_GET(DT_CHOSEN(zephyr_display))`, `cfb_framebuffer_init`, pick font,
  clear) and `display_oled_show(const char *line1, const char *line2)`
  (clear → `cfb_print` → `cfb_framebuffer_finalize`). Well-commented, minimal.
- [ ] **Step 3:** From the minimal app, call `display_oled_init()` +
  `display_oled_show("gestureband", "OLED v1")`.
- [ ] **Step 4 (verify, HW):** OLED displays the two lines; serial confirms init
  return code 0. Tune I2C address (0x3C/0x3D) and 128×32 vs 128×64 geometry on HW.

**M1 exit criteria:** custom board builds + flashes; console banner + `r` work; OLED
shows text. Commit. (Commit only when the user asks — per standing rule.)

> **✅ DONE 2026-08-26 (HW-verified).** Board boots (Zephyr 4.2.99), console + `r`
> work, OLED shows `gestureband`/`OLED v1 M1` at 0x3C 128×32 on SCL=D10/SDA=D9.
> **Flash:** `./flash_nrf54.sh` (probe-rs `--allow-erase-all` over USB CMSIS-DAP —
> chip ships APPROTECT-locked; OpenOCD/pyOCD/west-flash do NOT complete the recover).
> Needed `zephyr,concat-buf-size=<1088>` on the OLED I2C node. See
> `docs/nrf54lm20a-board.md`.

---

## Milestone 2 — BLE up + display-text characteristic

**Goal:** phone/Mac writes a UTF-8 string over a BLE characteristic → it renders on
the OLED. Proves the SoftDevice/controller on nRF54L + the output path end-to-end.

### Task 2.1: BLE stack boot + BAS
- [ ] Bring up the Zephyr BLE host + nRF54L controller (`CONFIG_BT=y`, board's BLE
  Kconfig); advertise; add **BAS** (battery). Verify connect from a host + `r`
  re-advertise on disconnect (mirror `main.cpp`'s `conn_recycled` pattern).

### Task 2.2: `display-text` GATT characteristic → OLED
- [ ] Define a minimal custom 128-bit GATT service, one **WRITE** char; on write,
  copy the bytes (bounded) and call `display_oled_show()` (split/trim to the OLED's
  2 lines). Add a NOTIFY status byte if useful for host ack. Reuse the `ble_audio.cpp`
  service-scaffolding style, not its audio payload.
- [ ] **Verify (HW):** from a BLE tool, write "hello world" → appears on OLED.

**M2 exit criteria:** host → BLE write → OLED text. Commit (on user ask).

> **✅ DONE 2026-08-26 (HW-verified).** SoftDevice Controller up (BT 6.0 / HCI 6.2),
> advertises as `gband-OLED` with battery + custom display service
> (svc `e9a10001-…`, text char `e9a10002-…`). Wrote text from nRF Connect mobile →
> rendered on OLED. App `apps/nrf54_ble`, module `src/ble_display`, host test
> `tools/nrf54_ble_display_test.py`. Advertising uses explicit `BT_LE_ADV_OPT_CONN`
> param (`BT_LE_ADV_CONN` removed in Zephyr 4.2).

---

## Milestone 3 — Sensors: PMIC rail → PDM mic + IMU → dictation

**Goal:** power the IMU/mic rail via the nPM1300, bring up the LSM6DS3-family IMU and
the PDM mic, and restore the `POSE_EAR`→`MODE_DICTATION` voice-gated path with the
mic→BLE **LC3 uplink** stream. HR stays compiled out.

### Task 3.1: nPM1300 PMIC bring-up (IMU&MIC LDO)
- [ ] Add the nPM1300 on IIC1 (`PMIC_SCL P1.17`/`PMIC_SDA P1.18`) using Zephyr's
  `nordic,npm1300` regulator driver; enable the **LDO1 / load-switch** that feeds
  `IMU&MIC_3V3`. Resolve board-ref open-item #3 (PMIC-LDO vs `P0.01` GPIO enable) on
  HW. Verify the rail comes up (measure / IMU WHO_AM_I responds).

### Task 3.2: LSM6DS3-family IMU
- [ ] Confirm the exact part (board-ref open-item #2 — likely **LSM6DS3TR-C**), set
  the correct Zephyr `compatible`, DT node on IIC0 (`P0.07/P0.08`, INT1 `P0.06`).
  Feed the existing `orientation`/`gesture_mode` acq path (100 Hz). The Mahony math
  is driver-agnostic; only the driver + tap-engine register map differ — re-verify
  the on-chip tap/sig-motion config against the LSM6DS3TR-C datasheet vs the LSM6DSL.
- [ ] **Verify (HW):** `v` pose trace + `g` gravity dump behave; pose detection arms.

### Task 3.3: PDM mic + dictation + LC3 uplink
- [ ] Remap `mic_vad` PDM pins to `P1.13/P1.14` (via DT `dmic`), confirm 16 kHz
  capture + adaptive floor + voice-onset. Restore `POSE_EAR` mic gate →
  `MODE_DICTATION` and the `audio_stream`→`lc3_codec`(encode)→`ble_audio` NOTIFY
  **uplink** (the phone still receives the user's voice).
- [ ] **Verify (HW):** `m` mic bench probe; raise-to-ear + speak → `MODE_DICTATION`
  logs; host receives LC3 uplink frames.

**M3 exit criteria:** dictation entry + mic uplink work on the OLED board. Commit.

> **M3c.1 ✅ DONE 2026-08-26 (HW-verified).** PDM mic (pdm20 via new `dmic0`
> alias — nRF54L can't carry a `pdm0` nodelabel; SoC validation reserves it)
> CLK P1.13/DIN P1.14, powered by nPM1300 LDO1. `mic_vad` capture + adaptive
> floor + M-of-N voiced-onset run: speech → veM ~1000× ambient, VOICE ONSET
> fires reliably, transients rejected. **nRF52840-tuned thresholds work as-is
> (no retune, no Cobra needed).** App `apps/nrf54_mic` (audio_stream stubbed).
> `mic_vad.cpp` now uses `DT_ALIAS(dmic0)` (both boards build). **M3c.2 next:**
> real audio_stream + lc3_codec + ble_audio LC3 uplink.

> **M3b ✅ DONE 2026-08-26 (HW-verified).** Ported `orientation` (Mahony) +
> `gesture_poses` onto a 100 Hz acq thread reading the LSM6DS3TR-C. Pitch/roll
> track tilt + settle (gravity-locked, drift-free), `at_rest` works, yaw drifts
> (expected, no mag). Pose classifier fired POSE_EAR at score 0.95 when gravity
> matched the canonical -> full pipeline works. App `apps/nrf54_pose`; `v` trace,
> `c` capture-canonical. **POSE_EAR needs wrist-mount recalibration** (canonical
> is nRF52840-tuned) -- deferred to a rough-mount session after firmware.
> **M3c next:** PDM mic + `mic_vad` + `MODE_DICTATION` + LC3 uplink.

> **M3a ✅ DONE 2026-08-26 (HW-verified).** nPM1300 LDO1 (boot-on, IIC1 i2c21
> @0x6b) powers the IMU/mic rail; LSM6DS3TR-C alive on IIC0 (i2c30) @0x6A via
> `st,lsm6dsl` at 104 Hz (ODR Kconfig is an INDEX: 4=104Hz, NOT the Hz value);
> gravity tracks tilt. App `apps/nrf54_imu`. No `P0.01` enable needed.
> **M3b next:** port `orientation` + `gesture_poses` → `POSE_EAR`.

---

## Milestone 4 — Strip audio-downlink + wire AI-text → OLED

**Goal:** make the OLED variant's build cleanly *exclude* the speaker/AEC world and
render Gemini's **text** replies on the OLED; the host stops sending LC3 audio down.

### Task 4.1: Kconfig-gate the removed stacks
- [ ] Introduce `CONFIG_GESTUREBAND_AUDIO_DOWNLINK` (default n on this board) guarding
  `audio_out`/`audio_downlink`/LC3-**decode**/downlink BLE char/clock-recovery, and
  `CONFIG_GESTUREBAND_HR` (default n) guarding MAX30102/`WearableDSP`/HRS + the
  HR-driven power states. Confirm `beta`/nRF52840 still builds with both = y
  (no behaviour change on the speaker board).
- [ ] Collapse the power state machine on this board to `run_idle` + the
  dictation/gesture path (HR states behind the HR guard), preserving the
  "chip taps serviced in all states" invariant.

### Task 4.2: Host — send Gemini text to the display char
- [ ] In `tools/voiceio`, add an OLED-variant path: instead of streaming LC3 audio
  downlink, forward Gemini's **text** transcript to the M2 `display-text` char.
  Uplink (voice → Gemini) unchanged. No DTLN/AEC/barge-in on this variant (no
  speaker ⇒ no echo) — the floor gate and convergence latch are simply not in this
  path.
- [ ] **Verify (HW):** speak → Gemini replies → text on OLED, full loop, no audio
  playback, no self-barge (structurally impossible here).

**M4 exit criteria:** end-to-end voice-in / text-out on the OLED board; audio-downlink
+ HR compiled out; nRF52840 speaker build unaffected. Commit.

---

## Cross-cutting notes
- **Both boards stay buildable.** `beta`/nRF52840 = speaker variant (audio downlink +
  HR on). This board = OLED variant (audio downlink + HR off, OLED on). Never both
  OLED and speaker in one device (one output interface) — enforced by Kconfig choice.
- **Constants:** any new tuned value (OLED I2C addr, line lengths, PMIC rail timing)
  gets a `[HOUSING]`/`[USER]`/`[UNIT]`/`[STRUCTURAL]` tag per the standing rule.
- **Deferred (future OLED rev):** HR/PPG — the MAX30102 wiring + `WearableDSP` + HRS
  flip back on via `CONFIG_GESTUREBAND_HR`; confirm the PPG footprint on this board.
- **Preliminary silicon caveat:** `nrf54lm20a` is engineering-sample ("enga"); expect
  driver rough edges. Verify each HW open item in `docs/nrf54lm20a-board.md` on the
  bench rather than assuming.
