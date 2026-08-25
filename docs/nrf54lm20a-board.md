# XIAO nRF54LM20A Sense — board hardware reference

Durable pin/power reference for the **OLED-output variant** firmware. Source of
truth = Seeed schematic **Rev V1.0** (`XIAO_nRF54LM20A_Schematic.pdf`, dated
2025-12-18, KiCad, marked **"PRELIMINARY"**) cross-checked against the Seeed pin
xlsx and the getting-started wiki. Where the xlsx and schematic disagree (the
xlsx carries superseded draft rows), **the schematic wins**.

> This board is a *different* SoC from the nRF52840 Sense (`beta` branch). Pins,
> IMU part, and power topology all differ. Do not copy nRF52840 pin numbers.

## SoC / toolchain
- **SoC:** Nordic **nRF54LM20A** (nRF54L family, Cortex-M33). In-tree as
  engineering-sample silicon: SoC Kconfig `SOC_NRF54LM20A_ENGA_CPUAPP`
  ("enga" = engineering sample A). App core target
  `nrf54lm20dk/nrf54lm20a/cpuapp`.
- **Zephyr/NCS:** in `/opt/nordic/ncs` (Zephyr 4.2.99 / NCS 3.2.x). SoC dtsi
  `zephyr/dts/vendor/nordic/nrf54lm20a.dtsi`; Nordic reference board
  `zephyr/boards/nordic/nrf54lm20dk/` (the copy-from for SoC-level nodes);
  XIAO-connector shape from `zephyr/boards/seeed/xiao_nrf54l15/` (same connector,
  *different* SoC/pins).
- **Debug/flash:** on-board **ATSAMD11D14A** CMSIS-DAP probe + USB. Console UART
  is bridged through it (see UART below).
  - **Flash with `./flash_nrf54.sh`** (probe-rs over the built-in USB — no
    external debugger). **VERIFIED on HW 2026-08-26.**
  - **The chip ships APPROTECT-LOCKED from the factory** — the first flash must
    recover (mass-erase). **Only probe-rs (>= 0.32, has native nRF54LM20A
    support) completes this over the SAMD11 CMSIS-DAP** via `--allow-erase-all`.
    Mainline OpenOCD 0.12 and the NCS-bundled pyOCD both connect to the probe and
    read the debug port, but their nRF54L CTRL-AP ERASEALL **times out** here —
    do not waste time on them. (`west flash` therefore also fails.)
  - probe-rs install (macOS arm64): download the prebuilt
    `probe-rs-tools-aarch64-apple-darwin` from GitHub releases → `probe-rs` is on
    PATH at `~/.cargo/bin/probe-rs`.
  - Connect-under-reset does NOT work (reset line via SAMD11 doesn't drive the
    sequence) — flash without it.

## Power topology (nPM1300 PMIC) — READ BEFORE POWER CODE
The board is powered through an **nPM1300 PMIC** (U2), not simple GPIO-fixed
regulators. Rails (schematic sheet 3 block diagram + sheet 4 power):

| Rail | Source | Powers | Firmware action needed? |
|------|--------|--------|--------------------------|
| **VSYS 3.3V** (nRF54 "Power IN") | nPM1300 **VOUT2**, 200 mA, **hardware-VSET** (R10=470K on VSET2) | the nRF54 core itself | **No** — autonomous at power-on (must be, or the CPU couldn't boot to run any code). "Set VOUT2 3.3V, do not use VOUT1." |
| **3V3_OUT** (XIAO header rail) | **TPS62840** DC/DC (U1), 600 mA | 28-pin header → **the OLED** | **Verify** — candidate enable `P1.12/DCDC_EN` (xlsx). Seeed XIAO nRF54 boards GPIO-gate rails (cf. the nrf54l15 template's `vbat_pwr`/`pdm_imu_pwr` hogs). Treat as possibly-gated: assert enable in the board def. |
| **IMU&MIC_3V3** | nPM1300 **LDO1** + load switch (LSIN/LSOUT) | LSM6DS3 IMU + PDM mic | **Yes, at M3** — enable via PMIC TWI (and/or `P0.01/IMU&MIC_3V3_EN` per xlsx). Not needed until sensors come up. |

- **PMIC control bus (IIC1 / TWI):** `PMIC_SCL = P1.17`, `PMIC_SDA = P1.18`.
  PMIC interrupt/GPIO lines to SoC: `npm_GPIO0 = P1.25`, `npm_GPIO1 = P1.26`.
- **Consequence for milestones:** boot + console + OLED (M1) need **no PMIC
  firmware** — VOUT2 is autonomous and the OLED rail is from the DC/DC. PMIC TWI
  bring-up (nPM1300 Zephyr regulator/charger driver) is an **M3** concern for the
  IMU/mic rail. This is why M1 is de-risked and comes first.

## Pin map (authoritative — schematic sheet 6)

### Console UART (via ATSAMD11 bridge → USB serial)
- `nRF54_TX = P1.11`, `nRF54_RX = P1.10`. Level-shifted through UM3204 + SAMD11.
- This is the `zephyr,console` / `zephyr,shell-uart`. Serial `r`/`b` console
  lands here (standing rule).

### OLED — I2C on the XIAO header (0.91" SSD1306)
- **Firmware maps the OLED bus to D10 (SCL, P1.06) + D9 (SDA, P1.05)** — NOT the
  default D4/D5 — so all four OLED wires land on four **consecutive right-edge
  pads** for neat single-side soldering. Right edge top→bottom: 5V, GND, 3V3,
  **D10, D9**, D8, D7. Wire order becomes GND · 3V3 · D10(SCL) · D9(SDA).
- The board's *standard* XIAO I2C pins are D4=P1.03/SDA, D5=P1.07/SCL (schematic
  U7 pins 5/6); this variant repurposes the header I2C solely for the OLED, hence
  the wiring-driven remap in `xiao_nrf54lm20a-pinctrl.dtsi` (i2c22, 400 kHz).
- OLED VCC = header **3V3_OUT** (see power table), GND = header GND. Most 0.91"
  I2C OLED breakouts carry their own SDA/SCL pull-ups — no external resistors.

### IMU — LSM6DS3 family on IIC0
- `IMU_SCL = P0.07`, `IMU_SDA = P0.08`, `IMU_INT1 = P0.06`, `IMU_CS = P3.12`
  (CS tied to select I2C mode). Block diagram: IMU on **IIC0**.
- **Part number to CONFIRM at M3:** Seeed docs indicate **LSM6DS3TR-C**
  (≠ the nRF52840 Sense's LSM6DSL). Different Zephyr `compatible`
  (`st,lsm6dso`-family vs `st,lsm6dsl`). The Mahony/orientation math on top is
  driver-agnostic; only the DT node + driver Kconfig change. Verify the exact
  part + on-chip tap-engine register map before porting the tap/pose paths.

### PDM mic
- `MIC_CLK = P1.13`, `MIC_DAT = P1.14`. (nRF52840 Sense used different pins —
  remap `mic_vad` pinctrl.) Powered by the IMU&MIC_3V3 LDO rail (M3).

### Buttons / LEDs / radio / storage
- **User button:** `P0.09 / USR_KEY`.
- **RGB LED:** `Red = P1.22`, `Blue = P1.23`, `Green = P1.24` (active-low
  typical on XIAO — verify polarity).
- **NFC:** `NFC1 = P1.01`, `NFC2 = P1.02`.
- **32.768 kHz LFXO crystal:** `P1.20 / P1.21` (XL1/XL2).
- **8 MB QSPI flash:** P2.00–P2.05 (`SPI_IO0..3`, `SPI_CLK`, `SPI_CS`).
- **RF switch control (antenna):** `RF_SW_PWR = P1.27`, `RF_SW_CTL = P1.28`
  (cf. nrf54l15 template `rfsw_ctl`/`rfsw_pwr` regulator hogs — port the same).

## HR/PPG (future OLED rev — NOT populated/used in v1)
- OLED **v1 ships without HR.** MAX30102 PPG + `WearableDSP` + BLE HRS are
  compiled out behind `CONFIG_GESTUREBAND_HR` (default `n` on this board).
- Confirm the MAX30102 footprint/pins on this board when the HR rev is scoped;
  the schematic reviewed here does not populate a PPG sensor.

## Open hardware-verification items (resolve on-device, not by assumption)
1. ~~Is header **3V3_OUT always-on or GPIO-gated**?~~ **RESOLVED 2026-08-26:**
   effectively always-on — the OLED powers from the header 3V3 with **no** GPIO
   enable in the board def (M1 OLED came up fine). No DCDC_EN hog needed.
2. Exact **IMU part**: **CONFIRMED LSM6DS3TR-C** (Seeed spec + upstream Zephyr
   board). Still verify I2C address (0x6A/0x6B) + tap-engine registers vs the
   nRF52840's LSM6DSL at M3.
3. **IMU&MIC_3V3 enable mechanism**: PMIC LDO1 via TWI vs. `P0.01` GPIO enable
   (xlsx shows both in draft rows) (M3).
4. RGB LED **polarity** (active-high vs -low).

## OLED wiring + driver (VERIFIED 2026-08-26)
- SSD1306 128×32 at I2C **0x3C** on `xiao_i2c` (i2c22): **SCL=D10/P1.06,
  SDA=D9/P1.05** — four consecutive right-edge pads (GND·3V3·D10·D9) for a
  GND/VCC/SCK/SDA module. Confirmed working.
- The I2C controller node needs **`zephyr,concat-buf-size = <1088>`** — CFB
  writes the full 512 B framebuffer + 1 cmd byte in one transfer and the nRF
  TWIM otherwise errors "internal buffer insufficient (1 + 512 > 16)".
