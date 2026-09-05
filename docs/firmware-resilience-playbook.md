# Firmware resilience playbook

Rules and code for making a device survive its own hardware faults. Written
after the 2026-09-05 failure and intended to be reused on **every** future
board, not just this one.

Implementation lives in `src/resilience.{h,cpp}`, `src/battery.{h,cpp}`,
`src/led_status.{h,cpp}`. Reference wiring: `apps/nrf54_button/`.

---

## 1. The incident, in one paragraph

A device that had run for two weeks stopped advertising. It was power-cycling
every 1.09s: `ssd1306` init failed at 514ms, and the board reset before
`main()` ever ran, so `bt_enable()` was never reached. Diagnosis took an
evening because **nothing on the device recorded why it restarted**. The
firmware had no watchdog, no `RESET_ON_FATAL_ERROR`, and no reset-cause
logging — which also proved software could not have caused the reset at all
(with that config a crash *halts*, it does not reboot), so it had to be
hardware: brownout or PMIC power cycle. The battery later read **2.53V**,
below the LiPo floor.

## 2. The two-failure model — internalise this

Every field failure is really two failures, and they need different responses.

| | What it is | Response |
|---|---|---|
| **A. The fault** | A part degraded: cell aged, joint cracked, connector fatigued | Manage the *rate*. You cannot eliminate it. |
| **B. The amplification** | That one part took down the **whole device** | **Prevent entirely. This is a firmware defect.** |

Failure A across a fleet is a certainty. Failure B is what turns a warranty
line item into a returned unit and a customer who thinks the product is broken.

**A display failing must degrade to "no display", never to "no device".**

## 3. Boot order — the single most important rule

```
1. sample the recovery input (button held?)
2. reset cause + boot-loop check      <- before anything can fail
3. RADIO                              <- comms BEFORE anything optional
4. watchdog                           <- armed once reachable
5. optional peripherals               <- guarded, skipped in safe mode
```

**Comms come up before any optional peripheral.** In the incident the display
was ahead of `bt_enable()`, so a display problem cost the radio — the one
channel that could have reported the problem.

**Never init an optional peripheral at `POST_KERNEL`.** That runs before
`main()`, where you cannot guard failure. The app *did* have a correct guard
(`if (display_oled_init() == 0)`) and it never got to run. Bind optional
devices from the application, behind a readiness check.

## 4. Record why you rebooted

Without this you are blind, and you will spend an evening bisecting.

```c
uint32_t reas = nrf_reset_resetreas_get(NRF_RESET);
nrf_reset_resetreas_clear(NRF_RESET, reas);   /* sticky - clear or it accumulates */
```

| Value | Meaning |
|---|---|
| **0** | **power-on or BROWNOUT** — the supply went away |
| `RESETPIN` | pin reset |
| `DOG0`/`DOG1` | watchdog — you hung |
| `SREQ` | software reboot |
| `LOCKUP` | CPU lockup |

Zero is the important one: no bit set *is* the signal. Log it in the first line
of `main()` and expose it over the wire.

> Note: `hwinfo` is gated to `SOC_SERIES_NRF54HX || NRF_SOC_SECURE_SUPPORTED`,
> so on nRF54L read `NRF_RESET` directly via `hal/nrf_reset.h`.

## 5. Boot-loop detection → safe mode

Count consecutive boots that never reached a healthy uptime. Past a threshold,
boot a minimal image: **comms only, no optional hardware**, under a distinct
advertised name so an app can find it and reflash. This converts a brick into a
recoverable device.

**The counter MUST be non-volatile.** A brownout loop wipes RAM, so a
RAM-retained counter resets to zero every cycle and never trips — it defeats
the exact failure it exists to catch. Use NVS.

Cost is ~2 flash writes per boot cycle; NVS wear-levelling absorbs that easily.
Clear the counter after a healthy uptime that comfortably exceeds full
peripheral bring-up, or a late-boot crash will clear it and defeat the detector.

## 6. Watchdog

No watchdog means a hang is **permanent and invisible**. Add one deliberately
and pair it with §4, so a watchdog reset is diagnosable rather than mysterious.
Zephyr's `task_wdt` already sets `WDT_OPT_PAUSE_HALTED_BY_DBG`; expect
`reset=WATCHDOG` anyway when a debugger holds the core.

## 7. Recovery a non-technical user can perform

Nothing here requires knowledge, tools, or returning the device.

| User action | Mechanism |
|---|---|
| **Plug into USB-C** (charger or computer) | External power bypasses a failing cell — the most common "won't turn on". On a PMIC with ship mode, applying power also wakes it. |
| **Hold button while plugging in** | Forces safe mode |
| **Hold button ~10s** | Clean restart |
| (nothing) | Watchdog recovers a hang; next boot logs why |

**Firmware long-press cannot save a dead MCU.** For real robustness the button
must also reach a hardware power path — on nPM1300 that is the **SHPHLD** pin,
which power-cycles in silicon regardless of firmware state. Wire the button
between SHPHLD and GND, and tap SHPHLD to a GPIO so firmware can read the same
press. **Design this in at PCB stage** — it cannot be retrofitted in software.

### 7.1 USB power is your strongest recovery path

Plugging in does three useful things, and it needs no knowledge from the user:

1. **Power bypass.** VBUS runs the system regardless of the cell, so a flat or
   damaged battery stops being a brick.
2. **Ship-mode wake.** On a PMIC with a hibernate state, applying external power
   wakes it — "plug it in" genuinely un-bricks a shipped-state device.
3. **It powers the DFU window**, giving safe mode time to be reflashed.

What it **cannot** do is reset a wedged-but-running MCU. The watchdog covers
hangs; a hardware power path (§7.2) covers the rest.

Optional extra: firmware can read VBUS state from the PMIC, so **plug/unplug
three times in 30s** works as a no-button recovery gesture. It only works while
the device stays alive between unplugs, so it needs a functioning cell.

### 7.2 Wiring the hardware recovery button

The button must reach a hardware power path, because firmware cannot restart a
dead MCU. On nPM1300 that pin is **SHPHLD**.

**One button can serve both purposes** — general input *and* hardware recovery:

```
   button:  SHPHLD ----o/ o---- GND      (long press -> PMIC power-cycles)
   sense:   SHPHLD ------------- GPIO    (short press -> firmware reads it)
```

**Check one voltage before you solder this.** SHPHLD is an input with a
pull-up, and if that pull-up goes to VBAT or VSYS it can idle **above** the
MCU's VDD — tying it straight to a GPIO would stress the pin.

> Multimeter, SHPHLD pad to GND, powered, button not pressed:
> * **at or below VDD** -> safe to share. Then configure the GPIO with **no
>   internal pull-up**; the existing pull-up already holds the node, and two
>   fighting pull-ups is a bug waiting to happen.
> * **above VDD** -> do not connect directly. Use two separate switches, or add
>   a level shift.

The two-switch fallback usually costs nothing, because most dev boards already
have an onboard user button you can point firmware at.

Also verify the PMIC's **long-press timing and ship-mode behaviour** against its
datasheet before relying on it — it is a configuration item, not a given.

## 8. LEDs are the only zero-knowledge UI

A user cannot read a serial log. The LED is the difference between "it's
charging, leave it" and "send it back". Make the cadences distinguishable
across a room:

| State | Indication |
|---|---|
| Charging | red, slow breath |
| Charged | green, solid |
| Flat / no battery | red, one blink every 4s |
| **Safe mode** | red, fast blink — outranks everything |

Drive them from a bare GPIO, and bring them up **even in safe mode** — there is
nothing to fail, and it is the only way the user learns recovery is needed.

## 9. Battery: report, don't control

Read voltage/current/charge state from the PMIC. **Do not enable charging
control with guessed parameters.** Charge current, termination voltage and
thermistor settings are cell-specific; getting them wrong on a LiPo is a safety
issue, not a bug. Most boards charge correctly with no firmware running at all,
so there is nothing to gain. Omit `charging-enable`; add it only with the
cell's real capacity and NTC fitment confirmed.

Never report a fault as healthy. A cell below ~3.0V is flat or absent — say so.
Reporting 2.53V as "charged" hides exactly the fault you need to see.

Voltage-based SoC is only good to roughly ±10% under load. If the product needs
better, use a real coulomb-counting gauge; do not add decimal places to a curve
that cannot support them.

### 9.1 Diagnosing a suspect cell

**Do not confuse your own status LED with the hardware charge LED.** This cost
us a false conclusion. The charge LED is hardware-driven and **steady**; our
firmware's "flat battery" indication is a **short blink every 4s**. If the only
light is blinking on a 4-second beat, the charger is not charging.

**The test: does the voltage climb?** Log battery voltage over 20-30 minutes on
USB. A cell accepting a trickle charge rises steadily. One that sits flat is not
taking charge.

**Beware rounding noise.** Ours read 2.53 -> 2.54 -> 2.53 -> 2.54V over three
and a half minutes: oscillation at the 10mV rounding boundary, *not* a trend.
Two samples are never a trend — the first two looked like a rise and were not.
Log to at least 1mV, and require a monotonic climb over minutes.

**Below ~2.5V, replace rather than revive.** The anode's copper current
collector starts dissolving and re-plates as dendrites on recharge, which can
bridge internally. That is a short-circuit risk appearing *later*, not a
capacity complaint — and it is not a trade worth making for anything worn
against skin. A latched protection IC presents the same way and may recover,
but a cell that reached that voltage has lost capacity regardless.

**A cell sagging under load causes periodic brownout power-cycling** that looks
exactly like a firmware boot loop. §10 is how to tell them apart.

## 10. Diagnosing this class of failure

Techniques that actually found it, in order of value:

1. **Watch USB enumerate/de-enumerate with timestamps.** A metronomic period
   (ours: 1.09s ±0.01 over 23 cycles) is a hardware timer or power event.
   Software crash loops jitter, because enumeration timing varies.
2. **No fault dump ⇒ not a caught software fault.** Check the *effective*
   config (`build/zephyr/.config`), not `prj.conf`. With no watchdog and no
   `RESET_ON_FATAL_ERROR`, software *cannot* reboot the board — so it's hardware.
   This one deduction eliminated most of the hypothesis space.
3. **Count the bytes.** Our truncated log line was exactly 64 bytes — one USB
   packet. The device died within ~1ms of printing, which rules out a graceful path.
4. **Check build artifact timestamps against the symptom.** Ours was 10 days
   old with a clean tree: the firmware was bit-identical to what had worked, so
   the change was physical.
5. **Change one thing.** Our replacement firmware removed the display *and* the
   PMIC *and* the IMU *and* the mic — so "it's fixed" identified nothing.
   A controlled bisect is worth the extra flash cycle.

## 11. Checklist for any new board

- [ ] Reset cause read, cleared, logged on boot, exposed over the wire
- [ ] Boot counter in **non-volatile** storage → safe mode at threshold
- [ ] Watchdog armed, fed from the main loop
- [ ] Comms up **before** any optional peripheral
- [ ] No optional peripheral at `POST_KERNEL`; all binds guarded
- [ ] Status LED with a distinct safe-mode pattern, live in safe mode
- [ ] Recovery button on a **hardware** power path (SHPHLD or equivalent)
- [ ] If that button is shared with a GPIO: idle voltage measured and <= VDD (§7.2)
- [ ] USB power alone brings the device up with a dead/absent cell
- [ ] Battery reporting read-only unless cell parameters are confirmed
- [ ] Brownout detection enabled; bulk capacitance sized; rails current-limited
- [ ] Telemetry: reset cause + boot count + uptime reported to the host


---

## Appendix A — XIAO nRF54LM20A Sense, board facts

Hard-won specifics for this board, so they are not rediscovered.

| Thing | Detail |
|---|---|
| Board target | `xiao_nrf54lm20a/nrf54lm20a/cpuapp` (out-of-tree, `BOARD_ROOT` = repo). `full_name` is "XIAO nRF54LM20A Sense" — there is **no** `/sense` variant like the nRF52840 has |
| Flashing | **No UF2 bootloader.** `probe-rs download --chip nRF54LM20A --allow-erase-all --reset` (`./flash_nrf54.sh`) |
| `probe-rs reset` | Leaves the core **halted** — the board goes silent and looks dead. Use `--reset` on `download` to actually run firmware |
| Serial console | **Output only.** The CMSIS-DAP bridge does not carry host->device RX, so single-letter console commands never arrive. Do not rely on them for recovery here |
| Probe + console | The **same** USB device. If firmware reset-loops, the probe flaps too and flashing fails with "No connected probes were found" — just retry |
| PMIC | nPM1300, **onboard**, I2C `i2c21`: SDA **P1.18**, SCL **P1.17** |
| Battery pads | `BAT+` / `BAT-` on the **underside** |
| SHPHLD | Labelled pad on the **underside**, next to a GND pad. Schematic shows R10 470K adjacent with VSYS_3V3 nearby (likely a 3V3 pull-up — verify, see §7.2) |
| RGB LED | R **P1.22**, G **P1.24**, B **P1.23**, all **active-low** |
| Onboard button | **P0.09**, already the `sw0` alias in the board dtsi — do not shadow it from an app overlay |
| Watchdog | `wdt31`, already aliased `watchdog0`; needs `status = "okay"` in the overlay |
| Storage | `storage_partition`, 32KB at `0x1dd000` — used for the NVS boot counter |
| `hwinfo` | Gated to `SOC_SERIES_NRF54HX \|\| NRF_SOC_SECURE_SUPPORTED`, so **not** available. Read `NRF_RESET` directly via `hal/nrf_reset.h` |
| Charger driver | `nordic,npm1300-charger` requires `dischg-limit-microamp` as well as the obvious properties |
