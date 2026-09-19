# Firmware resilience playbook

Rules and code for making a device survive its own hardware faults. Written
after the 2026-09-05 failure and intended to be reused on **every** future
board, not just this one.

Implementation lives in `src/resilience.{h,cpp}`, `src/battery.{h,cpp}`,
`src/led_status.{h,cpp}`, `src/button_gesture.{h,cpp}` (host-tested in
`tests/test_button_gesture.cpp`). Reference wiring: `apps/nrf54_button/`.

---

## 1. The incident — and its actual cause

A device that had run for two weeks stopped advertising. It was power-cycling
every 1.09s: `ssd1306` init failed at 514ms and the board reset before `main()`
ran, so `bt_enable()` was never reached.

**We chased the display for hours. It was never the cause.** The real cause was
a hand-written board definition that declared no PMIC, so nothing ever set
`charging-enable` and the battery had never been charged since the day it was
fitted. It drained to 2.53V, sagged the system rail under the LDO1 always-on
load, and the board browned out ~514ms into every boot. The display error was
simply the last thing printed before the supply collapsed.

Two things made this take an evening instead of a minute. **Nothing recorded why
the board restarted** — no reset-cause logging. And the effective config had no
watchdog and no `RESET_ON_FATAL_ERROR`, which meant software *could not* have
rebooted it (a crash halts), so the reset had to be hardware — a deduction that
was available immediately and that we reached late.

It ended with a dead board and a dead cell. Everything below exists so that the
next one is a log line instead.

## 2. The two-failure model — internalise this

Every field failure is really two failures, and they need different responses.

| | What it is | Response |
|---|---|---|
| **A. The fault** | A part degraded: cell aged, joint cracked, connector fatigued | Manage the *rate*. You cannot eliminate it. |
| **B. The amplification** | That one part took down the **whole device** | **Prevent entirely. This is a firmware defect.** |

Failure A across a fleet is a certainty. Failure B is what turns a warranty
line item into a returned unit and a customer who thinks the product is broken.

**A display failing must degrade to "no display", never to "no device".**

## 2a. Never hand-write a board definition — this cost us a battery and a board

The single most expensive bug in this project was not in application code. Our
out-of-tree board definition for the XIAO nRF54LM20A **declared no PMIC at all**.
The nPM1300's charger is disabled after power-on reset, and the vendor's own
board dtsi sets `charging-enable` precisely because firmware must turn it on.

We never had that node. **The battery was therefore never charged — for two
weeks.** It drained to 2.53V, sagged the system rail under the LDO1 always-on
load, and the board brownout-looped at 1.09s. The `ssd1306` error at 514ms that
we chased for hours was a *downstream symptom* of the supply collapsing.

A missing devicetree node presents as a hardware fault. It has no compile error,
no runtime warning, and no log line. Nothing tells you the charger is off.

**Rules that follow:**

1. **Use the vendor's published board definition.** Seeed's lives in
   `Seeed-Studio/platform-seeedboards` under `zephyr/boards/arm/<board>/`.
   Fetch it verbatim. Do not transcribe it, and do not write your own.
2. **If the vendor's board needs adapting to your SDK version, patch only the
   version-specific parts and say so in the file.** Ours needed two: an SoC
   dtsi filename and a pair of Kconfig symbol names. Neither changes a hardware
   description. Both are commented in place and revert on an SDK upgrade.
3. **Your overlay should contain only hardware YOU added.** After the migration
   ours is 30 lines: one external button, and enabling the watchdog instance.
4. **Verify it landed.** `grep` the generated `build/zephyr/zephyr.dts` — it
   annotates every property with the file and line it came from. That is how you
   prove `charging-enable` is actually present rather than assuming it.

## 2b. Hardware facts vs policy — label which is which

Every constant in firmware is one of two things, and conflating them is how
invented numbers end up looking authoritative.

**Hardware facts** — register bitmasks, pin numbers, charge parameters, voltage
ranges. These have exactly one correct value and it lives in the vendor's code
or datasheet. **Never guess one.** We guessed the nPM1300 charge-status bit
positions and got all four wrong, so the reported charge state was meaningless
for as long as it existed. The real masks (`COMPLETE`=BIT(1), `TRICKLE`=BIT(2),
`CC`=BIT(3), `CV`=BIT(4)) are in Nordic's own sample.

**Policy** — watchdog timeout, safe-mode threshold, debounce window, blink
cadence, low-battery warning level. These have no authoritative source; they are
design choices. Label them `POLICY` with the reasoning, so the next reader
argues with the choice instead of trusting it as a spec.

If you cannot name the source of a hardware constant, you do not have the
constant — you have a guess wearing a `#define`.

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

**Safe mode must also clear the counter — or it is a trap.** Our first version
only scheduled the healthy-uptime clear outside safe mode. Nothing else ever
cleared it, and **reflashing does not erase NVS**, so a device that tripped safe
mode stayed there permanently, even with fixed firmware. Correct behaviour: once
the minimal image has stayed up for the healthy window, clear the counter so the
next boot retries the full firmware. If it is still broken it returns to safe
mode after N more boots — a bounded retry, not a trap. (User-forced safe mode
clears immediately, since the user asked for it.)

**A debugger counts as failed boots.** Flashing over SWD halts the core while a
previous image's watchdog is still running, so each `probe-rs` flash produced
up to two `WATCHDOG` resets on the nRF54LM20A. Two quick flashes can approach
the threshold. The self-clear above makes that harmless.

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

Firmware cannot restart a dead MCU, so the button must also reach a hardware
power path. On nPM1300 that is **SHPHLD**.

**Measure SHPHLD's idle voltage before wiring it to anything.** On the XIAO
nRF54LM20A it measures **4.9V** with USB connected (and would sit at VBAT
unplugged) -- well above the nRF54's 3.3V rail. **A direct GPIO connection would
damage the pin.** There is no operating condition where the direct tap is safe.

A resistor divider is a poor fix: SHPHLD's pull-up is INTERNAL to the
nPM1300 (the Seeed schematic shows no external resistor on pin D4), its value
is not documented to us, and a divider's idle level depends on it. The diode
below does not care what the pull-up value is.

> Correction: an earlier version of this section said the pull-up was a 470K
> resistor, R10. The schematic shows R10 (470K) sits on VSET2, not SHPHLD. That
> was an inference from component proximity in the KiCad file, not a trace.

**One Schottky diode does solve it**, and keeps a single button:

```
        +---------------- SHPHLD --o/ o-- GND
        |                          (button)
   GPIO --|>|-- SHPHLD        anode at GPIO, cathode at SHPHLD
```

* **Open:** SHPHLD sits high; the diode is reverse-biased and blocks it. The
  GPIO reads high from its own internal pull-up.
* **Pressed:** SHPHLD goes to 0V, the diode conducts, and the pull-up current
  drains through it, pulling the GPIO to about one diode drop. Firmware sees the
  press; the PMIC sees SHPHLD low and power-cycles on a long hold.

Use a **Schottky**, not a silicon diode. Typical MCU VIL is 0.3 x VDD (0.99V at
3.3V); a silicon drop of 0.7V clears that by only 0.29V, a Schottky's ~0.25V by
nearly a volt. Confirm VIL in the SoC datasheet before committing.

If you would rather add no parts: skip SHPHLD entirely. Watchdog covers hangs,
the boot counter covers boot loops, and USB covers a flat battery or ship mode.
SHPHLD only covers an MCU wedged past its own watchdog -- rare once one is armed.

### 7.3 Gesture map for a single-button product

With one button, everyday and rare functions compete for the same gesture space.
Do not resolve that with longer presses -- **use context as a second axis.**

**A long-press primary action and a hold-duration escalation ladder cannot
coexist.** If press-and-hold starts a voice command, a long question will cross
any "hold 10s to restart" threshold. This is a real bug, not a corner case.

Everyday, works unplugged:

| Gesture | Action |
|---|---|
| press & hold | primary action; **any duration, no timeouts** |
| double tap | secondary toggle |
| single tap | reserved |

Maintenance -- gated on **USB connected**, entered by **double-tap-then-hold**
(a normal press starts with a hold, so it can never reach this):

| Held | Action |
|---|---|
| 5s | pairing mode |
| 10s | restart |

Boot-time only: button held while plugging in -> safe mode; N failed boots ->
safe mode automatically.

**Report a hold only after a confirm window** (~200ms of continuous contact).
At the instant of contact a tap and a hold are identical, so firing immediately
makes every tap open and close a session -- and a double tap fires the primary
action twice.

**What should NOT be reachable by gesture in a production device:** ship mode
(the device looks dead; put it in the companion app and enter it automatically
after N days idle) and factory reset (data loss behind a physical gesture earns
one-star reviews -- app only). Safe mode should be automatic via the boot
counter, with the manual hold-at-boot kept as an undocumented support
instruction rather than a user-facing feature.

Implementation: `src/button_gesture.{h,cpp}` -- a pure state machine with no
Zephyr dependency, so the timing rules are **host-tested**
(`tests/test_button_gesture.cpp`) rather than asserted.

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

## 9. Battery: charge it, protect it, report it honestly

**The nPM1300 charger is disabled after power-on reset. Firmware must enable
it.** The vendor's board dtsi sets `charging-enable` for exactly that reason.

> Correction: an earlier version of this section said "most boards charge
> correctly with no firmware running at all ... omit `charging-enable`". That is
> **wrong** for this PMIC, and it is the advice that let the first cell drain for
> two weeks. Do not omit it.

Rules:

1. **Charging config comes from the vendor's board definition** — fetched, not
   written (§2a).
2. **Cell-specific values are overridden in the app overlay** — charge current
   and discharge limit belong to *your* cell, not to the board (§9.4).
3. **Protect against over-discharge in firmware** — the PMIC stops over-charge
   in hardware, but nothing stops a device draining its own cell flat (§9.5).
4. **Never report a fault as healthy**, and never report a number you cannot
   defend (§9.2, §9.3).

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

### 9.2 Do not report a state-of-charge you cannot defend

A trustworthy battery percentage needs `nrf_fuel_gauge` **plus a `battery_model`
characterising the specific cell**, generated with Nordic's Battery Model
Characterizer. Vendor-supplied models describe the vendor's cell, not yours.

Without that model there is no honest percentage. A hand-written voltage curve
is invented precision — voltage-derived SoC is worth about ±10% under load even
when the curve is good, and worse while charging.

So: report **voltage, current, temperature and the charger's own status**. All
are real measurements. Omit the Bluetooth Battery Service rather than populate
it with a fabricated number, and add it when the cell has been characterised.

### 9.3 Cell temperature when the board has no thermistor

Charging a LiPo safely needs to know the cell's temperature. Heat is what turns
a charging fault into a fire, and JEITA-style charging (reduce or stop current
when cold or warm) depends on it.

**How production devices handle it -- mostly by not skipping it.** Wearables
either buy a **3-wire battery pack** with the thermistor built in, or place a
**10K NTC on the PCB pressed against the cell**, wired to the charger's NTC pin.
It is a few cents, and it is also what certification testing (IEC 62133 /
UL 2054 class) expects to see. For a production design: put it in the battery
spec or the PCB, not in firmware.

**What stacks on top of it, and what a prototype without one can lean on:**

1. **Low charge rate.** Resistive heating scales with current squared, so 0.5C
   produces a quarter of the I^2*R heat of 1C. This is the biggest lever you
   have in firmware alone.
2. **The cell's own protection circuit (PCM)** -- the small board under the tape
   at the leads. It cuts off over-charge, over-discharge, over-current and short
   circuit. It generally does **not** sense temperature.
3. **Charger die temperature as a proxy.** The PMIC sits at the battery input and
   can inhibit charging above a die-temperature threshold. It measures the chip,
   not the cell, so it catches board-level heat, not a cell heating on its own.
4. **Never report a fixed-resistor reading as a temperature.** A value that is
   always 25.0 C looks healthy and is not a measurement.

**Retrofit for a prototype:** replace the fixed resistor on the charger's NTC
pin with a real 10K thermistor glued to the cell, matching the configured beta
(Seeed's config is 10K / 3380). The firmware config already expects exactly
that part.

### 9.4 Size charge and discharge current to the cell

Vendor board defaults are sized for *a* cell, not yours. The XIAO nRF54LM20A
ships `current-microamp = 150000` and `dischg-limit-microamp = 1000000`. On a
120 mAh cell those are **1.25C** charge and **8.3C** discharge.

* **Charge current:** small-LiPo datasheets typically give 0.5C as the standard
  charge rate and 1C as the maximum. Use 0.5C when the device charges unattended
  or has no cell thermistor — heating scales with current squared. 120 mAh ->
  60 mA.
* **Discharge limit:** caps **total** battery current — board plus everything
  hanging off it. On nPM1300 the only options are 200 mA or 1000 mA; pick the
  one nearest the cell's rating.
* **Measure real load, do not estimate it.** The PMIC reports battery current.
  Fire the motor / radio / whatever peaks, and read it.
* Put the override in the **app overlay**, next to a comment naming the cell.
  If the cell changes, those lines must change with it.

Always check the allowed values in the binding
(`zephyr/dts/bindings/sensor/nordic,npm1300-charger.yaml`): charge current is
32-800 mA in 2 mA steps; discharge limit is an enum.

### 9.5 Over-discharge protection: ship mode

The PMIC prevents **over**charge in hardware. **Nothing prevents a device from
draining its own cell to death** while unplugged — the firmware keeps
advertising until the cell is flat. That is how the first cell reached 2.53V.

The pattern:

* On battery only, below a cutoff for N consecutive readings -> enter **ship
  mode** (everything off, including the MCU). On nPM1300, external power wakes
  it again, so the user recovery is simply "plug it in".
* Use the vendor call, not a raw register write:
  `regulator_parent_ship_mode()` on the nPM13xx regulator device — the same call
  Nordic's `samples/zephyr/drivers/regulator/ship_mode` uses. On success it does
  not return.
* **Never ship while USB is present.** It keeps the board flashable, and the
  PMIC will not stay in ship mode with external power anyway (the command
  returns 0 and the chip wakes straight back up).
* **Never ship on a failed read.** Require a valid voltage reading; if the
  USB-present read fails, assume USB is present.
* **Confirm over several readings** so a momentary sag (radio burst) cannot shut
  the device down.
* **Record the reason in NVS before shipping.** Waking from ship mode is a
  power-on reset — `RESETREAS = 0`, identical to a brownout. Without the note,
  the next boot reports exactly the ambiguity that cost us an evening.
* **Run it in safe mode too.** The PMIC is not an optional peripheral; skipping
  battery protection when the device is unhealthy is backwards.
* Cutoff is **policy**. We use 3.30V: it leaves charge in the cell so weeks in
  ship mode do not walk it toward the ~3.0V floor.

### 9.6 Detecting USB: read the documented bit, not the register

Over-discharge protection is only as good as its "is USB present?" signal. Ours
was wrong twice, and the cutoff silently never fired.

**Wrong 1 — `status != 0`** on nPM1300 `VBUSIN.VBUSINSTATUS` (0x07). This is
what Seeed's own battery example does. The register holds **six independent
flags**, and one stays set with no USB, so the check is never false. The board
ran on battery through three 68-104s unplugs without shutting down.

**Wrong 2 — the CC-detect register** (`VBUSIN.USBCDETECTSTATUS`, 0x05), which
Nordic's `npm13xx_fuel_gauge` sample uses. It reads the USB-C CC comparators.
**Seeed terminates the connector's CC lines with their own 5.1K resistors and
does not route them to the PMIC**, so on this board it is always zero — it
reported "no USB" while plugged in.

**Right — bit 0, `VBUSINPRESENT`**, from Nordic's register header
(`nordicsemi/npmx`, `adk/npm1300.h`):

| bit | VBUSINSTATUS field |
|---|---|
| **0** | **VBUSINPRESENT** |
| 1 | VBUSINCURRLIMACTIVE |
| 2 | VBUSINOVRPROTACTIVE |
| 3 | VBUSINUNDERVOLTAGE |
| 4 | VBUSINSUSPENDMODEACTIVE |
| 5 | VBUSINVBUSOUTACTIVE |

Measured plugged in: `0x21` = present + VBUSOUT active.

**Lessons that generalise:**
* A vendor example is not a specification. Seeed's was wrong here.
* A reference sample assumes the reference board's wiring. Nordic's was right
  for their EK and wrong for Seeed's schematic.
* **A status register is a set of flags.** Never test it with `!= 0`; find the
  named bit in the vendor's register header.

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
5. **Prove protection paths that rarely fire.** A low-battery cutoff that only
   triggers at 3.3V is code, not proof, until it has fired. Build a temporary
   variant with the threshold **above the current value** (we used 4.5V), then
   perform the triggering action (unplug) and check every link: detection,
   confirmation, action, recovery, logged reason. This is how we found the
   USB-detection bug — the protection had never worked. Mark the temporary value
   `TEMP TEST -- restore to X` and grep for it before shipping.
6. **Uptime tells you whether power was really lost.** On the nRF54L the GRTC
   keeps counting across a *soft* reset, so log timestamps continue. A timestamp
   that restarts from zero means the MCU genuinely lost power. That distinguished
   "ship mode worked" from "it rebooted" without extra instrumentation.
7. **Change one thing.** Our replacement firmware removed the display *and* the
   PMIC *and* the IMU *and* the mic — so "it's fixed" identified nothing.
   A controlled bisect is worth the extra flash cycle.

## 11. Checklist for any new board

- [ ] Board definition is the **vendor's**, fetched verbatim, not hand-written
- [ ] Every hardware constant traceable to vendor code/datasheet; policy constants labelled `POLICY`
- [ ] `charging-enable` (or equivalent) verified present in the **generated** devicetree
- [ ] Reset cause read, cleared, logged on boot, exposed over the wire
- [ ] Boot counter in **non-volatile** storage → safe mode at threshold
- [ ] Watchdog armed, fed from the main loop
- [ ] Comms up **before** any optional peripheral
- [ ] No optional peripheral at `POST_KERNEL`; all binds guarded
- [ ] Status LED with a distinct safe-mode pattern, live in safe mode
- [ ] Recovery button on a **hardware** power path (SHPHLD or equivalent)
- [ ] If that button is shared with a GPIO: idle voltage measured and <= VDD (§7.2)
- [ ] USB power alone brings the device up with a dead/absent cell
- [ ] Charging **enabled** via the vendor board definition; verified in generated DTS
- [ ] Charge current and discharge limit sized to **your** cell, in the app overlay
- [ ] "USB present" read from the **documented bit**, not a whole-register `!= 0`
- [ ] Over-discharge cutoff -> ship mode, **proven on hardware** with a temp-threshold build
- [ ] Safe mode clears its own counter after a healthy run
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
| SHPHLD | Labelled pad on the **underside**, next to a GND pad. nPM1300 pin D4, **no external pull-up** (internal to the PMIC). **Measured 4.9V idle on USB** — never tie it directly to a GPIO (§7.2) |
| NTC / cell temperature | **None.** nPM1300 pin D3 (NTC) goes to **R6, a fixed 10K 1% resistor to GND**. With Seeed's 10K NTC config it reads exactly 25.0 C forever. Do not report it. The PMIC **die** temperature (`SENSOR_CHAN_DIE_TEMP`) is real and sits at the battery input — use it as a proxy |
| Power rails | VSYS_3V3 (BUCK2) max **200mA**, powers nRF54 + SAMD11 debugger. LDO1 (IMU & mic) max **100mA**. Header 3V3 pin is a **separate DC-DC** (U1) max **600mA**, fed from VSYS — anything on it still draws through the battery discharge limit |
| RGB LED | R **P1.22**, G **P1.24**, B **P1.23**, all **active-low** |
| Onboard button | **P0.09**, already the `sw0` alias in the board dtsi — do not shadow it from an app overlay |
| Watchdog | `wdt31`, already aliased `watchdog0`; needs `status = "okay"` in the overlay |
| Storage | `storage_partition`, 32KB at `0x1dd000` — used for the NVS boot counter |
| `hwinfo` | Gated to `SOC_SERIES_NRF54HX \|\| NRF_SOC_SECURE_SUPPORTED`, so **not** available. Read `NRF_RESET` directly via `hal/nrf_reset.h` |
| Charger driver | `nordic,npm1300-charger` requires `dischg-limit-microamp` as well as the obvious properties |
| Board definition | **Use Seeed's**: `Seeed-Studio/platform-seeedboards` → `zephyr/boards/arm/xiao_nrf54lm20a/`. Two local patches needed on NCS 3.2.3, both commented in-file: SoC dtsi `nrf54lm20a_cpuapp.dtsi` → `nrf54lm20a_enga_cpuapp.dtsi`, and Kconfig `SOC_NRF54LM20A_CPUAPP` → `..._ENGA_CPUAPP` (engineering-A silicon naming) |
| Node labels | LEDs are `red_led` / `green_led` / `blue_led`; `led0`=**blue**; onboard button `button0` (`sw0`); IMU `lsm6ds3tr_c` |
| PMIC bus | `pmic_i2c`, **gpio-i2c bitbang** on P1.18/P1.17 — not a hardware TWIM instance |
| USB present | `VBUSINSTATUS` bit 0 (`VBUSINPRESENT`). Reads `0x21` plugged in. **Not** `!= 0`, and **not** the CC-detect register — USB-C CC lines are not routed to the PMIC on this board (§9.6) |
| Ship mode | `regulator_parent_ship_mode()` on `DT_PARENT(DT_NODELABEL(vsys_3v3))`. **Proven 2026-09-19:** cuts all power, USB wakes it. Returns 0 but does not hold with USB present |
| Flashing side effect | Each `probe-rs` flash can cause up to two `WATCHDOG` resets while the core is halted — each counts toward the safe-mode threshold |
| Uptime across resets | GRTC keeps counting through soft resets; a timestamp restarting at 0 means real power loss |
| Our cell | 120 mAh LiPo: charge overridden to 60 mA (0.5C), discharge limit to 200 mA — in `apps/nrf54_button/app.overlay` |
