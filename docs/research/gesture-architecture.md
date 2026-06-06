# Gesture detection architecture and roadmap

**Purpose:** definitive reference for the gesture-detection feature
track.  Captures the three product use cases (surface touchpad, air
mouse, advanced gestures), the unification insight that they share a
single underlying engine, verified hardware capabilities, the staged
implementation roadmap with realistic effort estimates, and the
research that grounds each decision.

**Status:** research and architecture complete; implementation has
not started.  Update this doc as we learn from real hardware behavior
during build-out.

> TODOs to resolve during implementation, not blocking architecture:
> - Measure actual gyro drift on LSM6DS3TR-C at our 104 Hz ODR.
>   Generic published range is 0.1-1 °/s; we need the actual number
>   for cursor-quality estimates.
> - Validate Zephyr HOGP profile on iOS specifically.  Nordic devzone
>   thread (referenced) flags some HID-on-iOS edge cases worth a
>   bring-up sanity check before committing to HID for the air-mouse
>   click protocol.
> - Confirm Edge Impulse Cortex-M4F (FPU) targeting -- documentation
>   typically lists "Cortex-M4" without explicit FPU mention.  Our
>   chip is M4F.
> - Specific feature-engineering choices for PPG gesture features
>   (Zhao 2018 details, windowing, derivative orders) to be locked
>   down before Stage 5 model training.

---

## TL;DR

Three product use cases (surface touchpad, air mouse, advanced
gestures) appear independent but share the same underlying engine.
Build one **unified gesture pipeline** with mode-aware output routing
rather than three separate implementations.

**Implementation order:** foundation (mode detector + HID profile +
cursor scaffolding) → chip-embedded gestures → air mouse v1 (IMU-only
pinch via Edge Impulse) → surface mode → custom macro gestures → PPG-
fused pinch (Apple-tier) → broader gesture vocabulary.  **Total to
ship all three use cases at usable quality: 8-12 weeks** with Claude
writing code + Edge Impulse handling the ML pipeline.

**Long pole:** labelled gesture data collection (Edge Impulse mobile
recorder, ~30 min per gesture for a usable model).  Not code, not
infrastructure.

---

## 1. The three product use cases

### Use Case 1: Surface touchpad

Wrist resting on a desk surface.  Trigger gesture enters mode.
Wrist motion glides cursor on the surface.  Single tap on surface
acts as right click, double tap as left click.  Wrist roll / tilt
performs scroll.

### Use Case 2: Air mouse

Wrist raised in air.  Trigger gesture enters mode.  Wrist orientation
+ motion controls cursor.  Index + thumb pinch performs left click;
double pinch performs right click.  Wrist roll performs scroll.

### Use Case 3: Apple-tier advanced gestures

Wrist in natural position.  No mode trigger needed -- gestures act
as ambient input.  Pinch, double pinch, clench, flick, finger-specific
taps detected via PPG-perceived tendon perturbations + IMU micro-
movements.  Each gesture maps to a phone-side action.

---

## 2. The unification insight

Looked at independently, these are three different builds.  Looked at
in terms of their underlying primitives, they are one engine with
three output bindings.

### Shared building blocks

| Primitive | Surface mode | Air mouse | Advanced gestures |
|---|---|---|---|
| Mode / orientation detector | enters surface mode | enters air mode | enters always-on |
| Trigger gesture (wake) | ✓ | ✓ | optional |
| Chip-embedded tap engine | retuned for desk impact | for fallback | for tap-on-band |
| Cursor pipeline (gyro integration + drift correction + HID) | ✓ surface tracking | ✓ air tracking | — |
| Wrist-roll → scroll | ✓ | ✓ | optional |
| Pinch classifier (ML on IMU + PPG) | optional disambiguation | ✓ click events | ✓ primary input |
| PPG-derived gesture features | optional | improves pinch | required |

### Architecture diagram

```
                IMU 100Hz + PPG 100Hz
                          |
                COMMON FEATURE EXTRACTION
              (windows, peaks, AC amplitude,
                  chip-embedded events)
                          |
              +-----------+----------+
              |                      |
         MODE DETECTOR          GESTURE CLASSIFIER
       (wrist orientation)      (Edge Impulse model,
                                  multi-class output)
              |                      |
              +-----------+----------+
                          |
                    ACTION ROUTER
       (mode-dependent binding of gesture events
        to BLE HID / BLE custom char / app msg)
                          |
              +-----------+----------+
              |           |           |
        CURSOR HID    CLICK HID   GESTURE EVT
       (mouse move)  (mouse btn)  (custom char)
```

**Build the foundation once.  Each use case is one output binding.**

---

## 3. Hardware capability check

Verified against datasheets and Zephyr documentation, not assumed.

### LSM6DS3TR-C (the IMU we have)

**Chip-embedded events that fire INT1 / INT2 pins for "free":**

- Single-tap recognition
- Double-tap recognition
- Wake-up detection
- Significant motion detection (we use this for workout wake)
- Free-fall detection
- 6D orientation detection
- Activity recognition (basic; LSM6DSV and LSM6DSOX have a full
  ML core, LSM6DS3TR-C does not)
- Pedometer (step detector + step counter -- chip accumulates,
  we read the register)

**What we don't get (would require newer chip):**

- Machine Learning Core (decision trees on-chip)
- Multiple programmable Finite State Machines for custom gestures
  (LSM6DSV has 16 FSM slots; LSM6DS3TR-C is a simpler chip)

**Verdict:** chip handles the foundation events natively.  We don't
need MCU code for tap, double-tap, wake-up, or step counting.
Anything more sophisticated (pinch, flick, finger-specific gestures)
runs on the Cortex-M4 with our own / Edge Impulse code.

### MAX30102 (the PPG sensor we have)

**Relevant for gestures:**

- IR LED + photodiode, single channel
- We sample at 100 Hz (driver config)
- 100 Hz is sufficient for PPG-feature-driven gesture detection
  (published wrist-PPG gesture papers use 50-200 Hz)
- Red LED unused -- could add SpO2 separately, not gesture-relevant

**Limitations:**

- Single channel -- can't do multi-PD spatial fusion
- Single wavelength -- can't do green/IR motion-artifact suppression
- Apple uses green for the same gesture role; we're slightly worse
  off but not blocked.  The PPG signal for gestures is the
  tendon/blood-flow signature -- IR sees this fine.

### nRF52840 + Zephyr BLE HID

**Verified:**

- Zephyr ships a `peripheral_hids_mouse` sample on nRF52840
- NCS includes both `peripheral_hids_mouse` and `peripheral_hids_
  keyboard` samples in production-quality form
- HID-over-GATT (HOGP) profile is standardized at the BLE-spec level
- Host compatibility: Windows / Linux / Mac / Android well-tested;
  iOS works but with some pairing-mode caveats (see TODO marker)

**Verdict:** we can implement the air-mouse / surface-touchpad cursor
+ click via standard BLE HID.  No proprietary protocol needed.

**Bonus:** because it's standard HID, the band works as a mouse for
*any* connected host without an app.  The phone app is only needed
for the advanced-gesture-to-action mapping in Use Case 3.

### Cortex-M4F + Edge Impulse / TFLite Micro

**Verified by reference deployments:**

- Typical 6-axis IMU gesture-recognition model: **20-40 KB** flash,
  similar order in working RAM
- With Edge Impulse EON Compiler + int8 quantization: significantly
  smaller (often <10 KB working RAM)
- Reference platforms include Cortex-M4 / M4F class chips with 256 KB
  RAM and 1 MB flash -- our exact configuration

**Our current footprint:**

- FLASH: 290 KB / 788 KB (37 %)
- RAM: 86 KB / 256 KB (34 %)

**Budget for gesture additions:** ~498 KB flash + ~170 KB RAM
headroom.  A 50 KB model + feature buffers + classifier scaffolding
fits comfortably with room for at least 2 more major subsystems.

**Verdict:** ML deployment is feasible.  No hardware blocker.

### Power budget

Per-component current draw, continuous mode:

| Component | Current | Source |
|---|---|---|
| nRF52840 active + BLE | ~3-5 mA | varies with radio scheduling |
| LSM6DS3TR-C continuous combo | ~0.9 mA | datasheet |
| MAX30102 PPG continuous | ~0.6 mA | datasheet |
| **Total active gesture mode** | **~5-7 mA** | |

Implications:

- For a typical 200-500 mAh coin / LiPo battery, **40-100 hours of
  continuous gesture-mode runtime** before recharge
- That's plenty for **triggered or duty-cycled** gesture mode (e.g.
  "active for 60 s after raise-to-wake")
- That's marginal for **always-on background gesture listening**
  (Use Case 3's ambient-input model).  Without duty cycling, battery
  drains in under 2 days even on a 500 mAh cell

**Architectural implication:** Use Cases 1 and 2 should be explicitly
triggered.  Use Case 3 needs a "smart-on" model where IMU listens
continuously for the trigger pattern (chip-embedded events, no MCU
wake) and the full PPG + ML pipeline only spins up after a candidate
gesture is detected.

---

## 4. Use Case 1 detail -- surface touchpad

### Closest reference: Tap Strap 2's Mouse Mode

Tap Strap 2 enters Mouse Mode automatically when the user's thumb
rests on a surface.  It uses an **optical mouse sensor on the thumb
ring** for cursor tracking -- not the IMU.

### What this means for us

We don't have an optical sensor.  Wrist-only IMU on a surface gives:

- **Cursor X/Y via gyro integration** -- jittery; gyro drift will be
  visible.  Aggressive zero-velocity update (detect wrist-still
  intervals and reset velocity) is the standard mitigation.
- **Surface tap detection** -- the LSM6 tap engine can be retuned for
  the impulse signature of a finger tapping the desk near the band
  (vs the chip's default of tap-on-device).  Empirical threshold tune.

### Realistic capability

| Sub-feature | Achievable | Quality |
|---|---|---|
| Mode entry (wrist-flat detection) | ✓ via accel gravity vector | Reliable |
| Cursor tracking | ✓ via gyro + ZVU | **Jittery** -- usable for slow cursor, frustrating for fine targeting |
| Single tap → right click | ✓ via LSM6 tap engine, retuned | Good after threshold tuning |
| Double tap → left click | ✓ via LSM6 double-tap engine | Good |
| Scroll | ✓ via wrist roll | Coarse but workable |

### Honest assessment

This is the **weakest fit for our hardware** of the three.  Tap Strap
2's surface mode works because of the optical sensor; ours won't
match.

**Recommended reframe:** call it "wrist remote on a surface" rather
than "real touchpad."  Optimize for **discrete gestures** (taps,
wrist-flick directions) rather than continuous cursor control.  This
fits the hardware much better and avoids user-expectation mismatch.

---

## 5. Use Case 2 detail -- air mouse

### Closest references

- **Doublepoint Wisp** (now Oura): IMU-only smartwatch detection of
  pinch + ray-cast pointing.  Documented in CHI 2022 paper "Enabling
  Hand Gesture Customization on Wrist-Worn Devices."
- **Tap Strap 2's Air Mouse Mode**: cursor + finger taps.  Uses
  multiple sensors across the hand.
- **WiiMote**: 6-axis IMU for cursor was the original proof of concept
  in 2006.

### What's achievable

| Sub-feature | Achievable | Quality | Approach |
|---|---|---|---|
| Mode entry | ✓ raise-to-wake + dwell | Reliable | Gravity vector + IMU still-detection |
| Cursor X/Y | ✓ gyro integration with gravity correction | **Smooth for sessions <30 s**, drift over longer use | Complementary filter or Madgwick |
| Pinch click (index + thumb) | ✓ IMU-only via Edge Impulse | **82-88 %** per published research | Train a small CNN/MLP on accel + gyro windows |
| Pinch click + PPG fusion | ✓ add PPG features | **88-94 %** | Same model with PPG features added |
| Double pinch | ✓ timing layer | ~80 % if single pinch is 90 % | Single-pinch detector + temporal classifier |
| Scroll | ✓ wrist roll | Coarse but usable | Roll angular rate → scroll steps |

### Why pinch detection works on IMU alone

Per Doublepoint and the PLOS One 2020 paper on multimodal hand
gesture recognition at the wrist:

> "Muscles and tendons at different locations of the forearm compress
> the arteries with different pressure and duration when performing
> a gesture."

The tendon recoil from a finger pinch propagates a small but
detectable signature to the wrist's accel + gyro.  IMU-only catches
~82 %; adding PPG (where the tendon-perfusion event also shows up)
lifts this to Apple-tier numbers.

### Honest assessment

This is the **best fit for our hardware** of the three.  We have the
exact sensor stack Apple Watch S9 uses for Double Tap, minus the
dedicated Neural Engine.  Without per-user fine-tuning we won't hit
Apple's "feels magical" reliability, but mid-80s to low-90s pinch
accuracy is achievable.

---

## 6. Use Case 3 detail -- Apple-tier advanced gestures

### Underlying physics

Per multiple peer-reviewed sources (Zhao 2018 UTK; Frontiers
Neuroscience 2022; multimodal PPG-FMG-ACC papers):

- Muscles and tendons compress arteries during finger movements
- PPG sees these as small blood-flow perturbations
- Different gestures produce different temporal patterns
- IMU sees the wrist micro-movements that accompany
- Combined signal, trained with ML, achieves usable classification

### Gesture vocabulary feasibility

| Gesture | Difficulty | Expected accuracy with our setup |
|---|---|---|
| Clench (full fist) | Easiest -- large tendon perturbation | ~92-95 % |
| Pinch (index + thumb) | Medium -- Apple Double-Tap class | ~85-92 % |
| Double pinch | Medium-hard | ~75-85 % |
| Single-finger flick | Hard -- small motion, brief signal | ~70-80 % |
| Specific-finger discrimination (which finger tapped) | Hardest -- inter-finger discrimination wants more sensor info than we have | ~60-70 %, may not be useful in practice |

### Why we'll be below Apple's accuracy

Apple advantages we lack:

- **Neural Engine** dedicated ML hardware (~10x our raw inference
  budget)
- **Hundreds of thousands of labelled gestures** for training
- **Per-user online fine-tuning** that improves accuracy over time
- **Multi-PD spatial fusion** on some Apple Watch generations

We compensate with:

- Same fundamental sensor stack (PPG + accel + gyro)
- Edge Impulse handling model design + training + Cortex-M4 codegen
- User can collect their own training data via Edge Impulse's mobile
  recorder
- Per-user fine-tune flow is buildable -- it's just a fine-tune pass
  on a base model

### Honest assessment

We can ship a 2-3 gesture vocabulary (pinch, double pinch, clench)
at usable accuracy.  Aiming for a richer Apple-tier vocabulary is
months of additional work and may never reach Apple parity without
hardware changes.

---

## 7. Implementation roadmap

Cumulative effort assumes each row builds on every row above.
Effort numbers reflect Claude writing C / C++ + Edge Impulse handling
the ML pipeline + user collecting training data with Edge Impulse's
mobile recorder.

| # | Item | Difficulty | Cumulative effort | Built using | Realistic accuracy |
|---|---|---|---|---|---|
| **0** | **Foundation:** mode detector, BLE HID profile (HOGP), cursor pipeline scaffolding | LOW | **3-5 days** | Zephyr `peripheral_hids_mouse` sample + custom mode FSM | n/a -- infrastructure |
| **1** | **Chip-embedded gestures** (tap-on-band, double-tap, wake, free-fall, sig-motion routing) | LOW | **+2-3 days** | LSM6DS3TR-C INT1 + register config | 95 %+ -- chip native |
| **2** | **Air mouse cursor + IMU-only pinch click** (Doublepoint-class) | MEDIUM | **+2-3 weeks** | Gyro integration + Edge Impulse pinch model | Cursor smooth <30 s; pinch 82-88 % |
| **3** | **Surface touchpad mode** (wrist on desk + LSM6 retuned tap + cursor reuse) | LOW-MEDIUM | **+3-5 days** | Reuses #0 cursor + retuned tap thresholds | Cursor jittery; clicks reliable |
| **4** | **Custom macro gestures** (wrist flick, twist, shake) | LOW-MEDIUM | **+3-5 days** | MCU-side threshold + duration classifier | ~90 % with tuning |
| **5** | **PPG-fused pinch** (Apple-tier approach) | MEDIUM-HIGH | **+2-3 weeks** | Add PPG features to Edge Impulse model + retrain | 88-94 % pinch (lift from #2) |
| **6** | **Multi-gesture vocabulary** (clench, double-pinch, finger flick, ...) | HIGH | **+3-6 weeks** | Edge Impulse broader gesture set + per-user fine-tune | 75-92 % varies by gesture |

**Total to ship all three use cases at usable quality: 8-12 weeks.**

### Phase milestones

- **End of Week 1**: foundation + chip gestures shipping.  Band acts
  as a basic HID with tap controls.
- **End of Week 3**: items 0-3 shipping.  All three use cases have v1
  functionality at usable-but-not-great quality.  This is the
  earliest "demoable" point.
- **End of Week 5**: items 4-5 shipping.  Pinch accuracy at Apple-
  tier numbers via PPG fusion.  Macro gestures available.
- **End of Week 12**: item 6 shipping.  Broader gesture vocabulary
  trained against real user data.

### Long pole: data collection

The technical work is fast.  The bottleneck is **labelled training
data**.  Realistic minimums:

- **Per gesture, basic model:** ~200-500 samples, ~30 min collection
- **Per gesture, excellent model:** ~1000-2000 samples, ~2 hours
- **Multi-user generalization:** multiply by user count

Edge Impulse's mobile recorder app collects + labels samples by
having the user perform the gesture on cue.  No special equipment.
Plan for ~30 minutes of focused data collection per gesture per phase.

---

## 8. Power budget

### Per-mode estimates

| Mode | Avg current | Battery life on 200 mAh | Battery life on 500 mAh |
|---|---|---|---|
| IDLE (current production) | ~50-100 µA | months | months |
| SNAPSHOT (current production, 22 s every 2 min) | ~250 µA avg | ~33 days | ~83 days |
| WORKOUT (current production) | ~5-7 mA | ~30 hours | ~75 hours |
| **Gesture mode active (continuous)** | ~5-7 mA | ~30 hours | ~75 hours |
| **Gesture mode triggered (5 min sessions)** | ~150-300 µA avg | weeks | weeks |
| **Gesture-chip-only (waiting for trigger)** | ~150 µA | months | months |

### Implications

- **Continuous always-on gesture detection is impractical** on coin-
  cell batteries.  500 mAh LiPo gives ~3 days at best.
- **Triggered gesture mode is fine.**  User performs raise-to-wake +
  air-mouse gesture, mode runs for 60-300 s, returns to chip-trigger
  listening.
- **Use Case 3 (ambient gestures) requires a duty-cycled architecture:**
  - Chip-embedded events (tap, sig-motion) listen continuously at
    ~150 µA on the LSM6 alone.  MCU sleeps.
  - When chip fires an event, MCU wakes, PPG enables, ML model runs.
  - Detection takes <1 s.  Result published, MCU returns to sleep.
  - **Average current: a few hundred µA**, battery acceptable.

### Architectural requirement

The mode router must support **"chip-armed sleep"**: gesture pipeline
suspended, LSM6 chip-embedded events still firing INT1, MCU wakes
only on event.  This is similar to our existing SNAPSHOT-IDLE state-
machine architecture and reuses the same power-control primitives
(`max30102_shutdown` etc.).

---

## 9. Implementation order and dependencies

```
[0] Foundation
      |
      +-> [1] Chip gestures
      |
      +-> [2] Air mouse + IMU pinch ---> [5] PPG-fused pinch
      |                                       |
      +-> [3] Surface mode (reuses [0] cursor + [1] tap)
      |
      +-> [4] Macro gestures (reuses [0] mode FSM)
                                              |
                                              v
                                  [6] Broader gesture set
```

Dependencies:

- **Item 0 unblocks everything else.**  Build it first.
- **Items 1, 2, 3, 4 are parallelizable** after item 0.
- **Item 5 needs item 2 working** (extends the same model).
- **Item 6 needs items 2 + 5** (extends the same model further).

Items 1, 3, 4 are each days of work and can be batched with item 0.

---

## 10. Known gaps to fill during implementation

Refinements that didn't block architecture but will need attention
during build-out:

### A. Gyro drift quantification

Generic literature says wrist gyros drift 0.1-1 °/s.  Our cursor-
quality estimates ("smooth <30 s") rely on this range.  **First
build-out task: characterize actual drift on our chip + ZVU
algorithm.**  Adjust cursor mode duration accordingly.

### B. PPG-feature engineering for gestures

Zhao 2018 (UTK) shows PPG-finger-gesture detection works at
~85-90 %, but the paper's specific feature engineering choices
(windowing, derivative orders, frequency bands) need to be extracted
and adapted to our 100 Hz IR signal before item 5 model training.

### C. iOS BLE HID validation

Zephyr HID samples work on Windows / Linux / Mac / Android.  iOS
support exists but has known pairing-mode caveats.  Validate with a
bring-up test before committing the air-mouse architecture to HID.
Fallback: custom GATT char + app-side translation.

### D. Latency budget for cursor / click feel

Haven't researched what latency is acceptable for cursor feel
(probably <50 ms) and click ack (probably <100 ms).  Validate against
real user testing during item 2 build-out.

### E. Edge Impulse Cortex-M4F (FPU) targeting

Documentation typically lists "Cortex-M4" without FPU detail.  Our
chip is M4F.  Verify FPU is utilized by EI codegen -- should give
meaningful inference speedup.

---

## 11. References

### Commercial reference products

- [Apple Watch double tap gesture available with watchOS 10.1 (Apple Newsroom)](https://www.apple.com/newsroom/2023/10/apple-watch-double-tap-gesture-now-available-with-watchos-10-1/)
- [Apple Watch Double Tap technical details (Medium / Michael Parekh)](https://medium.com/@mparekh/ai-apple-watchs-magical-double-tap-e6f20d71408f)
- [Apple Watch wrist flick gesture only works on newer models (Fahad X)](https://www.fahadx.com/posts/apple-watch-wrist-flick-gesture-only-works-on-newer-watch-models-for-a-reason)
- [Apple patent for next-gen EMG sensors (Patently Apple, 2025)](https://www.patentlyapple.com/2025/01/a-new-apple-patent-reveals-that-theyre-working-on-next-gen-emg-sensors-to-assist-apple-watch-gesture-technology.html)
- [Doublepoint product page](https://www.doublepoint.com/product)
- [Oura acquires Doublepoint (MobiHealthNews)](https://www.mobihealthnews.com/news/oura-acquires-doublepoint-gesture-recognition-technology-wearables)
- [Doublepoint XR gesture detection technology (Auganix)](https://www.auganix.org/xr-news-doublepoint-unveils-smartwatch-based-gesture-detection-technology-for-xr-device-control/)
- [Tap Strap 2 product page / how Tap works](https://www.tapwithus.com/how-tap-works/)
- [MouveMouse wearable motion-based mouse](https://mouvemouse.com/)

### Academic research

- [Enabling Hand Gesture Customization on Wrist-Worn Devices (CHI 2022 / ACM DL)](https://dl.acm.org/doi/fullHtml/10.1145/3491102.3501904)
- [Multimodal hand gesture recognition using single IMU and acoustic measurements at wrist (PLOS One 2020 / PMC 6957149)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6957149/)
- [WristSketcher: Creating Dynamic Sketches in AR with a Sensing Wristband (arXiv 2210.11674)](https://arxiv.org/pdf/2210.11674)
- [SurfaceXR: Fusing Smartwatch IMUs and Egocentric Hand Pose (arXiv 2603.19529)](https://arxiv.org/pdf/2603.19529)
- [PPG-based Finger-level Gesture Recognition Leveraging Wearables (Zhao 2018, IEEE)](https://ieeexplore.ieee.org/document/8486006/)
- [PPG-based Finger-level Gesture Recognition (Zhao 2018 UTK PDF)](https://web.eecs.utk.edu/~jliu/publications/zhao2018ppg.pdf)
- [Principal component analysis of PPG signals for improved gesture recognition (Frontiers Neurosci 2022)](https://www.frontiersin.org/journals/neuroscience/articles/10.3389/fnins.2022.1047070/full)
- [Gesture recognition of wrist motion based on wearables (Procedia 2022)](https://www.sciencedirect.com/science/article/pii/S1877050922015927/pdf)

### Hardware datasheets and verification

- [LSM6DS3TR-C datasheet (ST)](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)
- [LSM6DS3TR-C always-on application note (ST AN5130)](https://www.st.com/resource/en/application_note/an5130-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
- [LSM6DSOX machine learning core (ST AN5259)](https://www.st.com/resource/en/application_note/an5259-lsm6ds3trc-machine-learning-core-stmicroelectronics.pdf) -- comparison reference for what we lack
- [Bosch Sensortec IMUs for gesture control](https://www.bosch-sensortec.com/en/news/bosch-sensortec-imus-the-ultimate-solution-for-gesture-control-in-wearable-devices.html)

### BLE HID profile on nRF / Zephyr

- [HID Peripheral sample -- Zephyr Project Documentation](https://docs.zephyrproject.org/latest/samples/bluetooth/peripheral_hids/README.html)
- [Bluetooth: Peripheral HIDS mouse -- nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/samples/bluetooth/peripheral_hids_mouse/README.html)
- [BLE Peripheral HIDS mouse -- nRF52840 MDK USB Dongle](https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/guides/ncs/samples/ble/peripheral_hids_mouse/)
- [BLE mouse example for NRF52840 (denisgav GitHub)](https://github.com/denisgav/nrf52840_ble_mouse)
- [Nordic Q&A: HID mouse functionality with iOS on nRF52840 Dongle](https://devzone.nordicsemi.com/f/nordic-q-a/115855/inquiry-on-hid-mouse-functionality-with-ios-on-nordic-nrf52840-dongle)

### Edge Impulse / TinyML on Cortex-M4

- [Edge Impulse documentation -- Edge AI hardware overview](https://docs.edgeimpulse.com/docs/edge-ai-hardware/edge-ai-hardware)
- [Edge Impulse Inference performance documentation](https://docs.edgeimpulse.com/knowledge/metrics/inference-performance)
- [On-Device Gesture Recognition for Patient Communication (Edge Impulse blog)](https://www.edgeimpulse.com/blog/helping-patients-communicate-through-on-device-gesture-recognition/)
- [TinyML Made Easy: Gesture Recognition (Hackster.io)](https://www.hackster.io/mjrobot/tinyml-made-easy-gesture-recognition-ce13a5)
- [Gesture Recognition with Edge Impulse and Arduino (Leonardo Cavagnis / Medium)](https://leonardocavagnis.medium.com/gesture-recognition-with-edge-impulse-and-arduino-0da09c0873d5)
- [Gesture Recognition with the Thunderboard Sense 2 (Hackster.io)](https://www.hackster.io/gatoninja236/gesture-recognition-with-the-thunderboard-sense-2-d010ff)
- [ESP32-S3 Gesture Recognition with Edge Impulse (ForestHub)](https://www.foresthub.ai/resources/esp32-s3-gesture-recognition-edge-impulse)

### Power consumption references

- [nRF52840 Battery Life Estimation (Nordic devzone)](https://devzone.nordicsemi.com/f/nordic-q-a/63521/nrf52840-battery-life-estimation)
- [Current consumption of LSM6DS3 (ST community)](https://community.st.com/t5/mems-sensors/current-consumption-of-lsm6ds3/td-p/263934)
- [Optimizing Performance and Battery Life with the LSM6DS3TR in Portable Devices](https://www.ic-components.com/blog/optimizing-performance-and-battery-life-with-the-lsm6ds3tr-in-portable-devices.jsp)
- [More successful adventures in reducing nRF52840 power consumption (Tomas McGuinness)](https://tomasmcguinness.com/2025/01/02/more-adventures-in-nrf52840-power-consumption/)

### Cross-references

- See `hr-algorithm-decisions.md` for the parallel HR-algorithm
  decision doc.  Step counting and activity classification are parked
  in `software-optimization-roadmap.md` (Stage 5+ section).
- See `google-phrm-notes.md` for confidence-gated aggregation pattern
  -- some of the same UX patterns apply to gesture confidence
  thresholds.
