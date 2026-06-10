# Webcam-Free Drift-Mitigated Pointing

> **Source:** user design note (2026-06-10), saved verbatim as the reference
> spec for the air-mouse cursor subsystem.  We are NOT building the cursor
> pointing yet.  We ARE building the shared orientation foundation it needs
> (orientation filter + gyro-bias/ZARU + stillness detection + stillness
> re-zero), because the same machinery also gives us a path-independent
> dictation-vs-air-mouse discriminator and steadier pose detection.  See
> `docs/research/gesture-architecture.md` and the orientation-foundation
> work in `src/gesture_mode.cpp` / `src/orientation.*`.

*A self-contained gyro/IMU cursor solution for the Seeed XIAO nRF52840 Sense (OpenGearable)*

---

## 1. Scope and summary

This note specifies a drift-mitigation strategy for an air-mouse built on the XIAO nRF52840 Sense, with two hard constraints: **no magnetometer** (the onboard IMU is a 6-axis LSM6DS3) and **no webcam / external anchor**. Under these constraints, true mathematical drift-free pointing on all axes is not achievable, because the board carries no absolute heading reference. What *is* achievable is a system that feels drift-free in practice: one axis is genuinely drift-free, and the other is held in check by self-triggered correction requiring no button and no user gesture.

**Bottom line:** pitch is drift-free via gravity; yaw is gyro-integrated with live bias tracking and an automatic stillness re-zero (the wrist analogue of a zero-velocity update). Freezing the cursor while the hand is still makes the re-zero invisible. Residual drift only accumulates within a single continuous motion and is erased at every natural pause.

## 2. The drift problem

An orientation filter such as Madgwick or Mahony uses the accelerometer to measure the direction of gravity, which fixes pitch and roll against an absolute reference. Heading (yaw) has no such reference in a 6-axis device: it comes purely from integrating the gyroscope, and the documentation for these filters is explicit that gyro-derived heading drifts and a separate constant heading reference is required to hold it. [1] [2]

| Axis | Reference available? | Result |
|------|----------------------|--------|
| Pitch / Roll | Yes — gravity (accelerometer) | Absolute and drift-free. No correction needed. |
| Yaw | No — no magnetometer on this board | Gyro-integrated only; drifts unboundedly without an external fix. |

This asymmetry is the entire design problem. It is information-theoretic, not a tuning issue: no filter or model can recover an absolute heading that is not present in accelerometer + gyroscope data.

## 3. Hardware constraints

- **MCU / radio:** Nordic nRF52840, Bluetooth Low Energy. [6]
- **IMU:** LSM6DS3 — 3-axis accelerometer + 3-axis gyroscope, no magnetometer. [3] [4] [6]
- **Embedded IMU functions usable for free:** dual interrupt pins with wake-on-motion, tap detection and orientation/tilt events — useful for stillness and rest-pose flags without polling. [5]
- **Consequence:** yaw has no magnetic anchor; any absolute correction must come from motion structure (stillness / rest pose), not from a sensor.

## 4. The solution

Four components, all on-band, none requiring a button or magnetometer:

1. **Gravity-locked pitch (and roll).** Take pitch directly from the gravity-referenced orientation filter and map it to the vertical cursor axis. This axis is drift-free and needs no correction. [1]
2. **Gyro yaw with live bias tracking.** Map yaw to the horizontal axis. During low-motion windows, average the gyro output to estimate its bias and subtract it; this is the Zero Angular Rate Update principle — a motionless gyro should read zero, so any reading is bias/error to be removed. It sharply lowers the drift rate before any re-zero fires. [8]
3. **Stillness auto-rezero (ZUPT analogue).** When accelerometer variance and gyro magnitude both stay below threshold for a short dwell, declare "at rest" and re-zero yaw. This is the established zero-velocity-update mechanism, transplanted from the foot to the wrist: resetting at detected stationary intervals bounds the accumulation of integration drift. [7] [9]
4. **Freeze-on-stillness.** Move the cursor only when the hand is actively moving above a small motion threshold. The re-zero then happens while the cursor is parked, so corrections are invisible — no jump, no visible creep.

## 5. Expected behaviour and reliability

- **Hand still:** cursor does not move. Re-zero occurs while parked; the user never sees it.
- **Hand moving:** yaw drift accumulates only for the length of one continuous motion and is wiped at the next pause. Normal pointing has frequent micro-pauses, so a re-zero realistically fires every ~1–5 seconds of use.
- **Residual error per stretch ≈ (uncorrected bias) × (seconds since last pause).** After bias tracking this is small — on the order of a fraction of a degree of yaw over a typical 1–2 s motion, i.e. a few pixels, corrected at the next pause.
- **Known failure mode:** long, slow, continuous sweeps with no pause give the re-zero nothing to trigger on, so drift can build up visibly there. Mitigate with a deliberate recenter motion or a slow-decay yaw correction.

**Honesty note:** the exact residual bias — and how much it shifts as the band warms on the wrist — is specific to the individual LSM6DS3 unit. The figures above are order-of-magnitude, not measured. Log a 60-second stationary trace (warm and cold) to obtain the real numbers before trusting any threshold.

## 6. Rejected alternatives

| Approach | Why it was set aside | Verdict |
|----------|----------------------|---------|
| Add a magnetometer | Gives absolute yaw, but a desk is a worst-case magnetic environment (monitor, PC, steel frame). Heading becomes position-dependent; needs runtime distortion rejection that falls back to the gyro anyway. | Marginal value at a desk |
| Acoustic (mic + laptop speakers) | Proven to mm accuracy, but the nRF52840 PDM path very likely cannot capture the ~18–22 kHz ultrasonic band these methods rely on. | Likely blocked by mic bandwidth — verify |
| BLE radio to laptop as anchor | RSSI too noisy for cursor work; AoA / ranging need an antenna array or both-end support a stock laptop lacks. | Not feasible |
| MAX30102 (PPG) | A contact optical sensor at millimetre range; cannot see the laptop or perform desk-distance ranging. | Not applicable |

## 7. Optional desktop-side enhancement (AI)

Running a learned model on the host can lower the drift rate further — learned displacement priors let an IMU alone reach low-drift pose estimation, and learned filters can self-tune noise parameters. [10] Two caveats keep this an enhancement, not a cure. First, these methods cannot manufacture an absolute heading: even deep-network odometry concludes a globally consistent yaw needs an external absolute observation. [12] [13] Second, their accuracy leans on strong, repetitive human-motion priors (e.g. walking gait) that free-form arm pointing does not provide, and they often depend on an orientation estimate that itself can carry >20° error. [11] [14]

**Under-explored angle worth a prototype:** the desktop sees the user's own cursor corrections (overshoot-then-nudge-back), which are an implicit measurement of drift. Treating clickable UI targets as soft anchors could let a model estimate and cancel yaw drift online with no extra hardware. Promising but unvalidated — test before relying on it.

## 8. Implementation notes

- **All thresholds are empirical.** Stillness variance, dwell time, motion gate, and bias-window length are device- and mounting-specific. Derive them from your own still-vs-moving logs; do not use datasheet defaults.
- **Offload stillness/rest flags** to the LSM6DS3 embedded interrupts where possible to save power and MCU cycles. [5]
- **Re-zero only when truly stationary.** As in foot-mounted ZUPT, a too-loose stationary detector injects error; gate on both linear and angular quietness. [9]

## References

1. Madgwick orientation filter — AHRS documentation. <https://ahrs.readthedocs.io/en/latest/filters/madgwick.html>
2. S. O. H. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays". <https://www.researchgate.net/publication/361578937>
3. Adafruit LSM6DS3TR-C product page — 6-DoF IMU, no magnetometer. <https://www.adafruit.com/product/4503>
4. ST LSM6DS3 datasheet / AN4650. <https://cdn.sparkfun.com/assets/learn_tutorials/4/1/6/AN4650_DM00157511.pdf>
5. Axiometa LSM6DS3TR module — dual interrupt pins, tap, orientation-change. <https://www.axiometa.io/products/accelerometer-lsm6ds3>
6. Seeed XIAO nRF52840 Sense. <https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html>
7. Wahdan et al., ZUPT-aided pedestrian inertial navigation. <https://www.researchgate.net/publication/353898852>
8. Nazari et al., survey incl. ZARU (zero angular rate update). <https://arxiv.org/pdf/1906.05917>
9. Wahlström & Skog, "Fifteen Years of Progress at Zero Velocity". <https://arxiv.org/pdf/2008.09208>
10. Liu et al., "TLIO: Tight Learned Inertial Odometry". <https://arxiv.org/pdf/2007.01867>
11. Chen et al., "Deep Learning for Inertial Positioning: A Survey". <https://arxiv.org/pdf/2303.03757>
12. Wang et al., "DIDO". <https://arxiv.org/pdf/2203.03149>
13. Buchanan et al., "EqNIO". <https://arxiv.org/pdf/2408.06321>
14. Sun et al., "IDOL". <https://arxiv.org/pdf/2102.04024>
15. Wang & Gollakota, "MilliSonic". <https://arxiv.org/pdf/1901.06601>
16. Cao et al., "ReflecTrack". <https://pi.cs.tsinghua.edu.cn/wp-content/uploads/2021/11/ReflecTrack.pdf>
