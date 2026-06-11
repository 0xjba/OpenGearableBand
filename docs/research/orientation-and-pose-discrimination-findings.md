# Orientation Estimation & Pose Discrimination — Findings

> **⚠️ PARTIALLY SUPERSEDED (2026-06-11).** The **roll-based dictation
> discriminator** documented below (roll = atan2(gy,gz), threshold 85°) was a
> **WRONG TURN** — it reads the palm twist, which is in the gravity blind spot
> and contradicts the settled `decision_dictation_voice_gated_entry`
> (dictation = bring-to-face geometry + voice, NOT the twist). It mislabeled a
> steady near-vertical air-mouse pose as DICTATION. The corrected direction —
> an observability-aware classifier (roll-validity cone) + bring-to-face
> geometry + voice — is in
> [`observability-aware-pose-and-cursor-design.md`](observability-aware-pose-and-cursor-design.md).
> The orientation-foundation, gyro-integral-failure, and bias-measurement
> sections below remain valid; only the roll-split *conclusion* is retracted.

**Date:** 2026-06-10
**Status:** Orientation foundation valid; **roll-split conclusion retracted**
(see banner). Cursor pointing deferred.
**Scope:** Everything from the gyro-based addition (when the cursor spec
was handed over) through to the validated orientation foundation +
roll-based dictation discriminator.

Companion docs:
- [`cursor-drift-mitigation.md`](cursor-drift-mitigation.md) — the user's drift-free-cursor spec (the reason we built the orientation foundation).
- [`../superpowers/specs/2026-06-10-gesture-trigger-redesign-design.md`](../superpowers/specs/2026-06-10-gesture-trigger-redesign-design.md) — pose-gated trigger design.
- Memories: `project_orientation_foundation_2026_06_10`, `decision_dictation_voice_gated_entry_2026_06_10`, `hardware_wear_position`.

---

## 1. What we settled on (TL;DR)

1. **AIR_MOUSE and DICTATION are one "raised" pose**, split by **ROLL**
   (= `atan2(gy, gz)`), which is **gravity-derived and drift-free**.
   Threshold 85°: roll < 85° → AIR_MOUSE (palm to screen), roll ≥ 85°
   → DICTATION (palm supinated to face). Validated: settled air-mouse
   ~30–64°, settled dictation ~105–117°.
2. **The raised cone** is centred on the forearm axis `[1,0,0]` with
   tolerance 0.45 (arms within ~43.5°), wide enough to catch *inclined*
   raises (~38° off-axis) while rejecting NEUTRAL (~48°+) and SURFACE
   (~80°).
3. **A shared IMU orientation foundation** was built (Mahony filter +
   ZARU gyro-bias tracking + stillness detection + yaw re-zero) — it
   validated the roll discriminator, steadies pose detection, and is the
   base for the future drift-free cursor.
4. **Voice gating** is the chosen *future* backstop for the residual
   air-mouse/dictation overlap in the "blurry middle" (tilted poses).
5. **Snap-vs-tap gesture-variant discrimination** is NOT done here; it's
   separate future work (mic + IMU fusion).

---

## 2. The problem

On a wrist band (6-axis IMU, **no magnetometer**), distinguishing an
AIR_MOUSE raise (palm toward screen) from a DICTATION pose (palm
supinated toward the face) looked impossible at first:

- The two poses' **static gravity vectors overlap**. An air-mouse
  max-right lean and a dictation pose measured only **~4° apart** in the
  raw gravity direction.
- The distinguishing motion — **supination (palm flip)** — is a rotation
  *about the forearm axis*. When the arm is raised, the forearm axis is
  nearly aligned with gravity, so the flip is rotation *about gravity =
  yaw*, which an **accelerometer cannot sense** (it only measures the
  direction of "down"). This is the accelerometer's well-documented yaw
  blind spot. [1][2]

This led to an initial (wrong) decision to **merge** the two poses,
concluding "gravity can't separate them."

---

## 3. The empirical journey (three attempts)

### Attempt 1 — simple per-axis gyro integral. FAILED.

Hypothesis: the supination flip is a roll about the forearm axis (band
X), so the **net integral of gyro-X** over the move-into-pose would be
large for a dictation flip and small for an air-mouse raise.

Research backed the *idea*: gyroscope angular-velocity signatures do
distinguish pronation/supination, wrist flexion/extension, finger-snap,
etc. [3]

But hardware data killed the *implementation*:

| Gesture | net_X (deg) | net_Y (deg) |
|---|---|---|
| Air-mouse raises (×10) | −4 … +24 | all negative |
| Dictation flip *from lap* | **+2** | +61 |
| Dictation flip *from air-mouse* | **+24** | −46 |

`net_X` for dictation flips (+2, +24) sat **inside** the air-mouse range;
`net_Y` had **opposite signs for the same gesture by different paths**.

**Root cause (as the user predicted):** 3D rotations don't commute, so
integrating *body-frame* angular velocity over a natural *compound*
raise-and-twist is **path-dependent** — it doesn't decompose into a
clean per-axis "supination angle." People don't move mechanically
(raise → pause → flip); they raise-while-twisting in one motion, and the
integral smears across axes.

### Attempt 2 — full orientation tracking (Mahony filter). SUCCEEDED, via ROLL.

We built a Mahony complementary filter (accel+gyro → quaternion →
pitch/roll/yaw) to capture the **end orientation** (path-independent)
rather than a path-dependent integral. [1][2]

Two things fell out:

**(a) The yaw-as-discriminator idea backfired.** We auto-re-zero yaw at
rest (a ZUPT analogue, correct for the *cursor*). But that *erased* the
yaw signal the instant a pose settled — dictation yaw read 81° mid-flip
but **0° once held**. Yaw is the wrong signal for a *static* pose
discriminator.

**(b) ROLL was the answer all along — and it's drift-free.** Because the
raised forearm is **not** perfectly vertical (pitch ≈ −68°, so ~22° off
gravity), the supination has a component *perpendicular* to gravity that
the accelerometer **can** see, surfacing as Euler **roll**. Roll is
gravity-locked → drift-free → stable even when the pose is held:

| Pose (settled, `at_rest=1`) | pitch | **roll** |
|---|---|---|
| Air-mouse raise | −69 | **31** |
| Air-mouse raise | −68 | **59** |
| Dictation flip | −57 | **111** |
| Dictation flip | −66 | **117** |

~50° clean gap. Equivalent simple form: `roll = atan2(gy, gz)` from the
gravity-LPF (no quaternion needed for the discriminator itself — the
filter just revealed it).

### Correction of the over-generalization

"Gravity can't separate them" was **only true for the worst case** (an
air-mouse *max-right lean*, roll ~105°, which collides with dictation).
For air-mouse as actually performed (palm to screen, roll 30–64°),
gravity *does* separate it from dictation by ~50°. The initial merge was
concluded from the worst case, not the typical case.

### Attempt 3 — cone tuning for inclined poses

After re-splitting by roll, hardware testing showed inclined raises
("wrist not straight up", ~38° off the forearm axis) sometimes failed to
arm — the raised cone was too tight (armed within ~37°). Widened the
tolerance 0.60 → 0.45 (arms within ~43.5°), which catches inclined raises
while still rejecting NEUTRAL (~48°+) and SURFACE (~80°). Result: inclined
poses now arm reliably.

---

## 4. Final design

### Pose classification (in `gesture_mode.cpp` / `gesture_poses.cpp`)

- **SURFACE**: gravity-cosine canonical, Z-dominant `[0.18, 0.08, 0.98]`,
  tolerance 0.94 (±20°).
- **Raised hemisphere**: canonical `[1,0,0]` (forearm axis), tolerance
  0.45 (arms within ~43.5°). Covers straight + forward/left/right/inclined.
- **AIR_MOUSE vs DICTATION split**: within the raised hemisphere, compute
  `roll = atan2(gy, gz)`. `roll ≥ 85°` → DICTATION (positive high roll =
  supination toward face); otherwise AIR_MOUSE. Signed threshold — a left
  lean is negative roll and stays air-mouse.

### Orientation foundation (`orientation.{h,cpp}`)

A Mahony complementary filter fusing accel (gravity → drift-free
pitch/roll) and gyro (all axes; yaw drifts on a 6-axis IMU). Plus, from
the cursor spec [4]:
- **ZARU gyro-bias tracking** (the filter's integral term) — estimates
  and subtracts the gyro bias, hardest-converging when still. [8]
- **Stillness detection** — accel residual + gyro magnitude below
  threshold for a 300 ms dwell, gating on **both** linear and angular
  quietness. [9]
- **Yaw re-zero at rest** (ZUPT analogue) — bounds yaw drift to a single
  continuous motion. [7][9]

### Mode entry (two-step, in the commit handler)

Pose selects the mode; a **cadenced double-tap** (inter-tap 60–500 ms,
tuned to this user) confirms intent. Mode-transition is currently
LOG_INF-only (wire-up to the power state machine is future work F2).

---

## 5. Empirical data

### Roll separation (settled poses, drift-free)

- Air-mouse (palm to screen): roll **30–64°** (Z positive).
- Dictation (palm to face): roll **105–117°** clean, **88–103°** when
  inclined (Z near/below 0).
- Threshold 85° sits in the gap for clean poses.

### Gyro bias (this LSM6DSL unit, warm, two 6000-sample stationary traces)

| Axis | bias mean (dps) | noise σ (dps, clean trace) |
|---|---|---|
| X | +0.66 | ~0.77* |
| Y | **−1.59** | 0.16 |
| Z | −0.58 | 0.18 |

Two traces agreed to **within 0.04 dps** — stable and repeatable.
(*X σ inflated by slight motion; min/max excursions in the 2nd trace,
e.g. X −12.7 dps, were desk bumps, not noise — use σ.) Implication:
after ZARU subtracts the ~1.6 dps bias, residual ≈ 0.2 dps → ~0.4° yaw
drift over a 2 s motion (a few pixels), wiped at each stillness re-zero.
This is per-unit; production needs per-unit calibration, but the adaptive
ZARU means no value is hardcoded.

---

## 6. Known limitations

1. **Blurry-middle pose overlap.** *Tilted* air-mouse and *inclined*
   dictation are geometrically similar in gravity (X≈Y, Z≈0) and can
   mis-classify (~1 in 10, near the roll-85° boundary). Clean poses
   classify reliably. NOT tuned away, because tightening the threshold
   would re-break inclined dictation. Resolved later by **voice gating**.
2. **No absolute yaw** (no magnetometer). Yaw is gyro-only; "drift-free"
   is *practical* (stillness re-zero), not mathematical. Long, slow,
   pauseless sweeps can accumulate visible yaw drift (cursor concern).
3. **Mahony bias winds up during motion.** The Ki integral absorbs some
   linear acceleration as "bias" during active motion (observed Y bias
   creeping to ~4 dps mid-session); it self-settles when still. The `z`
   stationary trace gives the true bias. Freezing Ki during high motion
   is a future tuning option.
4. **All thresholds are device/mount/user-specific** — derived from this
   unit's logs, not datasheet defaults. Per-user calibration is a
   productionization item.

---

## 7. Future work

- **F2** — wire mode entry to the power state machine (LOG_INF → real).
- **Cursor pointing** — pitch→vertical, yaw→horizontal, with the
  ZARU/ZUPT drift mitigation; foundation is ready (see cursor spec).
- **Voice-gated dictation** — band-at-lips mic signal as the dictation
  confirmer; resolves the blurry-middle overlap.
- **Ki-freeze-during-motion** — prevent the bias estimate winding up.
- **Cold bias trace** — characterize cold→warm bias swing (post-recharge
  cold start); reminder pending for next session.
- **Per-unit / per-user calibration** of canonicals, roll threshold,
  cadence window, bias, stillness thresholds.

---

## 8. References

**Orientation filtering**
1. Madgwick orientation filter — AHRS docs (accel gives roll/pitch only; gyro-derived yaw drifts; needs a constant heading reference). <https://ahrs.readthedocs.io/en/latest/filters/madgwick.html>
2. S. O. H. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays." <https://www.researchgate.net/publication/361578937>

**Gyroscope gesture discrimination**
3. Gyroscope-Based Continuous Human Hand Gesture Recognition (MDPI Sensors 19(11):2562, 2019) — angular-velocity signatures distinguish pronation/supination, wrist flexion/extension, finger-snap, etc. <https://www.mdpi.com/1424-8220/19/11/2562>

**Accelerometer yaw blind spot / sensor fusion**
4. (User spec) `cursor-drift-mitigation.md`, this repo.
5. Accelerometer, Gyroscope and Magnetometer — Ericco Inertial (yaw cannot be estimated by accelerometers; only the gyroscope helps about the gravity axis, but drifts; magnetometer compensates). <https://www.ericcointernational.com/info/accelerometer-gyroscope-and-magnetometer.html>
6. Magnetometer-Based Drift Correction During Rest in IMU Arm Motion Tracking (PMC6471153) — 6-axis gyro-integrated heading drifts; rest-based correction. <https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6471153/>

**Zero-velocity / zero-angular-rate updates (drift mitigation)**
7. Wahdan et al., ZUPT-aided pedestrian inertial navigation — resetting at detected stationary intervals bounds integration drift. <https://www.researchgate.net/publication/353898852>
8. Nazari et al., survey describing ZARU — a motionless gyro should read zero; any reading is bias to remove. <https://arxiv.org/pdf/1906.05917>
9. Wahlström & Skog, "Fifteen Years of Progress at Zero Velocity" — survey of zero-velocity detection / drift correction; gate on genuine stationarity. <https://arxiv.org/pdf/2008.09208>

**Hardware (no magnetometer)**
10. Adafruit LSM6DS3TR-C — 6-DoF IMU, no magnetometer. <https://www.adafruit.com/product/4503>
11. ST LSM6DS3 datasheet / AN4650. <https://cdn.sparkfun.com/assets/learn_tutorials/4/1/6/AN4650_DM00157511.pdf>
12. Seeed XIAO nRF52840 Sense — nRF52840 BLE + onboard 6-axis LSM6DS3 + PDM mic. <https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html>
