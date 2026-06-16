#ifndef GESTURE_THRESHOLDS_H
#define GESTURE_THRESHOLDS_H

/*
 * ============================================================================
 *  gesture_thresholds.h — central catalog of empirically-tuned constants
 * ============================================================================
 *
 * Single home for the gesture / pose / orientation tuning constants, created
 * 2026-06-10 as productionization prep so re-tuning (when housing + a proper
 * strap arrive) is a one-file edit instead of hunting across source files.
 *
 * EVERY value here is empirical — derived from THIS unit, on an open PCB with
 * a duct-tape mount, for ONE user.  They are NOT datasheet defaults.  Each is
 * tagged with how it will need to change:
 *
 *   [HOUSING]    — will shift when a housing / proper strap is added (mechanical
 *                  coupling, contact pressure, how the band sits).  Re-measure.
 *   [USER]       — varies per user (gesture intensity, cadence, wrist range).
 *                  Target of the future per-user calibration ritual.
 *   [UNIT]       — varies per physical device (MEMS silicon).  Per-unit cal.
 *   [STRUCTURAL] — derived / fixed by the algorithm; do NOT tune to hardware.
 *
 * Productionization path (see project_productionization_gesture_calibration
 * memory): this header → NVS-backed runtime overrides → per-user/device
 * calibration ritual (companion app) → multi-user ML.  Re-measure with the
 * 'g' (gravity/pose), 'q' (PPG perfusion), and 'z' (gyro bias) serial
 * commands + the pose/cadence hardware tests.
 *
 * Findings & rationale: docs/research/gesture-architecture.md (§13 orientation
 * & pose-discrimination findings, §14 observability-aware pose).
 *
 * NOTE: a number of tunables physically cannot live in this C header
 * (Kconfig values, chip-register encodings, DSP-internal literals).  They are
 * CATALOGED at the bottom of this file with their real locations so this
 * header remains the single index of everything tunable.
 */

/* ---------------------------------------------------------------------------
 *  1. Pose canonicals (gravity direction per pose, band frame, unit vectors)
 *  ---------------------------------------------------------------------------
 *  Final mount: RIGHT wrist, VOLAR, THUMB-side.  Re-measured 2026-06-10 via
 *  the 'g' command.  tolerance_cos = min cos(angle-to-canonical) to match;
 *  higher = tighter.  cos(30°)=0.866, cos(20°)=0.940.  pose_score() uses these.
 *  [HOUSING][USER] — the band sits at a different fixed orientation in a
 *  housing, and users pose differently; re-measure all of these.
 */

/* AIR_MOUSE = the raised hemisphere (forearm axis).  Wide cone (tol 0.45 →
 * arms within ~43.5°) to catch inclined raises while rejecting NEUTRAL
 * (~48°+) and SURFACE (~80°).  The confirming gesture (double-tap) decides
 * the mode; roll-based splitting was removed 2026-06-11. */
#define POSE_AIRMOUSE_GX        1.0f
#define POSE_AIRMOUSE_GY        0.0f
#define POSE_AIRMOUSE_GZ        0.0f
#define POSE_AIRMOUSE_TOL       0.45f

/* DICTATION canonical is DISABLED for cosine matching (tol 2.0 → never
 * matches; cos can't exceed 1).  It was previously reached via a roll-based
 * split from POSE_AIR_MOUSE; that mechanism was removed 2026-06-11 (a held
 * max-right air-mouse is gravity-identical to dictation, so pose-only
 * discrimination is impossible — the confirming gesture decides the mode).
 * Values kept for future use when dictation entry is defined (clench + voice).
 * See 2026-06-11-pose-trigger-realignment spec. */
#define POSE_DICTATION_GX       0.92f
#define POSE_DICTATION_GY       0.39f
#define POSE_DICTATION_GZ       0.03f
#define POSE_DICTATION_TOL      2.0f

/* SURFACE = wrist flat on desk, volar up.  Z-dominant, slight +X lean.
 * Tighter cone (tol 0.94 ≈ ±20°) to reduce lap false positives. */
#define POSE_SURFACE_GX         0.18f
#define POSE_SURFACE_GY         0.08f
#define POSE_SURFACE_GZ         0.98f
#define POSE_SURFACE_TOL        0.940f

/* ---------------------------------------------------------------------------
 *  2. Pose FSM (arming)
 *  --------------------------------------------------------------------------- */

/* Pose-arm window: a matching gesture must arrive within this long of arming.
 * [USER] */
#define POSE_ARM_WINDOW_MS              3000

/* Minimum pose-match score (0..1) to count as "in a pose".  [STRUCTURAL] */
#define POSE_MATCH_THRESH               0.5f

/* (removed 2026-06-11) Roll-based AIR_MOUSE/DICTATION split deleted: a held
 * max-right air-mouse is gravity-identical to dictation, so pose-only
 * discrimination is impossible.  The confirming gesture decides the mode.
 * See 2026-06-11-pose-trigger-realignment spec. */

/* Gravity low-pass alpha (per-sample IIR) for the orientation/gravity
 * estimate used by pose detection, the cone gate, and `shadow`.  SLOW on
 * purpose -- stability is the feature there.  [STRUCTURAL] */
#define GRAVITY_LP_ALPHA                0.01f

/* Orientation classifier dwell, asymmetric (hysteresis, samples @100 Hz).
 * Entering a DEFINITE pose (UP_RAISED/DOWN_FLAT) is responsive (ENTER); a
 * NEUTRAL candidate is sticky (LEAVE) — a transient gx~=gy excursion mid
 * air-mouse sweep can't commit NEUTRAL while a definite pose is current.
 * LEAVE is the regression-tuned knob: raise it until the sweep log shows 0
 * spurious transitions.  [USER] */
#define ORIENTATION_ENTER_DWELL         15
#define ORIENTATION_LEAVE_DWELL         80

/* Axis-dominance ratio for the coarse orientation classifier.  [STRUCTURAL] */
#define DOMINANCE_RATIO                 1.3f

/* Raised-zone elevation ratio: UP_RAISED when gx (forearm axis, positive)
 * exceeds this * |gz| -- "arm up, not flat", IGNORING the gy SWEEP axis so a
 * wide air-mouse reach (gy large at the extremes) stays raised instead of
 * flapping to NEUTRAL (the 2026-06-11 sweep bug).  Part of the unified
 * gravity-component geometry (docs/research/observability-aware-pose-and-
 * cursor-design.md section 3.5).
 * EMPIRICAL: threshold = 1.1.  The observed MIN gx/|gz| across the raised
 * sweep was ~1.31 -- i.e. the sweep stays ~0.21 ABOVE the threshold; that gap
 * is the margin before a valid raised sweep would be clipped to not-raised.
 * Tune DOWNWARD only, against cross-session adversarial traces.  It is softer
 * than DOMINANCE_RATIO (1.3) on purpose: gy is already excluded here, so the
 * single gx-vs-|gz| test is geometrically tighter than the two-axis FLAT
 * check.  Regression test: 2026-06-11 sweep log -> 0 UP_RAISED<->NEUTRAL. [USER] */
#define RAISED_ELEVATION_RATIO          1.1f

/* ---------------------------------------------------------------------------
 *  3. Gesture / tap timing (cadenced double-tap, ringing, activity)
 *  --------------------------------------------------------------------------- */

/* Multi-tap accumulation window: the commit is rescheduled this long after
 * EACH tap, so a second tap must land within this window of the first to be
 * counted.  MUST be >= CADENCE_MAX_MS (enforced by BUILD_ASSERT in
 * gesture_mode.cpp) -- otherwise a valid-but-slow double-tap whose inter-tap
 * exceeds this window commits the FIRST tap alone (count=1) and is rejected,
 * even while perfectly still.  Was 250 (< the 500 cadence ceiling) -- raised
 * 2026-06-11 after hardware showed natural doubles at ~210-243 ms sitting
 * right at the old edge.  This is also the mode-entry latency after the last
 * tap (a future optimization could commit immediately once count==2 with
 * valid cadence, since triple-tap isn't a trigger).  [USER] */
#define MULTI_TAP_WINDOW_MS             550

/* Cadenced double-tap inter-tap interval [MIN, MAX] ms.  60 ms floor sits
 * above the 50 ms ringing refractory; 500 ms ceiling matches the slow-
 * deliberate range.  Lowered from 150 to 60 to match THIS user's fast
 * double-tap (measured 78-123 ms).  [USER] */
#define CADENCE_MIN_MS                  60
#define CADENCE_MAX_MS                  500

/* Ringing-rejection refractory: chip-tap events within this of the previous
 * one are dropped as mechanical-ringing duplicates.  Above the band's ~14 ms
 * ring, below the human inter-tap floor.  [HOUSING] (ring changes with
 * housing). */
#define RINGING_REFRACTORY_MS           50

/* Recent-gesture guard: while a pose is armed OR a chip-tap occurred within
 * this window, the significant-motion engine's WORKOUT_VERIFY transition is
 * SUPPRESSED — rapid gesture taps must not be mistaken for the sustained
 * motion of exercise (they otherwise preempt the gesture mid-sequence).
 * Covers the gesture sequence + margin; workout auto-detect only delays
 * during/just-after a gesture.  [USER] */
#define RECENT_GESTURE_GUARD_MS         3000

/* Activity gate (NO LONGER used in the trigger path — superseded by the pose
 * gate; kept for reference / possible in-session reuse).  Motion residual
 * above THRESH (m/s^2) resets the dwell; a tap was accepted only when the
 * dwell (samples @100 Hz) was reached.  [USER] */
#define ACTIVITY_GATE_THRESH            2.0f
#define ACTIVITY_GATE_DWELL             50

/* ---------------------------------------------------------------------------
 *  4. SURFACE discrimination (desk vs lap, via tap spectral content)
 *  --------------------------------------------------------------------------- */

/* Mid-band FFT energy above which a tap is taken to be on a HARD surface
 * (desk-feedback ringing) vs soft (lap).  v0 PROXY for post-event ring-down.
 * [HOUSING] — strongly coupling-dependent; re-measure on housed hardware. */
#define SURFACE_RESONANCE_MID_BAND_THRESH   5e7f

/* FFT band edges (bins) at 833 Hz / 512-pt (≈1.63 Hz/bin).  Low band
 * ~20-200 Hz (bone-conducted), mid ~200-400 Hz (impact/feedback).
 * [STRUCTURAL] (tied to ODR + FFT size). */
#define FFT_BIN_LOW_START               12     /* ~19.5 Hz */
#define FFT_BIN_LOW_END                 122    /* ~199 Hz, exclusive */
#define FFT_BIN_MID_START               122
#define FFT_BIN_MID_END                 246    /* ~400 Hz, exclusive */

/* ---------------------------------------------------------------------------
 *  5. Wrist-flick (gyro) — sharp flick gesture (detect + log; unbound since
 *     air-mouse extraction, kept as a hook for a future mode)
 *  --------------------------------------------------------------------------- */

/* Flick = a sharp gyro burst above THRESH (rad/s) followed by a sign-reversed
 * burst within WINDOW samples.  [USER] */
#define FLICK_BURST_THRESH_RPS          8.0f
#define FLICK_WINDOW_SAMPLES            25

/* ---------------------------------------------------------------------------
 *  6. Orientation filter (Mahony) + stillness / ZARU (shared IMU foundation:
 *     pose detection, dictation discriminator, future modes)
 *  --------------------------------------------------------------------------- */

/* Mahony proportional / integral gains.  Kp = how fast pitch/roll lock to
 * gravity; Ki = gyro-bias (ZARU) estimation rate.  [STRUCTURAL] (filter
 * tuning, not hardware). */
#define ORI_TWO_KP                      (2.0f * 0.5f)
#define ORI_TWO_KI                      (2.0f * 0.1f)

/* Stillness detection: "at rest" when |accel|-9.81 is within ACC_RESID (m/s^2)
 * AND total gyro is below GYRO_THRESH (rad/s) for DWELL samples.  Gates the
 * yaw re-zero + bias convergence.  Measured still-state noise ~0.2-1 dps, well
 * below the 0.10 rad/s (≈5.7 dps) threshold.  [UNIT] (gyro noise is per-chip). */
#define STILL_GYRO_THRESH_RPS           0.10f
#define STILL_ACC_RESID                 0.80f
#define STILL_DWELL_SAMPLES             30     /* 300 ms @100 Hz */

/*
 * ============================================================================
 *  CATALOG — tunables that physically live elsewhere (do NOT redefine here)
 * ============================================================================
 *
 *  These cannot be plain C macros (Kconfig, chip-register encodings, or
 *  DSP-internal literals), but are listed so this header is the single index.
 *
 *  --- prj.conf (Kconfig; rebuild to change) ---
 *   CONFIG_MAX30101_LED2_PA = 0x66   PPG LED drive (~20 mA).        [HOUSING]
 *   CONFIG_MAX30101_SR      = 1      PPG sample-rate index.         [STRUCTURAL]
 *   CONFIG_LSM6DSL_ACCEL_ODR= 4      accel ODR (104 Hz).            [STRUCTURAL]
 *   CONFIG_LSM6DSL_GYRO_ODR = 4      gyro ODR (104 Hz).             [STRUCTURAL]
 *   CONFIG_LSM6DSL_GYRO_FS  = 1000   gyro full-scale (±1000 dps).   [STRUCTURAL]
 *
 *  --- src/power_ctrl.cpp (chip registers) ---
 *   LSM6DSL_INT_DUR2_INITIAL = 0x7F  tap SHOCK/QUIET/DUR timing.    [HOUSING]
 *   tap threshold (TAP_THS)  = 0x08  (~500 mg) default; runtime
 *                                    adjustable via '+'/'-'.        [HOUSING][USER]
 *   accel/FIFO ODR           = 833 Hz (tap-engine + bio-acoustic).  [STRUCTURAL]
 *
 *  --- src/WearableDSP.h / .cpp (HR DSP) ---
 *   WEAR_PPG_THRESHOLD       = 10000   worn vs off-wrist PPG mean.  [HOUSING]
 *   SQI variance bounds      = 10 .. 1,000,000 (WearableDSP.cpp).   [HOUSING]
 *   imu_var motion bounds    = 0.01 / 0.5 (STATIONARY/MICRO/HEAVY). [USER]
 *   WEAR_PASSES_REQUIRED     = 4       windows to declare WORN.     [STRUCTURAL]
 *   MIN_BPM / MAX_BPM        = 40 / 200.                            [STRUCTURAL]
 *
 *  --- per-unit measured (not a constant; tracked live by ZARU) ---
 *   gyro bias ≈ [+0.66, -1.59, -0.58] dps (this unit, warm).       [UNIT]
 *     The Mahony integral tracks this adaptively; documented for reference.
 * ============================================================================
 */

#endif /* GESTURE_THRESHOLDS_H */
