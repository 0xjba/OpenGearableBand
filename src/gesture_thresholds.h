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
 * Findings & rationale: docs/research/orientation-and-pose-discrimination-
 * findings.md and docs/research/cursor-drift-mitigation.md.
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
 *  5. Cursor-mode cooldown / exit dwell (AIR_MOUSE)
 *  --------------------------------------------------------------------------- */

/* Cooldown after a cursor mode exits, during which re-entry is orientation-
 * only (no fresh gesture).  Samples @100 Hz (2000 = 20 s).  [USER] */
#define CURSOR_COOLDOWN_SAMPLES         2000

/* Orientation dwell to re-engage during cooldown.  Samples @100 Hz.  [USER] */
#define COOLDOWN_REENGAGE_DWELL         50

/* Entry grace (ms) after mode entry before exit logic arms.  [USER] */
#define AIR_MOUSE_ENTRY_GRACE           500

/* ---------------------------------------------------------------------------
 *  6. Wrist-flick (gyro) — flick-to-cancel a cursor mode
 *  --------------------------------------------------------------------------- */

/* Flick = a sharp gyro burst above THRESH (rad/s) followed by a sign-reversed
 * burst within WINDOW samples.  [USER] */
#define FLICK_BURST_THRESH_RPS          8.0f
#define FLICK_WINDOW_SAMPLES            25

/* ---------------------------------------------------------------------------
 *  7. Orientation filter (Mahony) + stillness / ZARU (cursor foundation)
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

/* ---------------------------------------------------------------------------
 *  8. Air-mouse cursor (pointing v1)
 *  ---------------------------------------------------------------------------
 *  See docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md.
 *  Y is an absolute gravity-anchored servo (counts/deg); X is relative/rate
 *  px/deg.  Constants are a mix of [USER] and [STRUCTURAL] — see inline tags. */

/* --- Absolute-Y cursor (gravity-anchored vertical) --------------------------
 * Y is no longer a relative px/deg gain.  CURSOR_GAIN_Y is now COUNTS/DEG: the
 * servo target is GAIN_Y * (vert - vert_top), tuned on '}' / '{' so a full
 * comfortable down-sweep fills the screen (this folds in the unknown host
 * count->pixel scale).  X stays relative px/deg.  See
 * docs/superpowers/specs/2026-06-11-absolute-y-cursor-design.md. */
#define CURSOR_GAIN_Y                   30.0f   /* counts/deg (servo); tune on HW [USER][HOUSING] */
/* X is driven by YAW (forearm sweep about the elbow), mapped linearly px/deg and
 * tuned live via the ']' / '[' serial knob.  CURSOR_YAW_HALF_SPAN_DEG records the
 * comfortable one-side sweep the gain is dialed against (sweep ~this far -> reach
 * the screen edge).  It is a documented tuning target, NOT a runtime anchor: X has
 * no fixed angle reference (no heading source on a 6-axis IMU), so the live gain
 * does the real tuning.  The SIGN sets the left/right direction for this mount
 * (negative here so a leftward sweep moves the cursor left); the ']' / '[' knob
 * scales MAGNITUDE only and preserves the sign. */
#define CURSOR_GAIN_X                   -8.0f   /* px/deg (yaw->X); sign = L/R for mount [USER][HOUSING] */
#define CURSOR_YAW_HALF_SPAN_DEG        35.0f   /* comfortable one-side sweep, deg [USER]          */

/* Sweep-plane coupling compensation: a tilted-plane forearm sweep drags elevation
 * into the Y driver, bowing the cursor into an arc.  corrected_vert = vert -
 * CURSOR_SWING_COMP_K * sweep_deg^2 cancels it (sweep_deg = horizontal cursor
 * displacement / X gain).  First guess from HW trace: ~15 deg of coupling at ~70
 * deg sweep -> k ~= 15/70^2 ~= 0.003.  Tuned live via 'o'/'p'; 0 disables. */
#define CURSOR_SWING_COMP_K             0.003f  /* deg elevation per deg^2 sweep [USER][HOUSING]   */

/* Vertical map calibration (deg). vert = acos(|gx|/|g|), 0=vertical..90=flat. */
#define CURSOR_VERT_SPAN_DEG            70.0f   /* nominal comfortable down-range [USER]           */
#define CURSOR_VERT_BOTTOM_MAX          85.0f   /* comfort clamp on implied bottom [USER]          */

/* Fixed top anchor (deg-from-vertical).  12deg keeps screen-top out of the
 * acos near-vertical jitter zone (~5x noise) while using almost the full
 * range. [USER][HOUSING] -- re-check after a re-tape/housing change. */
#define CURSOR_VERT_TOP_DEG             12.0f

/* Desk-settle exit (Amendment A.3).  gx_filt is the forearm-axis gravity
 * component in m/s^2 (~9.81 = 1g vertical, 0 = flat). */
/* COARSE capture gate: gx_filt below this = "flat-ish posture" where recording a
 * resting bottom anchor makes sense.  Deliberately GENEROUS (4.0 ~= vert > 65 deg)
 * so the bottom anchor is captured across re-wears -- the rest angle is mount-
 * dependent and has been measured anywhere from ~70 to ~84 deg (HW 2026-06-14: a
 * re-wear rested at vert=75 / gx~2.5, sitting exactly on the old 2.5 seed and
 * starving the gate of margin).  This is the ABSOLUTE gate for capture + the
 * stillness dwell; the desk-tap/stillness DISENGAGE keys off the calibrated rest
 * via CURSOR_LOW_ZONE_MARGIN_DEG instead, so it self-heals with the mount. [USER][HOUSING] */
#define CURSOR_LOW_ZONE_GX              4.0f

/* Anchor-relative low zone for the DISENGAGE gates.  "Near rest" = current vert is
 * within this many degrees of the auto-calibrated resting bottom (last_rest_vert).
 * Mount-INDEPENDENT (degrees from the user's own rest), unlike a fixed gx threshold:
 * when the mount shifts and the bottom anchor recalibrates, this zone moves with it.
 * 10 deg gives comfortable margin while staying clear of mid-screen pointing. [USER] */
#define CURSOR_LOW_ZONE_MARGIN_DEG      10.0f

/* Stillness-held disengage: consecutive settled-in-low-zone samples (still =
 * samples_since_activity >= ACTIVITY_GATE_DWELL) required to exit AIR_MOUSE.  Seed
 * 120 (~1.2 s) -- long enough a floating-to-point hand can't hold it, short enough
 * a real desk rest fires.  M1-tuned. [USER] */
#define STILL_EXIT_DWELL                120
#define CURSOR_PAST_PLANE_GX            (-1.0f) /* (b) no-desk exit: forearm past horizontal [USER] */
#define CURSOR_PAST_PLANE_DWELL         15      /* samples gx must stay past-plane [USER] */

/* Slam (manufactured edge clamp).  Over-travel is free (OS clamps); undershoot
 * poisons registration -> a GAIN_Y-INDEPENDENT floor guarantees the edge even
 * untuned. */
#define CURSOR_SLAM_MARGIN              2.0f    /* 2× max_counts [STRUCTURAL]                      */
#define CURSOR_SLAM_FLOOR_COUNTS        6000.0f /* absolute min over-travel, counts [USER][HOUSING] */
#define CURSOR_SLAM_MAX_REPORTS         80      /* generous burst cap (mis-tune) [STRUCTURAL]      */

/* Top re-pin hysteresis band (deg around vert_top). LEAVE >= ENTER. */
#define CURSOR_PIN_ENTER_DEG            3.0f    /* [USER] */
#define CURSOR_PIN_LEAVE_DEG            6.0f    /* [USER] */

/* Cone gate (X axis) with HYSTERESIS, on the GRAVITY-LPF shadow
 * sqrt(gy^2+gz^2): roll is unobservable near a vertical forearm.  Below
 * INVALIDATE -> gate X off; above REVALIDATE -> back on; between -> hold.
 * Brackets the measured cone (vert 15 -> shadow 2.4 invalid, vert 31 ->
 * shadow 5.0 valid).  REVALIDATE must be >= INVALIDATE (static_assert in
 * cursor_track.cpp). */
#define CURSOR_ROLL_SHADOW_INVALIDATE   3.5f
#define CURSOR_ROLL_SHADOW_REVALIDATE   4.5f

/* Freeze release: per-tick |Δangle| (deg) above which the freeze lets go
 * immediately (so SLOW precision moves aren't eaten).  Start LOW, just above
 * the still-pose per-tick angle-noise floor -- a HIGH value would freeze
 * precision pointing (and smuggle in a 'ratchet' clutch, which we do NOT want
 * here -- add a deliberate ratchet gesture later if the range bites). */
#define CURSOR_FREEZE_RELEASE_DELTA     0.05f

/* Discard any per-tick |Δangle| above this as a wrap/glitch (a real wrist
 * move is far smaller per 10 ms).  Belt-and-suspenders with wrap180(). */
#define CURSOR_MAX_DELTA_DEG            30.0f

/* ---- Natural entry-time cursor calibration (auto top+bottom anchors) ----
 * See docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md.
 * All seeds; tune from the per-entry [CAL] traces. */
#define CAL_SWEEP_MIN_DEG     25.0f   /* min (bottom - top) to count as a real raise [USER] */
#define CAL_TOP_MAX           30.0f   /* plausibility clamp: reject a top above this (lazy raise) [USER] */
#define CAL_MIN_DELTA         5.0f    /* ignore anchor shifts smaller than this (stable within a wear) [USER] */
#define CAL_BLEND_ALPHA       0.4f    /* adoption blend factor (1.0 effective on cold-start seed) [USER] */

#endif /* GESTURE_THRESHOLDS_H */
