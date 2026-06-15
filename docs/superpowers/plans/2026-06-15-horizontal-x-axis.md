# Horizontal (X) Cursor Axis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the air-mouse horizontal (X) cursor axis from a comfortable elbow-anchored forearm **sweep** (yaw), mapped linearly angle→pixels so it feels like a position "arc" (matching the vertical), with drift bounded by the existing cone-gate + at-rest freeze + Mahony bias tracking.

**Architecture:** The pure, host-tested `cursor_track` module already computes the X delta as `gain · (angle − prev_angle)` with a cone gate, an at-rest freeze, and a per-tick prev-resync — which is exactly the linear pseudo-absolute arc behavior we want. The only semantic change is the **angle source: roll → yaw**. We rename the X state/args from `roll` to `yaw`, wire the caller to pass `orientation.yaw_deg`, lock the behavior with characterization tests, add a documented comfort-span constant, then tune gain on hardware.

**Tech Stack:** C++ (Zephyr firmware, XIAO nRF52840 Sense / LSM6DSL). Build: `./build.sh`. Host test: `g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct` → expect `ALL PASS`. Full build/file map in `CLAUDE.md`. Design spec: `docs/superpowers/specs/2026-06-15-horizontal-x-axis-design.md`.

---

## File structure

- `src/cursor_track.h` — rename the X driver arg `roll_deg → yaw_deg` in `cursor_track_start` and `cursor_track_update`; update doc comments. (Public interface change.)
- `src/cursor_track.cpp` — rename X statics (`s_prev_roll → s_prev_yaw`, `s_roll_valid → s_x_valid`) and the X branch; **no behavioral change beyond the driver swap.** Y axis / slam / anchors / calibration untouched.
- `src/gesture_mode.cpp` — in the two `cursor_track_*` call sites pass `ori.yaw_deg` where `ori.roll_deg` was passed for X.
- `src/gesture_thresholds.h` — add `CURSOR_YAW_HALF_SPAN_DEG` (documented comfort span) and update the `CURSOR_GAIN_X` comment to say "yaw→X".
- `tests/test_cursor_track.cpp` — update the existing "X relative" comment to "yaw"; add characterization tests A–F locking the X-as-yaw contract.

**Note on TDD shape:** Task 1 is a mechanical rename of already-correct behavior, verified green by the unchanged host suite + build. Task 2's tests are **characterization tests** — they lock the contract and are expected to pass immediately; if any goes red it reveals a real gap to fix before proceeding. This is the honest shape for repurposing existing correct code; do not manufacture artificial red states.

---

### Task 1: Switch the X driver from roll to yaw

**Files:**
- Modify: `src/cursor_track.h` (the two prototypes + doc comments)
- Modify: `src/cursor_track.cpp` (statics + `cursor_track_start` + `cursor_track_update` X branch + `cursor_track_stop`)
- Modify: `src/gesture_mode.cpp` (the `cursor_track_start` and `cursor_track_update` call sites)

- [ ] **Step 1: Rename the X driver in the header**

In `src/cursor_track.h`, change the two prototypes and their doc comments. Replace:

```c
/* Begin a session: capture the reference angles (no entry jump) and gate X
 * until the shadow clearly clears the cone. */
void cursor_track_start(float vert_deg, float roll_deg);
```
with:
```c
/* Begin a session: capture the reference angles (no entry jump) and gate X
 * until the shadow clearly clears the cone. */
void cursor_track_start(float vert_deg, float yaw_deg);
```

Then replace the `cursor_track_update` doc + prototype:
```c
/* One tick (~100 Hz).
 *   vert_deg : angle-from-vertical (deg), the Y driver.  Use the GRAVITY-based
 *     inclination (acos(|gx|/|g|)), NOT Euler pitch -- hardware showed Euler
 *     pitch saturates at high roll (roll-contaminated) and freezes the cursor
 *     over the lower third of the stroke; the gravity inclination is immune.
 *   roll_deg : fused roll (orientation_get), the X driver.
 *   at_rest  : orientation stillness flag.
 *   shadow   : sqrt(gy^2+gz^2) from the GRAVITY-LPF (NEVER the fused
 *     quaternion -- inside the cone the fused roll drifts on gyro alone; the
 *     gate needs the raw gravity signal).
 * Writes the relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float vert_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy);
```
with:
```c
/* One tick (~100 Hz).
 *   vert_deg : angle-from-vertical (deg), the Y driver.  Use the GRAVITY-based
 *     inclination (acos(|gx|/|g|)), NOT Euler pitch -- hardware showed Euler
 *     pitch saturates at high roll (roll-contaminated) and freezes the cursor
 *     over the lower third of the stroke; the gravity inclination is immune.
 *   yaw_deg  : fused heading (orientation_get, bias-tracked, at-rest re-zeroed),
 *     the X driver.  X is a LINEAR angle->px map of the yaw delta (a forearm
 *     sweep about the elbow), so a given sweep angle moves a fixed screen
 *     distance regardless of speed.  No absolute heading reference exists on a
 *     6-axis IMU, so X is pseudo-absolute: cone-gated near vertical and
 *     re-anchored (prev resynced) at every at-rest pause to bound gyro drift.
 *   at_rest  : orientation stillness flag.
 *   shadow   : sqrt(gy^2+gz^2) from the GRAVITY-LPF (NEVER the fused
 *     quaternion -- it is the distance-from-vertical the cone gate needs; yaw
 *     degenerates at gimbal where shadow -> 0).
 * Writes the relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float vert_deg, float yaw_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy);
```

Also update the `cursor_track_set_gain` doc comment phrase "X gain may be negated to flip the X axis" — leave as-is (still true).

- [ ] **Step 2: Rename the X statics in cursor_track.cpp**

In `src/cursor_track.cpp`, replace:
```c
static float s_prev_roll      = 0.0f;
static bool  s_roll_valid     = false;
static bool  s_started        = false;
```
with:
```c
static float s_prev_yaw       = 0.0f;
static bool  s_x_valid        = false;
static bool  s_started        = false;
```

- [ ] **Step 3: Update cursor_track_start**

In `src/cursor_track.cpp`, replace the `cursor_track_start` body's roll lines. Change the signature and the two `s_prev_roll`/`s_roll_valid` lines:
```c
void cursor_track_start(float vert_deg, float roll_deg)
{
    (void)vert_deg;
    s_prev_roll  = roll_deg;
    s_roll_valid = false;
    s_cur_y      = 0.0f;
    s_at_top     = true;     /* we enter at the top */
    start_slam(-1);          /* entry slam to the top edge */
    s_started    = true;
}
```
becomes:
```c
void cursor_track_start(float vert_deg, float yaw_deg)
{
    (void)vert_deg;
    s_prev_yaw   = yaw_deg;
    s_x_valid    = false;
    s_cur_y      = 0.0f;
    s_at_top     = true;     /* we enter at the top */
    start_slam(-1);          /* entry slam to the top edge */
    s_started    = true;
}
```

- [ ] **Step 4: Update the X branch in cursor_track_update**

In `src/cursor_track.cpp`, change the signature `roll_deg → yaw_deg` and replace the X-axis block. The current code:
```c
void cursor_track_update(float vert_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
```
becomes:
```c
void cursor_track_update(float vert_deg, float yaw_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
```

Then replace this block:
```c
    /* ---- X axis: relative roll (UNCHANGED), but SUPPRESSED during a slam so
     * the slam emits only its Y burst (spec §5).  prev stays synced so X has no
     * jump when tracking resumes. ---- */
    float droll = wrap180(roll_deg - s_prev_roll);
    if (fabsf(droll) > CURSOR_MAX_DELTA_DEG) droll = 0.0f;
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_roll_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_roll_valid = false;
    bool x_frozen = at_rest && (fabsf(droll) < CURSOR_FREEZE_RELEASE_DELTA);
    float dx = 0.0f;
    if (!slamming && !x_frozen && s_roll_valid) dx = s_gain_x * droll;

    *out_dx = dx;
    *out_dy = dy;
    s_prev_roll = roll_deg;
}
```
with:
```c
    /* ---- X axis: LINEAR yaw-angle -> px (pseudo-absolute arc).  dx = gain *
     * per-tick yaw delta, so a given sweep ANGLE maps to a fixed screen distance
     * regardless of sweep speed.  Suppressed during a Y slam.  Cone-gated near
     * vertical (yaw degenerates at gimbal, shadow -> 0).  Frozen at rest: the
     * MAX_DELTA clamp + freeze absorb the filter's at-rest yaw re-zero
     * discontinuity, and s_prev_yaw is resynced EVERY tick (incl.
     * frozen/invalid/slamming) so X never jumps on resume. ---- */
    float dyaw = wrap180(yaw_deg - s_prev_yaw);
    if (fabsf(dyaw) > CURSOR_MAX_DELTA_DEG) dyaw = 0.0f;
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_x_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_x_valid = false;
    bool x_frozen = at_rest && (fabsf(dyaw) < CURSOR_FREEZE_RELEASE_DELTA);
    float dx = 0.0f;
    if (!slamming && !x_frozen && s_x_valid) dx = s_gain_x * dyaw;

    *out_dx = dx;
    *out_dy = dy;
    s_prev_yaw = yaw_deg;
}
```

- [ ] **Step 5: Update cursor_track_stop**

In `src/cursor_track.cpp`, replace:
```c
void cursor_track_stop(void)
{
    s_roll_valid = false;
    s_started    = false;
```
with:
```c
void cursor_track_stop(void)
{
    s_x_valid    = false;
    s_started    = false;
```
(Leave the rest of `cursor_track_stop` — the comment about absolute-Y state — unchanged.)

- [ ] **Step 6: Wire the caller to pass yaw**

In `src/gesture_mode.cpp` there are exactly two call sites, each with an `ori`
(`orientation_state_t`) already populated by an `orientation_get(&ori)` just above.

(a) The AIR_MOUSE-entry start (around line 604). Change:
```c
        cursor_track_start(top_now, ori.roll_deg);
```
to:
```c
        cursor_track_start(top_now, ori.yaw_deg);
```

(b) The per-tick update in `gesture_mode_update_gyro` (around line 1172). Change:
```c
        cursor_track_update(vert, ori.roll_deg, ori.at_rest,
                            shadow, &dx, &dy);
```
to:
```c
        cursor_track_update(vert, ori.yaw_deg, ori.at_rest,
                            shadow, &dx, &dy);
```
Keep the `vert` (1st) and `shadow` args unchanged. Do NOT touch the cone-gate
`shadow = sqrt(gy_filt^2 + gz_filt^2)` line just above the update — that stays on
the gravity-LPF (the comment near line 411 explains why it must NOT use the fused
quaternion).

- [ ] **Step 7: Build and run the existing host suite (verify green)**

Run:
```
./build.sh 2>&1 | tail -3
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
```
Expected: build prints the `Flashable artifact:` line (no errors); host test prints `ALL PASS`. The existing X test (`dx == GAIN_X*3` on a +3° step) passes unchanged because the math is identical — only the driver source changed.

- [ ] **Step 8: Commit**

```bash
git add src/cursor_track.h src/cursor_track.cpp src/gesture_mode.cpp
git commit -m "$(printf 'feat(cursor): drive horizontal X from yaw sweep, not roll\n\nThe air-mouse X axis now maps the fused yaw (forearm sweep about the\nelbow) linearly to pixels -- a sweep angle moves a fixed screen distance\nregardless of speed (arc feel), replacing the small-range relative-roll\ndriver. The X branch logic (linear delta, cone gate, at-rest freeze,\nper-tick prev resync) is unchanged; only the angle source is roll->yaw.\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 2: Lock the X-as-yaw contract with characterization tests

**Files:**
- Modify: `tests/test_cursor_track.cpp` (update one comment; append tests A–F before the final `printf`)

- [ ] **Step 1: Update the stale "X relative" comment**

In `tests/test_cursor_track.cpp`, the block beginning `/* --- X relative preserved; Y stillness via servo. --- */` still passes (yaw 50→53 = +3°, `dx = GAIN_X*3`). Change only that comment line to:
```c
    /* --- X = linear yaw delta (was roll); Y stillness via servo. --- */
```

- [ ] **Step 2: Append characterization tests A–F**

In `tests/test_cursor_track.cpp`, immediately BEFORE the line:
```c
    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
```
insert:
```c
    /* ===== X-axis (yaw) contract ===== */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* A: arc-linearity + speed-independence -- a 20deg sweep moves GAIN_X*20 px
     * whether fed as one big step or many small ones. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);   /* sync prev at yaw 0 */
    float x_big = 0.0f;
    cursor_track_update(TOP, 20.0f, false, V, &dx, &dy);
    x_big += dx;
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    float x_small = 0.0f;
    for (int i = 1; i <= 40; i++) {
        cursor_track_update(TOP, i * 0.5f, false, V, &dx, &dy);
        x_small += dx;
    }
    CHECK(fabsf(x_big   - CURSOR_GAIN_X * 20.0f) < 1e-3f);
    CHECK(fabsf(x_small - CURSOR_GAIN_X * 20.0f) < 1e-3f);
    CHECK(fabsf(x_big - x_small) < 1e-3f);

    /* B: at-rest yaw re-zero (a big discontinuity) is absorbed -> dx 0, and the
     * next real move is uncontaminated (prev resynced across the re-zero). */
    cursor_track_start(TOP, 100.0f);
    drain_slam(TOP, 100.0f);
    cursor_track_update(TOP, 100.0f, false, V, &dx, &dy);  /* sync prev at 100 */
    cursor_track_update(TOP, 0.0f, true, V, &dx, &dy);      /* re-zero: -100 jump */
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 2.0f, false, V, &dx, &dy);     /* 0->2, not 100->2 */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* C: freeze at rest for sub-release motion; releases for a real move. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 0.02f, true, V, &dx, &dy);     /* 0.02 < FREEZE_RELEASE */
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 1.02f, true, V, &dx, &dy);      /* +1.0 > FREEZE_RELEASE */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 1.0f) < 1e-3f);

    /* D: yaw wrap across +/-180 -> small real delta, no ~360 spike. */
    cursor_track_start(TOP, 179.0f);
    drain_slam(TOP, 179.0f);
    cursor_track_update(TOP, 179.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, -179.0f, false, V, &dx, &dy);  /* wraps to +2 deg */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* E: X suppressed during a Y slam. */
    cursor_track_start(TOP, 0.0f);
    CHECK(cursor_track_is_slamming());
    cursor_track_update(TOP, 10.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f);
    drain_slam(TOP, 10.0f);

    /* F: cone-gate hysteresis -- off below INVALIDATE, holds off in the band,
     * back on above REVALIDATE. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 2.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 0.5f, &dx, &dy);
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 4.0f, false,
        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f, &dx, &dy);
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 6.0f, false, CURSOR_ROLL_SHADOW_REVALIDATE + 0.5f, &dx, &dy);
    CHECK(dx != 0.0f);
```

- [ ] **Step 3: Build and run the host suite (verify green)**

Run:
```
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
```
Expected: `ALL PASS`. If any A–F fails, the X contract has a real gap — stop and reconcile against the spec (`docs/superpowers/specs/2026-06-15-horizontal-x-axis-design.md` §3/§5) before continuing; do not weaken the assertion to force a pass.

- [ ] **Step 4: Commit**

```bash
git add tests/test_cursor_track.cpp
git commit -m "$(printf 'test(cursor): lock the X-as-yaw contract (arc-linearity, freeze, wrap, cone)\n\nCharacterization tests for the yaw-driven X axis: speed-independent\nangle->px linearity, at-rest re-zero absorption + no-jump resume,\nsub-release freeze, +/-180 wrap, slam suppression, cone-gate hysteresis.\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 3: Document the comfort-span constant

**Files:**
- Modify: `src/gesture_thresholds.h:275` (the `CURSOR_GAIN_X` line + comment)

- [ ] **Step 1: Add the comfort-span constant and clarify the gain comment**

In `src/gesture_thresholds.h`, replace:
```c
#define CURSOR_GAIN_X                   8.0f    /* px/deg (relative roll->X) [USER][HOUSING]      */
```
with:
```c
/* X is driven by YAW (forearm sweep about the elbow), mapped linearly px/deg and
 * tuned live via the ']' / '[' serial knob.  CURSOR_YAW_HALF_SPAN_DEG records the
 * comfortable one-side sweep the gain is dialed against (sweep ~this far -> reach
 * the screen edge).  It is a documented tuning target, NOT a runtime anchor: X has
 * no fixed angle reference (no heading source on a 6-axis IMU), so the live gain
 * does the real tuning. */
#define CURSOR_GAIN_X                   8.0f    /* px/deg (yaw->X) [USER][HOUSING]                 */
#define CURSOR_YAW_HALF_SPAN_DEG        35.0f   /* comfortable one-side sweep, deg [USER]          */
```

- [ ] **Step 2: Build (verify clean)**

Run:
```
./build.sh 2>&1 | tail -3
```
Expected: prints the `Flashable artifact:` line with no errors.

- [ ] **Step 3: Commit**

```bash
git add src/gesture_thresholds.h
git commit -m "$(printf 'docs(cursor): record CURSOR_YAW_HALF_SPAN_DEG comfort span for X gain tuning\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 4: Hardware acceptance + gain tuning

**Files:** none (hardware validation; may end with a one-line `CURSOR_GAIN_X` default update + commit).

- [ ] **Step 1: Flash**

Run `./build.sh`, double-tap RESET on the Xiao, then:
```
cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/
```
Confirm macOS pointer acceleration is OFF (CLAUDE.md gotcha) and the BLE cursor is connected.

- [ ] **Step 2: Acceptance run (report observations, do not auto-tune blindly)**

Enter AIR_MOUSE (raise + double-tap). Verify against the spec §7 acceptance:
  1. Sweeping the forearm left/right (elbow anchored) moves the cursor horizontally as an arc — left aim → left, right aim → right.
  2. A given sweep covers a consistent screen distance regardless of speed.
  3. Pausing (hand still) does not jump the cursor; resuming continues smoothly (re-anchor invisible).
  4. Near a fully-raised/vertical forearm, X does not garbage/jitter (cone gate holds it).
  5. The only artifact is slow creep on a long, unbroken, slow sweep.

- [ ] **Step 3: Tune the X gain live**

Use the `]` (X gain up) / `[` (X gain down) serial commands so that a comfortable ~±35° sweep reaches each screen side without over-recruiting the shoulder. Note the value that feels right.

- [ ] **Step 4 (optional): Persist the tuned gain**

If the tuned value differs from `8.0f`, update `CURSOR_GAIN_X` in `src/gesture_thresholds.h` to the dialed value, rebuild (`./build.sh`, expect clean), and commit:
```bash
git add src/gesture_thresholds.h
git commit -m "$(printf 'tune(cursor): set CURSOR_GAIN_X from HW sweep validation\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

## Done when

- `./build.sh` clean; host suite (`test_cursor_track` + `test_cursor_calib`) `ALL PASS`.
- X tracks a left/right forearm sweep as a position arc on hardware, drift bounded by pauses, no garbage near vertical.
- Spec §8 out-of-scope items (per-user X calibration ritual, near-gimbal heading extraction) remain deferred — do NOT implement them here.
