# Air-Mouse Cursor (pointing v1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make air-mouse actually move the host cursor — wire AIR_MOUSE entry to the cursor and turn the drift-free orientation into relative mouse motion (roll→X, pitch→Y), with freeze-on-stillness and the observability cone gating the X axis.

**Architecture:** Three units. `gesture_mode` (FSM) transitions into `MODE_AIR_MOUSE` on a cadenced double-tap and, each ~100 Hz tick, hands the fused orientation to a NEW pure module `cursor_track`, which returns a relative `(dx,dy)`; `gesture_mode` injects that into the existing `cursor_pipeline` (125 Hz publish → BLE HID). `cursor_track` has no Zephyr/pipeline dependency, so its math is host-unit-tested.

**Tech Stack:** Zephyr / nRF52840 (XIAO Sense), C++, CMSIS. **Two test layers:** (a) a host unit test for the pure `cursor_track` math, compiled+run with `g++` (no hardware); (b) firmware build + on-host BLE-mouse verification for the integration. Firmware build command (used in every firmware build step):

```bash
export PATH=/opt/nordic/ncs/toolchains/185bb0e3b6/bin:$PATH
export ZEPHYR_BASE=/opt/nordic/ncs/zephyr
west build --build-dir build
```

Spec: `docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md`

---

### Task 1: Cursor tuning constants

**Files:**
- Modify: `src/gesture_thresholds.h`

- [ ] **Step 1: Add the cursor constants**

In `src/gesture_thresholds.h`, just before the final `#endif /* GESTURE_THRESHOLDS_H */`, add a new section:

```c
/* ---------------------------------------------------------------------------
 *  8. Air-mouse cursor (pointing v1)
 *  ---------------------------------------------------------------------------
 *  See docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md.
 *  Relative/rate: cursor moves by the per-tick CHANGE in the fused wrist
 *  angle.  All [USER]/empirical, tuned per-mount on hardware. */

/* Angle-delta -> pixels (SIGNED; flip on hardware if a direction is
 * inverted).  gain ~= reachable span: gain * usable-angle-range = total
 * travel.  ~80 deg usable twist * 8 = ~640 px (~1/3 of 1080p) -- EXPECT the
 * first tuning pass to ~triple this (and then a speed-dependent gain curve,
 * deferred, likely becomes necessary).  Start at 8 only to confirm direction
 * + stability. */
#define CURSOR_GAIN_Y                   8.0f
#define CURSOR_GAIN_X                   8.0f

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
```

- [ ] **Step 2: Firmware build**

Run the firmware build command. Expected: clean compile (unused `#define`s are harmless), FLASH ~46%.

- [ ] **Step 3: Commit**

```bash
git add src/gesture_thresholds.h
git commit -m "feat: air-mouse cursor tuning constants (gain, cone-gate hysteresis, freeze, clamp)"
```

---

### Task 2: cursor_track pure module + host unit test

**Files:**
- Create: `src/cursor_track.h`
- Create: `src/cursor_track.cpp`
- Create: `tests/test_cursor_track.cpp`
- Modify: `CMakeLists.txt` (add `src/cursor_track.cpp`)

- [ ] **Step 1: Write the header**

Create `src/cursor_track.h`:

```c
#ifndef CURSOR_TRACK_H
#define CURSOR_TRACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Air-mouse pointing: turn the drift-free orientation into a relative mouse
 * delta.  PURE -- no Zephyr / cursor_pipeline deps, so it is host-unit-
 * testable; the CALLER injects the returned delta into cursor_pipeline.
 * Relative/rate model.  Stateful singleton (one band / one cursor).
 * See docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md.
 */

/* Begin a session: capture the reference angles (no entry jump) and gate X
 * until the shadow clearly clears the cone. */
void cursor_track_start(float pitch_deg, float roll_deg);

/* One tick (~100 Hz).  pitch/roll: fused (orientation_get) angles in deg.
 * at_rest: orientation stillness flag.  shadow: sqrt(gy^2+gz^2) from the
 * GRAVITY-LPF (NEVER the fused quaternion -- inside the cone the fused roll
 * drifts on gyro alone; the gate needs the raw gravity signal).  Writes the
 * relative cursor delta (px) to *out_dx, *out_dy. */
void cursor_track_update(float pitch_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy);

/* End the session (clears latched state). */
void cursor_track_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_TRACK_H */
```

- [ ] **Step 2: Write the failing host test**

Create `tests/test_cursor_track.cpp`:

```cpp
#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

int main(void)
{
    float dx, dy;
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f; /* shadow: X valid */

    /* 1. Unchanged angles -> no movement. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* 2. Pitch->dy, roll->dx, scaled by gain (X valid). */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-58.0f, 53.0f, false, V, &dx, &dy);  /* dpitch=+2 droll=+3 */
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 2.0f) < 1e-3f);
    CHECK(fabsf(dx - CURSOR_GAIN_X * 3.0f) < 1e-3f);

    /* 3. Cone gate: shadow below INVALIDATE -> X gated, Y still moves. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);                          /* valid=true */
    cursor_track_update(-58.0f, 55.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy);
    CHECK(dx == 0.0f);
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 2.0f) < 1e-3f);

    /* 4. Hysteresis: shadow between thresholds holds the prior state. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);                          /* valid=true */
    cursor_track_update(-60.0f, 51.0f, false,
                        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f,
                        &dx, &dy);                                                    /* between -> holds */
    CHECK(dx != 0.0f);

    /* 5. Freeze: at_rest + tiny delta -> no motion. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, true, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* 6. Freeze releases on a move above the release delta even if at_rest. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-59.0f, 50.0f, true, V, &dx, &dy);  /* dpitch=+1 > release */
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 1.0f) < 1e-3f);

    /* 7a. Roll wrap at +/-180 is handled (not a teleport). */
    cursor_track_start(-60.0f, 179.0f);
    cursor_track_update(-60.0f, -179.0f, false, V, &dx, &dy);  /* raw -358 -> wrap +2 */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* 7b. A true glitch beyond MAX_DELTA is discarded. */
    cursor_track_start(0.0f, 0.0f);
    cursor_track_update(0.0f, CURSOR_MAX_DELTA_DEG + 10.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f);

    if (failures == 0) { printf("ALL PASS\n"); return 0; }
    printf("%d FAILURES\n", failures);
    return 1;
}
```

- [ ] **Step 3: Compile the test — verify it FAILS to link**

Run:
```bash
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp -lm -o /tmp/cursor_track_test
```
Expected: link error — `undefined reference to cursor_track_start/update/stop` (no implementation yet).

- [ ] **Step 4: Write the implementation**

Create `src/cursor_track.cpp`:

```cpp
#include "cursor_track.h"
#include "gesture_thresholds.h"

#include <math.h>

static_assert(CURSOR_ROLL_SHADOW_REVALIDATE >= CURSOR_ROLL_SHADOW_INVALIDATE,
              "CURSOR_ROLL_SHADOW_REVALIDATE must be >= INVALIDATE (cone-gate hysteresis)");

static float s_prev_pitch = 0.0f;
static float s_prev_roll  = 0.0f;
static bool  s_roll_valid = false;

/* Wrap an angle delta into [-180,180] so the roll (atan2) discontinuity at
 * +/-180 doesn't produce a ~360 deg jump. */
static float wrap180(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void cursor_track_start(float pitch_deg, float roll_deg)
{
    s_prev_pitch = pitch_deg;
    s_prev_roll  = roll_deg;
    s_roll_valid = false;   /* gate X until the shadow clearly clears the cone */
}

void cursor_track_update(float pitch_deg, float roll_deg, bool at_rest,
                         float shadow, float *out_dx, float *out_dy)
{
    float dpitch = wrap180(pitch_deg - s_prev_pitch);
    float droll  = wrap180(roll_deg  - s_prev_roll);

    /* (3c) Euler-wrap / glitch guard: a real wrist move is << this per tick. */
    if (fabsf(dpitch) > CURSOR_MAX_DELTA_DEG) dpitch = 0.0f;
    if (fabsf(droll)  > CURSOR_MAX_DELTA_DEG) droll  = 0.0f;

    /* (3a) Cone-gate the X axis with hysteresis (latched). */
    if (shadow >= CURSOR_ROLL_SHADOW_REVALIDATE)      s_roll_valid = true;
    else if (shadow <  CURSOR_ROLL_SHADOW_INVALIDATE) s_roll_valid = false;
    /* between the two thresholds: s_roll_valid holds */

    /* (3b) Asymmetric freeze: dwell-engage via at_rest, immediate release on
     * any per-tick motion (more sensitive than at_rest's gyro threshold, so
     * slow precision moves are not eaten). */
    float ang_speed = fabsf(dpitch) + fabsf(droll);
    bool frozen = at_rest && (ang_speed < CURSOR_FREEZE_RELEASE_DELTA);

    float dx = 0.0f, dy = 0.0f;
    if (!frozen) {
        dy = CURSOR_GAIN_Y * dpitch;
        dx = s_roll_valid ? (CURSOR_GAIN_X * droll) : 0.0f;
    }

    *out_dx = dx;
    *out_dy = dy;

    /* Always re-sync prev -> deltas are tick-to-tick; a frozen/gated tick
     * discards that motion instead of accumulating a jump. */
    s_prev_pitch = pitch_deg;
    s_prev_roll  = roll_deg;
}

void cursor_track_stop(void)
{
    s_roll_valid = false;
}
```

- [ ] **Step 5: Compile + run the test — verify it PASSES**

Run:
```bash
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/cursor_track_test && /tmp/cursor_track_test
```
Expected: `ALL PASS` (exit 0).

- [ ] **Step 6: Add to the firmware build**

In `CMakeLists.txt`, in the `target_sources(app PRIVATE ...)` list, add after `src/cursor_pipeline.cpp`:

```cmake
    src/cursor_track.cpp
```

- [ ] **Step 7: Firmware build**

Run the firmware build command. Expected: clean compile, FLASH ~46%.

- [ ] **Step 8: Commit**

```bash
git add src/cursor_track.h src/cursor_track.cpp tests/test_cursor_track.cpp CMakeLists.txt
git commit -m "feat: cursor_track pure pointing module + host unit test

Drift-free orientation -> relative cursor delta: roll->X (cone-gated,
hysteresis), pitch->Y, asymmetric freeze, wrap+clamp. Pure (no Zephyr/
pipeline deps); host unit test covers freeze/hysteresis/clamp/wrap.
See 2026-06-11-air-mouse-cursor spec."
```

---

### Task 3: Wire cursor_track into the gesture FSM (F2 + per-tick feed)

**Files:**
- Modify: `src/gesture_mode.cpp`

- [ ] **Step 1: Add includes**

In `src/gesture_mode.cpp`, with the other local includes (near `#include "orientation.h"`), add:

```c
#include "cursor_track.h"
#include "cursor_pipeline.h"
```

- [ ] **Step 2: F2 — enter AIR_MOUSE on the double-tap**

In `multi_tap_commit_handler`, replace the `case POSE_AIR_MOUSE:` body:

```c
    case POSE_AIR_MOUSE:
        LOG_INF("MODE ENTRY: AIR_MOUSE (pose + cadenced double-tap)");
        /* TODO (future task F2): wire to power state machine -- see
         * 't'/'y' serial commands in main.cpp for the AIR_MOUSE entry
         * pattern.  Logged-only for now to verify pose+gesture path
         * empirically first. */
        break;
```

with:

```c
    case POSE_AIR_MOUSE:
        LOG_INF("MODE ENTRY: AIR_MOUSE (pose + cadenced double-tap)");
        _transition_to(MODE_AIR_MOUSE);
        break;
```

- [ ] **Step 3: Start/stop the tracker on transition**

In `_transition_to`, immediately after `atomic_set(&mode_atomic, (atomic_val_t)new_mode);`, add:

```c
    /* Air-mouse cursor: start tracking on entry (capture the reference
     * angles so there's no jump), stop on any transition away. */
    if (new_mode == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        cursor_track_start(ori.pitch_deg, ori.roll_deg);
    } else {
        cursor_track_stop();
    }
```

- [ ] **Step 4: Feed the tracker each tick + inject the delta**

In `gesture_mode_update_gyro`, immediately after the `orientation_update(...)` call, add:

```c
    /* Air-mouse cursor tracking: fused angles -> relative delta -> pipeline.
     * Cone gate uses the GRAVITY-LPF shadow (gy_filt/gz_filt), NEVER the
     * fused quaternion (inside the cone the fused roll drifts on gyro alone). */
    if ((GestureMode)atomic_get(&mode_atomic) == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        float shadow = sqrtf(gy_filt * gy_filt + gz_filt * gz_filt);
        float dx = 0.0f, dy = 0.0f;
        cursor_track_update(ori.pitch_deg, ori.roll_deg, ori.at_rest,
                            shadow, &dx, &dy);
        cursor_pipeline_inject_motion(dx, dy);
    }
```

- [ ] **Step 5: Firmware build**

Run the firmware build command. Expected: clean compile (`cursor_track_*` and `cursor_pipeline_inject_motion` resolve; `gy_filt/gz_filt` are file-scope statics; `sqrtf` from `<math.h>` already included), FLASH ~46%.

- [ ] **Step 6: Verify at_rest log hygiene**

Run: `grep -rn "at_rest=%d" src/`
Expected: every hit passes `(int)ori.at_rest` (pose-trace + the ORI line). If any pass a bare bool, cast it to `(int)`. (Both known sites are already cast; this is a confirmation.)

- [ ] **Step 7: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "feat: wire air-mouse entry to cursor (F2) + per-tick cursor_track feed

Double-tap in the raised pose now _transition_to(MODE_AIR_MOUSE), starting
cursor_track; each ~100Hz tick feeds the fused orientation + gravity-LPF
shadow to cursor_track and injects the delta into cursor_pipeline (-> BLE
HID). Exit reuses the existing FSM orientation/flick logic.
See 2026-06-11-air-mouse-cursor spec."
```

---

### Task 4: On-host hardware verification (the real test)

**Files:** none (flash + pair + observe).

- [ ] **Step 1: Flash + pair**

Flash `build/zephyr/zephyr.uf2`. Pair the band to a host as a BLE mouse (it advertises HID). Confirm `ble_hid_notifications_enabled` (cursor only publishes when connected).

- [ ] **Step 2: Enter + basic pointing**

Raise into the air-mouse pose, settle, cadenced double-tap. Expect `MODE ENTRY: AIR_MOUSE` then `Mode transition: ... -> AIR_MOUSE`. Tilt wrist up/down → cursor moves vertically; twist wrist → cursor moves horizontally. If a direction is inverted, flip the sign of `CURSOR_GAIN_Y` / `CURSOR_GAIN_X` in `gesture_thresholds.h`, rebuild.

- [ ] **Step 3: Freeze + slow-precision**

Hold the hand still → cursor parks (no drift). Then make a SLOW deliberate move → it should respond immediately (not feel dead). If slow moves are eaten, lower `CURSOR_FREEZE_RELEASE_DELTA`; if it jitters when still, raise it slightly (toward the still-pose noise floor).

- [ ] **Step 4: Cone gate**

Raise the forearm toward vertical → horizontal (X) output should freeze while vertical (Y) keeps working; lower back to a normal raise → horizontal resumes with NO jump. Tune `CURSOR_ROLL_SHADOW_INVALIDATE/REVALIDATE` if the gate trips too early/late.

- [ ] **Step 5: Exit**

Lower out of the raised pose (or flick-to-cancel) → `Mode transition: AIR_MOUSE -> IDLE` and the cursor stops.

- [ ] **Step 6: Tune + record**

Expect the gain to need ~tripling for full-screen reach (per spec §4). Record the final `GAIN_X/Y`, shadow pair, and freeze value as the per-mount tuned values (commit the tuned constants).
