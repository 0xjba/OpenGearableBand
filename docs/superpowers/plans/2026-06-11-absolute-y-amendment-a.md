# Absolute-Y Amendment A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Switch the absolute-Y cursor to a fixed top anchor + expanded range, and replace the leave-raised-pose exit with a dual-trigger desk-settle exit (volar-contact impact, or forearm past the horizontal plane).

**Architecture:** Two tasks. A1 makes `cursor_track`'s top anchor a fixed constant and widens the span (pure module, host-tested). A2 rewrites the AIR_MOUSE exit in `gesture_mode.cpp` so the mode stays engaged from near-vertical to near-flat and disengages only on a desk-settle, plus telemetry to HW-tune the impact threshold.

**Tech Stack:** Zephyr / nRF52840, C++. `cursor_track` host-tested with g++; firmware built with `./build.sh`; A2 verified on hardware. Grounding: spec `docs/superpowers/specs/2026-06-11-absolute-y-cursor-design.md` **Amendment A**.

**Host test command:** `g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct`
**Firmware build:** `./build.sh`

---

## Task A1: Fixed top anchor + expanded range (cursor_track + constants + tests)

**Files:**
- Modify: `src/gesture_thresholds.h`
- Modify: `src/cursor_track.cpp`
- Modify: `tests/test_cursor_track.cpp` (rewrite numbers for the new anchor/span)

- [ ] **Step 1: Update constants in `src/gesture_thresholds.h`**

In the absolute-Y constants block:
- Change `CURSOR_VERT_SPAN_DEG` from `40.0f` to `70.0f`.
- Change `CURSOR_VERT_BOTTOM_MAX` from `60.0f` to `85.0f`.
- **Remove** the lines defining `CURSOR_VERT_TOP_DEFAULT` and `CURSOR_VERT_TOP_MAX` (obsolete under a fixed anchor).
- **Add** these defines (place with the other cursor constants):

```c
/* Fixed top anchor (deg-from-vertical).  12deg keeps screen-top out of the
 * acos near-vertical jitter zone (~5x noise) while using almost the full
 * range. [USER][HOUSING] -- re-check after a re-tape/housing change. */
#define CURSOR_VERT_TOP_DEG             12.0f

/* Desk-settle exit (Amendment A.3).  gx_filt is the forearm-axis gravity
 * component in m/s^2 (~9.81 = 1g vertical, 0 = flat). */
#define CURSOR_DESK_ZONE_GX             1.7f    /* near-flat zone: gx below this (vert>~80) [USER] */
#define CURSOR_PAST_PLANE_GX            (-1.0f) /* (b) no-desk exit: forearm past horizontal [USER] */
#define CURSOR_PAST_PLANE_DWELL         15      /* samples gx must stay past-plane [USER] */
#define CURSOR_IMPACT_THRESH            6.0f    /* (a) accel-residual spike, m/s^2 -- HW-TUNED seed [HOUSING] */
#define CURSOR_SETTLE_DWELL             40      /* post-impact stillness samples (~400ms) [USER] */
```

- [ ] **Step 2: Make the anchor fixed in `src/cursor_track.cpp`**

Replace the absolute-Y state declarations
```c
static float s_vert_top       = CURSOR_VERT_TOP_DEFAULT; /* top anchor (deg)        */
static float s_vert_top_good  = CURSOR_VERT_TOP_DEFAULT; /* RAM last-good fallback  */
```
with
```c
/* Fixed top anchor (Amendment A.1): no longer captured at entry. */
static const float s_vert_top = CURSOR_VERT_TOP_DEG;
```
(Delete `s_vert_top_good` entirely.)

Then replace the body of `cursor_track_start` — the capture/clamp/fallback block
```c
    /* Capture top anchor with a lazy-raise sanity clamp -> RAM last-good. */
    if (vert_deg <= CURSOR_VERT_TOP_MAX) {
        s_vert_top      = vert_deg;
        s_vert_top_good = vert_deg;
    } else {
        s_vert_top = s_vert_top_good;
    }
```
with
```c
    /* Fixed anchor (Amendment A.1): the entry inclination no longer sets the
     * top; vert_deg is unused here now but the arg is kept for API symmetry
     * (the slam + roll seed below still run on entry). */
    (void)vert_deg;
```
Leave the rest of `cursor_track_start` (the `s_prev_roll`/`s_roll_valid`/`s_cur_y`/`s_at_top`/`start_slam(-1)`/`s_started`) unchanged. `cursor_track_vert_top()` now returns the constant automatically.

- [ ] **Step 3: Rewrite `tests/test_cursor_track.cpp` for the fixed anchor + span 70**

Overwrite the file with:

```cpp
#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

static int drain_slam(float vert, float roll)
{
    float dx, dy;
    int n = 0;
    for (; n < CURSOR_SLAM_MAX_REPORTS + 2; n++) {
        if (!cursor_track_is_slamming()) break;
        cursor_track_update(vert, roll, false, 0.0f, &dx, &dy);
    }
    return n;
}

int main(void)
{
    float dx, dy;
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f;
    /* Fixed anchor: vert_top == CURSOR_VERT_TOP_DEG (12).  vert_bottom =
     * min(12+70, 85) = 82, so max_counts = GAIN_Y * 70 (no BOTTOM_MAX clamp). */
    const float TOP  = CURSOR_VERT_TOP_DEG;
    const float MAXC = CURSOR_GAIN_Y * CURSOR_VERT_SPAN_DEG;

    /* M1: after entry slam drains, cur_y == 0; holding vert==top yields dy 0. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_cur_y()) < 1e-3f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);

    /* M2: servo toward GAIN_Y*(vert-top); vert=top+10 -> 300 = 127+127+46. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 46.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    CHECK(fabsf(cursor_track_cur_y() - 300.0f) < 1e-3f);

    /* M3: target clamps to max_counts at the bottom (vert 90 >> 82). */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 30; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - MAXC) < 1e-3f);

    /* M4: post-stop update is a no-op. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_stop();
    dx = 1.0f; dy = 1.0f;
    cursor_track_update(40.0f, 80.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* M5: once clamped at the bottom, further-down vert holds (dy 0). */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 30; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - MAXC) < 1e-3f);
    cursor_track_update(95.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);

    /* --- Slam sizing: floor untuned-safe, cap respected. --- */
    cursor_track_set_gain(8.0f, 1.0f);          /* tiny gain_y */
    cursor_track_start(TOP, 50.0f);
    CHECK(cursor_track_is_slamming());
    int floor_reports = (int)ceilf(CURSOR_SLAM_FLOOR_COUNTS / 127.0f);
    int used = drain_slam(TOP, 50.0f);
    CHECK(used >= floor_reports);
    CHECK(used <= CURSOR_SLAM_MAX_REPORTS);
    cursor_track_set_gain(8.0f, 200.0f);         /* max=200*70 -> *2 capped */
    cursor_track_start(TOP, 50.0f);
    int used3 = drain_slam(TOP, 50.0f);
    CHECK(used3 == CURSOR_SLAM_MAX_REPORTS);
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* --- Fixed anchor: entry vert NEVER moves vert_top (replaces calibration). */
    cursor_track_start(40.0f, 50.0f);            /* "lazy" high entry */
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);
    cursor_track_start(14.0f, 50.0f);            /* normal entry */
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);

    /* --- Top re-pin hysteresis (anchored at the fixed top). --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);
    for (int i = 0; i < 25; i++) cursor_track_update(40.0f, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_cur_y() > 100.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);  /* <= top+ENTER -> re-pin */
    CHECK(cursor_track_is_slamming());
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_cur_y()) < 1e-3f);

    /* R2: no chatter while hovering in [ENTER,LEAVE]. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    cursor_track_update(TOP + (CURSOR_PIN_ENTER_DEG + CURSOR_PIN_LEAVE_DEG) * 0.5f,
                        50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());

    /* R3: leave past LEAVE then return re-arms exactly once; R3b no double-fire. */
    cursor_track_update(TOP + CURSOR_PIN_LEAVE_DEG + 5.0f, 50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_is_slamming());
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());

    /* --- X relative preserved; Y stillness via servo. --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 53.0f, false, V, &dx, &dy);
    CHECK(fabsf(dx - CURSOR_GAIN_X * 3.0f) < 1e-3f);
    cursor_track_update(TOP, 56.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy);
    CHECK(dx == 0.0f);

    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, true, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    cursor_track_update(TOP + 15.0f, 50.0f, true, V, &dx, &dy);
    CHECK(dy > 0.0f);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 4: Run host test → expect `ALL PASS`**

Run the host test command. If any check fails, the implementation diverges from the amendment — STOP and report (do not adjust numbers to force a pass without understanding why).

- [ ] **Step 5: Commit**

```bash
git add src/gesture_thresholds.h src/cursor_track.cpp tests/test_cursor_track.cpp
git commit -m "feat(cursor): fixed top anchor (12deg) + expanded range (span 70)"
```

---

## Task A2: Desk-settle exit + telemetry (gesture_mode.cpp)

**Files:**
- Modify: `src/gesture_mode.cpp` (the exit block ~lines 888–928, the residual-`r_mag` block ~794–804, the statics, `_transition_to`, `gesture_mode_init`)

**Goal:** For AIR_MOUSE, replace "exit when orientation leaves UP_RAISED" with the dual-trigger near-flat desk-settle exit. SURFACE's exit path is unchanged.

- [ ] **Step 1: Hoist `r_mag` so the exit logic can see it**

The activity-gate block currently computes `r_mag` inside its own `{ }` scope (around lines 794–804). Change it so `r_mag` is a local visible to the exit logic later in the function. Replace:
```c
    {
        float rx = ax - gx_filt;
        float ry = ay - gy_filt;
        float rz = az - gz_filt;
        float r_mag = sqrtf(rx * rx + ry * ry + rz * rz);
        if (r_mag > ACTIVITY_GATE_THRESH) {
            samples_since_activity = 0;
        } else if (samples_since_activity < ACTIVITY_GATE_DWELL) {
            samples_since_activity++;
        }
    }
```
with (drop the inner braces; declare `r_mag` at function-body scope so the exit code below can read it):
```c
    /* Motion residual (accel - gravity); drives the activity gate AND the
     * AIR_MOUSE desk-contact impact detector below. */
    float rx_resid = ax - gx_filt;
    float ry_resid = ay - gy_filt;
    float rz_resid = az - gz_filt;
    float r_mag = sqrtf(rx_resid * rx_resid + ry_resid * ry_resid + rz_resid * rz_resid);
    if (r_mag > ACTIVITY_GATE_THRESH) {
        samples_since_activity = 0;
    } else if (samples_since_activity < ACTIVITY_GATE_DWELL) {
        samples_since_activity++;
    }
```

- [ ] **Step 2: Add the desk-settle statics**

Near the other cursor statics (e.g. just after `static int cursor_exit_dwell` / `entry_grace_remaining` declarations), add:
```c
/* AIR_MOUSE desk-settle exit state (Amendment A.3). */
static bool air_impact_seen     = false;   /* (a) volar-contact impact latched in the near-flat zone */
static int  air_past_plane_dwell = 0;      /* (b) consecutive samples forearm is past horizontal */
```

- [ ] **Step 3: Replace the exit block with the per-mode exit**

Replace the entire exit block (from the `int exit_dwell_target = _exit_dwell_for(current_mode);` line through its closing `} else { cursor_exit_dwell = 0; }`, i.e. the lines 909–928 region) with:

```c
    /* --- Exit detection (per mode) ---
     * AIR_MOUSE (Amendment A.3): stay engaged through the full raised->near-flat
     * range; exit only on a desk-settle in the near-flat zone -- (a) volar
     * contact impact + settle (desk present), or (b) forearm past the
     * horizontal plane (no desk).  SURFACE keeps its orientation-drop exit. */
    if (in_cursor_mode && cursor_has_reached_pose) {
        bool do_exit = false;
        const char *exit_reason = "";

        if (current_mode == MODE_AIR_MOUSE) {
            bool near_flat = (gx_filt < CURSOR_DESK_ZONE_GX);

            /* (a) desk present: an impact while near-flat, then stillness. */
            if (near_flat && r_mag > CURSOR_IMPACT_THRESH) {
                air_impact_seen = true;
            }
            if (!near_flat) {
                air_impact_seen = false;   /* left the zone -> reset the latch */
            }
            bool desk_settled = air_impact_seen &&
                                (samples_since_activity >= CURSOR_SETTLE_DWELL);

            /* (b) desk absent: signed gx crosses past horizontal (drooping). */
            if (gx_filt < CURSOR_PAST_PLANE_GX) {
                if (air_past_plane_dwell < CURSOR_PAST_PLANE_DWELL) air_past_plane_dwell++;
            } else {
                air_past_plane_dwell = 0;
            }
            bool past_plane = (air_past_plane_dwell >= CURSOR_PAST_PLANE_DWELL);

            if (desk_settled)      { do_exit = true; exit_reason = "desk contact + settle"; }
            else if (past_plane)   { do_exit = true; exit_reason = "past horizontal plane (no desk)"; }

            /* Near-flat telemetry to HW-tune CURSOR_IMPACT_THRESH (Amendment A.4). */
            if (near_flat) {
                static int deskdbg = 0;
                if (++deskdbg % 10 == 0) {
                    LOG_INF("[DESK] gx=%d r_mag=%d still=%d impact=%d ppdwell=%d",
                            (int)gx_filt, (int)r_mag, (int)samples_since_activity,
                            (int)air_impact_seen, air_past_plane_dwell);
                }
            }
        } else {
            /* SURFACE (and any non-AIR_MOUSE cursor mode): orientation-drop exit. */
            int exit_dwell_target = _exit_dwell_for(current_mode);
            if (orientation_current != expected) {
                if (cursor_exit_dwell <= exit_dwell_target) {
                    cursor_exit_dwell++;
                    if (cursor_exit_dwell > exit_dwell_target) {
                        do_exit = true;
                        exit_reason = "wrist left desk plane (lift or drop)";
                    }
                }
            } else {
                cursor_exit_dwell = 0;
            }
        }

        if (do_exit) {
            LOG_INF("%s exit: %s -- starting %d ms re-engage cooldown",
                    _mode_str(current_mode), exit_reason, CURSOR_COOLDOWN_SAMPLES * 10);
            cursor_cooldown_mode = current_mode;
            cursor_cooldown_remaining = CURSOR_COOLDOWN_SAMPLES;
            air_impact_seen = false;
            air_past_plane_dwell = 0;
            _transition_to(MODE_IDLE);
        }
    } else {
        cursor_exit_dwell = 0;
        air_impact_seen = false;
        air_past_plane_dwell = 0;
    }
```

(The old `_exit_dwell_for` is still used by the SURFACE branch, so leave that function alone.)

- [ ] **Step 4: Reset the new statics on mode transitions and init**

In `_transition_to`, in the branch that resets cursor state when NOT entering a cursor mode (where `cursor_has_reached_pose = false;` is set), also reset:
```c
        air_impact_seen = false;
        air_past_plane_dwell = 0;
```
In `gesture_mode_init`, alongside the other resets (`cursor_has_reached_pose = false;` etc.), add:
```c
    air_impact_seen = false;
    air_past_plane_dwell = 0;
```

- [ ] **Step 5: Build the firmware**

Run: `./build.sh`
Expected: clean build, `==> Flashable artifact: build/zephyr/zephyr.uf2`, no new warnings.

- [ ] **Step 6: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "feat(cursor): AIR_MOUSE desk-settle exit (impact+settle / past-plane) + [DESK] telemetry"
```

- [ ] **Step 7: Hardware verification (manual, user)**

Flash, accel OFF, enter air-mouse (`t` raised). Verify:
1. Lowering all the way to near-flat **does NOT exit** anymore (cursor reaches/holds at screen bottom). Previously it exited ~mid-lower.
2. **No desk:** drop the forearm past horizontal → exits (`past horizontal plane (no desk)`), cooldown starts; raising re-engages.
3. **Desk:** lower onto the desk → exits (`desk contact + settle`). Capture the `[DESK]` log during a deliberate desk-landing to read the real `r_mag` impact spike; we set `CURSOR_IMPACT_THRESH` from it next.
4. Hover near-flat to point at the bottom (no impact, not past plane) → **stays engaged** (no false exit).

Report the `[DESK]` trace from a desk-landing so we tune `CURSOR_IMPACT_THRESH`. Do not claim success until observed on hardware.

---

## Self-Review

**Spec coverage (Amendment A):**
- A.1 fixed anchor → A1 Steps 1–2 (`CURSOR_VERT_TOP_DEG`, `s_vert_top` const, capture removed). ✓
- A.2 expanded range → A1 Step 1 (span 70, bottom_max 85). ✓
- A.3 dual-trigger exit → A2 Step 3 (near-flat zone, (a) impact+settle, (b) past-plane, hover stays). ✓
- A.4 telemetry → A2 Step 3 `[DESK]` line. ✓
- A.5 constants → A1 Step 1 (added/removed as listed). ✓
- A.6 cursor_track changes + test rewrite → A1 Steps 2–3. ✓
- A.7 out of scope (IMPACT_THRESH final, SURFACE revival) → IMPACT_THRESH seeded + HW-tuned in A2 Step 7; SURFACE untouched. ✓

**Placeholder scan:** none — all code given; `CURSOR_IMPACT_THRESH` is a seeded value explicitly HW-tuned later (not a blind TBD).

**Type consistency:** `gx_filt`/`r_mag`/`samples_since_activity` are existing file-scope/locals; `air_impact_seen` (bool), `air_past_plane_dwell` (int) declared once and reset in init + `_transition_to` + the exit `else`. Constants `CURSOR_VERT_TOP_DEG`, `CURSOR_DESK_ZONE_GX`, `CURSOR_PAST_PLANE_GX`, `CURSOR_PAST_PLANE_DWELL`, `CURSOR_IMPACT_THRESH`, `CURSOR_SETTLE_DWELL` defined in A1, used in A2. `MAXC = CURSOR_GAIN_Y * CURSOR_VERT_SPAN_DEG` holds because `12+70=82 < 85` (no bottom clamp).
