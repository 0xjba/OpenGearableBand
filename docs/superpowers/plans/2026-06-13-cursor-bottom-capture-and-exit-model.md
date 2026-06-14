# Cursor Bottom-Capture-at-Placement + Exit/Engage Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture the cursor's bottom anchor from where the wrist actually rests (grabbed at placement into a persistent non-aging scalar), add a stillness/tap disengage + raise re-engage exit model, and remove the dropped SURFACE mode (keeping the triple-tap gesture itself).

**Architecture:** A live "rest tracker" in `gesture_mode` records `last_rest_vert` whenever the wrist is settled in the low zone (replacing the dead `vert_hist` buffer). At AIR_MOUSE entry it feeds the *unchanged* `cursor_calib` adoption matrix, which is re-pointed from a buffer scan to that scalar. The AIR_MOUSE exit gains a stillness-held trigger alongside the existing chip-tap and past-plane. SURFACE-mode runtime is deleted; triple-tap becomes log-only.

**Tech Stack:** C++ (C-linkage modules), Zephyr/NCS on XIAO nRF52840 Sense. Host unit tests via `g++`; firmware via `./build.sh`.

**Spec:** `docs/superpowers/specs/2026-06-13-cursor-bottom-capture-and-exit-model-design.md`

---

## Plan-level decisions (deviations from the spec — flagged for the reviewer)

- **D1 — reuse the existing stillness counter instead of new variance constants.**
  The spec §7 listed `CAL_REST_STILL_DWELL` + `CAL_REST_VAR`. The codebase already
  has `samples_since_activity` (saturates at `ACTIVITY_GATE_DWELL=50` ≈ 0.5 s) as a
  stillness signal. "Settled" = in the low zone AND `samples_since_activity >=
  ACTIVITY_GATE_DWELL`; the saturated still-counter already encodes "low variance,"
  so a separate windowed-variance gate is omitted (DRY). Only **two** new constants
  are added: `CURSOR_LOW_ZONE_GX` (renames/retunes `CURSOR_DESK_ZONE_GX`) and
  `STILL_EXIT_DWELL`.
- **D2 — SURFACE removal keeps the `MODE_SURFACE` enum value as an unreachable stub.**
  All SURFACE *runtime* (entry routing, the exit branch, the motion-burst block,
  `surface_motion_*` state) is deleted and triple-tap becomes log-only, but the
  enum constant + its now-dead `_mode_str`/helper cases are left in place to keep
  this change focused and low-risk. Fully deleting the enum is a trivial follow-up.
- **Build-window note:** Task 1 changes `cursor_calib_decide`'s signature; the
  firmware caller is updated in Task 3. So `./build.sh` is NOT expected to pass
  between Task 1 and Task 3 — Task 1 is gated by its **host tests**, and the first
  firmware-build checkpoint is the END of Task 3.

## File structure

- `src/cursor_calib.h` / `src/cursor_calib.cpp` — pure decision engine. Signature
  moves from `(vert_chrono[], n)` to scalar `(bottom_candidate, bottom_valid)`;
  `find_plateau`/`median_of`/`plateau_t`/the TEST shim are removed; the adoption
  matrix is unchanged in behavior.
- `tests/test_cursor_calib.cpp` — reworked to the scalar signature.
- `src/gesture_thresholds.h` — add `CURSOR_LOW_ZONE_GX`, `STILL_EXIT_DWELL`; later
  retire the dead plateau/buffer constants.
- `src/gesture_mode.cpp` — rest tracker + `last_rest_vert`; entry rewire; exit
  model; SURFACE-runtime removal.
- `CLAUDE.md` — file-map note refresh.

---

## Task 1: `cursor_calib` — switch to a scalar bottom

**Files:**
- Modify: `src/cursor_calib.h`
- Modify: `src/cursor_calib.cpp`
- Test: `tests/test_cursor_calib.cpp` (rewrite)

Build/run line: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc` (expect `ALL PASS`).

- [ ] **Step 1: Rewrite the test file** `tests/test_cursor_calib.cpp` (replace ENTIRE contents — the old `find_plateau`/P1–P7 tests are gone):

```c
#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

int main(void)
{
    cursor_calib_result_t r;

    /* C1: cold-start + complete (top plausible, bottom valid, sweep>=25) -> SEED both. */
    r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f, /*bottom*/80.0f, /*valid*/true);
    CHECK(r.decision == CAL_SEED);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top - 12.0f) < 1e-3f);
    CHECK(fabsf(r.new_bottom - 80.0f) < 1e-3f);

    /* C2: cold-start, no bottom captured -> run on defaults, apply nothing. */
    r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f, 0.0f, false);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_COLD_START_DEFAULT);
    CHECK(r.apply == false);

    /* A1: have-calib, large shift -> ADOPT, blend both toward (20,60). */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 20.0f, 60.0f, true);
    CHECK(r.decision == CAL_ADOPT);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top    - (12.0f + CAL_BLEND_ALPHA * (20.0f - 12.0f))) < 1e-2f);
    CHECK(fabsf(r.new_bottom - (82.0f + CAL_BLEND_ALPHA * (60.0f - 82.0f))) < 1e-2f);

    /* A2: have-calib, both shifts < CAL_MIN_DELTA -> below-min-delta no-op. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 13.0f, 81.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_BELOW_MIN_DELTA);
    CHECK(r.apply == false);

    /* A3: have-calib, no bottom this session (mid-air) -> SHADOW-TRANSLATE, apply nothing. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 14.0f, 0.0f, false);
    CHECK(r.decision == CAL_SHADOW_TRANSLATE);
    CHECK(r.reason == CAL_REASON_MID_AIR_SHADOW);
    CHECK(r.apply == false);
    CHECK(fabsf(r.shadow_bottom - (14.0f + (82.0f - 12.0f))) < 1e-3f);  /* top + prior span */

    /* A4: implausible top (45 > CAL_TOP_MAX) even with a valid bottom -> REJECT implausible-top. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 45.0f, 80.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_IMPLAUSIBLE_TOP);
    CHECK(r.apply == false);

    /* A5: valid bottom but sweep too small (25 - 12 = 13 < CAL_SWEEP_MIN_DEG) -> insufficient-sweep. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 12.0f, 25.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_INSUFFICIENT_SWEEP);
    CHECK(r.apply == false);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 2: Run to verify failure.**

Run the build/run line. Expected: COMPILE error — `cursor_calib_decide` still has the old `(const float*, int)` signature, so the new 6-scalar calls don't match.

- [ ] **Step 3: Update the header** `src/cursor_calib.h`. Replace the result struct's plateau diagnostics and the function signature. Change the struct from the current fields to:

```c
typedef struct {
    cursor_calib_decision_t decision;
    cursor_calib_reason_t   reason;
    bool  apply;             /* true => caller MUST cursor_track_set_anchors(new_top,new_bottom) */
    float new_top;           /* anchor to apply (valid when apply==true)         */
    float new_bottom;        /* anchor to apply (valid when apply==true)         */
    /* Diagnostics for the [CAL] log line: */
    float bottom_candidate;  /* the captured rest vert handed in (deg); 0 if !bottom_valid */
    float sweep_deg;         /* bottom_candidate - top_now (deg); 0 if !bottom_valid       */
    float shadow_bottom;     /* would-be translate (CAL_SHADOW_TRANSLATE only)   */
} cursor_calib_result_t;
```

And replace the doc comment + signature (lines 53-64) with:

```c
/*
 * Decide whether/how to recalibrate the cursor anchors for this entry.
 *   have_calib       : false on the first entry since boot (RAM-only, no prior).
 *   prior_top/bottom : current anchors (defaults on cold start).
 *   top_now          : the snap-moment inclination (deg) = current_vert_deg().
 *   bottom_candidate : the most-recent settled desk-rest vert (deg), captured at
 *                      placement by the caller's rest tracker (does NOT age).
 *   bottom_valid     : false if no rest has been captured since boot.
 * Pure; returns a verdict. Never mutates global state.
 */
cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          float bottom_candidate, bool bottom_valid);
```

(Leave the `cursor_calib_decision_t` and `cursor_calib_reason_t` enums unchanged. `CAL_REASON_NO_PLATEAU` is now effectively unreachable but kept to avoid churn.)

- [ ] **Step 4: Rewrite `cursor_calib.cpp`.** Delete the `plateau_t` typedef, `median_of`, `find_plateau`, and the `cursor_calib_find_plateau_TEST` shim (lines 7-83). Keep the includes and `cal_blend`. Replace `cursor_calib_decide` (lines 91-167) with:

```c
cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          float bottom_candidate, bool bottom_valid)
{
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.bottom_candidate = bottom_valid ? bottom_candidate : 0.0f;
    r.sweep_deg        = bottom_valid ? (bottom_candidate - top_now) : 0.0f;
    r.shadow_bottom    = 0.0f;

    bool top_plausible = (top_now <= CAL_TOP_MAX);
    bool sweep_ok      = bottom_valid && ((bottom_candidate - top_now) >= CAL_SWEEP_MIN_DEG);
    bool ritual_complete = top_plausible && bottom_valid && sweep_ok;

    /* ---- Cold-start (RAM-only, no prior calibration). ---- */
    if (!have_calib) {
        if (ritual_complete) {
            r.decision = CAL_SEED; r.reason = CAL_REASON_OK;
            r.new_top = top_now; r.new_bottom = bottom_candidate; r.apply = true;
        } else {
            r.decision = CAL_REJECT; r.reason = CAL_REASON_COLD_START_DEFAULT; r.apply = false;
        }
        return r;
    }

    /* ---- Have prior calibration. ---- */
    if (ritual_complete) {
        bool top_moved = fabsf(top_now          - prior_top)    >= CAL_MIN_DELTA;
        bool bot_moved = fabsf(bottom_candidate  - prior_bottom) >= CAL_MIN_DELTA;
        if (!top_moved && !bot_moved) {
            r.decision = CAL_REJECT; r.reason = CAL_REASON_BELOW_MIN_DELTA; r.apply = false;
        } else {
            r.decision = CAL_ADOPT; r.reason = CAL_REASON_OK;
            r.new_top    = cal_blend(prior_top,    top_now);
            r.new_bottom = cal_blend(prior_bottom, bottom_candidate);
            r.apply      = true;
        }
        return r;
    }

    /* Ritual incomplete. Mid-air (plausible top, NO rest captured this session) ->
     * shadow only: log the would-be coupled translation, APPLY NOTHING (spec section 6). */
    if (top_plausible && !bottom_valid) {
        r.decision = CAL_SHADOW_TRANSLATE; r.reason = CAL_REASON_MID_AIR_SHADOW;
        r.shadow_bottom = top_now + (prior_bottom - prior_top);  /* preserve prior span */
        r.apply = false;
        return r;
    }

    /* Otherwise a hard reject; pick the most specific reason. */
    r.decision = CAL_REJECT; r.apply = false;
    if (!top_plausible)    r.reason = CAL_REASON_IMPLAUSIBLE_TOP;
    else if (bottom_valid) r.reason = CAL_REASON_INSUFFICIENT_SWEEP; /* rest exists, sweep too small */
    else                   r.reason = CAL_REASON_NO_PLATEAU;         /* unreachable (mid-air covers it) */
    return r;
}
```

- [ ] **Step 5: Run to verify pass.** Build/run line → `ALL PASS`.

- [ ] **Step 6: Commit.**
```bash
git add src/cursor_calib.h src/cursor_calib.cpp tests/test_cursor_calib.cpp
git commit -m "feat(calib): cursor_calib_decide takes a scalar bottom; retire find_plateau/vert_hist scan"
```

---

## Task 2: thresholds — add the low-zone + stillness-exit constants

**Files:**
- Modify: `src/gesture_thresholds.h`

- [ ] **Step 1: Add the new constants + alias the old name.** Find the existing line `#define CURSOR_DESK_ZONE_GX  ...` (currently `1.7f`). Replace that single line with:

```c
/* Low/near-flat zone: gx_filt below this = "low zone" for rest-capture + the
 * stillness/tap disengage.  Seed 2.5 ~= vert > 75 deg, ~5-9 deg margin below the
 * measured ~80-84 deg desk rest (cf. old DESK_ZONE 1.7 ~= vert>80).  M2-tuned. [USER] */
#define CURSOR_LOW_ZONE_GX              2.5f
/* Deprecated alias so existing call sites keep building until repointed (removed in
 * the cleanup task once no references remain). */
#define CURSOR_DESK_ZONE_GX             CURSOR_LOW_ZONE_GX

/* Stillness-held disengage: consecutive settled-in-low-zone samples (still =
 * samples_since_activity >= ACTIVITY_GATE_DWELL) required to exit AIR_MOUSE.  Seed
 * 120 (~1.2 s) -- long enough a floating-to-point hand can't hold it, short enough
 * a real desk rest fires.  M1-tuned. [USER] */
#define STILL_EXIT_DWELL                120
```

- [ ] **Step 2: Verify it still builds (host test unaffected; firmware build deferred per the build-window note).** Quick sanity: the cursor_calib host test must still pass (it includes this header):
Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: `ALL PASS`.

- [ ] **Step 3: Commit.**
```bash
git add src/gesture_thresholds.h
git commit -m "feat(cursor): add CURSOR_LOW_ZONE_GX (2.5) + STILL_EXIT_DWELL; alias old DESK_ZONE"
```

---

## Task 3: `gesture_mode` — rest tracker + remove vert_hist + rewire entry

**Files:**
- Modify: `src/gesture_mode.cpp` (statics ~308; `vert_hist_chronological` ~527-541; `cursor_calib_run_on_entry` ~566-596; the push ~1237-1239; rest tracker into `gesture_mode_update_accel` after the `samples_since_activity` update ~960)

This is the coupled core; **`./build.sh` is verified at the end of this task** (it closes the signature window opened in Task 1).

- [ ] **Step 1: Replace the `vert_hist` statics with the rest-tracker statics.** Find the block (the `vert_hist[VERT_HIST_SAMPLES]` / `vert_hist_idx` / `vert_hist_primed` declarations + their comment, ~lines 294-310) and replace it with:

```c
/* Bottom-anchor capture (replaces the old vert_hist[] buffer, which aged the rest
 * out before the >3 s raise->snap ritual completed -- see the 2026-06-13 [CALDBG]
 * finding).  `last_rest_vert` is a PERSISTENT scalar (never ages): the most-recent
 * settled desk-rest angle, captured at placement (the motion of setting the wrist
 * down keeps sampling awake long enough to register the settle).  Fed to
 * cursor_calib at AIR_MOUSE entry.  `rest_dwell` counts consecutive settled-low
 * samples and drives the stillness-held disengage. */
static float last_rest_vert  = 0.0f;
static bool  bottom_valid     = false;
static int   rest_dwell       = 0;
```

(`cursor_have_calib` already exists below this block — leave it.)

- [ ] **Step 2: Delete `vert_hist_chronological`.** Remove the whole function (the comment + body, ~lines 527-541).

- [ ] **Step 3: Add the rest tracker** in `gesture_mode_update_accel`, immediately AFTER the `samples_since_activity` update (the `if (...) samples_since_activity = 0; else if (samples_since_activity < ACTIVITY_GATE_DWELL) samples_since_activity++;` block, ~line 960). Insert:

```c
    /* ---- Rest tracker: capture the bottom anchor + drive the stillness-exit dwell.
     * "Settled" = in the low zone AND still (activity gate saturated -> low variance).
     * current_vert_deg() is stable when still, so it's a clean rest reading.  Runs in
     * ALL states; captured at placement (motion -> sampling awake -> settle).  The
     * scalar never ages, so it bridges the >3 s gap to the entry snap. ---- */
    {
        bool low_zone = (gx_filt < CURSOR_LOW_ZONE_GX);
        bool still    = (samples_since_activity >= ACTIVITY_GATE_DWELL);
        if (low_zone && still) {
            last_rest_vert = current_vert_deg();   /* auto-tracks the latest rest */
            bottom_valid   = true;
            if (rest_dwell < STILL_EXIT_DWELL) rest_dwell++;
            static int restdbg = 0;
            if (++restdbg % 10 == 0) {
                LOG_INF("[REST] vert=%d still=%d dwell=%d last_rest=%d",
                        (int)current_vert_deg(), (int)samples_since_activity,
                        rest_dwell, (int)last_rest_vert);
            }
        } else {
            rest_dwell = 0;
        }
    }
```

- [ ] **Step 4: Delete the vert_hist push** in `gesture_mode_update_gyro` (~lines 1235-1239 — the `/* Mirror push into the absolute-vert calibration history. */` comment + the three `vert_hist[...]`/`vert_hist_idx`/`vert_hist_primed` lines). Remove all of it.

- [ ] **Step 5: Rewire `cursor_calib_run_on_entry`** (~lines 569-596) to use the scalar. Replace the whole function with:

```c
/* Run natural calibration for an AIR_MOUSE entry.  MUST be called BEFORE
 * cursor_track_start (the entry slam is sized from the anchors).  top_now is the
 * snap-moment inclination; the bottom comes from the persistent last_rest_vert. */
static void cursor_calib_run_on_entry(float top_now)
{
    float prior_top    = cursor_track_vert_top();
    float prior_bottom = cursor_track_vert_bottom();

    cursor_calib_result_t res =
        cursor_calib_decide(cursor_have_calib, prior_top, prior_bottom,
                            top_now, last_rest_vert, bottom_valid);

    if (res.apply) {
        cursor_track_set_anchors(res.new_top, res.new_bottom);
        cursor_have_calib = true;
    }

    LOG_INF("[CAL] top %d->%d bottom %d->%d span=%d | "
            "bottom_valid=%d cand=%d sweep=%d | decision=%s reason=%s shadow_bottom=%d",
            (int)prior_top, (int)(res.apply ? res.new_top : prior_top),
            (int)prior_bottom, (int)(res.apply ? res.new_bottom : prior_bottom),
            (int)((res.apply ? res.new_bottom : prior_bottom) -
                  (res.apply ? res.new_top : prior_top)),
            (int)bottom_valid, (int)res.bottom_candidate, (int)res.sweep_deg,
            cal_decision_str(res.decision), cal_reason_str(res.reason),
            (int)res.shadow_bottom);
}
```

- [ ] **Step 6: Build the firmware** (closes the Task-1 signature window).
Run: `./build.sh`
Expected: clean build, `Flashable artifact: build/zephyr/zephyr.uf2`. No references to `vert_hist`/`vert_hist_chronological` remain (grep to confirm: `grep -n vert_hist src/gesture_mode.cpp` → nothing).

- [ ] **Step 7: Commit.**
```bash
git add src/gesture_mode.cpp
git commit -m "feat(calib): rest tracker (persistent last_rest_vert at placement); drop vert_hist; entry uses scalar"
```

---

## Task 4: `gesture_mode` — remove SURFACE-mode runtime

**Files:**
- Modify: `src/gesture_mode.cpp` (triple-tap ~1498-1512; exit `else` branch ~1088-1102; motion-burst block ~1126-1171; `surface_motion_*` statics ~102-111; `in_cursor_mode` ~1011-1012)

- [ ] **Step 1: Make triple-tap log-only.** In `gesture_mode_on_chip_triple_tap` (~1498), replace the `if (current_mode == MODE_IDLE) { ... _transition_to(MODE_SURFACE); }` body so it never enters a mode:

```c
void gesture_mode_on_chip_triple_tap(void)
{
    GestureMode current_mode = (GestureMode)atomic_get(&mode_atomic);
    /* SURFACE mode was dropped (roadmap 2026-06-13).  Triple-tap is KEPT as a free,
     * unbound trigger to repurpose later -- log-only for now, enters no mode. */
    LOG_INF("Chip triple-tap (unbound) from %s -- no mode entry (SURFACE removed)",
            _mode_str(current_mode));
}
```
(Delete the old `if/else` body entirely; keep the function signature.)

- [ ] **Step 2: Simplify `in_cursor_mode`** (~1011-1012). AIR_MOUSE is now the only cursor mode:
```c
    bool in_cursor_mode = (current_mode == MODE_AIR_MOUSE);
```

- [ ] **Step 3: Remove the SURFACE exit `else` branch.** In the exit block, the `if (current_mode == MODE_AIR_MOUSE) { ... } else { ... }` — delete the entire `else { /* SURFACE ... orientation-drop exit */ ... }` (~lines 1088-1102). The AIR_MOUSE `if` body stays (it's rewritten in Task 5). After this, the structure is just `if (current_mode == MODE_AIR_MOUSE) { ... }` with no else.

- [ ] **Step 4: Remove the SURFACE motion-burst block** (~lines 1119-1171) — the entire `/* SURFACE motion-burst exit detector. ... */` comment + `if (current_mode == MODE_SURFACE && cursor_has_reached_pose) { ... }` block. Delete all of it.

- [ ] **Step 5: Remove the `surface_motion_*` statics** (~lines 102-111): `surface_motion_burst_dwell`, `surface_motion_peak`, `surface_motion_log_counter`. Delete those declarations. Then grep for any remaining references and remove them:
Run: `grep -n "surface_motion" src/gesture_mode.cpp`
For each remaining hit (e.g. resets inside `_transition_to` ~638-640 and ~796-798), delete those lines too.

- [ ] **Step 6: Build.**
Run: `./build.sh`
Expected: clean build. (If the compiler flags an unused `_exit_dwell_for` or a `MODE_SURFACE` switch case, that's fine — `MODE_SURFACE` enum + its dead `_mode_str`/helper cases are intentionally left per decision D2; do NOT chase them unless the build errors.)

- [ ] **Step 7: Commit.**
```bash
git add src/gesture_mode.cpp
git commit -m "refactor(gesture): remove SURFACE-mode runtime; triple-tap is now log-only (unbound)"
```

---

## Task 5: `gesture_mode` — the exit/engage model (stillness-held + low-zone)

**Files:**
- Modify: `src/gesture_mode.cpp` (the AIR_MOUSE exit branch ~1054-1087 after Task 4's edits; the desk-tap setter in `gesture_mode_on_chip_single_tap` ~1327)

- [ ] **Step 1: Rewrite the AIR_MOUSE exit branch.** Replace the body of `if (current_mode == MODE_AIR_MOUSE) { ... }` (the `near_flat`/`desk_tap`/`past_plane`/`[DESK]` block, now ~1054-1087) with:

```c
        if (current_mode == MODE_AIR_MOUSE) {
            bool low_zone = (gx_filt < CURSOR_LOW_ZONE_GX);

            /* (a) chip tap in the low zone (desk landing). Read-and-clear the latch
             * every tick (atomic_set returns the prior value) so it can't go stale. */
            bool tapped   = (atomic_set(&air_desk_tap, 0) != 0);
            bool desk_tap = low_zone && tapped;

            /* (b) stillness-held: settled in the low zone for STILL_EXIT_DWELL (the
             * rest tracker maintains rest_dwell). Long enough a floating hand can't
             * hold it; a parked desk rest fires. */
            bool still_held = (rest_dwell >= STILL_EXIT_DWELL);

            /* (c) past-plane (no desk): signed gx crosses past horizontal (drooping). */
            if (gx_filt < CURSOR_PAST_PLANE_GX) {
                if (air_past_plane_dwell < CURSOR_PAST_PLANE_DWELL) air_past_plane_dwell++;
            } else {
                air_past_plane_dwell = 0;
            }
            bool past_plane = (air_past_plane_dwell >= CURSOR_PAST_PLANE_DWELL);

            if      (desk_tap)   { do_exit = true; exit_reason = "desk contact (chip tap)"; }
            else if (still_held) { do_exit = true; exit_reason = "rest-settle (stillness)"; }
            else if (past_plane) { do_exit = true; exit_reason = "past horizontal plane (no desk)"; }
        }
```
(The `[DESK]` periodic log is dropped — the new `[REST]` log in Task 3 covers the low-zone telemetry. The shared `if (do_exit) { ... }` block below stays unchanged.)

- [ ] **Step 2: Repoint the desk-tap setter** in `gesture_mode_on_chip_single_tap` (~1327). Change:
```c
    if (current_mode == MODE_AIR_MOUSE && gx_filt < CURSOR_DESK_ZONE_GX) {
```
to:
```c
    if (current_mode == MODE_AIR_MOUSE && gx_filt < CURSOR_LOW_ZONE_GX) {
```

- [ ] **Step 3: Build.**
Run: `./build.sh`
Expected: clean build, artifact produced. Confirm no `CURSOR_DESK_ZONE_GX` references remain except its alias definition:
Run: `grep -rn CURSOR_DESK_ZONE_GX src/` → only the alias line in `gesture_thresholds.h`.

- [ ] **Step 4: Commit.**
```bash
git add src/gesture_mode.cpp
git commit -m "feat(cursor): AIR_MOUSE exit = low-zone + (chip tap OR stillness-held OR past-plane)"
```

---

## Task 6: cleanup dead constants + docs + final verification

**Files:**
- Modify: `src/gesture_thresholds.h` (remove dead constants + the alias)
- Modify: `CLAUDE.md` (file-map note)

- [ ] **Step 1: Confirm the retired constants are truly unused**, then remove them.
Run: `grep -rn "VERT_HIST_SAMPLES\|CAL_PLATEAU_VAR\|CAL_PLATEAU_DWELL\|CURSOR_DESK_ZONE_GX" src/`
Expected: only their *definitions* in `gesture_thresholds.h` (no use sites). If any use site remains, STOP and report (a prior task missed something).

- [ ] **Step 2: Remove the dead definitions** from `src/gesture_thresholds.h`: delete the `VERT_HIST_SAMPLES`, `CAL_PLATEAU_VAR`, `CAL_PLATEAU_DWELL` lines (from the calibration block) and the `#define CURSOR_DESK_ZONE_GX CURSOR_LOW_ZONE_GX` alias line.

- [ ] **Step 3: Update `cursor_calib`'s header doc** in `src/cursor_calib.h` — the top comment still says "The CALLER owns the vert history buffer, linearises it...". Replace that sentence with: "The CALLER captures the resting bottom angle at placement (a persistent scalar) and the top at the snap, and applies the verdict via cursor_track_set_anchors()."

- [ ] **Step 4: Update `CLAUDE.md` file map** — the `src/cursor_calib.{h,cpp}` bullet mentions "extracts the resting-pose bottom anchor from a `vert` history". Change "from a `vert` history" to "from a placement-captured rest scalar (`last_rest_vert`, owned by `gesture_mode`)".

- [ ] **Step 5: Full verification.**
Run:
```
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc
./build.sh
```
Expected: both host suites `ALL PASS`; firmware builds clean with the artifact line.

- [ ] **Step 6: Commit.**
```bash
git add src/gesture_thresholds.h src/cursor_calib.h CLAUDE.md
git commit -m "chore(calib): retire vert_hist/plateau constants + DESK_ZONE alias; refresh docs"
```

---

## Verify-first measurements (HW, post-flash — NOT code tasks)

After flashing, before trusting the seeds, gather the data the spec's M1/M2 call for:
- **M1 (stillness dwell):** read `[REST] dwell=` while (a) resting on the desk and (b) floating low trying to hold the cursor at the bottom. Set `STILL_EXIT_DWELL` so a floating hand never reaches it; rebuild.
- **M2 (low-zone range):** read `[REST] vert=` for your actual rest(s). Set `CURSOR_LOW_ZONE_GX` ~5° below the measured lowest rest. If you ever rest much lower than ~80°, apply the spec §4 fallback (stillness-exit only in the deepest sub-band).

These tune the two `[USER]` seeds; they do not change code structure.

## HW acceptance (spec §9)
- rest → raise → hold any duration → double-tap → `[CAL] decision=SEED/ADOPT`, `bottom` ≈ your rest; cursor reaches both edges.
- re-wear / rest at a new height → next entry `decision=ADOPT`.
- settle flat on the desk in AIR_MOUSE → `exit: rest-settle (stillness)`; point low but keep moving → STAYS engaged.
- chip-tap near-flat → `exit: desk contact (chip tap)`; hand drops past plane → `exit: past horizontal plane`.
- triple-tap → `(unbound)` log, no mode entry.
