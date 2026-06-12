# Natural (entry-time) Cursor Calibration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Auto-recalibrate both ends of the absolute-Y cursor map (`vert_top`, `vert_bottom`) from the air-mouse entry ritual itself, so each day's band mount self-calibrates with no manual step.

**Architecture:** A new **pure** decision module `cursor_calib` (host-unit-tested) extracts the resting-pose "bottom" from a `vert` history buffer and scores the entry ritual, returning an adoption verdict. `gesture_mode` owns the history buffer and, at AIR_MOUSE entry, runs `decide → set_anchors → cursor_track_start` (strict order) and logs `[CAL]`. `cursor_track` gains runtime-variable anchors. RAM-only (no NVS).

**Tech Stack:** C++ (C-linkage modules), Zephyr/NCS on XIAO nRF52840 Sense. Host unit tests via `g++` per `CLAUDE.md`. Firmware build via `./build.sh`.

**Spec:** `docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md`

---

## Scope deferral (read before starting)

The spec §5 lists four ritual signals. **This plan implements signals 1 (plateau-from-rest) and 2 (sweep magnitude)** as the hard adoption gate — these alone constitute "came from rest AND swept a large range," and `find_plateau` enforces both. **Signals 3 (brief dwell-at-top) and 4 (gyro peak-rate corroboration) are deferred** — §5 marks signal 4 explicitly optional, and neither has a constant in the §8 table, so adding them now would be an un-specced expansion. The rich `[CAL]` telemetry (Task 6) will reveal on hardware whether top-quality needs the dwell gate; if so, it is a clean follow-up (one constant + one predicate). This deferral is intentional and YAGNI-aligned; flag it if the reviewer disagrees before building.

## File Structure

- **Create `src/cursor_calib.h`** — pure interface: result struct, decision/reason enums, `cursor_calib_decide()`. No Zephyr.
- **Create `src/cursor_calib.cpp`** — `find_plateau()` (median/variance/high-vert plateau scan) + `cursor_calib_decide()` (adoption matrix). No Zephyr.
- **Create `tests/test_cursor_calib.cpp`** — host unit tests (mirrors `tests/test_cursor_track.cpp` style: `CHECK` macro, `ALL PASS`/`FAILURES` print, `return failures?1:0`).
- **Modify `src/gesture_thresholds.h`** — add `VERT_HIST_SAMPLES` + `CAL_*` constants.
- **Modify `src/cursor_track.h` / `src/cursor_track.cpp`** — variable anchors: `cursor_track_set_anchors()`, `cursor_track_vert_bottom()`, `s_vert_bottom`.
- **Modify `tests/test_cursor_track.cpp`** — append anchor-setter tests.
- **Modify `src/gesture_mode.cpp`** — `vert_hist[300]` ring buffer + linearize helper; entry hook (decide→set→start) at `_transition_to` MODE_AIR_MOUSE branch; `[CAL]` telemetry.
- **Modify `CLAUDE.md`** — add `cursor_calib` to the file map.

---

## Task 1: Scaffold `cursor_calib` (interface + constants + buildable test)

**Files:**
- Create: `src/cursor_calib.h`
- Create: `src/cursor_calib.cpp`
- Create: `tests/test_cursor_calib.cpp`
- Modify: `src/gesture_thresholds.h` (before the final `#endif`)

- [ ] **Step 1: Add the constants** to `src/gesture_thresholds.h`, immediately before the closing `#endif /* GESTURE_THRESHOLDS_H */`:

```c
/* ---- Natural entry-time cursor calibration (auto top+bottom anchors) ----
 * See docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md.
 * All seeds; tune from the per-entry [CAL] traces. */
#define VERT_HIST_SAMPLES     300     /* 3 s of vert history @100Hz, ~1.2KB. Deep on
                                       * purpose: an undersized buffer fails INVISIBLY
                                       * as no-plateau (masquerades as a lazy entry). [STRUCTURAL] */
#define CAL_PLATEAU_VAR       4.0f    /* max vert variance (deg^2, ~2 deg std) for "resting still" [USER] */
#define CAL_PLATEAU_DWELL     30      /* min plateau length (samples, ~300 ms) [USER] */
#define CAL_SWEEP_MIN_DEG     25.0f   /* min (bottom - top) to count as a real raise [USER] */
#define CAL_TOP_MAX           30.0f   /* plausibility clamp: reject a top above this (lazy raise) [USER] */
#define CAL_MIN_DELTA         5.0f    /* ignore anchor shifts smaller than this (stable within a wear) [USER] */
#define CAL_BLEND_ALPHA       0.4f    /* adoption blend factor (1.0 effective on cold-start seed) [USER] */
```

- [ ] **Step 2: Create `src/cursor_calib.h`** (full interface — no stubs to fill later):

```c
#ifndef CURSOR_CALIB_H
#define CURSOR_CALIB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Natural (entry-time) cursor calibration -- PURE decision engine.
 * No Zephyr / no I/O / no threads, so it is host-unit-testable like
 * cursor_track.  The CALLER (gesture_mode) owns the vert history buffer,
 * linearises it into chronological order (oldest -> newest), and applies
 * the verdict via cursor_track_set_anchors().
 * Geometry: vert = acos(|gx|/|g|); SMALL when raised (screen top),
 * LARGE when flat (screen bottom).  So bottom > top in degrees.
 * See docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md.
 */

typedef enum {
    CAL_SEED,             /* cold-start: both anchors seeded from this entry      */
    CAL_ADOPT,            /* both anchors trusted, blended toward the new capture  */
    CAL_SHADOW_TRANSLATE, /* mid-air entry: NOTHING applied; shadow_bottom logged  */
    CAL_REJECT,           /* no change                                             */
} cursor_calib_decision_t;

typedef enum {
    CAL_REASON_OK,
    CAL_REASON_COLD_START_DEFAULT, /* cold-start + incomplete ritual -> run on defaults */
    CAL_REASON_NO_PLATEAU,
    CAL_REASON_INSUFFICIENT_SWEEP, /* a low-var rest existed but too close to top        */
    CAL_REASON_IMPLAUSIBLE_TOP,
    CAL_REASON_BELOW_MIN_DELTA,
    CAL_REASON_MID_AIR_SHADOW,
} cursor_calib_reason_t;

typedef struct {
    cursor_calib_decision_t decision;
    cursor_calib_reason_t   reason;
    bool  apply;             /* true => caller MUST cursor_track_set_anchors(new_top,new_bottom) */
    float new_top;           /* anchor to apply (valid when apply==true)         */
    float new_bottom;        /* anchor to apply (valid when apply==true)         */
    /* Diagnostics for the [CAL] log line: */
    bool  plateau_found;
    float plateau_var;       /* variance of the chosen plateau (deg^2)           */
    int   plateau_n;         /* length of the chosen plateau (samples)           */
    float bottom_candidate;  /* median vert of the plateau (deg); 0 if none      */
    float sweep_deg;         /* bottom_candidate - top_now (deg); 0 if no plateau */
    float shadow_bottom;     /* would-be translate (CAL_SHADOW_TRANSLATE only)   */
} cursor_calib_result_t;

/*
 * Decide whether/how to recalibrate the cursor anchors for this entry.
 *   have_calib    : false on the first entry since boot (RAM-only, no prior).
 *   prior_top/bottom : current anchors (defaults on cold start).
 *   top_now       : the snap-moment inclination (deg) = current_vert_deg().
 *   vert_chrono   : n samples of vert (deg) in CHRONOLOGICAL order (oldest first).
 * Pure; returns a verdict. Never mutates global state.
 */
cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n);

#ifdef __cplusplus
}
#endif

#endif /* CURSOR_CALIB_H */
```

- [ ] **Step 3: Create `src/cursor_calib.cpp`** as a compiling stub (real logic in Tasks 2-4). This lets the test harness build now:

```c
#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <math.h>

cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n)
{
    (void)have_calib; (void)top_now; (void)vert_chrono; (void)n;
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.plateau_found = false;
    r.plateau_var = 0.0f;
    r.plateau_n = 0;
    r.bottom_candidate = 0.0f;
    r.sweep_deg = 0.0f;
    r.shadow_bottom = 0.0f;
    return r;
}
```

- [ ] **Step 4: Create `tests/test_cursor_calib.cpp`** with the harness + one trivial assertion (stub returns REJECT):

```c
#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

/* Fill buf[0..n) with `val`. */
static void fill(float *buf, int n, float val) { for (int i=0;i<n;i++) buf[i]=val; }

int main(void)
{
    float v[VERT_HIST_SAMPLES];

    /* T0 (stub): empty/zero history -> REJECT (no plateau). */
    fill(v, VERT_HIST_SAMPLES, 0.0f);
    cursor_calib_result_t r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f,
                                                  v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.apply == false);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 5: Build + run the test** (establishes the build line used for the rest of the plan):

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: `ALL PASS`

- [ ] **Step 6: Commit**

```bash
git add src/cursor_calib.h src/cursor_calib.cpp tests/test_cursor_calib.cpp src/gesture_thresholds.h
git commit -m "feat(calib): scaffold pure cursor_calib module + constants"
```

---

## Task 2: `find_plateau` — resting-pose extraction

**Files:**
- Modify: `src/cursor_calib.cpp`
- Test: `tests/test_cursor_calib.cpp`

`find_plateau` scans the chronological `vert` history for the longest contiguous low-variance run whose median is HIGH (a lowered/resting posture, `median >= top_now + CAL_SWEEP_MIN_DEG`), returning that median as the bottom candidate. Greedy O(n) variance segmentation (Welford), median by sort.

- [ ] **Step 1: Write failing tests.** Append these blocks to `tests/test_cursor_calib.cpp` *before* the `printf(...)` line. They exercise an internal `find_plateau` exposed for testing — so first add its declaration to the top of the test file (under the includes):

```c
/* Test-only visibility into the plateau extractor (defined in cursor_calib.cpp). */
typedef struct { bool found; float median; float var; int len; bool any_lowvar; } plateau_t;
extern "C" plateau_t cursor_calib_find_plateau_TEST(const float *v, int n, float top_now);
```

Then the test blocks:

```c
    /* P1: a clean rest (flat ~80 for 1s) then a raise to ~12 -> plateau at 80. */
    fill(v, VERT_HIST_SAMPLES, 80.0f);                 /* whole buffer rest @80 */
    for (int i = VERT_HIST_SAMPLES - 50; i < VERT_HIST_SAMPLES; i++)
        v[i] = 80.0f - (float)(i - (VERT_HIST_SAMPLES - 50)); /* last 50: ramp down to ~30 */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 80.0f) < 1.5f);
        CHECK(p.len >= CAL_PLATEAU_DWELL);
    }

    /* P2: no rest (constant slow drift, never still) -> no plateau. */
    for (int i = 0; i < VERT_HIST_SAMPLES; i++) v[i] = 10.0f + 0.5f * i; /* var huge */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(!p.found);
    }

    /* P3: a low-var rest but too close to top (rest @25, top 12, sweep 13 < 25)
     *     -> not a qualifying plateau, but any_lowvar flags it (insufficient sweep). */
    fill(v, VERT_HIST_SAMPLES, 25.0f);
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(!p.found);
        CHECK(p.any_lowvar);
    }

    /* P4 (slow-raise / §3 invisible-starvation guard): rest plateau sits at the
     *    OLD edge of the buffer, followed by a long slow climb that fills the rest.
     *    Plateau must still be found (buffer is deep enough at 300). */
    fill(v, VERT_HIST_SAMPLES, 78.0f);                 /* default fill */
    for (int i = 0; i < 40; i++) v[i] = 78.0f;         /* oldest 40 = rest @78 */
    for (int i = 40; i < VERT_HIST_SAMPLES; i++)
        v[i] = 78.0f - 0.25f * (float)(i - 40);        /* slow ~0.25 deg/sample climb */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 78.0f) < 2.0f);
    }

    /* P5: two rests (one at top dwell @14, one lowered @70); pick the HIGH-vert one. */
    fill(v, VERT_HIST_SAMPLES, 70.0f);                 /* lowered rest fills most */
    for (int i = VERT_HIST_SAMPLES - 40; i < VERT_HIST_SAMPLES; i++) v[i] = 14.0f; /* recent top dwell */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 70.0f) < 1.5f);         /* the lowered rest, NOT the 14 dwell */
    }
```

- [ ] **Step 2: Run to verify failure.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: FAIL to **link** — `undefined reference to cursor_calib_find_plateau_TEST` (function not yet implemented).

- [ ] **Step 3: Implement `find_plateau`** in `src/cursor_calib.cpp`. Add at the top (after includes), and add the test shim. Replace the file's includes block and insert before `cursor_calib_decide`:

```c
#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <math.h>

/* Internal plateau result (kept in the .cpp; the test file re-declares a
 * layout-compatible struct for the TEST shim below). */
typedef struct { bool found; float median; float var; int len; bool any_lowvar; } plateau_t;

/* Median of a[0..n) via insertion sort on a local copy (n <= VERT_HIST_SAMPLES). */
static float median_of(const float *a, int n)
{
    float tmp[VERT_HIST_SAMPLES];
    for (int i = 0; i < n; i++) tmp[i] = a[i];
    for (int i = 1; i < n; i++) {
        float k = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    return (n & 1) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

/* Scan chronological vert[] for the highest-median low-variance resting plateau.
 * Greedy O(n) segmentation: grow a run via incremental (Welford) variance; when a
 * sample would push the run variance >= CAL_PLATEAU_VAR, close the run and start a
 * new one. A run qualifies as the resting bottom if it is long enough AND lowered
 * enough (median >= top_now + CAL_SWEEP_MIN_DEG). any_lowvar records that *some*
 * qualifying-length still run existed (even if too close to top) so the caller can
 * distinguish "no rest at all" from "rest, but insufficient sweep". */
static plateau_t find_plateau(const float *v, int n, float top_now)
{
    plateau_t best = { false, 0.0f, 0.0f, 0, false };
    float best_median = -1.0f;

    int   run_start = 0;
    double mean = 0.0, M2 = 0.0;
    int    cnt = 0;

    for (int i = 0; i <= n; i++) {
        bool close_run = (i == n);

        if (!close_run) {
            int    newcnt  = cnt + 1;
            double d       = (double)v[i] - mean;
            double newmean = mean + d / newcnt;
            double newM2   = M2 + d * ((double)v[i] - newmean);
            double var     = (newcnt > 1) ? newM2 / newcnt : 0.0;
            if (cnt > 0 && var >= CAL_PLATEAU_VAR) {
                close_run = true;            /* exclude v[i]; restart the run at i */
            } else {
                mean = newmean; M2 = newM2; cnt = newcnt;
            }
        }

        if (close_run) {
            if (cnt >= CAL_PLATEAU_DWELL) {
                float med = median_of(&v[run_start], cnt);
                float var = (float)(cnt > 1 ? M2 / cnt : 0.0);
                best.any_lowvar = true;      /* a long still run exists */
                if (med >= top_now + CAL_SWEEP_MIN_DEG && med > best_median) {
                    best_median = med;
                    best.found  = true;
                    best.median = med;
                    best.var    = var;
                    best.len    = cnt;
                }
            }
            run_start = i;
            if (i < n) { mean = v[i]; M2 = 0.0; cnt = 1; } else { cnt = 0; }
        }
    }
    return best;
}

/* Test-only shim: expose find_plateau to the host unit test. */
extern "C" plateau_t cursor_calib_find_plateau_TEST(const float *v, int n, float top_now)
{
    return find_plateau(v, n, top_now);
}
```

(Delete the now-duplicate `#include`/`#include`/`#include <math.h>` lines that were at the top of the Task-1 stub — keep only the single includes block shown above.)

- [ ] **Step 4: Run to verify pass.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: `ALL PASS`

- [ ] **Step 5: Commit**

```bash
git add src/cursor_calib.cpp tests/test_cursor_calib.cpp
git commit -m "feat(calib): find_plateau resting-pose extraction + tests"
```

---

## Task 3: `cursor_calib_decide` — cold-start path

**Files:**
- Modify: `src/cursor_calib.cpp`
- Test: `tests/test_cursor_calib.cpp`

- [ ] **Step 1: Write failing tests.** Append before the `printf` in `tests/test_cursor_calib.cpp`:

```c
    /* C1 (cold-start + complete ritual): rest @80 then raised; seed BOTH anchors. */
    fill(v, VERT_HIST_SAMPLES, 80.0f);
    for (int i = VERT_HIST_SAMPLES - 40; i < VERT_HIST_SAMPLES; i++) v[i] = 12.0f;
    r = cursor_calib_decide(false /*have_calib*/, 12.0f, 82.0f, 12.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_SEED);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top - 12.0f) < 1e-3f);
    CHECK(fabsf(r.new_bottom - 80.0f) < 1.5f);

    /* C2 (cold-start + incomplete ritual): no rest -> run on defaults, apply nothing. */
    for (int i = 0; i < VERT_HIST_SAMPLES; i++) v[i] = 10.0f + 0.5f * i;
    r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_COLD_START_DEFAULT);
    CHECK(r.apply == false);
```

- [ ] **Step 2: Run to verify failure.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: FAIL (stub returns `CAL_REJECT`/`CAL_REASON_NO_PLATEAU`, so C1 and C2's reason fail).

- [ ] **Step 3: Implement the cold-start branch.** Replace the stub body of `cursor_calib_decide` in `src/cursor_calib.cpp` with this (the have_calib branch is filled in Task 4 — for now it falls through to a REJECT so the function is complete and compiles):

```c
static float cal_blend(float old_v, float new_v)
{
    return old_v + CAL_BLEND_ALPHA * (new_v - old_v);
}

cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n)
{
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.shadow_bottom = 0.0f;

    bool top_plausible = (top_now <= CAL_TOP_MAX);
    plateau_t p = find_plateau(vert_chrono, n, top_now);

    r.plateau_found    = p.found;
    r.plateau_var      = p.var;
    r.plateau_n        = p.len;
    r.bottom_candidate = p.found ? p.median : 0.0f;
    r.sweep_deg        = p.found ? (p.median - top_now) : 0.0f;

    bool ritual_complete = top_plausible && p.found;  /* sweep enforced inside find_plateau */

    /* ---- Cold-start: every boot starts here (RAM-only, no prior calibration). ---- */
    if (!have_calib) {
        if (ritual_complete) {
            r.decision    = CAL_SEED;
            r.reason      = CAL_REASON_OK;
            r.new_top     = top_now;
            r.new_bottom  = p.median;
            r.apply       = true;
        } else {
            r.decision = CAL_REJECT;
            r.reason   = CAL_REASON_COLD_START_DEFAULT;  /* run on compile-time defaults */
            r.apply    = false;
        }
        return r;
    }

    /* ---- have_calib branch: filled in Task 4. ---- */
    (void)prior_bottom; (void)cal_blend;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    return r;
}
```

- [ ] **Step 4: Run to verify pass.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: `ALL PASS`

- [ ] **Step 5: Commit**

```bash
git add src/cursor_calib.cpp tests/test_cursor_calib.cpp
git commit -m "feat(calib): cold-start seed/default decision + tests"
```

---

## Task 4: `cursor_calib_decide` — have_calib paths (adopt / below-delta / shadow / reject)

**Files:**
- Modify: `src/cursor_calib.cpp`
- Test: `tests/test_cursor_calib.cpp`

- [ ] **Step 1: Write failing tests.** Append before the `printf`:

```c
    /* A1 (both trusted, large shift): re-wear moved mount; rest now @60, top now @20.
     *     Adopt via blend toward (20,60) from prior (12,82). */
    fill(v, VERT_HIST_SAMPLES, 60.0f);
    for (int i = VERT_HIST_SAMPLES - 40; i < VERT_HIST_SAMPLES; i++) v[i] = 20.0f;
    r = cursor_calib_decide(true, 12.0f, 82.0f, 20.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_ADOPT);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top    - (12.0f + CAL_BLEND_ALPHA * (20.0f - 12.0f))) < 1e-2f);
    CHECK(fabsf(r.new_bottom - (82.0f + CAL_BLEND_ALPHA * (60.0f - 82.0f))) < 0.6f);

    /* A2 (both trusted, tiny shift below CAL_MIN_DELTA): no-op. prior (12,82),
     *     capture ~ (13, 81) -> both deltas < 5 -> REJECT below-min-delta. */
    fill(v, VERT_HIST_SAMPLES, 81.0f);
    for (int i = VERT_HIST_SAMPLES - 40; i < VERT_HIST_SAMPLES; i++) v[i] = 13.0f;
    r = cursor_calib_decide(true, 12.0f, 82.0f, 13.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_BELOW_MIN_DELTA);
    CHECK(r.apply == false);

    /* A3 (mid-air: top plausible, no plateau): SHADOW-TRANSLATE; apply NOTHING. */
    for (int i = 0; i < VERT_HIST_SAMPLES; i++) v[i] = 10.0f + 0.5f * i; /* no rest */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 14.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_SHADOW_TRANSLATE);
    CHECK(r.reason == CAL_REASON_MID_AIR_SHADOW);
    CHECK(r.apply == false);                                   /* nothing applied */
    CHECK(fabsf(r.shadow_bottom - (14.0f + (82.0f - 12.0f))) < 1e-3f); /* top_now + prior span */

    /* A4 (implausible top: snapped at 45 > CAL_TOP_MAX): REJECT implausible-top, no-op. */
    fill(v, VERT_HIST_SAMPLES, 80.0f);
    r = cursor_calib_decide(true, 12.0f, 82.0f, 45.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_IMPLAUSIBLE_TOP);
    CHECK(r.apply == false);

    /* A5 (insufficient sweep: rest exists but only @25, top 12 -> sweep 13 < 25):
     *     REJECT insufficient-sweep, no-op. */
    fill(v, VERT_HIST_SAMPLES, 25.0f);
    r = cursor_calib_decide(true, 12.0f, 82.0f, 12.0f, v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_INSUFFICIENT_SWEEP);
    CHECK(r.apply == false);
```

- [ ] **Step 2: Run to verify failure.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: FAIL (have_calib branch still returns REJECT/NO_PLATEAU).

- [ ] **Step 3: Implement the have_calib branch.** In `src/cursor_calib.cpp`, replace the placeholder tail:

```c
    /* ---- have_calib branch: filled in Task 4. ---- */
    (void)prior_bottom; (void)cal_blend;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    return r;
```

with:

```c
    /* ---- Have prior calibration. ---- */
    if (ritual_complete) {
        bool top_moved = fabsf(top_now  - prior_top)    >= CAL_MIN_DELTA;
        bool bot_moved = fabsf(p.median - prior_bottom) >= CAL_MIN_DELTA;
        if (!top_moved && !bot_moved) {
            r.decision = CAL_REJECT;
            r.reason   = CAL_REASON_BELOW_MIN_DELTA;  /* stable within this wear */
            r.apply    = false;
        } else {
            r.decision   = CAL_ADOPT;
            r.reason     = CAL_REASON_OK;
            r.new_top    = cal_blend(prior_top,    top_now);
            r.new_bottom = cal_blend(prior_bottom, p.median);
            r.apply      = true;
        }
        return r;
    }

    /* Ritual incomplete. Mid-air (top plausible, no rest at all) -> shadow only:
     * log the would-be coupled translation, APPLY NOTHING (spec §6 -- the weakest
     * signal must never move the map; we gather data to justify it later). */
    if (top_plausible && !p.found && !p.any_lowvar) {
        r.decision     = CAL_SHADOW_TRANSLATE;
        r.reason       = CAL_REASON_MID_AIR_SHADOW;
        r.shadow_bottom = top_now + (prior_bottom - prior_top); /* preserve prior span */
        r.apply        = false;
        return r;
    }

    /* Otherwise a hard reject; pick the most specific reason for telemetry. */
    r.decision = CAL_REJECT;
    r.apply    = false;
    if (!top_plausible)        r.reason = CAL_REASON_IMPLAUSIBLE_TOP;
    else if (p.any_lowvar)     r.reason = CAL_REASON_INSUFFICIENT_SWEEP; /* rest existed, too shallow */
    else                       r.reason = CAL_REASON_NO_PLATEAU;
    return r;
```

Note the mid-air guard uses `!p.any_lowvar` so a *shallow rest* (A5) routes to `INSUFFICIENT_SWEEP`, while a *true mid-air* (A3, no still run anywhere) routes to `SHADOW_TRANSLATE`. This matches the spec's separation of reasons.

- [ ] **Step 4: Run to verify pass.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
Expected: `ALL PASS`

- [ ] **Step 5: Commit**

```bash
git add src/cursor_calib.cpp tests/test_cursor_calib.cpp
git commit -m "feat(calib): have-calib adoption matrix (adopt/blend, shadow, rejects) + tests"
```

---

## Task 5: `cursor_track` runtime-variable anchors

**Files:**
- Modify: `src/cursor_track.h`
- Modify: `src/cursor_track.cpp:29` (anchor decls), `:67-72` (`vert_bottom`), getters block `:184-187`
- Test: `tests/test_cursor_track.cpp` (append before the final `printf`)

- [ ] **Step 1: Write failing tests.** In `tests/test_cursor_track.cpp`, insert before the `printf(failures ? ...)` line:

```c
    /* --- Runtime anchors: set_anchors changes top + bottom; span derives. --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_set_anchors(20.0f, 80.0f);
    CHECK(fabsf(cursor_track_vert_top() - 20.0f) < 1e-3f);
    CHECK(fabsf(cursor_track_vert_bottom() - 80.0f) < 1e-3f);
    /* New top=20: holding vert=20 after slam yields dy 0; vert=30 servos to GAIN_Y*10. */
    cursor_track_start(20.0f, 50.0f);
    drain_slam(20.0f, 50.0f);
    cursor_track_update(20.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    for (int i = 0; i < 20; i++) cursor_track_update(30.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - CURSOR_GAIN_Y * 10.0f) < 1e-3f);

    /* --- Bottom still clamps to CURSOR_VERT_BOTTOM_MAX when set beyond it. --- */
    cursor_track_set_anchors(20.0f, 200.0f);
    CHECK(fabsf(cursor_track_vert_bottom() - CURSOR_VERT_BOTTOM_MAX) < 1e-3f);

    /* Restore defaults for hygiene (nothing runs after, but be explicit). */
    cursor_track_set_anchors(CURSOR_VERT_TOP_DEG, CURSOR_VERT_TOP_DEG + CURSOR_VERT_SPAN_DEG);
```

- [ ] **Step 2: Run to verify failure.**

Run: `g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct`
Expected: FAIL to link — `undefined reference to cursor_track_set_anchors` / `cursor_track_vert_bottom`.

- [ ] **Step 3: Declare the new API** in `src/cursor_track.h`, after the `cursor_track_vert_top` line (`:55`):

```c
float cursor_track_vert_bottom(void);     /* current bottom anchor (deg), clamped  */

/* Set both absolute-Y anchors at runtime (natural calibration, RAM-only).  MUST
 * be called BEFORE cursor_track_start() at entry: the entry slam is sized from
 * the anchors, so a stale anchor would slam the recalibrating entry against the
 * previous mount.  Does NOT itself re-slam.  See
 * docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md. */
void cursor_track_set_anchors(float vert_top, float vert_bottom);
```

- [ ] **Step 4: Make the anchors variable** in `src/cursor_track.cpp`. Change line 29 from:

```c
static const float s_vert_top = CURSOR_VERT_TOP_DEG;
```

to:

```c
/* Absolute-Y anchors -- now RUNTIME-VARIABLE (natural calibration sets them via
 * cursor_track_set_anchors).  Seeded from the compile-time defaults, which also
 * serve as the cold-start values until the first complete entry ritual. */
static float s_vert_top    = CURSOR_VERT_TOP_DEG;
static float s_vert_bottom = CURSOR_VERT_TOP_DEG + CURSOR_VERT_SPAN_DEG;
```

- [ ] **Step 5: Use the stored bottom** in `vert_bottom()` (lines 67-72). Change:

```c
static float vert_bottom(void)
{
    float vb = s_vert_top + CURSOR_VERT_SPAN_DEG;
    if (vb > CURSOR_VERT_BOTTOM_MAX) vb = CURSOR_VERT_BOTTOM_MAX;
    return vb;
}
```

to:

```c
static float vert_bottom(void)
{
    float vb = s_vert_bottom;                       /* runtime anchor (was top + SPAN) */
    if (vb > CURSOR_VERT_BOTTOM_MAX) vb = CURSOR_VERT_BOTTOM_MAX;
    return vb;
}
```

- [ ] **Step 6: Add the setter + getter** in `src/cursor_track.cpp`, next to the other getters (after `cursor_track_vert_top` at line 187):

```c
float cursor_track_vert_bottom(void)     { return vert_bottom(); }

void cursor_track_set_anchors(float vert_top, float vert_bottom_in)
{
    s_vert_top    = vert_top;
    s_vert_bottom = vert_bottom_in;
}
```

- [ ] **Step 7: Run both host tests to verify pass** (the existing assertions must stay green — defaults are unchanged):

Run:
```
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc
```
Expected: `ALL PASS` (both).

- [ ] **Step 8: Commit**

```bash
git add src/cursor_track.h src/cursor_track.cpp tests/test_cursor_track.cpp
git commit -m "feat(cursor): runtime-variable anchors (set_anchors + vert_bottom getter)"
```

---

## Task 6: `gesture_mode` plumbing — `vert_hist` buffer + entry hook + `[CAL]` telemetry

**Files:**
- Modify: `src/gesture_mode.cpp` — add include; `vert_hist` ring buffer (near `gyro_hist`, ~line 290); push site (near the `gyro_hist` push, ~line 1115); entry hook + telemetry inside `_transition_to` MODE_AIR_MOUSE branch (`:509-512`).

This task is **firmware, not host-testable** — verification is a clean `./build.sh` plus reading the `[CAL]` serial line against the expected format (per `CLAUDE.md`: hardware-in-the-loop, no on-target unit harness).

- [ ] **Step 1: Add the include** at the top of `src/gesture_mode.cpp`, next to the existing `#include "cursor_track.h"`:

```c
#include "cursor_calib.h"
```

- [ ] **Step 2: Declare the `vert_hist` ring buffer + calibration state**, immediately after the `gyro_hist` declaration block (after `static int gyro_hist_idx = 0;`, ~line 291):

```c
/* Absolute-vert history for natural cursor calibration (entry-time auto-anchor).
 * SEPARATE from gyro_hist (which is gyro rates, 1.5 s) -- this is absolute vert
 * (deg) and DEEPER (3 s) so a slow raise's resting plateau cannot fall out of the
 * window unseen (an undersized buffer fails invisibly as no-plateau).  Written on
 * the acq thread alongside gyro_hist; read on the acq thread at AIR_MOUSE entry
 * (same thread -- no cross-thread race).  See
 * docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md. */
static float vert_hist[VERT_HIST_SAMPLES];
static int   vert_hist_idx = 0;
static bool  vert_hist_primed = false;     /* false until the buffer has filled once */

/* RAM-only calibration state (no NVS): false until the first complete entry ritual
 * seeds the anchors.  cursor_track holds the anchor values; this just tracks "have
 * we calibrated since boot" for the cold-start branch. */
static bool cursor_have_calib = false;
```

- [ ] **Step 3: Push into `vert_hist`** at the same site the gyro buffer is pushed. After the `gyro_hist_idx = (gyro_hist_idx + 1) % GYRO_HIST_SAMPLES;` line (~line 1118), add:

```c
    /* Mirror push into the absolute-vert calibration history. */
    vert_hist[vert_hist_idx] = current_vert_deg();
    vert_hist_idx = (vert_hist_idx + 1) % VERT_HIST_SAMPLES;
    if (vert_hist_idx == 0) vert_hist_primed = true;
```

- [ ] **Step 4: Add a chronological-linearise helper + a calibration runner**, placed just above `_transition_to` (before line 500). The runner snapshots the ring into chronological order, calls the pure decider, applies the verdict (set_anchors) BEFORE the caller starts the cursor, flips `cursor_have_calib`, and logs `[CAL]`:

```c
/* Copy the vert_hist ring into chronological order (oldest -> newest) for the
 * pure decider.  Returns the count (== VERT_HIST_SAMPLES once primed, else the
 * number written so far). */
static int vert_hist_chronological(float *out)
{
    if (vert_hist_primed) {
        int k = 0;
        for (int i = 0; i < VERT_HIST_SAMPLES; i++) {
            out[k++] = vert_hist[(vert_hist_idx + i) % VERT_HIST_SAMPLES];
        }
        return VERT_HIST_SAMPLES;
    }
    for (int i = 0; i < vert_hist_idx; i++) out[i] = vert_hist[i];
    return vert_hist_idx;
}

static const char *cal_decision_str(cursor_calib_decision_t d)
{
    switch (d) {
        case CAL_SEED:             return "SEED";
        case CAL_ADOPT:            return "ADOPT";
        case CAL_SHADOW_TRANSLATE: return "SHADOW-TRANSLATE";
        default:                   return "REJECT";
    }
}
static const char *cal_reason_str(cursor_calib_reason_t r)
{
    switch (r) {
        case CAL_REASON_OK:                return "ok";
        case CAL_REASON_COLD_START_DEFAULT:return "cold-start-default";
        case CAL_REASON_NO_PLATEAU:        return "no-plateau";
        case CAL_REASON_INSUFFICIENT_SWEEP:return "insufficient-sweep";
        case CAL_REASON_IMPLAUSIBLE_TOP:   return "implausible-top";
        case CAL_REASON_BELOW_MIN_DELTA:   return "below-min-delta";
        case CAL_REASON_MID_AIR_SHADOW:    return "mid-air-shadow";
        default:                           return "?";
    }
}

/* Run natural calibration for an AIR_MOUSE entry.  MUST be called BEFORE
 * cursor_track_start (the entry slam is sized from the anchors).  top_now is the
 * snap-moment inclination. */
static void cursor_calib_run_on_entry(float top_now)
{
    static float chrono[VERT_HIST_SAMPLES];
    int n = vert_hist_chronological(chrono);

    float prior_top    = cursor_track_vert_top();
    float prior_bottom = cursor_track_vert_bottom();

    cursor_calib_result_t res =
        cursor_calib_decide(cursor_have_calib, prior_top, prior_bottom,
                            top_now, chrono, n);

    if (res.apply) {
        cursor_track_set_anchors(res.new_top, res.new_bottom);
        cursor_have_calib = true;
    }

    LOG_INF("[CAL] top %d->%d bottom %d->%d span=%d | "
            "plateau=%s(var=%d,n=%d) sweep=%d | decision=%s reason=%s shadow_bottom=%d",
            (int)prior_top, (int)(res.apply ? res.new_top : prior_top),
            (int)prior_bottom, (int)(res.apply ? res.new_bottom : prior_bottom),
            (int)((res.apply ? res.new_bottom : prior_bottom) -
                  (res.apply ? res.new_top : prior_top)),
            res.plateau_found ? "Y" : "N",
            (int)res.plateau_var, res.plateau_n, (int)res.sweep_deg,
            cal_decision_str(res.decision), cal_reason_str(res.reason),
            (int)res.shadow_bottom);
}
```

- [ ] **Step 5: Wire the entry hook** in `_transition_to` (the MODE_AIR_MOUSE branch, `:509-512`). Change:

```c
    if (new_mode == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        cursor_track_start(current_vert_deg(), ori.roll_deg);
    } else {
```

to (strict order: **decide → set_anchors (inside run_on_entry) → start**):

```c
    if (new_mode == MODE_AIR_MOUSE) {
        orientation_state_t ori;
        orientation_get(&ori);
        float top_now = current_vert_deg();
        /* Natural calibration FIRST: it may set_anchors() based on the entry
         * ritual.  cursor_track_start (below) sizes the entry slam from those
         * anchors, so the order MUST be decide -> set_anchors -> start, or the
         * recalibrating entry would slam against the previous mount. */
        cursor_calib_run_on_entry(top_now);
        cursor_track_start(top_now, ori.roll_deg);
    } else {
```

- [ ] **Step 6: Build the firmware** (per `CLAUDE.md`):

Run: `./build.sh`
Expected: a clean build ending with `Flashable artifact: build/zephyr/zephyr.uf2`, no errors/warnings from `cursor_calib` or `gesture_mode`.

- [ ] **Step 7: Commit**

```bash
git add src/gesture_mode.cpp
git commit -m "feat(calib): vert_hist buffer + entry-time calibration hook + [CAL] telemetry"
```

---

## Task 7: Documentation + final verification

**Files:**
- Modify: `CLAUDE.md` (file map)

- [ ] **Step 1: Add `cursor_calib` to the `CLAUDE.md` file map.** Under the file map section, after the `src/cursor_track.{h,cpp}` entry, add:

```markdown
- `src/cursor_calib.{h,cpp}` — **PURE** entry-time cursor calibration: extracts
  the resting-pose bottom anchor from a `vert` history + scores the air-mouse
  entry ritual → adoption verdict (`cursor_calib_decide`). Host-unit-tested
  (`tests/test_cursor_calib.cpp`). `gesture_mode` runs it at AIR_MOUSE entry
  (decide→set_anchors→start) and logs `[CAL]`. No Zephyr deps.
```

- [ ] **Step 2: Run the full host-test suite** (both pure modules) to confirm nothing regressed:

Run:
```
g++ -std=c++11 -Isrc tests/test_cursor_track.cpp src/cursor_track.cpp -lm -o /tmp/ct && /tmp/ct
g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc
```
Expected: `ALL PASS` (both).

- [ ] **Step 3: Final clean firmware build:**

Run: `./build.sh`
Expected: clean build, `build/zephyr/zephyr.uf2` produced.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add cursor_calib to CLAUDE.md file map"
```

---

## Hardware verification (post-flash, manual — not a plan step but the acceptance gate)

Flash (`cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`), open serial, and confirm against spec §10:
1. **Cold-start complete ritual:** from a desk/lap rest, raise fully, double-tap → `[CAL] ... decision=SEED reason=ok`; anchors match the observed top + resting vert; cursor reaches **both** screen edges.
2. **Lazy mid-air entry:** `decision=SHADOW-TRANSLATE` (or `REJECT reason=no-plateau`); `shadow_bottom` logged but **map unchanged**; entry still works.
3. **Re-wear:** shift the band, then a complete ritual → `decision=ADOPT`; mapping correct again with no manual step.
4. **Repeatability dataset:** `[CAL]` `top`/`bottom` values across several entries cluster (the spec §4.1 check) — also tells us if deferred signal-3 (top-dwell) is needed.
