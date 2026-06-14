# Spec: Cursor bottom-capture-at-placement + exit/engage model (+ SURFACE removal)

**Date:** 2026-06-13
**Status:** Design for review.
**Scope:** Three coupled changes to the air-mouse cursor's exit/calibration area:
1. **Bottom anchor** captured from where the wrist actually rests, grabbed at
   *placement* into a persistent scalar (`last_rest_vert`) that never ages —
   replacing the dead at-snap `vert_hist` buffer.
2. **Exit/engage model:** disengage on LOW + (chip tap OR stillness-held);
   re-engage on RAISE (auto in cooldown, else double-tap).
3. **Remove the SURFACE *mode*** (keep the triple-tap gesture itself, unbound).

These three land together because they all live in the same cursor-exit +
calibration code and share one "settled flat on the desk" signal.

**Grounding (HW + prior work):**
- `docs/superpowers/specs/2026-06-12-natural-cursor-calibration-design.md` — the
  calibration this revises. Its adoption matrix (SEED / ADOPT-blend /
  below-min-delta / implausible-top / mid-air-shadow) is KEPT; only the bottom
  *source* changes from a 3 s buffer to a persistent scalar.
- `[CALDBG]` HW finding 2026-06-13 (`span_ms=3083`, continuous sampling,
  `min=2 max=27 hi=0`): the desk rest is NOT in the 3 s buffer at the snap because
  the raise→pose-arm→double-tap ritual takes >3 s, so the rest ages out. The
  earlier "sensor sleeps during the rest" theory was overturned by this data. Fix =
  capture the bottom at placement into a non-aging scalar.
- `finding_desk_exit_tap_misses_volar_up_2026_06_12` — the desk-tap exit works when
  contact is genuinely near-flat (HW-confirmed 2026-06-13, rest ~80–84°). A *single,
  unconfirmed, pre-re-wear* observation once put an elbow-on-armrest contact at ~54°
  (flagged "to be confirmed", never reproduced, NOT seen in today's data) — it is a
  case to CHECK in M2, NOT a design premise. The stillness-held exit is the
  angle-robust fix regardless of the exact rest angle.
- `project_air_mouse_exit_model_2026_06_12` — the agreed exit model this realizes.
- Roadmap decision 2026-06-13: SURFACE touchpad (Use Case 1 / Item 3) is DROPPED.

---

## 1. Why

The natural-calibration build captures the TOP correctly (at the double-tap snap)
but cannot capture the BOTTOM: by snap time the desk rest is >3 s old and has
scrolled out of the 3 s `vert_hist` window (HW-proven, see `[CALDBG]` above). The
bottom must be grabbed **at placement** — when the wrist settles on the desk — and
held in a value that does not age. Separately, the AIR_MOUSE exit is unreliable
(the desk-tap gate only fires when contact is near-flat; a non-near-flat rest, if
the user ever has one, would be excluded — see M2). And SURFACE mode is dropped
from the roadmap, so its code is
removed here since it sits in the same exit branch.

## 2. The rest tracker (new) — `gesture_mode.cpp`

A small always-on detector in the per-sample acq pipeline (runs in ALL power
states, mode-independent):

```
on each sample (gesture_mode_update_*):
    in_low_zone = (gx_filt < CURSOR_LOW_ZONE_GX)         // low/near-flat, M2-tuned
    settled     = in_low_zone
                  AND samples_since_activity >= CAL_REST_STILL_DWELL
                  AND vert variance over the dwell < CAL_REST_VAR
    if settled:
        last_rest_vert = <settled vert>   // smoothed/median over the dwell
        bottom_valid   = true
```

- `last_rest_vert` is a **persistent scalar** (RAM); it does NOT age. Captured at
  placement (the motion of setting the wrist down keeps sampling awake long enough
  to register the settle), it stays valid indefinitely — bridging the >3 s gap to
  the snap that killed the buffer.
- "Auto-track every rest": each new settled rest overwrites `last_rest_vert`, so the
  bottom always reflects the most recent place you rested (re-wear, new chair).
- The settle detector reuses the *idea* of last build's `find_plateau`
  (low-variance run) but as a live single-scalar tracker, not a 3 s ring.

## 3. Entry: top at snap, bottom from the scalar — `cursor_calib`

At AIR_MOUSE entry (`gesture_mode._transition_to`, the MODE_AIR_MOUSE branch):
- `top_now = current_vert_deg()` (unchanged — captured at the snap, the user's
  deliberate vertical-pose extreme).
- `bottom_candidate = last_rest_vert`, with `bottom_valid`.
- Call the matrix; apply via `cursor_track_set_anchors` BEFORE `cursor_track_start`
  (the decide→set→start ordering invariant is preserved).

**`cursor_calib_decide` signature change:**
```
// OLD: (have_calib, prior_top, prior_bottom, top_now, const float *vert_chrono, int n)
// NEW:
cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          float bottom_candidate, bool bottom_valid);
```
- The adoption matrix body is UNCHANGED in behavior (SEED / ADOPT-blend /
  below-min-delta / implausible-top / mid-air-shadow), operating on the scalar:
  - `ritual_complete = top_plausible (top_now <= CAL_TOP_MAX)
                        AND bottom_valid
                        AND (bottom_candidate - top_now) >= CAL_SWEEP_MIN_DEG`.
  - cold-start seed / have-calib blend / min-delta no-op / implausible-top reject
    all as before.
  - "mid-air" (the SHADOW-TRANSLATE case) now = `top_plausible AND !bottom_valid`
    (no rest ever captured this session) → log `SHADOW-TRANSLATE`, apply nothing.
- **REMOVED from `cursor_calib`:** `find_plateau`, `median_of`, the `plateau_t`
  struct, the `cursor_calib_find_plateau_TEST` shim, and the `vert_chrono`/`n`
  plumbing. The diagnostic fields tied to the plateau (`plateau_found`,
  `plateau_var`, `plateau_n`) are dropped from the result struct; `bottom_candidate`
  and `sweep_deg` stay (now sourced from the scalar).

## 4. Exit / engage model — `gesture_mode.cpp` cursor-exit branch (AIR_MOUSE)

Disengage triggers, all gated to the **low zone** (`gx_filt < CURSOR_LOW_ZONE_GX`):
- **(a) chip tap** in-zone → exit "desk contact (chip tap)" (existing path;
  HW-confirmed working when near-flat).
- **(b) stillness-held** → settled in the low zone for `>= STILL_EXIT_DWELL`
  consecutive samples → exit "rest-settle (stillness)". NEW. The dwell is set so a
  floating-to-point hand cannot hold it (M1); a supported desk rest pegs `still`.
- **(c) past-plane** → signed `gx_filt < CURSOR_PAST_PLANE_GX` for
  `CURSOR_PAST_PLANE_DWELL` (no-desk path) → exit "past horizontal plane"
  (existing, unchanged).

Re-engage on **raise** (existing, unchanged): within the exit cooldown → auto
re-engage; cooldown expired → require the double-tap.

**Stillness-exit is gated to the low zone ONLY** (per the user's guardrail): going
still at a raised / mid angle = "holding to point" → STAY ENGAGED. Pointing at the
*bottom of the screen* (low + still briefly) is distinguished from "parked on the
desk" by the `STILL_EXIT_DWELL` dwell (M1).

**Potential tension — CONDITIONAL on M2, do NOT assume it.** The low zone must
cover the user's *actual measured* rest angle(s) (M2). Current HW data (2026-06-13)
shows the rest at **~80–84° (near-flat)**, in which case the low zone is NARROW
(`gx_filt` small, vert > ~75°), it barely overlaps the pointing range, and there is
little tension — the dwell + tap are plenty. The wide-zone problem only arises *if*
M2 finds the user routinely resting much lower (the historical, **unconfirmed,
pre-re-wear** ~54° armrest observation — one trace, flagged "to be confirmed", never
reproduced, and NOT seen in today's data). **If** M2 confirms a low rest, then the
wider zone makes part of the lower pointing range disengage-eligible-when-still, and
`STILL_EXIT_DWELL` becomes the sole separator — with the fallback: stillness-exit
only in the deepest near-flat sub-band, chip-tap for the shallower part. Decide from
M2 data; default expectation (per today's data) is the narrow, low-tension case.

## 5. SURFACE-mode removal

- Remove `MODE_SURFACE` and its routing: the triple-tap → SURFACE entry, the
  SURFACE branch in the cursor-exit logic (the orientation-drop exit), and the
  `surface_motion_*` / `_exit_dwell_for(SURFACE)` state.
- **KEEP** `gesture_mode_on_chip_triple_tap` and the `y` serial sim: leave the
  3-tap detector intact but **log-only** (e.g. `"Chip triple-tap (unbound)"`) — a
  free trigger to repurpose later. It no longer routes to any mode.
- AIR_MOUSE becomes the only cursor mode; the cursor-exit branch simplifies to the
  AIR_MOUSE logic in §4 (no `in_cursor_mode` SURFACE fork).

## 6. Verify-first measurements (do these FIRST; thresholds depend on them)

- **M1 — rest-vs-float stillness split.** With `[REST]` telemetry on: (i) rest the
  wrist flat on the desk, read `still` + the settle variance; (ii) float low and try
  to hold the cursor at the bottom, read the same. Set `STILL_EXIT_DWELL`,
  `CAL_REST_STILL_DWELL`, `CAL_REST_VAR` from the gap (floating must NOT reach the
  dwell). Seeds below are first guesses pending this.
- **M2 — measure the user's ACTUAL rest angle(s); set the zone to cover them.** Do
  NOT assume a number. Rest the wrist the way you actually work and read `vert`/`gx`
  (the `[REST]` log). Today's data says ~80–84° (near-flat) — if that holds, set
  `CURSOR_LOW_ZONE_GX` to a narrow near-flat gate. **Also check whether you ever rest
  notably lower** (e.g. elbow on an armrest below desk height — a historical,
  unconfirmed ~54° observation that did NOT appear in today's traces). If a lower
  rest is real and reproducible, widen `CURSOR_LOW_ZONE_GX` to cover it (gated on
  `gx_filt`/`vert` directly, not the strict `DOWN_FLAT` classifier) and apply the §4
  fallback. The point is to fit *measured* behavior, not a remembered number.

## 7. Constants (`gesture_thresholds.h`)

RETIRE: `VERT_HIST_SAMPLES`, `CAL_PLATEAU_VAR`, `CAL_PLATEAU_DWELL`.
REPLACE: `CURSOR_DESK_ZONE_GX` (1.7, the old near-flat gate) → renamed/retuned to
the M2-tuned `CURSOR_LOW_ZONE_GX` (likely similar if rests are near-flat per today's
data; only wider if M2 confirms a lower rest). Audit existing uses of
`CURSOR_DESK_ZONE_GX` in the exit branch and repoint them to the new gate.

ADD (all `[USER]`, seeds tuned from M1/M2):
| Constant | Seed | Meaning |
|---|---|---|
| `CURSOR_LOW_ZONE_GX` | `2.5` (≈ vert > 75°) | `gx_filt` below this = "low zone" for rest-capture + stillness/tap exit. Seed gives ~5–9° margin below the measured ~80–84° rest (cf. old `CURSOR_DESK_ZONE_GX`=1.7 ≈ vert>80°). M2 finalizes: set ~5° under the measured *lowest* rest; widen only if a notably lower rest is confirmed |
| `CAL_REST_STILL_DWELL` | ~40 (~400 ms) | min still-samples to call a low-zone pose a settled rest (capture `last_rest_vert`) |
| `CAL_REST_VAR` | `4.0` | max `vert` variance (deg²) over the dwell for "settled" |
| `STILL_EXIT_DWELL` | tune (M1) | consecutive settled low-zone samples to DISENGAGE (long enough a floating hand can't hold it) |

KEEP: `CAL_SWEEP_MIN_DEG` (25), `CAL_TOP_MAX` (30), `CAL_MIN_DELTA` (5),
`CAL_BLEND_ALPHA` (0.4); `CURSOR_PAST_PLANE_GX`/`_DWELL`; the cursor_track anchor
defaults (`CURSOR_VERT_TOP_DEG`/`SPAN`/`BOTTOM_MAX`).

## 8. Telemetry

- `[CAL]` (entry): unchanged shape; `bottom` now sourced from `last_rest_vert`.
  Drop the `plateau=...` field; add `bottom_valid=Y/N`.
- `[REST]` (new): on each settle, `vert / still / var` — this is the dataset that
  tunes M1 and confirms auto-track.
- Exit log: add the reason (`chip-tap` / `stillness` / `past-plane`).

## 9. Testing

- **Host (`cursor_calib`):** rework `tests/test_cursor_calib.cpp` to the scalar
  signature. Keep the matrix coverage: cold-start SEED (bottom_valid + sweep), both-
  trusted ADOPT-blend, below-min-delta no-op, implausible-top reject, `!bottom_valid`
  → SHADOW-TRANSLATE. Remove the P1–P7 plateau tests (find_plateau retired).
  `cursor_track` tests unchanged.
- **Firmware:** rest tracker + exit + SURFACE removal are in `gesture_mode.cpp` →
  verify with clean `./build.sh` + reading `[REST]`/`[CAL]`/exit logs (no on-target
  unit harness, per CLAUDE.md).
- **HW acceptance:**
  - rest → raise → hold any duration → double-tap → `[CAL]` bottom ≈ your rest;
    cursor reaches both edges.
  - re-wear / rest at a new height → next entry `decision=ADOPT`.
  - settled flat on the desk in AIR_MOUSE → `stillness` disengage; pointing low but
    moving → STAYS engaged.
  - chip-tap on placement near-flat → `chip-tap` disengage; hand drops past plane →
    `past-plane` disengage.
  - triple-tap → log-only, enters no mode.

## 10. Out of scope

Ritual signals 3 (top-dwell) / 4 (gyro corroboration) — still deferred. NVS
persistence — deferred (RAM-only; `last_rest_vert` re-captures on the first rest
after boot). Mid-air SHADOW-TRANSLATE promotion — unchanged (shadow-only).
Horizontal-X axis, cursor smoothing (`o`/`p` / gyro-fused), `[CURSOR]` log strip —
all separate work items.
