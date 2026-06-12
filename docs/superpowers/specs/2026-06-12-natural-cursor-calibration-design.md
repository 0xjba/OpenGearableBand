# Spec: Natural (entry-time) cursor calibration

**Date:** 2026-06-12
**Status:** Design for review.
**Scope:** Auto-anchor **both ends** of the absolute-Y cursor map (`vert_top`,
`vert_bottom`) from the air-mouse **entry ritual itself** — raise from a resting
pose, reach the top, snap (double-tap) — so each day's mount self-calibrates
with no manual step. **RAM-only** (NVS deferred). This repays the documented
cost of the fixed anchor introduced in Amendment A.1.

**Grounding (specs + memory):**
- `docs/superpowers/specs/2026-06-11-absolute-y-cursor-design.md` — §4 (the
  original entry-capture "option A"), §7 (the designated non-circular A→B
  self-calibration), and **Amendment A.1**, which fixed the top anchor at 12°
  "for maximum cross-session determinism" *and wrote down the cost*:
  > Mount caveat: fixed angles assume the forearm-axis mounting is consistent.
  > After a re-tape / housing change the absolute mapping shifts — re-check
  > `CURSOR_VERT_TOP_DEG`. The former §4.1 cross-session check becomes the
  > recalibration trigger.
  This spec *is* that recalibration trigger, automated.
- `finding_desk_exit_tap_misses_volar_up_2026_06_12` — desk contact happens at
  `vert ≈ 54°` (elbow on an armrest below desk height), not the near-flat 80°+
  the desk-exit gate assumed. That is a **bottom-anchor** shift, and it is why
  leaving the bottom hardcoded (option B below) re-creates today's problem at
  the other end.
- `project_productionization_orientation_calibration` /
  `project_productionization_gesture_calibration_2026_06_09` — per-mount /
  per-user calibration was always a flagged productionization need; this is its
  first concrete, non-circular instalment.
- `principle_real_world_grounding_before_engineering` — every threshold here is
  a seed, tuned from the user's own entry recordings, not hardcoded blind.

---

## 1. Why (the problem this kills)

The absolute-Y map anchors a wrist inclination (`vert = acos(|gx|/|g|)`) to a
screen height via two numbers: `vert_top` (→ screen top) and `vert_bottom`
(→ screen bottom, currently `vert_top + SPAN`). Amendment A.1 made these
**compile-time constants** (12° / 70° span) to get cross-session determinism.

That determinism only holds **if the forearm-axis mounting is stable**. It is
not: re-wearing the band the next morning rotates/shifts it on the wrist, so the
same physical posture now produces a different `gx` → a different `vert`, and the
fixed 12°/82° no longer line up with where the wrist actually is. Observed
2026-06-12: "when I move my wrist up I don't have to reach 12° to hit the top of
the screen." Both ends drift, not just the top (the desk-exit finding shows the
bottom landing at ~54° for this user's posture).

**Fix:** recapture both anchors automatically from the gesture the user already
performs to enter air-mouse. No calibration ritual — the entry *is* the
calibration.

## 2. Architecture — three focused units

```
gesture_mode (acq thread)                cursor_calib (PURE)            cursor_track (PURE)
─ vert_hist[150] ring buffer  ── arrays ─▶ extract bottom plateau   ─ anchors ─▶ set_anchors(top,bottom)
─ at AIR_MOUSE entry:                      score ritual completeness            map uses top + bottom
    call cursor_calib_decide()             adoption matrix → verdict            (span = bottom − top)
    apply verdict, set_anchors             (+ new anchors)
    emit [CAL] log
```

Three units, each independently understandable and (for two of them) host-testable:

- **`cursor_calib.{h,cpp}` — NEW, PURE (no Zephyr deps), host-unit-tested.**
  The decision engine. Pure functions over plain arrays + scalars; no I/O, no
  threads, no Zephyr. This is where every non-trivial rule lives (plateau
  extraction, ritual scoring, the adoption matrix, the blend), so all of it is
  testable like `cursor_track`. Interface in §6.
- **`gesture_mode.cpp` — plumbing only.** Owns a new `vert_hist[150]` ring
  buffer of absolute `vert` (deg) at 100 Hz (parallel to the existing
  `gyro_hist`, ~600 B). At AIR_MOUSE entry (`gesture_mode.cpp:512`, where
  `cursor_track_start` is already called) it snapshots the buffers, calls
  `cursor_calib_decide()`, applies the returned anchors via
  `cursor_track_set_anchors()`, and emits the `[CAL]` telemetry line.
- **`cursor_track.{h,cpp}` — consumer.** `s_vert_top` changes from `const` to a
  variable; a new `s_vert_bottom` variable replaces the derived
  `vert_top + SPAN` (span becomes `bottom − top`, still clamped to
  `CURSOR_VERT_BOTTOM_MAX`). New setter `cursor_track_set_anchors(top, bottom)`
  and getter `cursor_track_vert_bottom()`. `CURSOR_VERT_TOP_DEG` /
  `CURSOR_VERT_SPAN_DEG` are **retained as cold-start defaults**, not removed.

## 3. Data: the `vert_hist` ring buffer

- `static float vert_hist[VERT_HIST_SAMPLES]`, `VERT_HIST_SAMPLES = 150` (1.5 s
  at 100 Hz), written every sample in the acq
  pipeline alongside `gyro_hist`, storing `current_vert_deg()` (the FAST
  cursor-filter inclination — the same signal that drives the cursor Y).
- ~600 B static. The 1.5 s depth matches `gyro_hist`; it must be long enough to
  contain the resting plateau **plus** the raise. If HW traces show a slow
  raise overruns 1.5 s, lengthen it (a `[USER]` structural constant), do not
  silently accept a truncated plateau.

## 4. Bottom extraction — plateau, not last-sample

The resting `vert` is the **median of a detected low-motion, high-vert plateau**
in `vert_hist`, *not* the last sample before motion (already contaminated by
raise onset).

```
find_plateau(vert_hist):
    # the buffer's most-recent samples are the raise + top; the resting
    # plateau is in the OLDER portion, preceding the climb to top_now.
    among all contiguous runs R where
        variance(R) < CAL_PLATEAU_VAR  AND  len(R) >= CAL_PLATEAU_DWELL :
        keep those whose median(vert) is HIGH (lowered posture), i.e.
        median(R) >= top_now + CAL_SWEEP_MIN_DEG
    if any qualify: bottom_candidate = median(highest-vert qualifying run)
    else:           no plateau (bottom untrusted this entry)
```

Median (not mean) so a single spike inside the run does not pull the anchor.
Variance gate so "resting" means genuinely still, not mid-drift. The
high-`vert` requirement disambiguates the *resting* plateau from a brief
low-`vert` dwell at the top (which is also low-variance) — the plateau is the
lowered end, the top is the raised end.

## 5. Ritual-completeness — sweep magnitude from `vert_hist` directly

The adoption gate is **ritual completeness**, not delta magnitude alone. A
complete entry ritual is: came from a resting plateau → swept a large vertical
range → brief dwell at the top → snap. Signals:

1. **Plateau exists** (§4) — the move started from rest.
2. **Sweep magnitude** `= bottom_candidate − top_now >= CAL_SWEEP_MIN_DEG`
   (`vert` is *small* when raised = top, *large* when flat = bottom, so the
   range is bottom minus top), computed **directly from the two
   `vert_hist`-derived endpoints**. We do *not*
   integrate a gyro axis for this: we already have both endpoints, and reading
   the range off `vert` avoids identifying which `gyro_hist` axis maps to
   forearm elevation (fragile geometry).
3. **Brief dwell at the top** before the snap — `vert` stable near `top_now` for
   a short window (the user "reached top and held" rather than snapping
   mid-swing).
4. **(Optional corroboration)** `gyro_signature()` peak-rate on the move — a
   real raise has a deliberate angular-velocity profile, not a slow creep. Used
   only to corroborate, never as the sole gate.

`top_now` is the entry `vert` at the snap, subject to a plausibility clamp:
`top_now <= CAL_TOP_MAX` (a lazy half-raise that snaps at, say, 45° is rejected
as the top — it is not trusted even if everything else passes).

## 6. Adoption decision matrix

`cursor_calib_decide(prior_top, prior_bottom, have_calib, top_now, vert_hist[])
 → { decision, reason, new_top, new_bottom }`

| Situation | Decision |
|---|---|
| **Cold-start** (`!have_calib`, every boot) + ritual complete | **Seed** both immediately (blend α = 1): `new_top = top_now`, `new_bottom = bottom_candidate`. |
| **Cold-start** + ritual incomplete | **No adopt.** Run absolute mode on the compile-time defaults (`CURSOR_VERT_TOP_DEG` / `+SPAN`); log `cold-start-default`. Self-heals on first complete ritual. |
| Both trusted + ritual complete + `|Δtop| > CAL_MIN_DELTA` (or `|Δbottom| > CAL_MIN_DELTA`) | **Adopt (blend).** `new_top = blend(prior_top, top_now)`, `new_bottom = blend(prior_bottom, bottom_candidate)`. Span = `new_bottom − new_top`. |
| **Top trusted, no bottom plateau** (entered from mid-air) | **Coupled adoption `[VALIDATE]`:** adopt top (blend), and *translate* bottom to preserve the last span: `new_bottom = new_top + (prior_bottom − prior_top)`. **Never** pair a fresh top with a stale-epoch bottom as independent absolutes. |
| Ritual incomplete / neither trusted / `|Δ| < CAL_MIN_DELTA` | **No-op.** Keep current anchors; log the reason. |

**Cold-start path is a first-class path, not an edge case** (RAM-only means
every session starts here). The deliberate choice is **compile-time defaults**,
not "refuse absolute / fall back to relative": it keeps the deterministic
absolute map, avoids a relative-mode regression, and is exactly today's
behaviour — least surprise — while self-healing on the first complete ritual.

**Anti-jitter — blend, not hard-swap.** A complete-ritual capture moves the
anchor by `blend(old, new) = old + CAL_BLEND_ALPHA * (new − old)`
(`CAL_BLEND_ALPHA ≈ 0.4`), seeded at α = 1 on cold-start. One odd-but-plausible
entry that survives the ritual gate moves the map only partially; the next good
entry corrects it. *(Alternative considered: require two consecutive
agreeing entries — strictly rejects a single odd entry but costs one
miscalibrated engagement after a genuine re-wear. Chose blend; revisit if HW
shows blend converges too slowly after a re-wear.)*

**Epoch rule (explicit):** `top` and `bottom` must always come from the *same*
epoch — both fresh from one ritual, or bottom translated from the new top. The
matrix never stores a top from entry N beside a bottom from entry M as two
independent absolutes.

## 7. Telemetry — `[CAL]` on every entry

```
[CAL] top old=<a> new=<b>  bottom old=<c> new=<d>  span=<d-b> |
      plateau=<Y/N>(var=<v>,n=<k>)  sweep=<deg>  topdwell=<n> |
      decision=<ADOPT|SEED|TRANSLATE|REJECT>  reason=<...>
```

`reason ∈ { ok, cold-start-default, no-plateau, insufficient-sweep,
implausible-top, below-min-delta }`.

Calibration that silently shifts the map is the subsystem we least want opaque —
this project debugs on trace evidence. As a side effect, the per-entry `[CAL]`
stream **is** the cross-session `vert_top` repeatability dataset that §4.1 of
the original spec demanded before trusting entry-capture — the validation
prerequisite fulfils itself as a logging by-product.

## 8. Constants (`gesture_thresholds.h`, all `[USER]`, seeds tuned from traces)

| Constant | Seed | Meaning |
|---|---|---|
| `VERT_HIST_SAMPLES` | `150` | 1.5 s of `vert` history at 100 Hz (`[STRUCTURAL]`; lengthen if a slow raise overruns) |
| `CAL_PLATEAU_VAR` | `4.0` | max `vert` variance (deg², ~2° std) for "resting still" |
| `CAL_PLATEAU_DWELL` | `30` (~300 ms) | min plateau length (samples) |
| `CAL_SWEEP_MIN_DEG` | `25.0` | min `top − bottom` to count as a real raise |
| `CAL_TOP_MAX` | `30.0` | plausibility clamp: reject a top above this (lazy raise) |
| `CAL_MIN_DELTA` | `5.0` | ignore anchor shifts smaller than this (stable within a wear) |
| `CAL_BLEND_ALPHA` | `0.4` | adoption blend factor (1.0 effective on cold-start seed) |

`CURSOR_VERT_TOP_DEG` (12) and `CURSOR_VERT_SPAN_DEG` (70) are **retained** as
the cold-start defaults. `CURSOR_VERT_BOTTOM_MAX` (85) still clamps the derived
bottom in `cursor_track`.

## 9. `cursor_track` API changes (host-testable)

- `s_vert_top`: `const float` → `float` variable.
- Add `static float s_vert_bottom` (init `CURSOR_VERT_TOP_DEG +
  CURSOR_VERT_SPAN_DEG`). `vert_bottom()` returns `min(s_vert_bottom,
  CURSOR_VERT_BOTTOM_MAX)`; `max_counts()` / `target_counts()` unchanged
  otherwise (span = `vert_bottom() − s_vert_top`, now both variable).
- `void cursor_track_set_anchors(float vert_top, float vert_bottom)` — sets both;
  used by `gesture_mode` after a calibration verdict. (Does **not** itself
  re-slam; the entry slam in `cursor_track_start` still runs as today.)
- `float cursor_track_vert_bottom(void)` — getter for telemetry.
- `cursor_track_vert_top()` now returns the variable.

## 10. Testing

**Host unit tests (both modules are pure):**
- `cursor_calib`: plateau extraction (median, variance gate, dwell floor, no-
  plateau case); sweep gate; plausibility clamp; the full adoption matrix —
  cold-start seed, both-trusted blend, coupled/translate, all reject reasons,
  below-min-delta no-op; blend arithmetic; epoch rule (no cross-epoch pairing).
- `cursor_track`: `set_anchors` changes the map; span derived correctly; bottom
  clamp at `CURSOR_VERT_BOTTOM_MAX`; `max_counts`/`target`/slam behave with
  non-default anchors.
- Build/run per `CLAUDE.md`:
  `g++ -std=c++11 -Isrc tests/test_cursor_calib.cpp src/cursor_calib.cpp -lm -o /tmp/cc && /tmp/cc`
  (and the existing `test_cursor_track`).

**Hardware acceptance:**
- Enter from a desk/lap rest with a full raise → `[CAL] decision=ADOPT/SEED`;
  anchors match the observed top and resting `vert`; cursor then reaches **both**
  screen edges across the wrist range.
- Lazy mid-air entry → `decision=REJECT reason=no-plateau` (or
  `insufficient-sweep`); map unchanged; entry still works.
- Simulated re-wear (shift the band) → first complete ritual adopts the new
  mount's anchors; mapping is correct again without a manual step.
- `[CAL]` lines across several sessions show `vert_top` clustering (the §4.1
  cross-session repeatability check).

## 11. Out of scope

- **Desk-EXIT** stays a *separate* problem (this spec is entry calibration). The
  captured `vert_bottom` may later feed the desk-exit gate (it is the same
  mount-dependent contact angle), but that wiring is not in this spec.
- **NVS persistence** — deferred (RAM-only here); the per-entry recapture makes
  reboot self-healing, so NVS is marginal until a later productionization pass.
- **Coupled adoption** (epoch translation, §6) is tagged `[VALIDATE]` — it
  assumes a mount shift is ~a rigid translation of both anchors; confirm on HW
  before trusting it beyond a fallback.
- **Absolute-X**, clicks, the cursor smoothing/`o`-`p` knob, and the live-gain
  keys are untouched.
