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
─ vert_hist[300] ring buffer  ── arrays ─▶ extract bottom plateau   ─ anchors ─▶ set_anchors(top,bottom)
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
- **`gesture_mode.cpp` — plumbing only.** Owns a new `vert_hist[300]` ring
  buffer of absolute `vert` (deg) at 100 Hz (separate from `gyro_hist`, ~1.2 KB).
  At AIR_MOUSE entry (`gesture_mode.cpp:512`, where `cursor_track_start` is
  already called) it runs a **strict three-step order**:
  **(1) `cursor_calib_decide()` → (2) `cursor_track_set_anchors()` →
  (3) `cursor_track_start()`**, then emits the `[CAL]` line.
  **The ordering is load-bearing:** the entry slam and the `map(vert)` placement
  in `cursor_track_start` are *sized from the anchors*, so the anchors MUST be
  updated *before* `start` — otherwise the very entry that recalibrates the map
  executes its slam-then-place against the *previous* mount's anchors, and the
  first placement of every recalibrated session lands wrong.
- **`cursor_track.{h,cpp}` — consumer.** `s_vert_top` changes from `const` to a
  variable; a new `s_vert_bottom` variable replaces the derived
  `vert_top + SPAN` (span becomes `bottom − top`, still clamped to
  `CURSOR_VERT_BOTTOM_MAX`). New setter `cursor_track_set_anchors(top, bottom)`
  and getter `cursor_track_vert_bottom()`. `CURSOR_VERT_TOP_DEG` /
  `CURSOR_VERT_SPAN_DEG` are **retained as cold-start defaults**, not removed.

## 3. Data: the `vert_hist` ring buffer

- `static float vert_hist[VERT_HIST_SAMPLES]`, `VERT_HIST_SAMPLES = 300` (**3 s**
  at 100 Hz), written every sample in the acq pipeline alongside `gyro_hist`,
  storing `current_vert_deg()` (the FAST cursor-filter inclination — the same
  signal that drives the cursor Y). This is a **separate, deeper** buffer than
  `gyro_hist` (which stays 1.5 s for its own dictation-flip purpose).
- **~1.2 KB static — and deliberately generous (3 s, not 1.5 s).** The buffer
  must contain the resting plateau **plus** the entire raise. The failure mode
  of an undersized buffer is insidious: a slow raise pushes the plateau out of
  the window, which surfaces as `no-plateau`, which is indistinguishable in the
  telemetry from a genuine lazy mid-air entry — so recalibration would silently
  starve with **nothing in the log pointing at the buffer**. The trigger
  condition is invisible by construction, so we size up front (1.2 KB on 256 KB
  RAM is trivial) rather than wait for a symptom we cannot see. Trim only if
  someone later has a concrete reason to.

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
| **Top plausible, no bottom plateau** (entered from mid-air) | **No-op + SHADOW log.** Adopt nothing (the ritual is *incomplete* — signal 1 failed — so per §5 it must not move the map). But emit the *would-have-been* coupled translation `shadow_bottom = top_now + (prior_bottom − prior_top)` in the `[CAL]` line as `decision=SHADOW-TRANSLATE`, applying nothing. |
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
*(Future option, noted so it is not re-derived: an **adaptive α** — a larger
blend factor for a large but fully-vouched delta (clear re-wear) and a small one
for minor drift — would speed re-wear convergence without losing jitter damping.
Dropped from v1 for simplicity; revisit alongside the two-consecutive trigger.)*

**Why the mid-air case is shadow-only (not a live translate).** A translation
moves *both* anchors at once, yet the only evidence behind it is a top that
passed the plausibility clamp — the weakest signal in the system, on an entry
whose ritual is by definition incomplete (no plateau). Letting that move the
whole map contradicts §5's gate. So we **log the prediction without applying
it**: after a few weeks, compare each `SHADOW-TRANSLATE` prediction against the
*next* full-ritual bottom; promote the translation to a live decision only if
the rigid-shift assumption survives that data. This keeps §5's principle absolute
— **incomplete ritual → no adoption, ever** — while still gathering the evidence
to justify the feature later.

**Epoch rule (explicit):** `top` and `bottom` must always come from the *same*
epoch — both fresh from one complete ritual. The matrix never stores a top from
entry N beside a bottom from entry M as two independent absolutes (the mid-air
case adopts neither; it only shadow-logs).

## 7. Telemetry — `[CAL]` on every entry

```
[CAL] top old=<a> new=<b>  bottom old=<c> new=<d>  span=<d-b> |
      plateau=<Y/N>(var=<v>,n=<k>)  sweep=<deg>  topdwell=<n> |
      decision=<ADOPT|SEED|SHADOW-TRANSLATE|REJECT>  reason=<...>
      [shadow_bottom=<s>]   # only on SHADOW-TRANSLATE: the would-be, NOT applied
```

`reason ∈ { ok, cold-start-default, no-plateau, insufficient-sweep,
implausible-top, below-min-delta, mid-air-shadow }`. The `SHADOW-TRANSLATE`
stream is the dataset that, after a few weeks, decides whether the mid-air
coupled translation is ever promoted to live (§6).

Calibration that silently shifts the map is the subsystem we least want opaque —
this project debugs on trace evidence. As a side effect, the per-entry `[CAL]`
stream **is** the cross-session `vert_top` repeatability dataset that §4.1 of
the original spec demanded before trusting entry-capture — the validation
prerequisite fulfils itself as a logging by-product.

## 8. Constants (`gesture_thresholds.h`, all `[USER]`, seeds tuned from traces)

| Constant | Seed | Meaning |
|---|---|---|
| `VERT_HIST_SAMPLES` | `300` | 3 s of `vert` history at 100 Hz, ~1.2 KB (`[STRUCTURAL]`; deep on purpose — an undersized buffer fails *invisibly* as `no-plateau`, see §3) |
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
  re-slam.) **Must be called BEFORE `cursor_track_start()` at entry** (the §2
  decide→set→start order), so the entry slam in `cursor_track_start` — which is
  sized from the anchors — uses the *just-calibrated* values, not the previous
  mount's.
- `float cursor_track_vert_bottom(void)` — getter for telemetry.
- `cursor_track_vert_top()` now returns the variable.

## 10. Testing

**Host unit tests (both modules are pure):**
- `cursor_calib`: plateau extraction (median, variance gate, dwell floor, no-
  plateau case, **and a slow-raise case where the plateau sits near the OLD edge
  of the buffer** — guards the §3 invisible-starvation failure); sweep gate;
  plausibility clamp; the full adoption matrix — cold-start seed, both-trusted
  blend, **mid-air → SHADOW-TRANSLATE (asserts nothing is applied, shadow_bottom
  is computed)**, all reject reasons, below-min-delta no-op; blend arithmetic;
  epoch rule (no cross-epoch pairing — the mid-air case adopts neither anchor).
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
- Lazy mid-air entry → `decision=SHADOW-TRANSLATE` (top plausible, no plateau)
  or `decision=REJECT reason=no-plateau/insufficient-sweep`; **map unchanged
  either way**; entry still works. Confirm the `shadow_bottom` prediction is
  logged but not applied.
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
- **Coupled adoption** (mid-air epoch translation, §6) ships in **shadow mode
  only** — logged as `SHADOW-TRANSLATE`, never applied. Promoting it to a live
  decision is explicitly out of scope here; it happens only if the shadow
  dataset later shows the rigid-shift assumption holds.
- **Absolute-X**, clicks, the cursor smoothing/`o`-`p` knob, and the live-gain
  keys are untouched.
