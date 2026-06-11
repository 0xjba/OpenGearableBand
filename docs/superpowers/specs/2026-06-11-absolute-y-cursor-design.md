# Spec: Absolute-Y air-mouse cursor (gravity-anchored vertical)

**Date:** 2026-06-11
**Status:** Design for review.
**Scope:** Replace the **vertical (Y)** axis of the air-mouse cursor with an
**absolute** gravity-anchored map. **X stays relative** (roll → dx, unchanged,
cone-gated). No clicks. This is "rung 2" of the four-rung ladder (edge re-reg
→ band-side absolute emulation → host servo → true absolute HID); we build
band-side absolute-Y emulation because macOS pointer-acceleration can be
disabled (below), which makes the band-side position estimate trustworthy.
**Grounding:**
- `docs/research/observability-aware-pose-and-cursor-design.md` §3.5 (gx/gy/gz
  geometry, the cone).
- `docs/superpowers/specs/2026-06-11-air-mouse-cursor-design.md` (the relative
  v1 this revises) + `cursor_track.{h,cpp}`, `cursor_pipeline.{h,cpp}`.
- Hardware trace 2026-06-11: entry `vert ≈ 13–15°`, usable down to `vert ≈ 55°`
  at pose-exit; Euler pitch saturates at high roll (already fixed: Y now driven
  by `vert`, the gravity inclination).

---

## 1. Why (the problem this kills)

Relative/rate pointing maps wrist **motion → cursor motion**, never wrist
**angle → cursor position**. Two things float independently at every entry:
(1) where the cursor happens to be, (2) the inclination you entered at (captured
as the relative zero). Result: "inclination X → screen height Y" has no stable
answer — confirmed by the user being unable to characterise behaviour because
it depends on entry conditions. Absolute-Y ties a given gravity inclination to
a fixed screen height, every entry. The Y signal is already `vert =
acos(|gx|/|g|)` (roll-immune; Euler pitch was roll-contaminated and saturated —
see the relative spec's hardware note).

## 2. Prerequisite — host pointer acceleration OFF

The band-side position estimate assumes **emitted dy counts ∝ pixels moved**.
macOS applies an acceleration *curve* by default (same delta → different pixels
by speed), which corrupts that estimate continuously. Disable it:

- **macOS Sonoma+:** System Settings → Mouse → Advanced → **Pointer
  acceleration → off** (linear 1:1). Older macOS: `defaults write` or
  LinearMouse.
- **This is a GLOBAL mouse setting** — it changes the feel of the user's
  everyday mouse too. If that annoys daily use, **LinearMouse supports
  per-device config** (disable accel only for "Xiao_Pulse"), avoiding a future
  "why does my Logitech feel weird".
- With accel off, the count→pixel scale `k` is a *constant* (the tracking-speed
  setting) but **unknown to the band** and possibly ≠ 1. The band never needs
  `k`: it works entirely in **emitted-count** units. `GAIN_Y` is tuned by feel
  (`}`/`{`) until a full `vert` sweep traverses the screen — which folds `k`
  into `GAIN_Y` implicitly. The slam over-travels generously (§5) so it reaches
  the edge regardless of `k`.

If acceleration cannot be disabled in a given deployment, this design degrades
to "frequent re-pins as damage control" and the honest next rung is true
absolute HID (digitizer) — out of scope here, noted in §9.

## 3. Architecture

```
gesture_mode ──vert,roll,at_rest,shadow──▶ cursor_track (absolute-Y) ──dx,dy──▶ cursor_pipeline ──▶ BLE HID
                                            Y: servo to map(vert)
                                            X: relative roll (unchanged)
```

`cursor_track` gains internal absolute-Y state. It still emits **relative dy**
to `cursor_pipeline` (HID is relative) — but those deltas now servo the cursor
toward an absolute target derived from `vert`, with the entry slam + edge
re-pins providing the known anchor points.

## 4. The vertical map + calibration (option A)

Map `vert → target_counts` from the top anchor. The vertical range is a single
tuned gain plus a comfort-clamped bottom; there is **no independent
"full-screen count" constant** — the reachable range *is* `GAIN_Y × span`:

```
span_deg     = vert_bottom - vert_top            // vert_bottom = comfort-clamped (§4.3)
max_counts   = GAIN_Y * span_deg                 // full-range travel, in counts
target_counts = GAIN_Y * (vert - vert_top)       // vert_top -> 0 (screen top)
target_counts = clamp(target_counts, 0, max_counts)
```

Servo each tick (anchored, NOT pure delta):
```
err = target_counts - cur_y
dy  = clamp(err, -127, +127)        // int8 HID; multi-tick to converge on big jumps
cur_y += dy                         // band's position estimate (counts from top)
```

**Calibration = option A (live top + span knob):**
- `vert_top` is **captured live on AIR_MOUSE entry** (the raised pose is
  reliably near-top in all traces so far). NOT hardcoded. The persisted
  fallback (§4.2) is a **last resort, not a trusted prior** — a band remount or
  a different chair shifts the geometry day to day, so a stale persisted
  `vert_top` can itself be wrong; the §4.1 cross-session check is what tells us
  how much this matters.
- The usable wrist span is **one tunable: `GAIN_Y` (counts/deg)** — tuned on
  `}` / `{` so a comfortable down-sweep fills the screen. `vert_bottom` is the
  comfort-clamped bottom (§4.3); `max_counts = GAIN_Y × (vert_bottom −
  vert_top)` is derived (no independent full-screen constant).

**A's load-bearing assumption — entry-pose repeatability — must be guarded:**
1. **Cross-session validation REQUIRED before trusting A:** log captured
   `vert_top` on every entry; confirm it stays ≈13–15° across several
   sessions/days. Within-session repeatability is not enough.
2. **Sanity clamp on capture:** if entry `vert_top` is implausibly high (e.g.
   `> VERT_TOP_MAX`, a lazy half-raise), WARN and **fall back to the last
   session's `vert_top`** (persist it) rather than anchoring the whole map to a
   bad entry.
3. **Far-end comfort clamp:** if the implied `vert_bottom` lands beyond the
   comfortable lowering range (cursor bottom unreachable), clamp the implied
   `vert_bottom` to measured comfort (`VERT_BOTTOM_MAX`), shrinking the screen
   mapping rather than stranding the bottom edge.

## 5. Slam mechanics (entry + re-pin)

HID deltas are **int8 (±127)**, so a slam is a **burst of reports**, not one
write:
- **Burst size: a GAIN_Y-independent FLOOR, not just a margin on the tuned
  range.** Sizing the slam purely as `SLAM_MARGIN × max_counts` is circular —
  `max_counts = GAIN_Y × span`, so an undertuned `GAIN_Y` makes the slam
  *undershoot*, the band falsely believes the cursor is at the edge, and
  registration is silently corrupted from the first second (and the re-pins
  inherit the undershoot rather than healing it). **Overshoot is free (the OS
  clamps); undershoot is poison.** So:
  ```
  slam_counts  = max(SLAM_MARGIN * max_counts, SLAM_FLOOR_COUNTS)
  N            = min(ceil(slam_counts / 127), SLAM_MAX_REPORTS)
  ```
  `SLAM_FLOOR_COUNTS` is a generous absolute over-travel (≫ any plausible
  screen-height-in-counts at any reasonable tracking speed), so the slam
  reaches the edge **even with `GAIN_Y` completely untuned**. `SLAM_MARGIN`
  (≥ 2×) lets it scale up further for large tuned ranges. `SLAM_MAX_REPORTS` is
  a *generous* cap (a too-tight cap is just another undershoot path).
- **Entry slam = top:** burst of `-127` dy (up) → OS clamps cursor at `y=0`.
  Then set `cur_y = 0` and begin servoing to `map(vert)`.
- **SUPPRESS normal tracking during a slam:** a `slam_active` flag; while set,
  `cursor_track` emits only the slam burst — user wrist motion must NOT mix in,
  or the clamp is corrupted. Re-enable tracking when the burst completes.
- Bursts are emitted across consecutive `cursor_pipeline` publish ticks (or a
  dedicated rapid sequence); spec the cadence so the whole slam completes in
  < ~200 ms (imperceptible-ish, but a deliberate visible jump is accepted).

**"slam-then-place", not "pin":** the band cannot place or read the cursor; it
*manufactures* a known position by slamming to an edge, then servos. Entry
therefore **abandons the prior cursor position** (determinism over continuity —
adopted knowingly).

## 6. Re-pin triggers (drift correction at extremes)

- **Top re-pin:** when `vert <= vert_top` (fully raised) — re-slam to top,
  `cur_y = 0`, and update `vert_top` (running min) **only while clamped** (see
  §7).
- **Bottom re-pin:** option-A minimal = bottom handled purely by clamping
  `target_counts` to `max_counts` (no bottom slam). Bottom slam + self-calib is
  the §7 upgrade.
- **Hysteresis / dwell on EVERY pin trigger.** Fully-raised / fully-lowered
  detection near the calibrated extremes is another chattering boundary (same
  failure class as the orientation classifier and the cone gate). Require a
  dwell or enter/leave hysteresis band on the pin trigger; thresholds `[USER]`
  empirical. No bare single-sample threshold.

## 6b. Out-of-span behaviour + the single stillness authority

- **`vert < vert_top`** (raised past the top anchor): `target_counts` clamps to
  `0`; this is the top re-pin region (§6).
- **`vert > vert_bottom`** (lowered past the comfort bottom): `target_counts`
  clamps to `max_counts` — cursor parks at the bottom edge. In option A nothing
  else happens; this is exactly where the §7 B-upgrade later extends the span
  (only while clamped).
- **Stillness has ONE owner on Y, and it's the servo — not the freeze gate.**
  With the servo, when the wrist is still `vert` is static → `target_counts` is
  static → `err → 0 → dy → 0` *naturally*. So the relative-mode `at_rest` freeze
  gate is **redundant on Y and must be disabled for the absolute-Y axis** (two
  authorities fighting over Y stillness is a bug). The `at_rest` freeze gate
  **remains only on X** (still relative). State authority explicitly in code so
  a later reader doesn't "restore" the Y freeze.

## 7. Designated A→B upgrade (non-circular self-calibration)

Option B (self-calibrating `vert_bottom`) is **circular if naive** (the bottom
re-pin needs the bottom to know when to fire, but the bottom is what's being
calibrated) and, if resolved as a running-max, **rescales the map mid-session —
reintroducing the non-determinism this build exists to kill.**

**The fix, specced now so B isn't re-invented badly later:** rescale the map
**only while the cursor is clamped at an edge.** When the user reaches a new
lowest `vert` AND the cursor is pinned at the bottom edge, extend `vert_bottom`
(one-directionally, growing `max_counts`) *at that pinned moment* — the cursor
is already at the edge,
so the rescale is invisible. This yields a clean A→B path: ship A (top anchor +
clamp), then let a bottom re-pin extend the span only while pinned. Do NOT
rescale while the cursor is mid-screen.

## 8. Constants (`gesture_thresholds.h`)

| Constant | Initial | Tag | Meaning |
|---|---|---|---|
| `CURSOR_GAIN_Y` | retune (~30) | `[USER]` | counts/deg (now servo gain; tuned on `}`/`{` so full sweep ≈ full screen) |
| `CURSOR_SLAM_MARGIN` | 2.0 | `[STRUCTURAL]` | over-travel factor on `max_counts` (overshoot free, undershoot poison) |
| `CURSOR_SLAM_FLOOR_COUNTS` | 6000 | `[STRUCTURAL]` | GAIN_Y-independent minimum over-travel; slam clamps even untuned |
| `CURSOR_SLAM_MAX_REPORTS` | 80 | `[STRUCTURAL]` | generous cap on burst length (too-tight = undershoot path) |
| `CURSOR_VERT_TOP_MAX` | 25.0 | `[USER]` | reject entry `vert_top` above this (lazy-raise guard) → fall back to persisted |
| `CURSOR_VERT_BOTTOM_MAX` | 60.0 | `[USER]` | comfort clamp for the implied bottom (caps `span_deg`) |
| `CURSOR_PIN_ENTER_DEG` / `_LEAVE_DEG` | 3.0 / 6.0 | `[USER]` | hysteresis band (deg) around an extreme for the re-pin trigger (LEAVE ≥ ENTER) |

`max_counts = GAIN_Y × (vert_bottom − vert_top)` is **derived**, not a stored
constant (avoids the COUNT_FULL/GAIN_Y redundancy).

X-axis constants (`CURSOR_GAIN_X`, cone-gate shadow pair, freeze, max-delta)
are **unchanged** — X stays relative.

## 9. Telemetry / testing
- Extend `[CURSOR]` log: add `vert_top`, `target_counts`, `cur_y`, `slam`
  state, `pin` events.
- **Persist + log `vert_top` per entry** for the §4.1 cross-session check.
- Test (accel OFF): enter → cursor slams to top, descends with the wrist;
  raise fully → cursor returns to top (re-pin); a given inclination lands the
  cursor at the same height across entries (the determinism acceptance test).
- Verify the slam burst doesn't fight a moving wrist (suppression works).
- **Slam-undershoot acceptance test (makes the silent failure visible):** fully
  raise — **the cursor MUST visibly clamp at the very top edge with margin** (it
  should "stick" there, not stop short). If it stops short, `GAIN_Y` is
  undertuned and/or `SLAM_FLOOR_COUNTS` too small — increase `GAIN_Y` (`}`)
  until the slam clamps hard. Until this passes, position truth is unreliable
  and every other Y observation is suspect — run it FIRST each session.

## 10. Out of scope (this spec)
- Absolute **X** (roll has a gravity reference too — a later rung, same pattern,
  only after absolute-Y proves out).
- True absolute HID (digitizer) — the rung above this; needs a separate
  "search-and-confirm how macOS treats a BLE digitizer" pass first.
- Host-side servo, clicks, SURFACE cursor, the tap-entry power-state fix.
