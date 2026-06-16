# X Auto-Anchor + Center-on-Entry — Design Spec

**Date:** 2026-06-16
**Status:** design approved (Sections 1–2); file-level plan (Section 3) + implementation PENDING
**Branch:** feature/air-mouse
**Builds on:** yaw-driven X axis + sweep-coupling compensation
(`2026-06-15-horizontal-x-axis-design.md`, `2026-06-16-sweep-coupling-compensation-design.md`)

---

## 1. Problem

X is a *relative, floating* axis (Y is *absolute, anchored* — it slams to the top
edge on entry). With relative HID output the firmware has no idea where the cursor
sits horizontally, so on entry the X origin = wherever the Mac pointer happened to
be. Observed on HW: "reach the left edge fully but not the right" — the origin sat
left-of-center, so a comfortable sweep reached left easily but ran out before right.
Root: no absolute X reference (the no-magnetometer limit, same family as the arc).

## 2. Approach (chosen: "A — automatic anchor on entry")

The only absolute reference X can get is a **screen edge** (Y uses the top edge).
So on each AIR_MOUSE entry, after the Y top-snap, X auto-anchors with **no user
ritual**:

1. **Edge-snap (the calibration):** slam the cursor to the **left** screen edge.
   The OS pins it there, so the firmware finally knows the absolute horizontal
   position. (Mandatory — without it the asymmetry stays.)
2. **Slide-to-center (for the user):** then move the cursor right by exactly one
   comfortable half-sweep's worth of travel, so **forearm-ahead = screen center**.
   Calibration is already done at the edge-snap; this step is purely ergonomic so
   both edges are reachable symmetrically. User confirmed: ahead = center.
3. Then normal yaw-driven sweep tracking continues from center.

**Coupling that keeps it simple:** the slide-to-center distance and the sweep reach
use the *same* X gain, so tuning the X gain (`]`/`[`) until a comfortable half-sweep
reaches an edge makes "ahead = center" fall out automatically — no separate knob.
center_counts = |gain_x| * CURSOR_YAW_HALF_SPAN_DEG.

## 3. Drift / pauses / arc (Section 2, approved)

- **Cone-gate near vertical:** X freezes (holds position), resumes cleanly (prev
  re-synced) — unchanged.
- **Pauses (at_rest):** X freezes; the gyro at-rest re-zero lands invisibly.
- **Drift caveat (no-magnetometer limit):** heading slowly drifts over a long
  unbroken session, so the center can wander and the reach asymmetry can *slowly*
  creep back. Fix is automatic: **re-entering air-mouse re-runs the edge-snap +
  recenter**, resetting it.
- **Arc-comp folds in:** the sweep distance for the swing-coupling correction is now
  measured from the established center (cursor displacement from mid-screen), instead
  of a floating zero. Arc strength stays the separate `o`/`p` dial — decoupled from
  centering (centering fixes L/R reach, `k` fixes the vertical bow).

## 4. Implementation sketch (Section 3 — NOT yet detailed/built)

Likely: convert X from pure relative-delta to an absolute-ish servo mirroring Y, OR
keep relative-delta + a one-time entry "slam-left + burst-to-center" sequence (lighter).
Reuse the existing slam machinery (`start_slam`, the publish-thread 127-counts/tick
drain). `cursor_track` already tracks `s_cur_x` (added for the arc-comp); the center
established at entry becomes the arc-comp reference. Edge/at-rest re-anchor of the
center mirrors the existing prev-yaw re-sync. Host-test centering + clamp + re-anchor.
Decide servo-vs-relative-recenter at plan time.

## 5. Status note (parked 2026-06-16)

Air-mouse was parked to `feature/air-mouse` and extracted from the foundation. This
design is captured here for when air-mouse development resumes. Prior interim work on
this branch (committed alongside): L/R sign flip + sign-preserving gain clamp,
sweep-coupling compensation (`CURSOR_SWING_COMP_K`, `o`/`p` knob), `[XARC]` diagnostic
(temporary — remove when the X-anchor + arc are validated together on HW).
