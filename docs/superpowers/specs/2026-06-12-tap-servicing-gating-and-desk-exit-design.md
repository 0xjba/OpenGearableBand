# Spec: Tap-servicing power-state gating fix + desk-exit via the chip tap engine

**Date:** 2026-06-12
**Status:** Design for review.
**Scope:** (1) Service the LSM6DSL chip tap interrupt in **all** power states, not
just `PS_IDLE`. (2) Once verified on hardware, switch the AIR_MOUSE desk-settle
exit from the software `r_mag` impact detector to the chip tap engine, dropping
`CURSOR_IMPACT_THRESH`.
**Grounding (memory):**
- `project_sig_motion_interference_2026_06_09` — Issue 2 (tap dispatch gated to
  IDLE); fix option (3) "route tap dispatch in all power states" is the
  recommended move once tap loss is observed outside IDLE (now confirmed: the
  HR SNAPSHOT fires during air-mouse and drops taps). The sig-motion→WORKOUT
  hijack (Issue 1) is already RESOLVED via the `gesture_mode_recent_activity()`
  guard — do NOT re-open it.
- `finding_tap_while_moving_2026_06_11` — the chip tap engine MISSES taps during
  continuous arm motion (accel railed, no shock→QUIET). A desk landing is
  shock-then-rest (cleaner), but Part 2 is **gated on empirical confirmation**.
- `project_chip_tap_and_calibration_pickup_2026_06_08` — the calibration-log
  discipline used to verify tap behavior empirically before committing thresholds.

---

## 1. Why

`gesture_mode_update_accel` (acq thread) runs every sample in all states, but the
**chip tap interrupt is only serviced in `run_idle()`** (`main.cpp`). The INT1
ISR gives `motion_wake_sem`; only `run_idle` consumes it (reads TAP_SRC +
FUNC_SRC1, dispatches taps + the guarded sig-motion transition). `run_snapshot`,
`run_workout_verify`, `run_workout` never consume it, so **any tap during those
states is silently lost** (the binary sem just sits set until IDLE returns, by
which time the multi-tap window / desk moment has passed).

This bites two ways:
- **Entry taps** during the periodic HR **SNAPSHOT** (~22 s every ~couple min,
  and it fires *during* air-mouse — confirmed in the 2026-06-12 trace) are
  dropped → intermittent "double-tap doesn't enter."
- **Desk-exit via the tap engine** (Part 2) is impossible while taps die in
  SNAPSHOT, which is exactly when you're air-mousing.

## 2. Part 1 — service the chip INT in all power states

Keep servicing **on the power thread** (so `power_state` is never mutated
cross-thread) by factoring the `run_idle` demux into a shared helper and calling
it from every state loop's wait point.

### 2.1 `service_chip_int1(bool handle_sigmotion)`
Extract the existing `run_idle` demux body into:
```
service_chip_int1(handle_sigmotion):
    if !sem_take(motion_wake_sem, NO_WAIT): return
    read TAP_SRC, FUNC_SRC1          # MUST read both -> clears both LIR latches,
                                     # de-asserts INT1 (else it sticks high)
    if tap_fired:
        [CAL] log (if tap_calibration_mode)
        gesture_mode_bio_acoustic_on_tap()
        gesture_mode_on_chip_single_tap(axis, sign)   # gesture FSM; atomic, no power mutation
    if sigm_fired and handle_sigmotion:
        # UNCHANGED IDLE behavior incl. the recent-gesture guard
        if gesture_mode_recent_activity(): log "suppressed"
        else: k_sem_reset(snapshot_tick_sem); transition_to_workout_verify(); return SIGM_WORKOUT
    # sigm_fired and !handle_sigmotion -> latch already cleared, ignore (mid-snapshot/workout
    #   a sig-motion is irrelevant; we're already sampling)
    return OK
```
- **Tap dispatch runs in EVERY state** → the gesture FSM sees taps always.
- **Sig-motion→WORKOUT stays IDLE-only** (`handle_sigmotion=true` only from
  `run_idle`). A sig-motion mid-SNAPSHOT/WORKOUT is meaningless; we just clear it.
- `run_idle` calls `service_chip_int1(true)`; it still also waits on
  `snapshot_tick_sem` for the IDLE→SNAPSHOT timer.

### 2.2 Wait-point changes per loop
- **`run_idle`** — unchanged structure: `k_poll(snapshot_tick_sem,
  motion_wake_sem)`; on motion → `service_chip_int1(true)`.
- **`run_snapshot`** — currently `while(deadline){ k_sleep(500ms); wear check }`.
  Change the wait to `k_poll(motion_wake_sem, timeout=500ms)`:
  - woke on motion → `service_chip_int1(false)`, loop (no extra sleep).
  - timed out → do the existing 500 ms wear/abort check.
  This keeps the wear cadence **and** services taps promptly.
- **`run_workout_verify` / `run_workout`** — add the same `service_chip_int1(false)`
  to their wait points (lower priority — air-mouse during a workout is unlikely —
  but include for the "gesture detection always works" guarantee; mirror the
  snapshot pattern).

### 2.3 Constraints
- I2C: TAP_SRC/FUNC_SRC1 reads now happen from the power thread in more states,
  but still **only on the power thread** (never the ISR, never acq) — no new
  bus-concurrency surface beyond what `run_idle` already had.
- The bio-acoustic FIFO capture on tap must still function in SNAPSHOT (the
  accel FIFO runs independently of the MAX30102/PPG). Verify on hardware.
- No change to the ISR (`lsm6dsl_int1_isr` keeps giving `motion_wake_sem`).

### 2.4 Part 1 acceptance (hardware)
- Enter air-mouse, wait for `PowerState: IDLE -> SNAPSHOT`, then double-tap to
  test entry / or single-tap: the `Chip single-tap` log MUST appear **during**
  the SNAPSHOT window (today it does not). Confirm no INT1 stuck-high (taps keep
  working after several SNAPSHOT cycles).

## 3. Part 2 — desk-exit via the chip tap engine (verification-gated)

**Do NOT build Part 2 until Part 2.1 confirms the chip tap fires on a desk
landing.**

### 3.1 Verify first (real-world grounding)
With Part 1 flashed, air-mouse and deliberately lower onto the desk several
times. Watch for a `Chip single-tap` log at the moment of contact (near-flat).
- If it fires **reliably** on desk contact → proceed to 3.2.
- If the descent motion masks it (per `finding_tap_while_moving`) → STOP; keep
  the `r_mag` exit, and note that a motion-aware / high-ODR path is needed. Part 1
  still stands on its own (entry-tap reliability).

### 3.2 Switch the exit (only if 3.1 passes)
- Add a hook: when a chip tap fires AND `gesture_mode_get()==MODE_AIR_MOUSE` AND
  near-flat (`gx_filt < CURSOR_DESK_ZONE_GX`), set an atomic `air_desk_tap` flag.
- In the AIR_MOUSE exit branch, **replace the `r_mag`-impact path** (trigger (a))
  with: `air_desk_tap` set → desk contact. Keep the optional settle confirm if
  desired, or exit immediately (clicks not implemented; a near-flat tap == landing).
- **Remove** `CURSOR_IMPACT_THRESH`, `CURSOR_SETTLE_DWELL`, and the `r_mag`-based
  impact latch (`air_impact_seen`) — the chip's `TAP_THS` is now the (hardware,
  debounced) bar. Keep `r_mag` only if still used by the activity gate.
- **Trigger (b) past-plane (`gx < CURSOR_PAST_PLANE_GX`) is UNCHANGED** — pure
  geometry, no tap needed, the no-desk path.
- Clear `air_desk_tap` on exit / mode transition / when leaving near-flat.

### 3.3 Why this is better (the user's point)
A desk hit IS a tap; the chip's 833 Hz hardware peak-detect + SHOCK/QUIET
debounce is purpose-built for it and catches sharp impacts that 100 Hz `r_mag`
can under-sample. It also deletes a software threshold (`TAP_THS` is the bar).
The only reason we used `r_mag` first was the gating this spec's Part 1 fixes.

## 4. Out of scope
- Tap-while-moving in-session clicking (separate finding; high-ODR path).
- sig-motion sensitivity tuning (Issue 1 already mitigated).
- Any SURFACE revival.

## 5. Order
Part 1 (gating fix) → flash → §2.4 + §3.1 hardware observation → Part 2 only if
§3.1 confirms. Part 1 is committed/valuable regardless of Part 2's outcome.
