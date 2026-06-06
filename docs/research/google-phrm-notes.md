# Notes on Google PHRM (rPPG) — applicability to gestureband

**Source:** https://research.google/blog/towards-passive-heart-health-monitoring-via-smartphone-camera/

**Status:** raw working notes, captured 2026-06-05 while reviewing the
blog post. Not yet polished into a design doc. The takeaways below are
correct in substance; the framing / prioritization will change once we
actually start the snapshot-aggregation work.

> TODO when we get to that stage:
> - Re-verify the published MAPE numbers against the underlying paper
>   (the blog post is a summary). The paper may break the error down by
>   activity class, which would let us compare apples-to-apples against
>   our worn/stationary vs nlms-heavy paths instead of against an
>   undifferentiated free-living average.
> - Decide whether the per-snapshot confidence number is exposed over
>   BLE as a raw 0-100 or as a discrete tier (e.g. green/yellow/red).
>   App-side ergonomics, not a firmware decision.

---

## Their setup vs ours (so the comparisons below stay honest)

| Axis | Google PHRM | gestureband |
|---|---|---|
| Sensor | Phone camera, rPPG | MAX30102 contact PPG (IR ~880 nm) |
| Compute | Phone SoC, CNN | Cortex-M4 @ ~64 MHz, classical DSP |
| Window | 8-second clips | 5.12-second windows, 50% overlap |
| Use case | Passive RHR over a day | Live HR + snapshots + workout |
| Motion handling | None at v1; listed as future work | NLMS + IMU shadow mask today |

Different problems. The architectural patterns transfer; the
implementation does not.

---

## Transferable ideas

### 1. Two-tier Kalman: per-window for live, per-snapshot for aggregation

Their pipeline:
- per-clip BPM + confidence score
- gate out low-confidence clips
- Kalman across surviving clips → one daily RHR

We currently Kalman per-window in real time and that's the only Kalman
in the pipeline. The on-demand snapshot feature (already on the v0.3
backlog) is the natural place to add a second aggregator that:

1. Collects every window of the snapshot
2. Keeps only windows that cleared SQI + delta + (motion == STATIONARY
   OR autocorr path)
3. Emits one BPM as `kalman_over_surviving_windows.x`, plus a count

> TODO: decide whether the snapshot aggregator should also be a Kalman
> filter or just a trimmed-mean / median. Kalman is overkill if we
> already discarded the low-confidence stuff. Median-of-N is robust,
> trivial, and explains itself in one line of code.

### 2. Confidence score, exposed alongside BPM

Google doesn't publish the confidence-score recipe. For us the components
are already computed; we just need to combine them:

- `sqi_passed` (bool)
- `motion == STATIONARY` (bool, strong vs MICRO/HEAVY)
- `delta_ok` (bool)
- `|raw_bpm - kalman.x|` post-update (small = good)
- `wear_state == WORN` (bool)

Cheap path: confidence = fraction of snapshot windows that cleared every
gate. 80/100 → confidence 80%. Trivial, defensible, observable.

> TODO: instrument the per-window innovation magnitude
> (`raw_bpm - kalman.x` *before* the update) and log it. This lets us
> later build a calibration curve mapping innovation → expected error
> from real traces, instead of guessing weights.

### 3. Benchmark numbers we can hold ourselves to

From the blog (numbers verbatim):

- HR MAPE < 10% across all skin tones (their bar for "industry standard")
- Free-living MAPE: 6.09% overall (Group 1 5.04%, Group 2 5.12%, Group 3 7.84%)
- RHR MAE: target < 5 bpm; achieved 4.39 bpm overall
- Bland-Altman bias: -0.64 bpm for HR, -0.1 bpm for RHR

Where we sit by user observation:
- Worn / stationary / autocorr path: roughly ±1-2 BPM vs Apple Watch.
  Already well inside 6%.
- nlms-heavy during sprint climb: peaked at 122 vs Apple's ~133.
  ~8% error on that single trace. Currently outside their bar.
- Cooldown / autocorr return: "almost perfect."

So the heavy-motion path is the only one with daylight between us and
their numbers. Dynamic IMU shadow mask (commit `51e8b1f`) is the
current attempt to close that gap; we will know after the next sprint
trace.

> TODO: once we have a clean trace post-`51e8b1f`, compute our own
> MAPE on the heavy-motion windows specifically and put the number
> here. Don't compare to their free-living average — compare to a
> matched activity-class subset if the paper has one.

---

## Not transferable

- **Temporal-shift CNN trained on ~350k clips from ~700 participants.**
  We have ~256 KB RAM and no labelled dataset. Classical DSP is the
  right call for our form factor; the paper does not change that.
- **8-second windows.** They can afford it (no battery constraint on a
  phone). We can't, and our autocorr does fine at 5.12 s with the
  AUTOCORR_TRANSIENT_SKIP of 80 samples. Not a signal we should chase.
- **Specific peak-detection algorithm.** Blog doesn't disclose. Nothing
  to copy at the DSP level.

---

## What gestureband already does better than them (at v1)

- **Accelerometer-based gating** is listed as *future work* in their
  paper. Our `imu_smv` motion classifier + chained NLMS + dynamic
  shadow mask is that whole feature already.
- **Real-time HR**, not just daily-aggregate RHR. They are explicitly
  passive / RHR-focused.
- **Wear detection.** Their pipeline assumes the user pointed the
  phone at their face; ours has WEAR_NOT_WORN / STABILIZING / WORN
  with hysteresis.

Not bragging — just calibrating expectations about what "lessons from
Google" actually means for our roadmap. Most of their advantage is
data and compute; most of our advantage is contact + IMU + always-on.

---

## Skin-tone note

Their Group 3 (darkest skin tones) had MAPE 7.84% vs Group 1's 5.04%
— a real and reported gap on rPPG (visible-wavelength imaging).

For us, MAX30102 IR @ ~880 nm penetrates melanin much better than green
or visible light, so we should fare better than they did. But "should"
is not "do." Once we have something shippable, we should run sanity
checks across the skin-tone range available to us.

> TODO: add a paragraph here once we have any real cross-skin-tone
> data. Until then this section is a reminder, not a finding.

---

## Concrete actions if/when we revisit

1. When wiring the on-demand BLE characteristic, return
   `{ bpm, confidence_pct, window_count }` not just `bpm`.
2. Log `|raw_bpm - kalman.x|` (innovation magnitude) per window so we
   can build a confidence-calibration curve from traces.
3. After the next sprint trace, compute our own MAPE for the
   nlms-heavy path windows specifically and put the number in the
   benchmark section above.

Both of (1) and (2) are ~30-line additions and neither blocks current
work.
