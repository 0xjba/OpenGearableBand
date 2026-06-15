# Software optimization roadmap — closing the gap with high-end commercial bands

**Source of this doc:** research session 2026-06-06, prompted by sub-harmonic
autocorr failure observed once in trace, generalized to "what are commercial
high-end bands doing that we aren't, and which of those should we adopt?"

**Status:** research notes + concrete 4-stage proposal. Nothing implemented
yet. Picked up later when we return from the sprint-accuracy work.

**Thesis:** Our hardware is hobbyist (MAX30102 + LSM6DS3TR on Cortex-M4),
but software is essentially free. The published gap between our current
algorithm and what Apple / Galaxy / Fitbit publish in patents and
reference designs is reachable with ~150 lines of new code and ~1 ms
extra compute per stationary window. We should close it before any
"ship to others" milestone.

> TODO before implementing:
> - Re-verify each stage against the actual referenced source — the blog/
>   patent summaries we pulled may simplify. Particularly the harmonic-sum
>   method (Stage 2) — check the arXiv paper directly for the exact
>   formulation.
> - Decide whether to expose confidence over BLE as raw 0-100 or as
>   discrete tiers. App-side ergonomics question, not a firmware decision.
> - Decide whether multi-feature SQI replaces the current variance gate or
>   composes with it.  Default plan: replace.

---

## Why this exists (context)

In our v0.3 sprint-accuracy work we hit a sub-harmonic failure mode in
the autocorrelation path — raws of 42-44 BPM when the true cardiac rate
was ~85. The instinct was to add a sub-harmonic guard. The user pushed
back: *"is this what fitness bands actually do, or are we band-aiding?"*

Research showed: multi-method decision fusion with confidence-weighted
Kalman is documented industry standard in higher-end commercial bands
(Apple patent verbatim, Analog Devices reference designs, Samsung
Galaxy Watch BioActive descriptions). Single-method autocorr-only is
the **low-end commercial pattern.**

We can match the high-end pattern on our hardware cheaply. This doc
captures the plan so we don't lose it.

---

## Gap analysis

| Stage | High-end commercial pipeline | gestureband current | Gap severity |
|---|---|---|---|
| Bandpass + detrend | 4th-order Butterworth, linear detrend | 4th-order Butterworth, endpoint detrend | none |
| Signal quality assessment | Multi-feature SQI: skewness, kurtosis, perfusion, template-matching, zero-crossing, entropy | Single variance threshold | **big** |
| Motion artifact removal | NLMS / Wiener / SSA / sparse reconstruction | 3-axis chained NLMS | moderate |
| Spectral estimation | FFT + autocorr + sometimes MUSIC/sparse | FFT (motion) OR autocorr (stationary) | moderate |
| Peak picking | Harmonic-sum, sub-harmonic guard, harmonic consistency | First-prominent-peak walk + IMU/stride mask | **big** |
| Multi-method fusion | Weighted decision fusion across methods | Single method per motion state | **big** |
| State tracking | Kalman with adaptive measurement noise (RAKF) | 1D Kalman with motion-state-dependent noise | minor |
| Per-window confidence | Continuous 0-1 score (Apple patent verbatim) | Binary in_range / delta_ok | moderate |
| Adaptive method selection | Quality-aware: high-SQI → FFT, low-SQI → autocorr or skip | Motion-state-only switching | moderate |

**Apple's verbatim pattern from US patent 9,826,940:**
> "weighted combination of the predicted pulse rate and the determined
> frequency of the local maximum, based on a weighting factor related
> to the determined confidence level."

That's Kalman update with confidence-weighted measurement gain. They
explicitly tie estimate trust to a measured-confidence number.

**Published academic state-of-the-art for wrist PPG during heavy motion:**
~2.08-2.34 BPM MAE using TROIKA / JOSS (SSA + sparse spectral
reconstruction). This is significantly better than commercial bands
during activity (~9.5 BPM MAE on average across 10 commercial bands).
The accuracy ceiling is set by algorithm choice, not by hardware.

---

## What we deliberately won't chase

Honest exclusion list with reasoning, so we don't revisit these later
without justification:

- **SSA / sparse spectral reconstruction (TROIKA, JOSS).** Best published
  accuracy (~2 BPM MAE during running) but RAM-heavy matrix ops.
  Borderline on M4; high implementation risk; we'd be at the bleeding
  edge of embedded DSP.
- **MUSIC subspace algorithm.** Marginal gain over FFT at our 5.12 s
  window sizes; high compute.
- **Multi-light-path fusion.** Hardware limitation — MAX30102 has IR +
  Red but our pipeline only uses IR. Could revisit if we ever care about
  SpO2.
- **Deep learning models (foundation models for PPG).** Out of scope on
  M4; need MB of memory; need labelled training data.
- **Adaptive template-matching SQI.** Per the literature: *"feature-
  specific algorithms are computationally expensive for embedded
  wearables where power for processing information is critical for
  battery-based systems."* Skewness gives 86 % of the benefit at 5 %
  of the cost.

These exclusions are deliberate. If we later have a reason to revisit
(e.g. the simpler approaches don't get us where we need to be), the
reason needs to be in writing.

---

## Stage 1 — Multi-feature SQI

**Replace the binary variance gate with a continuous quality score.**

### Why

Variance alone is a poor SQI. A noisy signal with cardiac content can
have similar variance to a clean weak signal; we either reject good
windows or accept bad ones. Multi-feature SQI distinguishes them.

Published evaluation across 8 SQI metrics
([Optimal SQI for PPG, PMC 2017](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5597264/)):
- **Skewness alone:** F1 = 86.0 % at "excellent vs acceptable",
  87.2 % at "excellent vs unfit", 79.1 % at "acceptable vs unfit"
- Outperformed perfusion, kurtosis, relative power, non-stationarity,
  zero crossing, entropy, and matching of systolic wave detectors
- Single most predictive metric

### What

Combine three cheap features:
1. **Skewness** — primary discriminator, F1 ≈ 86 %
2. **Kurtosis** — secondary, complements skewness
3. **Perfusion ratio** — AC amplitude / DC level (already implicit in
   our `ppg_var / ppg_dc` but not used)

Output: continuous 0-100 SQI score per window. Map to:
- > 70 = high quality (full processing)
- 30-70 = degraded (conservative method selection)
- < 30 = unusable (hold Kalman)

### Cost

~30 lines, three CMSIS-DSP statistics calls (`arm_var`, manual
kurtosis if no DSP function, perfusion = arithmetic), no new memory.

### Code touch points

`WearableDSP::processHeartRate` — replace `sqi_passed` bool with
`int sqi_score`. Threshold checks become tiered.

> TODO: CMSIS-DSP doesn't ship `arm_kurtosis_f32` AFAIK — verify and
> if true, hand-roll a O(N) one-pass implementation.

---

## Stage 2 — Dual-method cross-validation at STATIONARY

**Run both autocorr and FFT on every stationary window; reconcile
with harmonic awareness.**

### Why

The sub-harmonic failure mode that triggered this whole roadmap. Single-
method autocorr at rest can lock onto the 2T peak when the cardiac T-peak
is weaker than the global max in the in-band correlation. Adding an
isolated sub-harmonic guard catches this one failure mode but doesn't
generalize. The principled fix is what Apple, Galaxy, Analog Devices
all do: run both methods, reconcile.

From the patent corpus (US patent 10,285,651):
> "Heart rate estimation methods use both autocorrelation and Discrete
> Fourier Transform (DFT) approaches, computing autocorrelation over
> one time period and DFT over a second, longer time period, with the
> final heart rate determined based on both estimates."

### What

Per stationary window, compute both:
- `bpm_autocorr` from `runAutocorrelation()`
- `bpm_fft` from a new `runStationaryFFT()` (FFT on the same band-pass
  buffer, peak in [0.6, 3.3 Hz], no NLMS, no stride masking)

Decision logic:
1. **Agree within ±5 BPM:** confidence = HIGH, use average, kalman.r =
   low
2. **Disagree, but harmonic check resolves it:**
   - Does FFT show a peak at 2× the smaller candidate? At 3× the
     smaller candidate?
   - Harmonic-consistent candidate wins, confidence = MEDIUM
3. **Disagree, no harmonic structure:** confidence = LOW, hold Kalman
   (no update)

### Cost

~80 lines, one extra FFT per stationary window (~1 ms on M4 with
`arm_rfft_fast_f32`), no new memory (reuse `fft_output`, `fft_magnitudes`).

### Code touch points

- New `WearableDSP::runStationaryFFT(float* ppg)` (no stride masking
  unlike the motion-path `runFFT`)
- `WearableDSP::processHeartRate` STATIONARY case rewritten to compute
  both and reconcile
- May want a helper `harmonicallyConsistent(float candidate, float* fft_mag)`

---

## Stage 3 — Per-window confidence score (0-100)

**Replace `delta_ok` / `in_range` booleans with a continuous confidence
number propagated to caller and (eventually) to BLE.**

### Why

- App-side UX: returning `{bpm, confidence_pct}` lets the app decide
  whether to display the value, average across recent windows, or warn
  the user
- Foundation for Stage 4 (adaptive Kalman r)
- Foundation for the snapshot-aggregation backlog item from
  `google-phrm-notes.md` ("confidence = fraction of snapshot windows
  that cleared every gate")
- Apple patent describes it verbatim; not exotic, just structured.

### What

Combine these into a single 0-100 number per window:
- SQI score from Stage 1 (0-100)
- Method-agreement from Stage 2 (1.0 if methods agreed, 0.5 if
  resolved by harmonic, 0.0 if disagreed without resolution)
- Delta-from-kalman penalty: 1.0 if `|raw - kalman.x|` < small,
  scaling down as delta grows
- Wear-state factor: 1.0 only when `WEAR_WORN`
- Motion-state factor: 1.0 stationary, 0.7 micro, 0.5 heavy

`confidence = SQI × agreement × delta_factor × wear_factor × motion_factor`

Log it as part of the `dsp:` line so we can see distributions.

### Cost

~20 lines.

---

## Stage 4 — Adaptive Kalman measurement noise (RAKF pattern)

**Scale Kalman's measurement-noise term `r` inversely with confidence.**

### Why

Current Kalman has three fixed `r` values (0.5 / 2.0 / 5.0 for
stationary / micro / heavy). Apple's patent and the RAKF pattern
published in
[Advanced rPPG framework, PMC 12818640](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12818640/)
both vary `r` per-window based on measurement confidence.

The RAKF paper reports **MAE = 0.72 BPM on the PURE dataset** — that's
the published ceiling for confidence-adaptive Kalman + DWT on rPPG.
We can match the structural pattern; we won't match the absolute
number on a wrist contact PPG (different problem).

### What

```cpp
kalman.r = R_BASE * (1.0f - confidence / 100.0f) + R_FLOOR;
```

With `R_BASE` ~ 5.0 and `R_FLOOR` ~ 0.3. High confidence → low r →
Kalman trusts measurement → fast convergence. Low confidence → high
r → Kalman trusts state → smoothing dominates.

### Cost

~5 lines. Defer until we've seen real confidence-score distributions
from Stage 3 traces and can tune `R_BASE` / `R_FLOOR` against data.

---

## Implementation order

Recommended: **1 → 2 → 3 → defer 4 until we see trace distributions.**

| Stage | LOC | Compute | New RAM | Risk | Order |
|---|---|---|---|---|---|
| 1 — Multi-feature SQI | ~30 | trivial | 0 | low | first |
| 2 — Dual-method cross-validate | ~80 | +1 ms / window | 0 | medium | second |
| 3 — Per-window confidence | ~20 | trivial | 0 | low | third |
| 4 — Adaptive Kalman r | ~5 | trivial | 0 | low | after seeing Stage 3 data |

Total: ~135 lines, ~1 ms / window extra compute, zero new RAM.

**Why not all at once:** each stage feeds the next (SQI feeds confidence;
dual-method feeds confidence; confidence feeds Kalman). Shipping in
order lets us validate each layer against real traces before adding
the next.

---

---

## Stage 5+ — non-HR features parked here for future work

These aren't HR-pipeline optimizations but live on the same IMU
acquisition path, share the same MCU budget, and need to be tracked
somewhere.  Filed here so they don't get lost; promote to their own
doc if/when they grow.

### A. Step counting

**What:** count user steps over time, publish via BLE.

**Hardware:** LSM6DS3TR-C has a **built-in pedometer** that runs on
the chip with effectively zero MCU cost.  Steps accumulate in
internal registers; we just read them periodically.

**Effort:** 3-5 days.  Enable the pedometer register, set up a
periodic read, publish to BLE (could reuse battery service pattern
or add a custom characteristic).

**Accuracy:** chip-pedometer is typically ±2-5 % over a day for
normal walking.  Less accurate during running or unusual gaits;
adequate for casual fitness use.

**Notes:** would NOT conflict with our existing significant-motion
INT1 routing -- pedometer counter accumulates independently.  No
algorithmic decisions to make here, it's pure integration work.

### B. Activity classification

**What:** classify user state as sitting / standing / walking /
running / cycling / other.  Publish on transitions via BLE so the
app can update context (workout type detection, sedentary alerts,
sleep onset signal).

**Approach:** IMU-side classifier on accel+gyro features.

- **Option B1:** simple threshold-based classifier on accel variance
  + frequency content (same SMV we already compute).  Distinguishes
  stationary / walking / running cleanly.  Cycling is harder
  (lower-amplitude periodic motion).  **Effort: 1-2 weeks.**

- **Option B2:** Edge Impulse trained classifier with broader
  activity set (sitting vs standing, biking, swimming-prep, etc.).
  **Effort: 2-3 weeks** including data collection.

**Hardware:** runs on Cortex-M4 in either option.  No chip-embedded
activity classifier in LSM6DS3TR-C (that's newer LSM6DSOX with the
ML core).  We classify in MCU.

**Accuracy expectation:** Option B1 ~85-90 % on the common 3-4
classes.  Option B2 ~90-95 % across a broader class set per
published wrist-IMU classifier work.

**Dependency:** would feed the gesture mode machine (Use Case 1/2/3
from the gesture roadmap) -- knowing the user is walking vs sitting
helps decide when gesture detection should be active vs muted.
Build activity classification BEFORE shipping gesture v2 for this
reason.

### C. Posture detection (sub-feature of activity)

**What:** standing / sitting / lying-down.  Useful for sleep
detection and sedentary alerts.

**Approach:** gravity vector orientation from accel + recent motion
history.

**Effort:** 3-5 days.

**Accuracy:** ~90 % typical for the three classes.

### D. Future IMU features (placeholder list)

Tracked but not yet investigated:
- Fall detection (LSM6 has free-fall interrupt; would need
  follow-up confirmation logic)
- Sleep onset / wake detection (HR + IMU stillness combined)
- Posture-change alerts (long sit detection)
- Hand-wash detection (motion pattern + duration heuristic)

---

## Hardware-platform optimization candidates — MAX30101 / MAX30105 switch (UN-TRIAGED)

**Status:** candidate brain-dump captured 2026-06-15, **gated on the planned PPG
sensor swap** (MAX30102 → MAX30101 or MAX30105). NOT yet verified, NOT triaged,
NOT scheduled. This list is here so we can brainstorm → research-verify → decide
which are apt → spec → implement. Apply the same discipline as the rest of this
doc: **re-verify every claim against the actual datasheet / source before
building** (the bullet summaries below are unconfirmed engineering folklore until
checked against the MAX3010x datasheet + cited literature).

**Why this section exists / why the swap matters:** the single biggest reason to
move off the MAX30102 is the **green LED (≈530 nm)**. The MAX30102 is Red + IR
only — it has *no* green emitter, which is the wavelength commercial bands use for
wrist HR because of its superior SNR on superficial capillary beds. The MAX30101
(Red + IR + **Green**) and MAX30105 (adds particle/proximity sensing) both add it.
So several items below (green-drive, multi-light-path) are simply *unavailable* on
our current part and become possible only post-swap. This also reactivates the
"Multi-light-path fusion" line currently in **What we deliberately won't chase**
(excluded *because* the MAX30102 lacks the third path) — revisit that exclusion
once the part changes.

> Triage note: each candidate must be tagged on review as one of —
> **[apt-now]** (do post-swap), **[overlaps-existing]** (already partly built —
> reconcile, don't duplicate), **[conflicts]** (contradicts a deliberate exclusion
> — needs explicit justification to pursue), or **[drop]**. Pre-tags are my first
> read, not decisions.

### Scenario A — Steady-state HR (at rest)
- **Maximize the Green LED (530 nm) drive current** for the best SNR on
  superficial capillary beds. *[apt-now — this is the headline reason to swap.]*
- **Physical light baffle / opaque optical barrier** between LEDs and photodiode
  under the cover glass to kill internal optical crosstalk. *[apt-now — but it's a
  mechanical/housing change, ties into the productionization-housing thread, not a
  firmware task. Cross-ref `hardware_wear_position.md` + the open-PCB/duct-tape
  mount caveat.]*

### Scenario B — Minor motion (typing, gesturing)
- **Adaptive band-pass filter ~0.5–3 Hz** to isolate the pulse band from small
  muscular twitches. *[overlaps-existing — we already run a 4th-order Butterworth
  band-pass; verify whether "adaptive" adds anything over our fixed band, or is
  just a re-description of what we have.]*
- **Offload to a dedicated biometric hub (MAX32664)** for on-chip AGC + continuous
  smoothing. *[conflicts/large — this is adding a *second IC* and moving the whole
  pipeline off our M4, which throws away the ~135-LOC software roadmap above. Big
  architecture decision; treat as its own brainstorm, not a tweak. The MAX32664
  also pairs specifically with MAX3010x parts.]*

### Scenario C — Major motion (running, walking)
- **3-axis accel reference for adaptive noise cancellation (ANC)** — subtract
  motion spikes from the optical signal. *[overlaps-existing — we already do
  3-axis chained NLMS motion-artifact removal (see gap table). Reconcile: is the
  proposal a different/better ANC, or the same thing?]*
- **Increase MAX30101 sample rate** for higher-resolution waveforms → better
  separation of true cardiac peaks from footstep impacts. *[apt-now — verify the
  power/I2C-bandwidth cost vs. our current ODR; interacts with Stage 2's FFT
  window sizing.]*

### Scenario D — Hand-down / arm-lowered (venous blood pooling)
- **Detect downward arm angle via accelerometer** → lower the confidence score,
  lean on recent averages. *[overlaps-existing — we already have the gravity
  vector + orientation filter (cursor work) AND a confidence-score plan (Stage 3).
  This is a *new input* to Stage 3's confidence formula (an "arm-down factor"),
  not new machinery. Cheap, apt — fold into Stage 3.]*
- **Dynamically boost LED brightness + photodiode sensitivity** (via the AFE) to
  penetrate pooled venous blood when arm is down. *[apt-now — this is AGC applied
  to the hand-down case; verify against the MAX3010x AFE register set.]*

### Additional / cross-scenario
- **Built-in proximity detection to shut down main LEDs on wrist-off** (big power
  win). *[overlaps-existing — we already have a wear-state (`WEAR_WORN`); the
  MAX3010x proximity mode could *drive* it in hardware instead of inferring it.
  Apt — power + correctness win. MAX30105 has the strongest proximity support.]*
- **Adaptive Gain Control (AGC)** generally. *[apt-now — AFE-level; underpins
  several items above (C-rate, D-brightness). Verify the MAX3010x AGC register
  behavior.]*
- **Generative waveform reconstruction — GANs / Edge Impulse, trained on
  PPG-DaLiA.** *[CONFLICTS — directly contradicts the existing "What we
  deliberately won't chase → Deep learning models (foundation models for PPG):
  out of scope on M4; need MB of memory; need labelled training data." If we want
  to revisit, the doc's own rule says the justification must be written down. Note:
  PPG-DaLiA is a real, well-known wrist-PPG+accel dataset, so the *data* exists;
  the blocker is M4 memory/compute, not data. Edge Impulse can target M4-class
  MCUs with small models — so the honest open question is whether a *tiny*
  on-device model (not a GAN) is feasible within our RAM, which is a different and
  much narrower claim than "run a GAN." Research before pursuing.]*

> Open cross-cutting questions for the brainstorm:
> 1. MAX30101 vs MAX30105 — do we need the 30105's particle/proximity extras, or
>    is the 30101 (Green+Red+IR) enough? Proximity-driven wear-detect leans 30105.
> 2. MAX32664 hub (offload) vs. keep-everything-on-M4 (the software roadmap above)
>    — these are mutually-exclusive architectures. Decide early; it reframes the
>    whole HR pipeline.
> 3. How much of items B/C/D is genuinely NEW vs. a re-description of band-pass +
>    NLMS + confidence-score we already have planned/built? Reconcile before
>    speccing so we don't rebuild existing work.

> TODO before implementing any of the above: add datasheet + literature citations
> to the Sources block (MAX30101/30105 datasheet, MAX32664 user guide, PPG-DaLiA
> dataset paper, Edge-Impulse-on-Cortex-M feasibility) — none are cited yet
> because nothing here is verified.

---

## Sources

Listed by which stage they primarily support.

**General / multi-method industry pattern:**
- [MUSIC-Based Algorithm for On-Demand Heart Rate Estimation Using PPG Signals on Wrist (Analog Devices)](https://www.analog.com/en/resources/analog-dialogue/articles/music-based-algorithm-for-on-demand-heart-rate-estimation.html)
- [Validation and Performance of a Wearable Heart-Rate Monitoring Algorithm (Analog Devices)](https://www.analog.com/en/resources/technical-articles/validation-and-performance-of-a-wearable-heartrate-monitoring-algorithm.html)
- [Using Apple Watch to measure heart rate, calorimetry, and activity (Apple Health Whitepaper)](https://www.apple.com/health/pdf/Heart_Rate_Calorimetry_Activity_on_Apple_Watch_November_2024.pdf)
- [Optical tracking of heart rate using PLL optimization (US patent 9,826,940 — Apple)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9826940)
- [On-demand heart rate estimation based on optical measurements (US patent 10,285,651)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10285651)
- [Heart rate detection method and electronic device (US patent 12,369,803)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12369803)
- [End-to-End PPG Processing Pipeline for Wearables (UCI Future Health)](https://futurehealth.uci.edu/wp-content/uploads/2023/12/End-to-End-PPG-Processing-Pipeline-for-Wearables-From-Quality-Assessment-and-Motion-Artifacts-Removal-to-HRHRV-Feature-Extraction.pdf)
- [Accuracy of Optical Heart Rate Measurements for 10 Commercial Wearables (PMC)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12912460/)
- [Investigating sources of inaccuracy in wearable optical heart rate sensors (npj Digital Medicine)](https://www.nature.com/articles/s41746-020-0226-6)
- [PPG-Based Heart Rate Accuracy in Diverse Populations (arXiv 2601.22377)](https://arxiv.org/pdf/2601.22377)
- [Adaptive Estimation Algorithm for PPG Heart Rate Based on Finite State Machine (MDPI)](https://www.mdpi.com/2076-3417/14/24/11631)
- [PPG heart rate extraction algorithm based on motion artifact intensity Classification framework (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S1746809424003458)
- [Frontiers — Research on heart rate estimation algorithm based on dynamic PPG (2026)](https://www.frontiersin.org/journals/signal-processing/articles/10.3389/frsip.2026.1724468/full)
- [Quality-Aware Signal Processing Mechanism of PPG Signal (PMC)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11207506/)

**Stage 1 — SQI:**
- [Optimal Signal Quality Index for Photoplethysmogram Signals (PMC 5597264)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5597264/)
- [Optimized Signal Quality Assessment for PPG Signals using Feature Selection (PMC 9478959)](https://pmc.ncbi.nlm.nih.gov/articles/PMC9478959/)
- [System and method for PPG signal quality assessment (US patent 10,575,786)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10575786)
- [Photoplethysmography Signal Processing and Synthesis (Peter Charlton chapter)](https://peterhcharlton.github.io/publication/ppg_sig_proc_chapter/PPG_sig_proc_Chapter_20210612.pdf)

**Stage 2 — Dual-method + harmonic disambiguation:**
- [Harmonic Sum-based Method for Heart Rate Estimation using PPG Signals Affected with Motion Artifacts (arXiv 1610.05112)](https://arxiv.org/pdf/1610.05112)
- [A Real-Time PPG Peak Detection Method for Accurate Determination of Heart Rate (PMC 8869811)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8869811/)
- [Estimating Respiratory and Heart Rates from the Correntropy Spectral Density of the PPG (PMC 3899260)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3899260/)

**Stage 4 — Adaptive Kalman / RAKF:**
- [Advanced signal-processing framework for rPPG: Adaptive Kalman + DWT (PMC 12818640)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12818640/)
- [Bounded Kalman filter method for motion-robust, non-contact heart rate estimation (PMC 5854085)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5854085/)
- [An adaptive Kalman filter approach for cardiorespiratory signal extraction and fusion (BMC)](https://bmcmedinformdecismak.biomedcentral.com/articles/10.1186/1472-6947-14-37)

**TROIKA / JOSS (excluded but cited as the published ceiling):**
- [TROIKA framework (arXiv 1409.5181)](https://arxiv.org/pdf/1409.5181)
- [Heart rate monitoring from wrist-type PPG based on SSA with motion decision (PubMed 28269055)](https://pubmed.ncbi.nlm.nih.gov/28269055/)
- [PPG-based Heart Rate Monitoring via Joint Sparse Spectrum Reconstruction (arXiv 1503.00688)](https://arxiv.org/pdf/1503.00688)

**Hardware / positioning context (cross-reference with `hardware_wear_position.md`):**
- [Impact of Anatomical Placement on the Accuracy of Wearable HR Monitors (PMC 12788198)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12788198/)
- [A Comparison of Reflective Photoplethysmography (PMC 6514840)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6514840/)
