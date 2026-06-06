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
