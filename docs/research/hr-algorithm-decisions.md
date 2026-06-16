# HR algorithm decisions and feature roadmap

**Purpose:** definitive reference for the algorithmic choices that
shaped main, the alternatives we rejected and why, what those choices
cost us in feature reach, and the architectural path to add the
trade-off features later if/when they become requirements.  Also
contains: the staged software optimization roadmap for closing the
commercial-band gap (§10), and applicability notes from Google PHRM /
rPPG research (Appendix §11).

**Status:** captures decisions made through v0.4 production-track
(merge `6e65d66` on main).  Update this doc when major algorithm or
hardware decisions change.

> TODO before treating this as final:
> - Reverify Apple's algorithmic patents annually -- the corpus we
>   cited is current as of 2026-06; published patent claims can shift.
> - Confirm SpO2 hardware path (MAX30102 red LED + SpO2 mode in driver)
>   is actually viable on our specific board variant before committing
>   to SpO2 as a future feature.

---

## TL;DR

We picked **spectral analysis (FFT + autocorrelation + harmonic-sum
check)** as the primary HR algorithm.  We rejected **time-domain beat
detection** because of hardware-specific constraints that make it
underperform spectral during motion on our setup.  This costs us a
defined set of beat-level features (HRV, AFib, premature beats, beat-
to-beat transients, pulse arrival time) but preserves the standard
fitness-band feature set.  HRV and similar can be added later via a
**stationary-only parallel beat-detection pass** without disturbing
the spectral pipeline -- a documented hybrid architecture some
commercial bands use internally.

---

## 1. Algorithm: spectral analysis (chosen)

### What we run today

Per 5.12 s windowed buffer, 50 % overlap (publish every 2.56 s):

1. **Endpoint detrending** — subtracts a line through first/last
   samples to kill DC drift residual (eliminates IIR ringing on
   step input)
2. **4th-order Butterworth band-pass IIR**, 0.6-3.3 Hz at 100 Hz Fs
3. **Two parallel cardiac estimators on STATIONARY windows:**
   - `runAutocorrelation()`: arm_correlate_f32 on the band-passed
     buffer, peak-find via "first prominent local max above 50% of
     in-band global max" rule (`findSecondPeak`)
   - `runStationaryFFT()`: arm_rfft_fast_f32 + arm_cmplx_mag_f32,
     peak-find in cardiac band [bin 3, bin 17] = [35, 199] BPM
4. **`reconcileStationary()`**: dual-method consensus
   - Within 10 BPM → average, confidence 1.0, kalman r = 0.5
   - Either silent → trust the other, confidence 0.5, kalman r = 2.0
   - 2:1 ratio → pick larger (sub-harmonic disambiguation),
     confidence 0.5
   - Otherwise → confidence 0.0, hold Kalman
5. **`applyHarmonicCheck()`**: harmonic-sum metric
   - Score(K) = FFT[K] + FFT[2K] + FFT[3K]
   - If Score(2K) > Score(K), candidate is the sub-harmonic; snap
     to 2K's BPM
6. **1D Kalman filter** smoothing (gain ~0.32 at r=0.5)
7. **In-range gate** ([40, 200] BPM) before Kalman update

### Why we picked it

- **CMSIS-DSP primitives are first-class on M4.**
  `arm_rfft_fast_f32` and `arm_correlate_f32` are tuned for our chip
  and run in ~1 ms each.  Time-domain peak detection would be hand-
  rolled.
- **No labelled training data.**  Beat-detection algorithms in
  commercial bands lean heavily on ML calibration (Apple cites
  >100,000 training workout sessions).  We don't have that data and
  building it would dominate the project.
- **Motion robustness is the dominant accuracy driver in wrist PPG.**
  Spectral methods average over the window; single-beat corruption
  is diluted instead of cascading.  Beat detection makes every
  corrupt beat a wrong measurement.
- **Sub-harmonic protection requires spectral structure.**  Our
  `applyHarmonicCheck` is fundamentally a frequency-domain idea --
  no time-domain equivalent for catching "is this peak a real
  cardiac fundamental or the 2T of one?".
- **Motion-artifact cancellation composes cleanly with spectral.**
  NLMS, Wiener, spectral exclusion all assume continuous signals,
  not discrete events.  The hybrid mask work on
  `feature/motion-path-experiments` only makes sense in frequency
  domain.
- **Mathematical transparency.**  Bin math and lag math are
  inspectable from logs.  Beat-detection errors are harder to
  diagnose because they're cascade-dependent.

### Decisions inherited from this choice

- **Window length is bound to FFT frequency resolution.**  N=512 at
  100 Hz Fs → resolution 0.195 Hz = 11.72 BPM/bin.  N=256 would be
  too coarse (23.4 BPM/bin loses cardiac).  N=1024 doubles latency.
- **Update cadence = 50 % overlap step = 2.56 s.**  Could go to
  75 % overlap for 1.28 s updates at higher compute cost.  No reason
  to chase that today.
- **Precision degrades at high HR.**  At 180 BPM (bin 15.4) we round
  to bin 15 (175.78 BPM) or bin 16 (187.5 BPM).  Autocorr's
  lag-step granularity refines this somewhat at low HR (lag 83 ≈
  72 BPM is ±0.5 BPM) but not at high (lag 33 ≈ 182 BPM is ±5-6).

---

## 2. Algorithm: time-domain beat detection (rejected)

### What it is

Find the systolic peaks in the raw PPG waveform.  Measure the
intervals between consecutive peaks.  Each interval inverted gives an
instantaneous HR.  Output: stream of (timestamp, HR, confidence) per
beat.

Standard implementations:
- Pan-Tompkins-style (originally for ECG, adapted for PPG)
- Wavelet-decomposition-based
- CNN-based (Apple, Galaxy)

### Why we rejected it for our hardware

The fundamental issue is **motion robustness**, and the hardware
stacks against us:

| Constraint | Impact on beat detection during motion |
|---|---|
| Single IR LED (no green) | IR is meaningfully more motion-sensitive than green wavelengths.  Apple uses green for cardiac, IR mainly for SpO2 and background readings.  Without green, motion artifacts have larger amplitude than cardiac peaks → false peaks dominate. |
| No multi-wavelength fusion | Commercial bands use green + IR or green + red difference to suppress motion.  Not available to us with our sensor wired in IR-only mode. |
| No labelled beat-classifier ML model | Apple's beat detection is partly learned (per US patents 10,123,746 / 10,736,575 / 11,744,520, ~100k workout sessions of training).  We'd be hand-rolling, no calibration set. |
| Single PD channel | Some peak algorithms use spatial redundancy across multiple photodiode sites.  Galaxy Watch BioActive has 4 channels; MAX30102 has 1. |
| Cortex-M4 compute | Real-time peak finding with motion-aware rejection + dicrotic-notch handling is feasible but not cheap. |

### Performance vs spectral, factually

On our hardware:
- **At rest:** beat detection would work fine.  ±1-2 BPM, comparable
  to spectral.
- **At MICRO motion (typing, gestures):** degraded.  False peaks
  from desk motion cause occasional wrong readings.  Maybe ±5-10
  BPM after smoothing.
- **At HEAVY motion (running):** catastrophic.  Motion peaks
  dominate cardiac.  The algorithm would track stride cadence
  instead of HR.  Same failure region as our spectral path, but
  **less recoverable** -- the harmonic-sum check we ship has no
  time-domain equivalent.

So strictly: beat detection runs on our hardware, but it strictly
underperforms spectral during motion.  No winning move there for our
primary HR-display use case.

### What we'd need before reconsidering

- Multi-wavelength sensor (green + IR), OR
- Labelled beat-classifier training data on our exact hardware, OR
- Move sensor off-wrist to a less motion-corrupted site (chest, arm,
  ear), OR
- Accept that beat-mode is rest-only and is for HRV/AFib specifically,
  not live HR display

The last option is **exactly the hybrid path** in §5.

---

## 3. Other algorithms we surveyed but did not adopt

### TROIKA / JOSS (Singular Spectrum Analysis + sparse spectral reconstruction)

State-of-the-art academic algorithm for wrist PPG during heavy motion.
~2.08-2.34 BPM MAE on running data.

**Why deferred:**
- RAM-heavy matrix operations.  Borderline on M4.
- High implementation risk -- we'd be at bleeding-edge embedded DSP.
- Marginal gain over what we could achieve with simpler chained
  NLMS + spectral exclusion (on the experiments branch) IF we ever
  ship a HEAVY path.

### MUSIC subspace method

High-resolution spectral estimation via eigendecomposition of the
covariance matrix.

**Why rejected:**
- High compute (eigendecomposition).
- Marginal accuracy gain over FFT at our window sizes per published
  comparisons.

### Deep learning / foundation models (Pulse-PPG etc.)

End-to-end neural network on raw PPG.

**Why rejected:**
- Out of scope on M4: model sizes are MB+.
- No labelled training data.
- Black-box failures hard to diagnose from logs.

### Multi-wavelength fusion (green + IR difference)

Subtract motion-correlated content between two wavelengths to isolate
the cardiac signal.

**Why blocked:**
- Hardware constraint.  MAX30102 driver in IR mode is single-channel
  for our path.  Adding green would need a second sensor or a driver
  rewrite to drive both LEDs and sample synchronously.
- Worth reconsidering if v2 hardware uses MAX30101 with green.

### Adaptive template matching (correlate PPG against a stored cardiac waveform template)

**Why rejected:**
- Per-user template calibration required (template differs by skin,
  age, vasculature).
- Computationally expensive per the SQI literature.

---

## 4. Features lost vs preserved with the spectral choice

### Lost (require beat-level resolution, blocked without beat detection)

| Feature | Why blocked |
|---|---|
| **Time-domain HRV** (RMSSD, SDNN, pNN50) | Requires precise inter-beat intervals.  We get one BPM per 2.56 s window. |
| **AFib detection** | Detected from irregular RR-interval patterns.  No RR intervals → no detection. |
| **Premature beat detection** (PACs, PVCs) | Requires individual beat timings to identify out-of-rhythm beats. |
| **HR Turbulence** | Cardiac autonomic test based on RR-interval response after a premature ventricular contraction.  Pure beat-level analysis. |
| **Beat-to-beat instantaneous HR** | A 4-beat tachycardia burst inside one of our windows is invisible -- it averages with surrounding beats. |
| **Pulse Arrival Time / Pulse Transit Time** | BP proxy derived from ECG-to-PPG timing.  We lack ECG, and we lack beat-precise PPG timing. |
| **Sub-second stress events** (galvanic-like brief autonomic events) | Need beat-by-beat HR variability over very short windows. |
| **Anaerobic threshold from mid-interval HR drift** | Requires beat-level resolution during the workout to detect the inflection. |

### Preserved (achievable with spectral + existing infrastructure)

| Feature | How we'd implement it |
|---|---|
| **Live HR display** | Already shipped. |
| **Resting HR + trends** | Aggregate Kalman state during WORN+STATIONARY+settled windows over a day.  Snapshot-aggregation backlog item from §11 below. |
| **Workout HR (averaged)** | Track Kalman through workout.  Already most of the way -- POWER state machine already classifies workout. |
| **Cardio fitness / VO2max estimate** | HR + activity (IMU) + duration.  Apple's algorithm is HR + GPS + activity; we'd use IMU.  No beat-level data. |
| **Recovery HR after exercise (HRR)** | Track Kalman drop in the 60 s after exercise stop.  Window cadence is fine -- HRR is measured over tens of seconds. |
| **Respiratory rate** | Extract from low-frequency PPG (0.15-0.4 Hz band) via FFT.  Same FFT we already run, just a different peak-find range.  ~30 lines added. |
| **SpO2** | MAX30102 has a red LED unused today.  SpO2 = ratio of red AC/DC to IR AC/DC.  Hardware feature unlock, separate from HR algorithm. |
| **Step counting / activity classification** | Pure IMU.  Independent of PPG. |
| **Sleep onset / wake detection** | HR trend (low + flat) + IMU stillness.  Spectral cadence is fine -- sleep transitions are minutes-long. |
| **Resting HR alerts** | Threshold on averaged Kalman.  No beat data needed. |
| **Workout calories** | HR-based estimate.  Standard formulas use averaged HR. |
| **Energy expenditure / TDEE** | HR + IMU. |
| **Frequency-domain HRV** (LF/HF ratio) | Compute LF (0.04-0.15 Hz) and HF (0.15-0.4 Hz) bands of the HR tachogram over 5+ minute windows.  Less precise than time-domain HRV but produces a stress score.  Requires storing the Kalman series, not raw beats. |

---

## 5. Hybrid path -- if HRV/AFib become must-haves

If a future product requirement adds HRV or arrhythmia detection, the
clean architecture is **additive, not replacement**.

### Architecture

```
                    PPG buffer (5.12 s windowed)
                            |
              +-------------+-------------+
              |                           |
         SPECTRAL PASS              BEAT-DETECTION PASS
       (current pipeline)         (new; STATIONARY only)
              |                           |
         Live HR (BLE HRS)           RR intervals
              |                           |
                                    HRV metrics
                                    (RMSSD, SDNN, pNN50)
                                          |
                                    AFib classifier
                                    (Poincaré plot or
                                     learned model)
                                          |
                                   Health data over BLE
                                   (separate characteristic
                                    or custom service)
```

### Why this works

- **Beat detection only runs on STATIONARY windows.**  Motion failure
  modes are bypassed at the gate.
- **HRV is mostly measured at rest anyway** (Apple's HRV is mostly
  captured during sleep / quiet sitting -- not workouts).  Stationary-
  only scope is the right scope.
- **Spectral keeps delivering live HR.**  Display latency is unchanged.
  Users see continuous HR.
- **No regression risk to existing path.**  Beat detection is a
  parallel module, gated and feeding a different output channel.

### Implementation cost estimate

- **Beat-detection module:** ~150-250 lines.  Bandpass-filtered PPG
  in (we already have it), array of timestamps + intervals out.
  Implementation: Pan-Tompkins-adapted or wavelet-based.  CMSIS-DSP
  helps with bandpass and threshold ops.
- **RR-interval buffer:** ~500 floats (5 minutes at 90 BPM ≈ 450
  beats).  ~2 KB RAM.
- **HRV computation:** ~50 lines for RMSSD/SDNN/pNN50.  Runs on
  demand (per query) or per snapshot.
- **AFib classifier:** Poincaré-plot-based heuristic = ~100 lines,
  reasonable specificity.  ML classifier = larger, would need
  training data.
- **BLE exposure:** custom GATT characteristic or extend existing
  HRS with vendor-specific fields.

Compute: ~1-2 ms per stationary window for beat detection, less for
HRV (only runs when buffer fills).

### When NOT to do this

- If the product positioning never needs HRV / arrhythmia.  Sleep /
  stress / VO2max can be done without it.
- If we're still resource-constrained on the M4 by then.

---

## 6. Hardware path -- features that need hardware changes

Some features cannot be unlocked by software alone on current
hardware.  Captured here so they don't get re-debated:

| Feature | Hardware requirement | Notes |
|---|---|---|
| Multi-wavelength motion artifact removal | Green LED active in addition to IR | MAX30101 supports green; MAX30102 has IR + red.  Would need to switch sensor or rewire driver to drive green channel. |
| ECG | Electrodes + analog frontend | Out of scope on current PCB. |
| SpO2 | Red LED + algorithm | Already present on MAX30102.  Software-only unlock. |
| Continuous body temp | Skin thermistor | Not currently wired. |
| GPS-derived pace / cadence | GNSS receiver | Not on this board.  Phone-derived possible via BLE. |
| Multi-PD spatial fusion | Multiple photodiode sites | Single PD on MAX30102.  Would need different sensor (Apple PPG / Galaxy Watch sensors are 4-channel). |

---

## 7. Decision matrix summary

| Capability | Spectral (now) | Beat detection alone | Hybrid (later) |
|---|---|---|---|
| Live HR at rest | ✓ ±1-2 BPM | ✓ ±1-2 BPM | ✓ ±1-2 BPM |
| Live HR at MICRO motion | ✓ ±2-4 BPM (research target) | degraded ±5-10 | ✓ same as spectral |
| Live HR at HEAVY motion | ✗ holds Kalman (today) | ✗ catastrophic | ✗ same as spectral |
| Time-domain HRV | ✗ | ✓ | ✓ (rest only) |
| AFib detection | ✗ | ✓ (rest only, ML-dependent) | ✓ (rest only) |
| Premature beat detection | ✗ | ✓ at rest | ✓ at rest |
| Respiratory rate | ✓ (FFT low-freq band) | ✓ (interval analysis) | ✓ either |
| SpO2 | ✓ (hardware unlock) | ✓ (hardware unlock) | ✓ |
| Sleep stages | ✓ (HR + IMU heuristic) | ✓ (HRV-based, more precise) | ✓ better |
| Stress score | ✓ (freq-domain HRV, less precise) | ✓ (time-domain HRV) | ✓ best |
| Cardio fitness / VO2max | ✓ | ✓ | ✓ |
| Workout calories | ✓ | ✓ | ✓ |
| Step counting | ✓ (IMU) | ✓ (IMU) | ✓ |
| Compute on M4 | ✓ fits today | borderline with ML | ✓ fits |
| Implementation risk | low | high | medium |

---

## 8. References

### Algorithm choice and pipeline

- [Photoplethysmographic Time-Domain Heart Rate Measurement Algorithm for Resource-Constrained Wearable Devices (PMC 7146569)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7146569/)
- [End-to-End PPG Processing Pipeline for Wearables (UCI Future Health)](https://futurehealth.uci.edu/wp-content/uploads/2023/12/End-to-End-PPG-Processing-Pipeline-for-Wearables-From-Quality-Assessment-and-Motion-Artifacts-Removal-to-HRHRV-Feature-Extraction.pdf)
- [The 2023 wearable photoplethysmography roadmap (Charlton et al.)](https://khan.usc.edu/assets/files/charlton20232023.pdf)
- [Frontiers — Research on heart rate estimation algorithm based on dynamic PPG (2026)](https://www.frontiersin.org/journals/signal-processing/articles/10.3389/frsip.2026.1724468/full)

### Apple Watch algorithm details

- [Using Apple Watch to measure heart rate, calorimetry, and activity (Apple Health Whitepaper, Nov 2024)](https://www.apple.com/health/pdf/Heart_Rate_Calorimetry_Activity_on_Apple_Watch_November_2024.pdf)
- [Optical tracking of heart rate using PLL optimization (US patent 9,826,940 — Apple)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9826940)
- [On-demand heart rate estimation based on optical measurements (US patent 10,285,651)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10285651)
- [Accuracy of heart rate estimation from PPG signals (US patents 10,736,575 / 11,744,520 / 10,123,746)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10736575)
- [Heart rate estimation apparatus with state sequence optimization (US patent 10,321,831 / 11,129,538 / 12,042,257)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10321831)
- [Monitor your heart rate with Apple Watch (Apple Support)](https://support.apple.com/en-us/120277)

### Spectral methods (FFT, harmonic-sum)

- [Harmonic Sum-based Method for Heart Rate Estimation using PPG Signals Affected with Motion Artifacts (arXiv 1610.05112)](https://arxiv.org/pdf/1610.05112)
- [TROIKA: A General Framework for Heart Rate Monitoring Using Wrist-Type PPG Signals (arXiv 1409.5181)](https://arxiv.org/pdf/1409.5181)
- [PPG-based Heart Rate Monitoring via Joint Sparse Spectrum Reconstruction (arXiv 1503.00688)](https://arxiv.org/pdf/1503.00688)
- [Heart rate monitoring from wrist-type PPG based on SSA with motion decision (PubMed 28269055)](https://pubmed.ncbi.nlm.nih.gov/28269055/)

### Beat detection algorithms

- [A Real-Time PPG Peak Detection Method for Accurate Determination of Heart Rate during Sinus Rhythm and Cardiac Arrhythmia (PMC 8869811)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8869811/)
- [Robust PPG Peak Detection Using Dilated Convolutional Neural Networks (PMC 9414657)](https://pmc.ncbi.nlm.nih.gov/articles/PMC9414657/)
- [TAU: Modeling Temporal Consistency Through Temporal Attentive U-Net for PPG Peak Detection (arXiv 2503.10733)](https://arxiv.org/pdf/2503.10733)
- [Robust Heart Rate Detection via Multi-Site Photoplethysmography (arXiv 2412.17538)](https://arxiv.org/pdf/2412.17538)

### Signal quality and motion artifact

- [Quality-Aware Signal Processing Mechanism of PPG Signal for Long-Term Heart Rate Monitoring (MDPI Sensors)](https://www.mdpi.com/1424-8220/24/12/3901)
- [Optimal Signal Quality Index for Photoplethysmogram Signals (PMC 5597264)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5597264/)
- [Motion Artifact Reduction for Wrist-Worn PPG Sensors Based on Different Wavelengths (MDPI Sensors)](https://www.mdpi.com/1424-8220/19/3/673)
- [PPG signal motion artifacts correction algorithm based on feature estimation (Optik 2019)](https://www.sciencedirect.com/science/article/abs/pii/S0030402618313883)
- [PPG heart rate extraction algorithm based on motion artifact intensity Classification framework (Biomed SP&C 2024)](https://www.sciencedirect.com/science/article/abs/pii/S1746809424003458)

### Activity-specific accuracy validation

- [Accuracy of Optical Heart Rate Measurements for 10 Commercial Wearables in Different Climate Conditions and Activities (PMC 12912460)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12912460/)
- [Wrist-Worn and Arm-Worn Wearables for Monitoring Heart Rate During Sedentary and Light-to-Vigorous Physical Activities (PMC 11951816 / JMIR Cardio 2025)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11951816/)
- [Investigating sources of inaccuracy in wearable optical heart rate sensors (npj Digital Medicine)](https://www.nature.com/articles/s41746-020-0226-6)
- [Activity, but not skin tone, can impact wearables' PPG heart rate accuracy (Duke study via MobiHealthNews)](https://www.mobihealthnews.com/news/duke-study-activity-not-skin-tone-can-impact-wearables-ppg-heart-rate-accuracy)
- [Heart rate measurements of wearable monitors vary by activity, not skin color (ScienceDaily 2020)](https://www.sciencedaily.com/releases/2020/02/200212121950.htm)
- [PPG-Based Heart Rate Accuracy in Diverse Populations: Investigating Inequities Across Body Composition and Skin Tones (arXiv 2601.22377)](https://arxiv.org/pdf/2601.22377)

### Sensor positioning

- [Impact of Anatomical Placement on the Accuracy of Wearable Heart Rate Monitors (PMC 12788198)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12788198/)
- [A Comparison of Reflective Photoplethysmography for Detection of HR, SpO2, and Respiration Rate at Various Anatomical Locations (PMC 6514840)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6514840/)
- [Optimization and pre-use suitability selection for wrist PPG-based HR monitoring in patients with cardiac disease (EHJ Digital Health)](https://academic.oup.com/ehjdh/article/6/5/1024/8211204)

### HRV and adaptive Kalman

- [Advanced signal-processing framework for rPPG: Adaptive Kalman + DWT (PMC 12818640)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12818640/)
- [Bounded Kalman filter method for motion-robust, non-contact HR estimation (PMC 5854085)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5854085/)
- [Adaptive Estimation Algorithm for PPG HR Based on Finite State Machine (MDPI Applied Sciences 2024)](https://www.mdpi.com/2076-3417/14/24/11631)

### Cross-reference

- See §10 (Software optimization roadmap) in this document for the
  staged plan that brought us from baseline to v0.4 production-track,
  including the multi-feature SQI and adaptive Kalman items still parked.
- See §11 (Appendix: rPPG / camera-HR) in this document for findings
  from Google's rPPG research (camera-based, but pipeline patterns
  translate).

---

## 9. Things this doc does NOT cover (yet)

- Power consumption tradeoffs of each algorithm (when/if we measure
  on a coulomb counter).
- Specific HRV thresholds or stress scoring formulas -- defer to when
  the hybrid path is on the table.
- Per-user calibration strategies (no consensus in literature).
- BLE characteristic design for HRV / AFib / extended health data --
  app-side coordination required first.

---

## 10. Software optimization roadmap — closing the commercial-band gap

> **Provenance:** folded in from `docs/research/software-optimization-roadmap.md`
> (research session 2026-06-06; original doc deleted 2026-06-17).

**Source of this section:** research session 2026-06-06, prompted by sub-harmonic
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

### Why this exists (context)

In our v0.3 sprint-accuracy work we hit a sub-harmonic failure mode in
the autocorrelation path — raws of 42-44 BPM when the true cardiac rate
was ~85. The instinct was to add a sub-harmonic guard. The user pushed
back: *"is this what fitness bands actually do, or are we band-aiding?"*

Research showed: multi-method decision fusion with confidence-weighted
Kalman is documented industry standard in higher-end commercial bands
(Apple patent verbatim, Analog Devices reference designs, Samsung
Galaxy Watch BioActive descriptions). Single-method autocorr-only is
the **low-end commercial pattern.**

We can match the high-end pattern on our hardware cheaply. This section
captures the plan so we don't lose it.

---

### Gap analysis

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

### What we deliberately won't chase

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

### Stage 1 — Multi-feature SQI

**Replace the binary variance gate with a continuous quality score.**

#### Why

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

#### What

Combine three cheap features:
1. **Skewness** — primary discriminator, F1 ≈ 86 %
2. **Kurtosis** — secondary, complements skewness
3. **Perfusion ratio** — AC amplitude / DC level (already implicit in
   our `ppg_var / ppg_dc` but not used)

Output: continuous 0-100 SQI score per window. Map to:
- > 70 = high quality (full processing)
- 30-70 = degraded (conservative method selection)
- < 30 = unusable (hold Kalman)

#### Cost

~30 lines, three CMSIS-DSP statistics calls (`arm_var`, manual
kurtosis if no DSP function, perfusion = arithmetic), no new memory.

#### Code touch points

`WearableDSP::processHeartRate` — replace `sqi_passed` bool with
`int sqi_score`. Threshold checks become tiered.

> TODO: CMSIS-DSP doesn't ship `arm_kurtosis_f32` AFAIK — verify and
> if true, hand-roll a O(N) one-pass implementation.

---

### Stage 2 — Dual-method cross-validation at STATIONARY

**Run both autocorr and FFT on every stationary window; reconcile
with harmonic awareness.**

#### Why

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

#### What

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

#### Cost

~80 lines, one extra FFT per stationary window (~1 ms on M4 with
`arm_rfft_fast_f32`), no new memory (reuse `fft_output`, `fft_magnitudes`).

#### Code touch points

- New `WearableDSP::runStationaryFFT(float* ppg)` (no stride masking
  unlike the motion-path `runFFT`)
- `WearableDSP::processHeartRate` STATIONARY case rewritten to compute
  both and reconcile
- May want a helper `harmonicallyConsistent(float candidate, float* fft_mag)`

---

### Stage 3 — Per-window confidence score (0-100)

**Replace `delta_ok` / `in_range` booleans with a continuous confidence
number propagated to caller and (eventually) to BLE.**

#### Why

- App-side UX: returning `{bpm, confidence_pct}` lets the app decide
  whether to display the value, average across recent windows, or warn
  the user
- Foundation for Stage 4 (adaptive Kalman r)
- Foundation for the snapshot-aggregation backlog item (see §11,
  transferable idea #1): "confidence = fraction of snapshot windows
  that cleared every gate"
- Apple patent describes it verbatim; not exotic, just structured.

#### What

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

#### Cost

~20 lines.

---

### Stage 4 — Adaptive Kalman measurement noise (RAKF pattern)

**Scale Kalman's measurement-noise term `r` inversely with confidence.**

#### Why

Current Kalman has three fixed `r` values (0.5 / 2.0 / 5.0 for
stationary / micro / heavy). Apple's patent and the RAKF pattern
published in
[Advanced rPPG framework, PMC 12818640](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12818640/)
both vary `r` per-window based on measurement confidence.

The RAKF paper reports **MAE = 0.72 BPM on the PURE dataset** — that's
the published ceiling for confidence-adaptive Kalman + DWT on rPPG.
We can match the structural pattern; we won't match the absolute
number on a wrist contact PPG (different problem).

#### What

```cpp
kalman.r = R_BASE * (1.0f - confidence / 100.0f) + R_FLOOR;
```

With `R_BASE` ~ 5.0 and `R_FLOOR` ~ 0.3. High confidence → low r →
Kalman trusts measurement → fast convergence. Low confidence → high
r → Kalman trusts state → smoothing dominates.

#### Cost

~5 lines. Defer until we've seen real confidence-score distributions
from Stage 3 traces and can tune `R_BASE` / `R_FLOOR` against data.

---

### Implementation order

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

### Stage 5+ — non-HR features parked here for future work

These aren't HR-pipeline optimizations but live on the same IMU
acquisition path, share the same MCU budget, and need to be tracked
somewhere.  Filed here so they don't get lost; promote to their own
doc if/when they grow.

#### A. Step counting

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

#### B. Activity classification

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

#### C. Posture detection (sub-feature of activity)

**What:** standing / sitting / lying-down.  Useful for sleep
detection and sedentary alerts.

**Approach:** gravity vector orientation from accel + recent motion
history.

**Effort:** 3-5 days.

**Accuracy:** ~90 % typical for the three classes.

#### D. Future IMU features (placeholder list)

Tracked but not yet investigated:
- Fall detection (LSM6 has free-fall interrupt; would need
  follow-up confirmation logic)
- Sleep onset / wake detection (HR + IMU stillness combined)
- Posture-change alerts (long sit detection)
- Hand-wash detection (motion pattern + duration heuristic)

---

### Hardware-platform optimization candidates — MAX30101 / MAX30105 switch (UN-TRIAGED)

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

#### Scenario A — Steady-state HR (at rest)
- **Maximize the Green LED (530 nm) drive current** for the best SNR on
  superficial capillary beds. *[apt-now — this is the headline reason to swap.]*
- **Physical light baffle / opaque optical barrier** between LEDs and photodiode
  under the cover glass to kill internal optical crosstalk. *[apt-now — but it's a
  mechanical/housing change, ties into the productionization-housing thread, not a
  firmware task. Cross-ref `hardware_wear_position.md` + the open-PCB/duct-tape
  mount caveat.]*

#### Scenario B — Minor motion (typing, gesturing)
- **Adaptive band-pass filter ~0.5–3 Hz** to isolate the pulse band from small
  muscular twitches. *[overlaps-existing — we already run a 4th-order Butterworth
  band-pass; verify whether "adaptive" adds anything over our fixed band, or is
  just a re-description of what we have.]*
- **Offload to a dedicated biometric hub (MAX32664)** for on-chip AGC + continuous
  smoothing. *[conflicts/large — this is adding a *second IC* and moving the whole
  pipeline off our M4, which throws away the ~135-LOC software roadmap above. Big
  architecture decision; treat as its own brainstorm, not a tweak. The MAX32664
  also pairs specifically with MAX3010x parts.]*

#### Scenario C — Major motion (running, walking)
- **3-axis accel reference for adaptive noise cancellation (ANC)** — subtract
  motion spikes from the optical signal. *[overlaps-existing — we already do
  3-axis chained NLMS motion-artifact removal (see gap table). Reconcile: is the
  proposal a different/better ANC, or the same thing?]*
- **Increase MAX30101 sample rate** for higher-resolution waveforms → better
  separation of true cardiac peaks from footstep impacts. *[apt-now — verify the
  power/I2C-bandwidth cost vs. our current ODR; interacts with Stage 2's FFT
  window sizing.]*

#### Scenario D — Hand-down / arm-lowered (venous blood pooling)
- **Detect downward arm angle via accelerometer** → lower the confidence score,
  lean on recent averages. *[overlaps-existing — we already have the gravity
  vector + orientation filter (cursor work) AND a confidence-score plan (Stage 3).
  This is a *new input* to Stage 3's confidence formula (an "arm-down factor"),
  not new machinery. Cheap, apt — fold into Stage 3.]*
- **Dynamically boost LED brightness + photodiode sensitivity** (via the AFE) to
  penetrate pooled venous blood when arm is down. *[apt-now — this is AGC applied
  to the hand-down case; verify against the MAX3010x AFE register set.]*

#### Additional / cross-scenario
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

### Sources (§10)

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

---

## 11. Appendix: rPPG / camera-HR (Google PHRM) applicability notes

> **Provenance:** folded in from `docs/research/google-phrm-notes.md`
> (raw working notes captured 2026-06-05; original doc deleted 2026-06-17).

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

### Their setup vs ours (so the comparisons below stay honest)

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

### Transferable ideas

#### 1. Two-tier Kalman: per-window for live, per-snapshot for aggregation

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

#### 2. Confidence score, exposed alongside BPM

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

#### 3. Benchmark numbers we can hold ourselves to

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

### Not transferable

- **Temporal-shift CNN trained on ~350k clips from ~700 participants.**
  We have ~256 KB RAM and no labelled dataset. Classical DSP is the
  right call for our form factor; the paper does not change that.
- **8-second windows.** They can afford it (no battery constraint on a
  phone). We can't, and our autocorr does fine at 5.12 s with the
  AUTOCORR_TRANSIENT_SKIP of 80 samples. Not a signal we should chase.
- **Specific peak-detection algorithm.** Blog doesn't disclose. Nothing
  to copy at the DSP level.

---

### What gestureband already does better than them (at v1)

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

### Skin-tone note

Their Group 3 (darkest skin tones) had MAPE 7.84% vs Group 1's 5.04%
— a real and reported gap on rPPG (visible-wavelength imaging).

For us, MAX30102 IR @ ~880 nm penetrates melanin much better than green
or visible light, so we should fare better than they did. But "should"
is not "do." Once we have something shippable, we should run sanity
checks across the skin-tone range available to us.

> TODO: add a paragraph here once we have any real cross-skin-tone
> data. Until then this section is a reminder, not a finding.

---

### Concrete actions if/when we revisit

1. When wiring the on-demand BLE characteristic, return
   `{ bpm, confidence_pct, window_count }` not just `bpm`.
2. Log `|raw_bpm - kalman.x|` (innovation magnitude) per window so we
   can build a confidence-calibration curve from traces.
3. After the next sprint trace, compute our own MAPE for the
   nlms-heavy path windows specifically and put the number in the
   benchmark section above.

Both of (1) and (2) are ~30-line additions and neither blocks current
work.
