# HR algorithm decisions and feature roadmap

**Purpose:** definitive reference for the algorithmic choices that
shaped main, the alternatives we rejected and why, what those choices
cost us in feature reach, and the architectural path to add the
trade-off features later if/when they become requirements.

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
| **Resting HR + trends** | Aggregate Kalman state during WORN+STATIONARY+settled windows over a day.  Snapshot-aggregation backlog item from `google-phrm-notes.md`. |
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

- See `google-phrm-notes.md` for findings from Google's rPPG research
  (camera-based, but pipeline patterns translate).
- See `software-optimization-roadmap.md` for the staged plan that
  brought us from baseline to v0.4 production-track, including the
  multi-feature SQI and adaptive Kalman items still parked.

---

## 9. Things this doc does NOT cover (yet)

- Power consumption tradeoffs of each algorithm (when/if we measure
  on a coulomb counter).
- Specific HRV thresholds or stress scoring formulas -- defer to when
  the hybrid path is on the table.
- Per-user calibration strategies (no consensus in literature).
- BLE characteristic design for HRV / AFib / extended health data --
  app-side coordination required first.
