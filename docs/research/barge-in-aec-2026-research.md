# Robust barge-in / self-echo — cross-domain research (2026-06-30)

> Deep multi-source research (29 sources, 25 claims adversarially verified, 22 survived). General
> full-duplex voice problem, NOT wearable-specific. Each finding tagged with confidence + sources.
> The headline: **the field fixes false barge-ins at the acoustic FRONT-END (by modelling the
> NONLINEAR reference), not at the detector.** We've been fighting it at the detector.

## THE core insight (why we've struggled)
We kept patching the *detector* (energy VAD, abs floor, onset grace, convergence gate). The field's
actual fix is upstream: make the residual *small enough* that detection is easy — and specifically
**handle loudspeaker nonlinearity by having a DNN estimate the nonlinearly-distorted reference** that
the canceller subtracts. Our DTLN-aec does NOT do nonlinear-reference estimation, which is exactly our
rail-sag/cheap-speaker failure.

## A. Hybrid acoustic front-end = linear AEC → neural RES — CONFIRMED (high)
The dominant production pattern is a **cascade: linear adaptive filter FIRST (removes linear echo),
then a NEURAL residual-echo suppressor (RES) SECOND** (removes nonlinear residual).
- Sources: EchoFree (arXiv 2508.06271, 2025, verbatim "cascaded framework ... linear filtering
  followed by neural post filter"); Technion DTD-aided RES (MDPI Acoustics 2022); Alibaba mobile-AEC
  (arXiv 2508.07561); Carbajal (2005.09237); MDPI Electronics 12/15/3258.
- HONEST CAVEAT: literature says "a common approach," not "THE standard." The single strongest 2023
  model, **DeepVQE** (arXiv 2306.03177), is an end-to-end JOINT network (no separate linear filter)
  that beat the challenge top-1 — but it has **NO open weights** (Microsoft Teams production).
- Verdict: cascade = dominant lightweight/practical hybrid (our DTLN belongs here); joint = closed SOTA.

## (A-nonlinear) DNN-estimated nonlinear reference — CONFIRMED (high) — DIRECTLY our failure
The strongest models beat cheap-loudspeaker/rail-sag distortion by having a DNN estimate the
**nonlinearly-distorted reference**:
- **Deep Adaptive AEC** (ICASSP 2022, Amazon/OSU, minjekim.com/.../icassp2022_hzhang.pdf): traditional
  AEC "cannot estimate nonlinearities introduced ... by amplifiers and loudspeakers"; fixed with a
  "DNN-estimated nonlinear reference signal" via a learned mask.
- **NeuralKalman / NKF-AEC** (arXiv 2301.12363; code github.com/fjiang9/NKF-AEC): "employs a DNN to
  estimate nonlinearly distorted far-end signals." Open code, 5.3K params, RTF ~0.09.
- DeepVQE (2306.03177), JAECBF (2111.04904, but MULTI-CHANNEL → not single-mic).
- This is the literature directly matching our documented rail-sag-distorted coupling.

## C. Double-talk detection (DTD) with the known reference — CONFIRMED (high) + honest knock
Coherence (magnitude-squared coherence between reference & mic) and cross-correlation DTD are the
canonical "is this the user or my own echo?" gate.
- Sources: Tashev (Microsoft) "Coherence Based DTD with Adaptive Threshold"; Gänsler et al. 1996
  "A double-talk detector based on coherence" (IEEE); cross-correlation DTD for NONLINEAR AEC (IEEE
  2014); lineage Benesty/Gänsler/Cho-Morgan (IEEE TSAP 2000).
- **CRITICAL CAVEAT (our worst regime):** coherence/cross-correlation DTD precision **degrades exactly
  pre-convergence and under strong loudspeaker nonlinearity** — i.e. where we need it most. DTD helps
  but is NOT a pre-convergence silver bullet. Mitigation: pair with the nonlinear-reference neural
  models above. (This tempers yesterday's "build coherence DTD" plan — it's necessary but not sufficient.)

## (C-neural) Neural DTD-aided RES — CONFIRMED (high) but MODEST gains
Embedding a neural DTD + feeding the reference as an input channel to the RES gives ~2 dB ERLE /
0.1–0.2 PESQ (Technion MDPI 2022, Table 1: ERLE 40.4→42.3; DT PESQ 2.74→2.84; full system 44.3/2.94).
A refinement, not the primary lever. JAECBF integrates DTD end-to-end but is multi-channel.

## B. Audio-aware turn / interruption detection — CONFIRMED complementary (high) — but NOT for self-echo
- **LiveKit Adaptive Interruption Handling**: "audio encoder with a CNN" that "operates AFTER voice
  activity detection" to reject brief backchannels, coughs/sighs, typing/music/chatter.
- **Smart Turn v3** (Daily/Pipecat): open, **12 ms CPU inference**; used WITH Silero VAD.
- **LiveKit Turn Detector v1 / v1-mini** (open-weight, CPU).
- **CRITICAL CAVEAT:** these reject **human** false-fires (backchannels/noise). They do **NOT** solve
  "the speech IS residual echo" — if the residual sounds like fluent speech, a turn classifier will fire
  on it. So B is complementary, NOT a fix for self-echo false barges.
- Vendor metrics unaudited: LiveKit's "rejects 51% of VAD false positives" was **REFUTED** (1-2) in
  adversarial review as not independently corroborated. Treat all vendor barge-in numbers skeptically.

## (Practical lever) Two-output AEC: suppress harder for the detector than for the uplink — CONFIRMED (single source)
Alibaba mobile-AEC (arXiv 2508.07561, 2025): emit two task-tailored signals via parametric Wiener
filtering — **β=0.6 (more echo suppression) for VAD/detection vs β=0.2 (more speech) for ASR/uplink**.
Directly actionable for our two-failure split: feed barge-in detection a HEAVILY-suppressed branch,
feed Gemini a gentler branch. CAVEAT: single source; their "VAD" is ASR endpointing, so applying it to
barge-in is a reasonable inference, not their stated claim. Same paper corroborates our failure modes
(device/loudspeaker nonlinearity + variable ref-vs-mic latency "a few to several hundred ms").

## D. Our DTLN-aec status + open menu — CONFIRMED (high)
DTLN-aec = open (MIT), TF-Lite, CPU-real-time, **but placed 3rd in the MS AEC Challenge = solid
baseline, NOT SOTA**. Open + CPU-real-time menu to build a stronger stack:
- Linear front stage: **WebRTC AEC3** or **SpeexDSP** (classical).
- Neural RES / hybrid: **DTLN-aec** (have it), **NKF-AEC** (open code, estimates nonlinear reference).
- Strongest (DeepVQE) = closed weights, multi-channel options (JAECBF) need a mic array → not for us.

## E + F-problem2 — EVIDENCE GAP (low / unverified)
Krisp BVC / VoiceFilter-Lite / Personal VAD claims did NOT survive verification this batch (gap, not
refutation). Exact "stop on barge-in" event names (OpenAI `response.cancel`/`conversation.item.truncate`,
Gemini Live activity handling, Pipecat interruption frames, LiveKit interrupt) were NOT corroborated —
the raw docs were fetched (platform.openai.com realtime, ai.google.dev live-api, pipecat
interruption-strategies, OpenAI community truncation-bug thread) but not verified into claims. **Needs a
dedicated follow-up.** Canonical Problem-2 fix remains: server cancel + flush LOCAL playout + truncate
context to actually-heard audio.

## REFUTED (honesty)
- "Kalman foundation gives inherent double-talk robustness + rapid convergence" — REFUTED 0-3.
- "Linear filters inherently CANNOT handle nonlinear distortion" — REFUTED 0-3 (overstated; they handle
  the linear part; neural handles residual).
- LiveKit "51% false-positive rejection" — REFUTED 1-2 (unaudited vendor claim).

## RECOMMENDED production ordering (software-only, single-mic) + what's free
1. **Linear AEC** (WebRTC AEC3 / SpeexDSP) — fast linear subtraction + alignment. [free/open]
2. **Neural RES that models the NONLINEAR reference** — our actual lever for rail-sag. DTLN-aec (have,
   baseline) → evaluate **NKF-AEC** (open, estimates nonlinear reference). [free/open]
3. **DTD gate (coherence/cross-correlation) on the known reference** — weak pre-convergence, pair with #2. [build]
4. **Detector runs on a MORE aggressively suppressed branch** than the uplink (β trick). [build]
5. **Audio-aware turn classifier (Smart Turn v3, 12ms CPU)** — rejects human backchannels only; later. [free/open]
6. **Problem 2:** server cancel + local playout flush + context truncation (needs targeted follow-up for exact events).
