# Gesture detection architecture and roadmap

**Purpose:** definitive reference for the gesture-detection feature
track.  Captures the three product use cases (surface touchpad, air
mouse, advanced gestures), the unification insight that they share a
single underlying engine, verified hardware capabilities, the staged
implementation roadmap with realistic effort estimates, and the
research that grounds each decision.

**Status:** research and architecture complete; implementation has
not started.  Update this doc as we learn from real hardware behavior
during build-out.

> TODOs to resolve during implementation, not blocking architecture:
> - Measure actual gyro drift on LSM6DS3TR-C at our 104 Hz ODR.
>   Generic published range is 0.1-1 °/s; we need the actual number
>   for cursor-quality estimates.
> - Validate Zephyr HOGP profile on iOS specifically.  Nordic devzone
>   thread (referenced) flags some HID-on-iOS edge cases worth a
>   bring-up sanity check before committing to HID for the air-mouse
>   click protocol.
> - Confirm Edge Impulse Cortex-M4F (FPU) targeting -- documentation
>   typically lists "Cortex-M4" without explicit FPU mention.  Our
>   chip is M4F.
> - Specific feature-engineering choices for PPG gesture features
>   (Zhao 2018 details, windowing, derivative orders) to be locked
>   down before item 6 model training.
> - PDM driver bring-up on Zephyr for XIAO BLE Sense: confirm the
>   `nrfx_pdm` or `zephyr,pdm` binding works with our existing
>   board overlay; verify the MSM261D3526H1CPM is wired the way the
>   Seeed wiki expects.
> - Push-to-talk vs wake-word architecture decision: v1 will be
>   push-to-talk per the power analysis, but reconsider once we
>   have real battery measurements with PDM duty-cycled.
> - Final keyword vocabulary -- the section 6.5 list (volume /
>   brightness / youtube / music / presentation) is a starting
>   point.  Real vocabulary will depend on which apps the phone-
>   side team prioritizes and what users actually want.
> - END-OF-SESSION UX for the air-mouse pose: the current Item 0
>   build exits AIR_MOUSE on orientation-drop (wrist below armrest
>   level) with a 3 s cooldown re-engage window.  Long-term UX
>   needs deeper thought: how do we handle session boundaries
>   when the user is briefly distracted vs genuinely done?  Does
>   a long cursor-inactivity timer make sense as an additional
>   exit path?  Should the cooldown window vary based on recent
>   activity (e.g. shorter cooldown if user just air-moused for
>   30 s straight, longer if they were mid-gesture)?  Defer this
>   investigation until we have hands-on user testing data;
>   document the eventual design here when it lands.

---

## TL;DR

Three product use cases (surface touchpad, air mouse, advanced
gestures) plus a cross-cutting voice-context capability share the
same underlying engine.  Build one **unified gesture + voice pipeline**
with mode-aware and context-aware output routing rather than separate
implementations.

**Implementation order:** foundation (mode detector + HID profile +
cursor scaffolding) → chip-embedded gestures → air mouse v1 (IMU-only
pinch via Edge Impulse) → surface mode → custom macro gestures →
**voice-activated context state machine (KWS)** → PPG-fused pinch
(Apple-tier) → broader gesture vocabulary.  **Total to ship all three
use cases + voice context at usable quality: 11-15 weeks** with
Claude writing code + Edge Impulse handling the ML pipeline.

**Long poles:** labelled gesture data collection AND labelled keyword
data collection (both via Edge Impulse's mobile recorder, ~30 min per
gesture or keyword for usable models).  Not code, not infrastructure.

---

## 1. The three product use cases

### Use Case 1: Surface touchpad

Wrist resting on a desk surface.  Trigger gesture enters mode.
Wrist motion glides cursor on the surface.  Single tap on surface
acts as right click, double tap as left click.  Wrist roll / tilt
performs scroll.

### Use Case 2: Air mouse

Wrist raised in air.  Trigger gesture enters mode.  Wrist orientation
+ motion controls cursor.  Index + thumb pinch performs left click;
double pinch performs right click.  Wrist roll performs scroll.

### Use Case 3: Apple-tier advanced gestures

Wrist in natural position.  No mode trigger needed -- gestures act
as ambient input.  Pinch, double pinch, clench, flick, finger-specific
taps detected via PPG-perceived tendon perturbations + IMU micro-
movements.  Each gesture maps to a phone-side action.

### Cross-cutting: Voice-activated context state machine

A capability that **multiplies the power of all three use cases above
without changing them**.  The user speaks a context keyword
("volume", "brightness", "youtube", etc.) and the band enters that
context for the next N seconds.  Subsequent gestures execute within
that context rather than their default binding.

**Examples of the product feel:**

- Say "volume" → pinch up/down adjusts system volume
- Say "brightness" → pinch up/down adjusts display brightness
- Say "youtube" → tap seeks, double-tap toggles captions, pinch zooms
- Say nothing → gestures perform their default binding (mouse click etc.)

Context is set per-app on the phone-side configuration UI, so the
same gestures mean different things in different apps within different
contexts.  This turns the band from "input device" into "intent-aware
remote."

---

## 2. The unification insight

Looked at independently, these are three different builds.  Looked at
in terms of their underlying primitives, they are one engine with
three output bindings.

### Shared building blocks

| Primitive | Surface mode | Air mouse | Advanced gestures | Voice context |
|---|---|---|---|---|
| Mode / orientation detector | enters surface mode | enters air mode | enters always-on | n/a |
| Trigger gesture (wake) | ✓ | ✓ | optional | activates voice listen window |
| Chip-embedded tap engine | retuned for desk impact | for fallback | for tap-on-band | push-to-talk trigger |
| Cursor pipeline (gyro integration + drift correction + HID) | ✓ surface tracking | ✓ air tracking | — | n/a |
| Wrist-roll → scroll | ✓ | ✓ | optional | bound per active context |
| Pinch classifier (ML on IMU + PPG) | optional disambiguation | ✓ click events | ✓ primary input | bound per active context |
| PPG-derived gesture features | optional | improves pinch | required | n/a |
| **Audio pipeline (PDM + MFCC + KWS model)** | — | — | — | ✓ context-keyword detection |
| **Active-context state** (timed, per active context) | ✓ alters tap mapping | ✓ alters click mapping | ✓ alters gesture binding | sets / clears the context |
| **Phone-side context-gesture binding table** | ✓ per app config | ✓ per app config | ✓ per app config | ✓ defines available contexts |

### Architecture diagram

```
       IMU 100Hz + PPG 100Hz       PDM Mic (push-to-talk gated)
              |                                |
   COMMON FEATURE EXTRACTION              MFCC + KWS MODEL
   (windows, peaks, AC amplitude,        (Edge Impulse,
       chip-embedded events)              MFCC frontend +
              |                            small CNN)
              |                                |
  +-----------+----------+               KEYWORD DETECTED
  |                      |                     |
 MODE DETECTOR     GESTURE CLASSIFIER          v
(wrist orient.)   (Edge Impulse model,    ACTIVE CONTEXT STATE
   |               multi-class output)   (timed; auto-clears
   |                      |               after N seconds)
   |                      |                     |
   +----------------------+---------------------+
                          |
                    ACTION ROUTER
       (mode + context aware binding of gesture
        events to BLE HID / custom char / app msg)
                          |
              +-----------+----------+
              |           |           |
        CURSOR HID    CLICK HID   GESTURE EVT
       (mouse move)  (mouse btn)  (custom char,
                                 includes context)
```

**Build the foundation once.  Each use case is one output binding.
The voice context layer multiplies the meaning of each gesture
without changing the gesture detection path.**

---

## 3. Hardware capability check

Verified against datasheets and Zephyr documentation, not assumed.

### LSM6DS3TR-C (the IMU we have)

**Chip-embedded events that fire INT1 / INT2 pins for "free":**

- Single-tap recognition
- Double-tap recognition
- Wake-up detection
- Significant motion detection (we use this for workout wake)
- Free-fall detection
- 6D orientation detection
- Activity recognition (basic; LSM6DSV and LSM6DSOX have a full
  ML core, LSM6DS3TR-C does not)
- Pedometer (step detector + step counter -- chip accumulates,
  we read the register)

**What we don't get (would require newer chip):**

- Machine Learning Core (decision trees on-chip)
- Multiple programmable Finite State Machines for custom gestures
  (LSM6DSV has 16 FSM slots; LSM6DS3TR-C is a simpler chip)

**Verdict:** chip handles the foundation events natively.  We don't
need MCU code for tap, double-tap, wake-up, or step counting.
Anything more sophisticated (pinch, flick, finger-specific gestures)
runs on the Cortex-M4 with our own / Edge Impulse code.

### MAX30102 (the PPG sensor we have)

**Relevant for gestures:**

- IR LED + photodiode, single channel
- We sample at 100 Hz (driver config)
- 100 Hz is sufficient for PPG-feature-driven gesture detection
  (published wrist-PPG gesture papers use 50-200 Hz)
- Red LED unused -- could add SpO2 separately, not gesture-relevant

**Limitations:**

- Single channel -- can't do multi-PD spatial fusion
- Single wavelength -- can't do green/IR motion-artifact suppression
- Apple uses green for the same gesture role; we're slightly worse
  off but not blocked.  The PPG signal for gestures is the
  tendon/blood-flow signature -- IR sees this fine.

### nRF52840 + Zephyr BLE HID

**Verified:**

- Zephyr ships a `peripheral_hids_mouse` sample on nRF52840
- NCS includes both `peripheral_hids_mouse` and `peripheral_hids_
  keyboard` samples in production-quality form
- HID-over-GATT (HOGP) profile is standardized at the BLE-spec level
- Host compatibility: Windows / Linux / Mac / Android well-tested;
  iOS works but with some pairing-mode caveats (see TODO marker)

**Verdict:** we can implement the air-mouse / surface-touchpad cursor
+ click via standard BLE HID.  No proprietary protocol needed.

**Bonus:** because it's standard HID, the band works as a mouse for
*any* connected host without an app.  The phone app is only needed
for the advanced-gesture-to-action mapping in Use Case 3.

### Cortex-M4F + Edge Impulse / TFLite Micro

**Verified by reference deployments:**

- Typical 6-axis IMU gesture-recognition model: **20-40 KB** flash,
  similar order in working RAM
- With Edge Impulse EON Compiler + int8 quantization: significantly
  smaller (often <10 KB working RAM)
- Reference platforms include Cortex-M4 / M4F class chips with 256 KB
  RAM and 1 MB flash -- our exact configuration

**Our current footprint:**

- FLASH: 290 KB / 788 KB (37 %)
- RAM: 86 KB / 256 KB (34 %)

**Budget for gesture additions:** ~498 KB flash + ~170 KB RAM
headroom.  A 50 KB model + feature buffers + classifier scaffolding
fits comfortably with room for at least 2 more major subsystems.

**Verdict:** ML deployment is feasible.  No hardware blocker.

### Xiao BLE Sense PDM microphone

**Hardware:**

- **MSM261D3526H1CPM** (NOT MP34DT05 -- that's the Arduino Nano BLE
  33 Sense; the XIAO BLE Sense uses a different mic IC, verified
  against Seeed's wiki).
- Pulse Density Modulation (PDM) digital microphone.
- Connected to nRF52840's built-in PDM peripheral.
- Sampling rate typically 16 kHz for keyword spotting use cases
  (down from the chip's native PDM rate of 1.024 MHz after
  decimation, which the nRF52840 PDM peripheral does in hardware).

**Verified Edge Impulse path:**

Seeed has an official tutorial: *"Speech Recognition based on Edge
Impulse"* for the XIAO BLE Sense -- proven workflow for training
custom wake-words / keywords on this exact hardware.  Edge Impulse
also has a generic *"Keyword spotting"* end-to-end tutorial that
matches our deployment target (Cortex-M4 + PDM).

**Benchmark performance on Cortex-M4 (from TinyML literature):**

- DS-CNN model, **~20 KB**, achieves **90.5 % accuracy** at 15 ms
  per-inference latency, 0.3 mJ per inference
- Optimized variants: **5 ms latency**, 0.05 mJ (6x better)
- Standard architecture: MFCC features → small CNN → softmax
- Typical input: 1-second sliding windows, 500 ms stride

**For our use case (5-15 context keywords):**

- ~30-50 KB flash for the model + MFCC frontend code
- ~5-10 KB RAM for audio buffers + inference working set
- **Expected accuracy: 88-95 %** for a closed vocabulary of 5-15
  context keywords + a "negative" class for everything else

**Power impact -- the architectural decision:**

PDM mic continuous listening + KWS inference is **expensive on
battery** (~1-3 mA combined).  Continuous always-on voice listening
is impractical on wearable batteries.

Three architectural patterns considered:

| Pattern | Power | UX | Recommendation |
|---|---|---|---|
| Always-on KWS | ~1-3 mA continuous | Most natural | Skip -- battery cost too high for wrist |
| Push-to-talk via gesture | Mic off until triggered | Requires deliberate user action | **Recommended for v1** |
| Wake-word + KWS | Low-power VAD always on, KWS gated | Hands-free, good UX | v2 -- requires VAD model in addition |

**Push-to-talk v1 flow:**

1. User taps the band (chip-embedded tap event, ~zero cost)
2. Band enables PDM + KWS pipeline for ~3-5 seconds
3. User speaks a context keyword ("volume" / "youtube" / etc.)
4. KWS classifies, publishes context state, mic disables
5. Context times out after ~10 seconds of gesture inactivity, or
   immediately on next tap

This gives near-Apple feel (just tap-and-speak) at a fraction of the
power cost of always-on listening.

**Verdict:** PDM + KWS via Edge Impulse is a verified path on this
exact hardware.  Power is the only constraint; addressed by push-to-
talk gating.

### Power budget

Per-component current draw, continuous mode:

| Component | Current | Source |
|---|---|---|
| nRF52840 active + BLE | ~3-5 mA | varies with radio scheduling |
| LSM6DS3TR-C continuous combo | ~0.9 mA | datasheet |
| MAX30102 PPG continuous | ~0.6 mA | datasheet |
| **Total active gesture mode** | **~5-7 mA** | |

Implications:

- For a typical 200-500 mAh coin / LiPo battery, **40-100 hours of
  continuous gesture-mode runtime** before recharge
- That's plenty for **triggered or duty-cycled** gesture mode (e.g.
  "active for 60 s after raise-to-wake")
- That's marginal for **always-on background gesture listening**
  (Use Case 3's ambient-input model).  Without duty cycling, battery
  drains in under 2 days even on a 500 mAh cell

**Architectural implication:** Use Cases 1 and 2 should be explicitly
triggered.  Use Case 3 needs a "smart-on" model where IMU listens
continuously for the trigger pattern (chip-embedded events, no MCU
wake) and the full PPG + ML pipeline only spins up after a candidate
gesture is detected.

---

## 4. Use Case 1 detail -- surface touchpad

### Closest reference: Tap Strap 2's Mouse Mode

Tap Strap 2 enters Mouse Mode automatically when the user's thumb
rests on a surface.  It uses an **optical mouse sensor on the thumb
ring** for cursor tracking -- not the IMU.

### What this means for us

We don't have an optical sensor.  Wrist-only IMU on a surface gives:

- **Cursor X/Y via gyro integration** -- jittery; gyro drift will be
  visible.  Aggressive zero-velocity update (detect wrist-still
  intervals and reset velocity) is the standard mitigation.
- **Surface tap detection** -- the LSM6 tap engine can be retuned for
  the impulse signature of a finger tapping the desk near the band
  (vs the chip's default of tap-on-device).  Empirical threshold tune.

### Realistic capability

| Sub-feature | Achievable | Quality |
|---|---|---|
| Mode entry (wrist-flat detection) | ✓ via accel gravity vector | Reliable |
| Cursor tracking | ✓ via gyro + ZVU | **Jittery** -- usable for slow cursor, frustrating for fine targeting |
| Single tap → right click | ✓ via LSM6 tap engine, retuned | Good after threshold tuning |
| Double tap → left click | ✓ via LSM6 double-tap engine | Good |
| Scroll | ✓ via wrist roll | Coarse but workable |

### Honest assessment

This is the **weakest fit for our hardware** of the three.  Tap Strap
2's surface mode works because of the optical sensor; ours won't
match.

**Recommended reframe:** call it "wrist remote on a surface" rather
than "real touchpad."  Optimize for **discrete gestures** (taps,
wrist-flick directions) rather than continuous cursor control.  This
fits the hardware much better and avoids user-expectation mismatch.

---

## 5. Use Case 2 detail -- air mouse

### Closest references

- **Doublepoint Wisp** (now Oura): IMU-only smartwatch detection of
  pinch + ray-cast pointing.  Documented in CHI 2022 paper "Enabling
  Hand Gesture Customization on Wrist-Worn Devices."
- **Tap Strap 2's Air Mouse Mode**: cursor + finger taps.  Uses
  multiple sensors across the hand.
- **WiiMote**: 6-axis IMU for cursor was the original proof of concept
  in 2006.

### What's achievable

| Sub-feature | Achievable | Quality | Approach |
|---|---|---|---|
| Mode entry | ✓ raise-to-wake + dwell | Reliable | Gravity vector + IMU still-detection |
| Cursor X/Y | ✓ gyro integration with gravity correction | **Smooth for sessions <30 s**, drift over longer use | Complementary filter or Madgwick |
| Pinch click (index + thumb) | ✓ IMU-only via Edge Impulse | **82-88 %** per published research | Train a small CNN/MLP on accel + gyro windows |
| Pinch click + PPG fusion | ✓ add PPG features | **88-94 %** | Same model with PPG features added |
| Double pinch | ✓ timing layer | ~80 % if single pinch is 90 % | Single-pinch detector + temporal classifier |
| Scroll | ✓ wrist roll | Coarse but usable | Roll angular rate → scroll steps |

### Why pinch detection works on IMU alone

Per Doublepoint and the PLOS One 2020 paper on multimodal hand
gesture recognition at the wrist:

> "Muscles and tendons at different locations of the forearm compress
> the arteries with different pressure and duration when performing
> a gesture."

The tendon recoil from a finger pinch propagates a small but
detectable signature to the wrist's accel + gyro.  IMU-only catches
~82 %; adding PPG (where the tendon-perfusion event also shows up)
lifts this to Apple-tier numbers.

### Honest assessment

This is the **best fit for our hardware** of the three.  We have the
exact sensor stack Apple Watch S9 uses for Double Tap, minus the
dedicated Neural Engine.  Without per-user fine-tuning we won't hit
Apple's "feels magical" reliability, but mid-80s to low-90s pinch
accuracy is achievable.

---

## 6. Use Case 3 detail -- Apple-tier advanced gestures

### Underlying physics

Per multiple peer-reviewed sources (Zhao 2018 UTK; Frontiers
Neuroscience 2022; multimodal PPG-FMG-ACC papers):

- Muscles and tendons compress arteries during finger movements
- PPG sees these as small blood-flow perturbations
- Different gestures produce different temporal patterns
- IMU sees the wrist micro-movements that accompany
- Combined signal, trained with ML, achieves usable classification

### Gesture vocabulary feasibility

| Gesture | Difficulty | Expected accuracy with our setup |
|---|---|---|
| Clench (full fist) | Easiest -- large tendon perturbation | ~92-95 % |
| Pinch (index + thumb) | Medium -- Apple Double-Tap class | ~85-92 % |
| Double pinch | Medium-hard | ~75-85 % |
| Single-finger flick | Hard -- small motion, brief signal | ~70-80 % |
| Specific-finger discrimination (which finger tapped) | Hardest -- inter-finger discrimination wants more sensor info than we have | ~60-70 %, may not be useful in practice |

### Why we'll be below Apple's accuracy

Apple advantages we lack:

- **Neural Engine** dedicated ML hardware (~10x our raw inference
  budget)
- **Hundreds of thousands of labelled gestures** for training
- **Per-user online fine-tuning** that improves accuracy over time
- **Multi-PD spatial fusion** on some Apple Watch generations

We compensate with:

- Same fundamental sensor stack (PPG + accel + gyro)
- Edge Impulse handling model design + training + Cortex-M4 codegen
- User can collect their own training data via Edge Impulse's mobile
  recorder
- Per-user fine-tune flow is buildable -- it's just a fine-tune pass
  on a base model

### Honest assessment

We can ship a 2-3 gesture vocabulary (pinch, double pinch, clench)
at usable accuracy.  Aiming for a richer Apple-tier vocabulary is
months of additional work and may never reach Apple parity without
hardware changes.

---

## 6.5 Voice-activated context state machine detail

### Concept

Voice doesn't replace gestures; it **adds context that changes what
each gesture means**.  Apple Watch's Siri equivalent is "do
something specific via voice."  Our model is different: "tell the
band what app/context you're in, then do it with a gesture."

The pattern in multimodal-interaction research is well-established
(see CHI 2023 *Voice-Accompanying Hand-to-Face Gesture Recognition*,
US patents on transmodal input fusion for wearables): **voice sets
intent, gesture executes action**.  Voice is good at categorical
labels ("youtube", "volume", "brightness"); gesture is good at
continuous / spatial actions (move cursor, adjust value, tap).
Each plays to its strength.

### How the state machine works

```
   Default state: NO ACTIVE CONTEXT
            |
            | (user taps band, says keyword)
            v
     CONTEXT = "volume"
     timer = N seconds
            |
            | (user performs gesture)
            v
   gesture is bound through CURRENT CONTEXT
   to "volume up" / "volume down" / etc.
            |
            | (timer expires OR user taps again)
            v
   Returns to NO ACTIVE CONTEXT
   (gestures revert to default mouse / cursor binding)
```

State transitions:

- **Push-to-talk trigger** (tap on band) opens a ~3-5 second voice
  listening window
- **Keyword recognized** sets the active context with a ~10 s
  inactivity timeout
- **Gesture performed** resets the inactivity timer and routes
  through the context's binding table
- **Inactivity timeout** OR **explicit cancel gesture** clears the
  active context, gestures revert to default

### Context vocabulary (initial v1 targets)

Five high-leverage contexts that pair well with our gesture set:

| Context keyword | Pairs with | Example bindings |
|---|---|---|
| "volume" | scroll / pinch / tap | scroll = volume +/-, tap = mute toggle |
| "brightness" | scroll / pinch | scroll = brightness +/- |
| "youtube" | tap / double-tap / pinch | tap = seek, double-tap = caption toggle, pinch = zoom |
| "music" | tap / pinch / scroll | tap = play/pause, scroll = next/prev track |
| "presentation" | flick / tap | flick = next/prev slide, tap = laser pointer |

Phone-side app holds the binding table.  User configures per-app
gesture meanings via the app UI.  Band just reports the active
context + gesture event to the app over BLE; app applies the
binding.

### What the band sends over BLE

When a context+gesture event fires:

```
struct GestureEvent {
    uint8_t context_id;        // 0 = no context, 1+ = keyword index
    uint8_t gesture_id;         // tap / pinch / flick / etc.
    uint8_t modifier;           // up / down / direction
    uint16_t confidence;        // 0-1000 for app-side gating
    uint32_t timestamp_ms;
};
```

Published over a **custom GATT characteristic** (this part requires
the app, unlike the standard HID mouse path which works app-less).
HID mouse path remains available when no context is active --
context aware gestures route through the custom char.

### Accuracy realistic for our setup

Edge Impulse KWS on Cortex-M4 with custom training data, vocabulary
of 5-15 keywords:

- **Push-to-talk path (no VAD challenges):** ~92-96 % keyword
  accuracy in quiet environments, ~85-90 % in moderate noise (cafe,
  office)
- **Always-on with VAD (v2 future):** ~85-92 % accuracy depending on
  noise and false-wake rate
- **Per-user training:** lifts accuracy 5-10 % at the cost of an
  onboarding flow

### Effort estimate

- PDM driver integration (Zephyr has it native, Seeed has tutorial):
  ~2-3 days
- Audio capture pipeline + MFCC frontend (Edge Impulse generated):
  ~2-3 days
- KWS model training: ~3-5 days of data collection across keywords
  + user accents (Edge Impulse mobile recorder collects samples
  with cloud training)
- Context state machine in firmware: ~2-3 days
- BLE custom characteristic + GATT extension: ~1-2 days
- Phone-app binding table + UX (app-side, separate from band):
  parallel work
- Testing and tuning across environments: ~5-7 days

**Total band-side: ~2-3 weeks.**

### Honest limitations

- **Wakeword false positives** -- "volume" might fire when watching
  TV news (mention of "volume"); push-to-talk gating mostly avoids
  this by requiring a band-tap first
- **Background noise** -- KWS accuracy drops in noisy environments;
  per-user training and increased confidence threshold help
- **Multi-keyword conflict** -- if two contexts have similar-sounding
  keywords, classifier may confuse them; vocabulary design matters
- **Onboarding friction** -- user has to learn the keyword set; this
  is a UX challenge more than a technical one
- **Privacy perception** -- a "listening wearable" raises concerns
  even if listening is gated to short windows; clearly communicate
  push-to-talk model to users

---

## 7. Implementation roadmap

Cumulative effort assumes each row builds on every row above.
Effort numbers reflect Claude writing C / C++ + Edge Impulse handling
the ML pipeline + user collecting training data with Edge Impulse's
mobile recorder.

| # | Item | Difficulty | Cumulative effort | Built using | Realistic accuracy |
|---|---|---|---|---|---|
| **0** | **Foundation:** mode detector, BLE HID profile (HOGP), cursor pipeline scaffolding | LOW | **3-5 days** | Zephyr `peripheral_hids_mouse` sample + custom mode FSM | n/a -- infrastructure |
| **1** | **Chip-embedded gestures** (tap-on-band, double-tap, wake, free-fall, sig-motion routing) | LOW | **+2-3 days** | LSM6DS3TR-C INT1 + register config | 95 %+ -- chip native |
| **2** | **Air mouse cursor + IMU-only pinch click** (Doublepoint-class) | MEDIUM | **+2-3 weeks** | Gyro integration + Edge Impulse pinch model | Cursor smooth <30 s; pinch 82-88 % |
| **3** | **Surface touchpad mode** (wrist on desk + LSM6 retuned tap + cursor reuse) | LOW-MEDIUM | **+3-5 days** | Reuses #0 cursor + retuned tap thresholds | Cursor jittery; clicks reliable |
| **4** | **Custom macro gestures** (wrist flick, twist, shake) | LOW-MEDIUM | **+3-5 days** | MCU-side threshold + duration classifier | ~90 % with tuning |
| **5** | **Voice-activated context state machine** (push-to-talk KWS + context binding) | MEDIUM-HIGH | **+2-3 weeks** | nRF52840 PDM peripheral + Edge Impulse KWS model + custom GATT char | KWS 88-95 % closed vocabulary; mode router gates voice power |
| **6** | **PPG-fused pinch** (Apple-tier approach) | MEDIUM-HIGH | **+2-3 weeks** | Add PPG features to Edge Impulse model + retrain | 88-94 % pinch (lift from #2) |
| **7** | **Multi-gesture vocabulary** (clench, double-pinch, finger flick, ...) | HIGH | **+3-6 weeks** | Edge Impulse broader gesture set + per-user fine-tune | 75-92 % varies by gesture |

**Total to ship all three use cases + voice context at usable
quality: 11-15 weeks.**

### Phase milestones

- **End of Week 1**: foundation + chip gestures shipping.  Band acts
  as a basic HID with tap controls.
- **End of Week 3**: items 0-3 shipping.  All three use cases have v1
  functionality at usable-but-not-great quality.  This is the
  earliest "demoable" point.
- **End of Week 5**: items 4 shipping.  Custom macro gestures
  available.
- **End of Week 7-8**: item 5 shipping.  Voice context layer
  live -- band feels intent-aware ("say a word + do a gesture").
  This is the **defining product moment** that distinguishes us
  from any IMU-only gesture band on the market.
- **End of Week 10**: item 6 shipping.  Pinch accuracy at
  Apple-tier numbers via PPG fusion.
- **End of Week 15**: item 7 shipping.  Broader gesture vocabulary
  trained against real user data.

### Long pole: data collection

The technical work is fast.  The bottleneck is **labelled training
data**, for both gestures (items 2, 5/PPG, 6) and keywords (item 5/
voice).  Realistic minimums:

**For gestures:**
- Per gesture, basic model: ~200-500 samples, ~30 min collection
- Per gesture, excellent model: ~1000-2000 samples, ~2 hours
- Multi-user generalization: multiply by user count

**For keywords (KWS):**
- Per keyword, basic model: ~50-100 utterances per speaker, ~10-15
  minutes collection per speaker
- Per keyword, multi-speaker model: ~50 utterances × 10+ speakers
  = 500+ samples per keyword
- Background "negative class": ~30 minutes of varied ambient audio

Edge Impulse's mobile recorder app collects + labels samples for
both modalities (IMU on the band, audio via phone mic / band mic).
No special equipment.  Plan for ~30 minutes per gesture per phase
and ~15 minutes per keyword per speaker.

---

## 8. Power budget

### Per-mode estimates

| Mode | Avg current | Battery life on 200 mAh | Battery life on 500 mAh |
|---|---|---|---|
| IDLE (current production) | ~50-100 µA | months | months |
| SNAPSHOT (current production, 22 s every 2 min) | ~250 µA avg | ~33 days | ~83 days |
| WORKOUT (current production) | ~5-7 mA | ~30 hours | ~75 hours |
| **Gesture mode active (continuous)** | ~5-7 mA | ~30 hours | ~75 hours |
| **Gesture mode triggered (5 min sessions)** | ~150-300 µA avg | weeks | weeks |
| **Gesture-chip-only (waiting for trigger)** | ~150 µA | months | months |
| **Voice mode active (3-5 s push-to-talk windows)** | adds ~1-3 mA during window only | minimal averaged impact | minimal averaged impact |
| **Voice always-on (NOT recommended)** | adds ~1-3 mA continuous | <2 days | ~3-5 days |

### Implications

- **Continuous always-on gesture detection is impractical** on coin-
  cell batteries.  500 mAh LiPo gives ~3 days at best.
- **Triggered gesture mode is fine.**  User performs raise-to-wake +
  air-mouse gesture, mode runs for 60-300 s, returns to chip-trigger
  listening.
- **Use Case 3 (ambient gestures) requires a duty-cycled architecture:**
  - Chip-embedded events (tap, sig-motion) listen continuously at
    ~150 µA on the LSM6 alone.  MCU sleeps.
  - When chip fires an event, MCU wakes, PPG enables, ML model runs.
  - Detection takes <1 s.  Result published, MCU returns to sleep.
  - **Average current: a few hundred µA**, battery acceptable.

### Architectural requirement

The mode router must support **"chip-armed sleep"**: gesture pipeline
suspended, LSM6 chip-embedded events still firing INT1, MCU wakes
only on event.  This is similar to our existing SNAPSHOT-IDLE state-
machine architecture and reuses the same power-control primitives
(`max30102_shutdown` etc.).

---

## 9. Implementation order and dependencies

```
[0] Foundation
      |
      +-> [1] Chip gestures
      |
      +-> [2] Air mouse + IMU pinch ---> [6] PPG-fused pinch
      |                                        |
      +-> [3] Surface mode (reuses [0] cursor + [1] tap)
      |
      +-> [4] Macro gestures (reuses [0] mode FSM)
      |
      +-> [5] Voice context (needs [0] action router + custom GATT)
                                                 |
                                                 v
                                    [7] Broader gesture set
```

Dependencies:

- **Item 0 unblocks everything else.**  Build it first.
- **Items 1, 2, 3, 4 are parallelizable** after item 0.
- **Item 5 (voice) needs only item 0** -- the action router and
  custom GATT characteristic.  Can be built in parallel with
  items 2-4.  Most product impact for the effort.
- **Item 6 needs item 2 working** (extends the same gesture model).
- **Item 7 needs items 2 + 6** (extends the same gesture model
  further).

Items 1, 3, 4 are each days of work and can be batched with item 0.
Item 5 (voice) is the next-biggest product lever after item 2 and
can run in parallel with item 6.

---

## 10. Known gaps to fill during implementation

Refinements that didn't block architecture but will need attention
during build-out:

### A. Gyro drift quantification

Generic literature says wrist gyros drift 0.1-1 °/s.  Our cursor-
quality estimates ("smooth <30 s") rely on this range.  **First
build-out task: characterize actual drift on our chip + ZVU
algorithm.**  Adjust cursor mode duration accordingly.

### B. PPG-feature engineering for gestures

Zhao 2018 (UTK) shows PPG-finger-gesture detection works at
~85-90 %, but the paper's specific feature engineering choices
(windowing, derivative orders, frequency bands) need to be extracted
and adapted to our 100 Hz IR signal before item 5 model training.

### C. iOS BLE HID validation

Zephyr HID samples work on Windows / Linux / Mac / Android.  iOS
support exists but has known pairing-mode caveats.  Validate with a
bring-up test before committing the air-mouse architecture to HID.
Fallback: custom GATT char + app-side translation.

### D. Latency budget for cursor / click feel

Haven't researched what latency is acceptable for cursor feel
(probably <50 ms) and click ack (probably <100 ms).  Validate against
real user testing during item 2 build-out.

### E. Edge Impulse Cortex-M4F (FPU) targeting

Documentation typically lists "Cortex-M4" without FPU detail.  Our
chip is M4F.  Verify FPU is utilized by EI codegen -- should give
meaningful inference speedup.

---

## 11. References

### Commercial reference products

- [Apple Watch double tap gesture available with watchOS 10.1 (Apple Newsroom)](https://www.apple.com/newsroom/2023/10/apple-watch-double-tap-gesture-now-available-with-watchos-10-1/)
- [Apple Watch Double Tap technical details (Medium / Michael Parekh)](https://medium.com/@mparekh/ai-apple-watchs-magical-double-tap-e6f20d71408f)
- [Apple Watch wrist flick gesture only works on newer models (Fahad X)](https://www.fahadx.com/posts/apple-watch-wrist-flick-gesture-only-works-on-newer-watch-models-for-a-reason)
- [Apple patent for next-gen EMG sensors (Patently Apple, 2025)](https://www.patentlyapple.com/2025/01/a-new-apple-patent-reveals-that-theyre-working-on-next-gen-emg-sensors-to-assist-apple-watch-gesture-technology.html)
- [Doublepoint product page](https://www.doublepoint.com/product)
- [Oura acquires Doublepoint (MobiHealthNews)](https://www.mobihealthnews.com/news/oura-acquires-doublepoint-gesture-recognition-technology-wearables)
- [Doublepoint XR gesture detection technology (Auganix)](https://www.auganix.org/xr-news-doublepoint-unveils-smartwatch-based-gesture-detection-technology-for-xr-device-control/)
- [Tap Strap 2 product page / how Tap works](https://www.tapwithus.com/how-tap-works/)
- [MouveMouse wearable motion-based mouse](https://mouvemouse.com/)

### Academic research

- [Enabling Hand Gesture Customization on Wrist-Worn Devices (CHI 2022 / ACM DL)](https://dl.acm.org/doi/fullHtml/10.1145/3491102.3501904)
- [Multimodal hand gesture recognition using single IMU and acoustic measurements at wrist (PLOS One 2020 / PMC 6957149)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6957149/)
- [WristSketcher: Creating Dynamic Sketches in AR with a Sensing Wristband (arXiv 2210.11674)](https://arxiv.org/pdf/2210.11674)
- [SurfaceXR: Fusing Smartwatch IMUs and Egocentric Hand Pose (arXiv 2603.19529)](https://arxiv.org/pdf/2603.19529)
- [PPG-based Finger-level Gesture Recognition Leveraging Wearables (Zhao 2018, IEEE)](https://ieeexplore.ieee.org/document/8486006/)
- [PPG-based Finger-level Gesture Recognition (Zhao 2018 UTK PDF)](https://web.eecs.utk.edu/~jliu/publications/zhao2018ppg.pdf)
- [Principal component analysis of PPG signals for improved gesture recognition (Frontiers Neurosci 2022)](https://www.frontiersin.org/journals/neuroscience/articles/10.3389/fnins.2022.1047070/full)
- [Gesture recognition of wrist motion based on wearables (Procedia 2022)](https://www.sciencedirect.com/science/article/pii/S1877050922015927/pdf)

### Hardware datasheets and verification

- [LSM6DS3TR-C datasheet (ST)](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)
- [LSM6DS3TR-C always-on application note (ST AN5130)](https://www.st.com/resource/en/application_note/an5130-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
- [LSM6DSOX machine learning core (ST AN5259)](https://www.st.com/resource/en/application_note/an5259-lsm6ds3trc-machine-learning-core-stmicroelectronics.pdf) -- comparison reference for what we lack
- [Bosch Sensortec IMUs for gesture control](https://www.bosch-sensortec.com/en/news/bosch-sensortec-imus-the-ultimate-solution-for-gesture-control-in-wearable-devices.html)

### BLE HID profile on nRF / Zephyr

- [HID Peripheral sample -- Zephyr Project Documentation](https://docs.zephyrproject.org/latest/samples/bluetooth/peripheral_hids/README.html)
- [Bluetooth: Peripheral HIDS mouse -- nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/samples/bluetooth/peripheral_hids_mouse/README.html)
- [BLE Peripheral HIDS mouse -- nRF52840 MDK USB Dongle](https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/guides/ncs/samples/ble/peripheral_hids_mouse/)
- [BLE mouse example for NRF52840 (denisgav GitHub)](https://github.com/denisgav/nrf52840_ble_mouse)
- [Nordic Q&A: HID mouse functionality with iOS on nRF52840 Dongle](https://devzone.nordicsemi.com/f/nordic-q-a/115855/inquiry-on-hid-mouse-functionality-with-ios-on-nordic-nrf52840-dongle)

### Voice-activated context / Keyword spotting (KWS)

- [PDM Usage for XIAO nRF52840 Sense (Seeed Studio Wiki)](https://wiki.seeedstudio.com/XIAO-BLE-Sense-PDM-Usage/)
- [Speech Recognition based on Edge Impulse (Seeed Studio Wiki)](https://wiki.seeedstudio.com/XIAO-BLE-PDM-EI/)
- [XIAO nRF52840 Sense Speech Recognition with Edge Impulse (Bruno Santos / Medium)](https://medium.com/@feiticeir0/xiao-nrf52840-sense-speech-recognition-with-edge-impulse-cc1d9911109)
- [Edge Impulse end-to-end keyword spotting tutorial](https://docs.edgeimpulse.com/tutorials/end-to-end/keyword-spotting)
- [TinyML Made Easy: KeyWord Spotting (Hackster.io)](https://www.hackster.io/mjrobot/tinyml-made-easy-keyword-spotting-kws-5fa6e7)
- [TinyML: Wake Word Detection (Hackster.io)](https://www.hackster.io/zy43/tinyml-wake-word-detection-c28b82)
- [Wake Word Detection technical overview (Arun Baby)](https://www.arunbaby.com/speech-tech/0040-wake-word-detection/)
- [Keyword Spotting and Voice Wake Word (SiliconWit Edge AI / TinyML)](https://siliconwit.com/education/edge-ai-tinyml/keyword-spotting-voice-wake-word/)
- [TinyML Platforms Benchmarking (arXiv 2112.01319)](https://arxiv.org/pdf/2112.01319)

### Multimodal voice + gesture interaction (research / patents)

- [Enabling Voice-Accompanying Hand-to-Face Gesture Recognition with Cross-Device Sensing (CHI 2023)](https://dl.acm.org/doi/10.1145/3544548.3581008)
- [GazePointAR: A Context-Aware Multimodal Voice Assistant (CHI 2024)](https://dl.acm.org/doi/10.1145/3613904.3642230)
- [Transmodal input fusion for a wearable system (US patent 11,983,823)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11983823)
- [Transmodal input fusion for a wearable system (US patent 10,861,242)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10861242)
- [Multimodal task execution and text editing for a wearable system (US patent 11,960,636)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11960636)
- [Providing a context related view with a wearable apparatus (US patent 10,409,464)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10409464)
- [Designing Multimodal AI Interfaces: Voice, Vision & Gestures (Fuselab Creative)](https://fuselabcreative.com/designing-multimodal-ai-interfaces-interactive/)

### Edge Impulse / TinyML on Cortex-M4

- [Edge Impulse documentation -- Edge AI hardware overview](https://docs.edgeimpulse.com/docs/edge-ai-hardware/edge-ai-hardware)
- [Edge Impulse Inference performance documentation](https://docs.edgeimpulse.com/knowledge/metrics/inference-performance)
- [On-Device Gesture Recognition for Patient Communication (Edge Impulse blog)](https://www.edgeimpulse.com/blog/helping-patients-communicate-through-on-device-gesture-recognition/)
- [TinyML Made Easy: Gesture Recognition (Hackster.io)](https://www.hackster.io/mjrobot/tinyml-made-easy-gesture-recognition-ce13a5)
- [Gesture Recognition with Edge Impulse and Arduino (Leonardo Cavagnis / Medium)](https://leonardocavagnis.medium.com/gesture-recognition-with-edge-impulse-and-arduino-0da09c0873d5)
- [Gesture Recognition with the Thunderboard Sense 2 (Hackster.io)](https://www.hackster.io/gatoninja236/gesture-recognition-with-the-thunderboard-sense-2-d010ff)
- [ESP32-S3 Gesture Recognition with Edge Impulse (ForestHub)](https://www.foresthub.ai/resources/esp32-s3-gesture-recognition-edge-impulse)

### Power consumption references

- [nRF52840 Battery Life Estimation (Nordic devzone)](https://devzone.nordicsemi.com/f/nordic-q-a/63521/nrf52840-battery-life-estimation)
- [Current consumption of LSM6DS3 (ST community)](https://community.st.com/t5/mems-sensors/current-consumption-of-lsm6ds3/td-p/263934)
- [Optimizing Performance and Battery Life with the LSM6DS3TR in Portable Devices](https://www.ic-components.com/blog/optimizing-performance-and-battery-life-with-the-lsm6ds3tr-in-portable-devices.jsp)
- [More successful adventures in reducing nRF52840 power consumption (Tomas McGuinness)](https://tomasmcguinness.com/2025/01/02/more-adventures-in-nrf52840-power-consumption/)

### Cross-references

- See `hr-algorithm-decisions.md` for the parallel HR-algorithm
  decision doc.  Step counting and activity classification are parked
  in `software-optimization-roadmap.md` (Stage 5+ section).
- See `google-phrm-notes.md` for confidence-gated aggregation pattern
  -- some of the same UX patterns apply to gesture confidence
  thresholds.
