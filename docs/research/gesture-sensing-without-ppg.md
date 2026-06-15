# Gesture sensing without PPG — the realistic verb menu for an IMU + mic wristband

**Source:** deep-research session 2026-06-15 (fan-out web search → fetch →
3-vote adversarial verification → synthesis; 149 verified claims). Compiled
from the verified-claim set after the workflow stalled before its own final
synthesis step.

**Status:** research reference. **REFER TO THIS WHEN BUILDING GESTURE VERBS** —
it defines what the hardware can/can't do so we don't spec a gesture that needs
sensors we don't have.

**Hardware grounding:** Seeed XIAO nRF52840 Sense (Cortex-M4F), **LSM6DSL**
6-axis IMU (accel+gyro, **NO magnetometer**) over I2C with on-chip tap +
sig-motion + 6D-orientation engines and a tap INT line, **PDM mic**, worn RIGHT
wrist / VOLAR / radial(thumb)-side, output = BLE HID mouse+keyboard to a Mac.
**No PPG used for gestures, no EMG** (architectural decision — see
`decision_ppg_fusion_session_only_2026_06_10`).

---

## Bottom line

The reliable menu on this hardware is **whole-wrist motions + orientation poses
+ tap-class impulses**, plus — if we build a high-ODR bio-acoustic path — **on-body
taps and finger flicks/rubs**. The one thing genuinely **lost without PPG** is the
**subtle finger-pinch with near-zero wrist motion** (Apple "Double Tap"). True
per-finger discrimination (ASL hand-shapes) needs *active* acoustics + a thumb
ring, not a passive band.

---

## The verb menu (tagged by confidence on OUR hardware)

| Verb | Modality on our band | Confidence | Notes |
|---|---|---|---|
| Single / double tap (band or surface) | LSM6DSL **on-chip tap engine** (416 Hz, 2g, INT) | **High** — already in use | dedicated single+double-tap engines, threshold/shock/quiet/dur regs, INT1/INT2 |
| Static wrist/forearm poses (gravity) | Accel gravity vector / on-chip **6D orientation** | **High** | per-axis face data, no mag needed |
| Shake, tilt, lift, "hands up" (coarse) | Accel+gyro, ~100 Hz | **High** — >98% / 0.995 in studies | robust whole-wrist; needs null class |
| Flick/swipe, thumbs up/down, fist, okay, victory | Accel+gyro (+ optional mic fusion) | **Medium–High** — per-user ~80%, cross-user ~75% | pick distinct-motion verbs |
| Static finger/grip poses (extend, pinch-shape, grip) | Accelerometer (**tendon micro-movement**) | **Medium** — accel *beat* EMG here | gyro is useless for static holds |
| On-body taps (forearm/palm/back-of-hand), finger flicks/rubs/scratches/claps | **High-ODR accel bio-acoustics** (ViBand ~4 kHz) | **Medium** — needs high-ODR path we don't have yet | localizes on-body tap site |
| Snap / knock | Mic (airborne) OR high-ODR accel (contact) | **Medium** — mic noise-prone; contact-accel noise-immune | prefer contact-accel |
| Held-object / tool ID (toothbrush, drill) | High-ODR accel bio-acoustics | **Medium** (bonus) | distinct vibration signatures |
| Cursor / continuous pointing | Accel+gyro tilt (gravity-referenced) | **High** — already built | cf. Apple AssistiveTouch "Motion Pointer" |

---

## Modality cheat-sheet (which sensor earns its keep)

- **Accelerometer (low-rate):** gravity poses, tap impulses, and — surprisingly —
  **static finger poses via tendon micro-movement**. The workhorse.
- **Gyroscope:** dynamic/rotational gestures (flicks, twists, shake). **Useless
  for static poses** (no rotational velocity) — don't rely on it for holds.
- **On-chip engines (free, ~zero MCU):** tap/double-tap, 6D orientation, tilt,
  free-fall, wake-up, pedometer — all on the LSM6DSL with an INT line.
- **High-ODR accelerometer (~4 kHz) = the big unlock:** body-coupled vibration
  "mic" → on-body taps, finger flicks/rubs, object ID. **Noise-immune** (senses
  mechanical coupling, not airborne sound). Highest-leverage R&D direction;
  aligns with the deferred ViBand thread (`finding_surface_tap_not_dead_2026_06_08`).
- **PDM mic:** snaps/taps acoustically + voice (dictation path). ~7% over
  IMU-only in fusion — real but secondary (one study: only 2 of top-25 features
  were acoustic).

---

## Reliability / gotchas (now evidence-backed)

- **Tap-while-moving / segmentation is a *named* hard problem.** Onset detection
  in continuous IMU data needs real machinery (GestureKeeper used recurrence-
  quantification analysis, not a threshold). = our "2nd tap lost while sweeping"
  finding (`finding_tap_while_moving_2026_06_11`).
- **MUST train an explicit null / everyday-activity class** or false positives
  ruin it. The best real-world numbers (0.6 FP/hr, 0.995 balanced acc) all
  depended on a rest/negative class.
- **Similar-wrist-motion gestures get confused** (okay/victory/stop share a
  twist; minimal-motion taps blur). Choose verbs with *distinct gross motions*.
- **Per-user calibration matters:** cross-subject ~75% vs intra-subject >80%+;
  few-shot custom gestures hit ~87% at 5 shots. A short enrollment ritual pays off.
- **No-magnetometer caveat:** several high IMU accuracies (88–89%) used a
  magnetometer and/or 4–8 sensors. Our single 6-axis volar band targets the
  IMU-only / 2-sensor numbers (~75–95% w/ per-user tuning), not those ceilings.
- **Drift:** gyro-only orientation drifts (no mag for yaw) — why the cursor uses
  the gravity-locked `vert` signal. Gravity-referenced poses are drift-free;
  yaw-based verbs are not.

---

## The hard line — what PPG/EMG buys that we can't replicate

- **Subtle finger-pinch with ~no wrist motion (Apple "Double Tap"): PPG-dependent,
  full stop.** Fuses accel+gyro **with optical blood-flow disruption**, on a
  dedicated neural engine, gated to display-on for power. Without PPG we do **not**
  get this exact motionless pinch reliably. *(A pinch/clench that DOES move the
  wrist/tendons is partially recoverable via accel tendon-micro-movement or
  high-ODR bio-acoustics — it's the* motionless *pinch that's lost.)*
- **Fine individual-finger discrimination (ASL shapes, thumb-to-each-phalanx):**
  94–96% achievable — but only with **FingerPing-style *active* acoustics** (thumb-
  ring chirp emitter 20 Hz–6 kHz + multiple receivers). A passive IMU+mic band
  can't; needs extra worn hardware.
- **Pinch/grip *force*, intent during arm motion:** EMG/SNC territory (Mudra
  Band) — the excluded modality.
- **Degrades badly in bursty real use:** finger-level or minimal-motion gestures
  while the arm moves; anything needing held stillness a moving hand can't give;
  mic-based snap/tap in noisy rooms (use contact-accel instead).

---

## Roadmap implications

- **Safe high-confidence verb set (most already on-chip):** single/double/triple
  tap, orientation poses, tilt-cursor, shake/lift, a few distinct-motion
  flicks/swipes.
- **Worth R&D:** high-ODR accelerometer **bio-acoustics (ViBand path)** for
  on-body taps + finger flicks + noise-robust snap detection. This is the realistic
  way to expand the vocabulary without PPG.
- **Do NOT spec:** a *motionless* finger pinch, per-finger ASL discrimination, or
  grip-force — they need PPG / EMG / active-acoustic hardware we don't have.

---

## Sources (surfaced & verified; 3-vote adversarial)

- Apple Watch **Double Tap** — accel+gyro+**PPG** fusion on S9 neural engine
  (Apple docs + coverage). The proof the marquee wrist gesture is *not* IMU-only.
- Apple Watch **AssistiveTouch** — tap / double-tap / clench / double-clench +
  **Motion Pointer** tilt-cursor (Apple accessibility docs).
- **ViBand** — Laput, Xiao, Harrison; CMU Future Interfaces Group, **UIST 2016**.
  4 kHz accel bio-acoustics; on-body taps/flicks + object ID; noise-immune.
- **FingerPing** — Cheng Zhang et al., Georgia Tech, **CHI 2018**. *Active*
  acoustic (thumb-ring chirps + 4 receivers); 22 poses incl. ASL 1–10;
  93.77%/95.64%. Not achievable passively.
- **Siddiqui & Chan 2020**, *Multimodal hand gesture recognition using single IMU
  + acoustic at wrist*, **PLOS ONE** 15(1):e0227039. 13–14 gestures; +~7% from
  mic; cross-subject ~75%, intra >80%.
- **GestureKeeper** — IMU-only (accel+gyro), RQA onset detection + SVM, >98%.
- IMU-only cross-user gesture study — 95.7% cross-user, 0.6 FP/hr, few-shot
  custom gestures (1/3/5 shots → 55/83/87%).
- IMU-only shake/tilt vs everyday-activity — >98%, 0.995 balanced acc, explicit
  null class, small CNN feasible on Cortex-M-class MCU.
- Accel-vs-EMG static-pose study — accel-only 84±8% **beat** EMG 76±11% on 17
  poses; gyro poor at static; tendon micro-movement is the mechanism;
  magnetometer/8-sensor caveat noted.
- **LSM6DSL** datasheet / app notes — on-chip single+double tap (416 Hz/2g,
  INT1/INT2), 6D orientation, tilt, free-fall, wake-up, pedometer.
