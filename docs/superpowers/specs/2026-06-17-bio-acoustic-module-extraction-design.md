# Bio-Acoustic Module Extraction + Dead-Code Removal — Design Spec

**Date:** 2026-06-17
**Status:** approved (scope confirmed); spec for the implementer
**Branch:** beta

---

## 1. Goal

Two foundation code-health changes, behavior-preserving:
1. **Remove dead `gesture_mode_set()`** — 0 callers since the air-mouse extraction
   (its only user was the removed mouse-test mode). Delete the declaration
   (`gesture_mode.h`) + definition (`gesture_mode.cpp`).
2. **Extract the bio-acoustic DSP subsystem** out of `gesture_mode.cpp` into a new
   pure-ish module **`src/bio_acoustic.{h,cpp}`**, so the FSM hub shrinks to a
   focused gesture state machine and the tap-feature DSP is independently
   understandable. No behavior change.

## 2. What moves into `bio_acoustic.{h,cpp}`

From `gesture_mode.cpp`, the self-contained bio-acoustic block (≈ the tap-capture /
FFT / feature-extraction / worker section, currently ~lines 883–1245):
- **State:** `tap_capture_buf`, `tap_capture_sample_count`, `tap_capture_sem`,
  `tap_capture_busy`, `last_tap_mid_band_energy`, the feature struct, FFT state
  (`fft_initialised` + the `arm_math` instance(s)), and the `TAP_CAPTURE_*` sizing.
- **Functions:** `_parse_sample`, `_abs16`, `_extract_peak`, `_compute_band_energies`,
  `surface_spectral_confirms_hard_surface`, `bio_acoustic_worker` (+ its
  `K_THREAD_DEFINE`), `gesture_mode_bio_acoustic_init`, `gesture_mode_bio_acoustic_on_tap`.
- **Includes it needs:** `<arm_math.h>`, `<arm_const_structs.h>`, `power_ctrl.h`
  (for the `lsm6dsl_fifo_*` API it already uses), `gesture_thresholds.h` (FFT bin
  edges, `SURFACE_RESONANCE_MID_BAND_THRESH`), Zephyr kernel/log/atomic, `<math.h>`.
  Remove these from `gesture_mode.cpp` IF nothing else there still needs them
  (verify — `arm_math` is likely bio-only; `power_ctrl.h` and `math.h` may still be
  used by the FSM, so check before deleting an include).

## 3. Interface (`bio_acoustic.h`)

Rename the public entry points to the module's own namespace:
- `void  bio_acoustic_init(void);`            (was `gesture_mode_bio_acoustic_init`)
- `void  bio_acoustic_on_tap(void);`          (was `gesture_mode_bio_acoustic_on_tap`)
- `bool  bio_acoustic_last_was_hard_surface(void);` (was the file-static
  `surface_spectral_confirms_hard_surface()` — `gesture_mode.cpp` reads it in one
  diagnostic log line, so expose it).

Keep `_parse_sample`/`_abs16`/`_extract_peak`/`_compute_band_energies`/the worker as
file-static **inside** `bio_acoustic.cpp` (not in the header).

## 4. Wiring changes

- **`gesture_mode.h`:** remove the `gesture_mode_bio_acoustic_init/on_tap`
  declarations and the dead `gesture_mode_set` declaration.
- **`gesture_mode.cpp`:** delete the moved block + `gesture_mode_set`; the one log
  line that called `surface_spectral_confirms_hard_surface()` now calls
  `bio_acoustic_last_was_hard_surface()` (add `#include "bio_acoustic.h"`); reset of
  `last_tap_mid_band_energy` in `gesture_mode_init` moves into `bio_acoustic_init`
  (or a reset there) — verify no other FSM code touches it.
- **`src/main.cpp`:** the INT1/boot callers of `gesture_mode_bio_acoustic_init()` /
  `gesture_mode_bio_acoustic_on_tap()` now call `bio_acoustic_init()` /
  `bio_acoustic_on_tap()` (add `#include "bio_acoustic.h"`).
- **`CMakeLists.txt`:** add `src/bio_acoustic.cpp`.

## 5. Constraints

- **Behavior identical** — this is a move + rename, not a redesign. Same FFT, same
  thresholds, same worker logic, same FIFO reads via `lsm6dsl_fifo_*`.
- The bio module must NOT reach back into gesture_mode FSM/pose/mode state. If the
  worker references any such state (it appears not to — it owns its buffers +
  `last_tap_mid_band_energy`), STOP and report; do not create a circular dependency.
- Keep the existing `[BIO]` log strings as-is.

## 6. Verification

- `./build.sh` clean (artifact line, no errors).
- `grep -n "gesture_mode_set\|gesture_mode_bio_acoustic\|surface_spectral_confirms" src/`
  → only the new `bio_acoustic.*` definitions + their callers via the new names;
  zero in `gesture_mode.{h,cpp}` except the `#include "bio_acoustic.h"` + the one
  `bio_acoustic_last_was_hard_surface()` call.
- `gesture_mode.cpp` shrinks by ~300 lines; `bio_acoustic.cpp` is the new ~300-line module.
- Boot-smoke (HW, user): a chip tap still logs `[BIO] features …`; HR + pose/tap unaffected.
