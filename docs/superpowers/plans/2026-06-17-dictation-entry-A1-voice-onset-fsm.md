# Dictation Entry A.1 — POSE_EAR + Voiced-Onset FSM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`). Firmware is hardware-in-the-loop: code "verify" = clean `./build.sh` + host test; thresholds come from the Task 2 HW measurement; final behavior is the Task 6 HW verification (user). Spec: `docs/superpowers/specs/2026-06-17-dictation-entry-A1-voice-onset-fsm-design.md`. Build/flash + file map: `CLAUDE.md`. Scope discipline: build EXACTLY the spec — detect + log only, no audio stream, no HID, no regulator power-gate, no KWS/Cobra. Do not delete the tap counter/cadence/commit machinery.

**Goal:** Enter `MODE_DICTATION` (detect + log only) when `POSE_EAR` is held + still AND a voiced-onset is detected by a spectral (voiced-band energy, adaptive latched-floor, M-of-N count-in-window) check on the wrist PDM mic — and reconcile the vestigial pose model left by the air-mouse extraction.

**Architecture:** Apple-Watch "Raise to Speak" pattern. The IMU pose (`POSE_EAR`) gates *when* the mic runs; an in-window voiced-band spectral check (FFT on the existing `mic_vad` capture, reusing CMSIS-DSP) decides voice-onset against an ambient floor latched during the held-silent window; two-factor (pose AND voice) enters the mode.

**Tech Stack:** Zephyr DMIC (`mic_vad` from A.0), CMSIS-DSP `arm_rfft_fast_f32` (already enabled: `CONFIG_CMSIS_DSP_TRANSFORM=y`, used by `bio_acoustic`), the gravity/pose classifier (`gesture_poses`), the Mahony orientation filter (`orientation_get().at_rest`).

---

## File structure
- `src/mic_vad.h` / `src/mic_vad.cpp` — add voiced-band spectral feature (FFT), latched-floor + sustained-onset state machine, `mic_vad_voice_onset()`; extend the `[MIC]` log. (`mic_vad_rms.cpp` gains the pure band-sum helper.)
- `src/gesture_poses.{h,cpp}` — remove `POSE_AIR_MOUSE`; rename `POSE_DICTATION` → `POSE_EAR` (measured canonical, re-enabled); `POSE_SURFACE` unchanged.
- `src/gesture_thresholds.h` — drop `POSE_AIRMOUSE_*`; `POSE_DICTATION_*` → `POSE_EAR_*` (measured canonical); add `VAD_*` constants.
- `src/gesture_mode.{h,cpp}` — add `MODE_DICTATION`; remove the `POSE_AIR_MOUSE` commit-switch case (keep the rest of the tap machinery); add the `POSE_EAR`-held mic gate + voice-onset entry/exit.
- `tests/test_mic_vad.cpp` — extend with band-sum helper cases.

---

### Task 1: `mic_vad` voiced-band spectral feature + `[MIC]` log (measurement instrument)

**Files:** Modify `src/mic_vad.h`, `src/mic_vad_rms.cpp`, `src/mic_vad.cpp`, `tests/test_mic_vad.cpp`

- [ ] **Step 1: Add a failing host test for the pure band-sum helper**

In `tests/test_mic_vad.cpp`, add inside `main()` before the final `printf`:
```cpp
    /* Band-sum: sum of bins [lo,hi] inclusive, clamped to [0,n). */
    float e[8] = {0,1,2,3,4,5,6,7};
    CHECK(mic_vad_band_sum(e, 8, 2, 4) == 9.0f);      /* 2+3+4 */
    CHECK(mic_vad_band_sum(e, 8, 0, 100) == 28.0f);   /* hi clamped to 7 */
    CHECK(mic_vad_band_sum(e, 8, 5, 1) == 0.0f);      /* lo>hi -> 0 */
    CHECK(mic_vad_band_sum(e, 0, 0, 3) == 0.0f);      /* empty */
```

- [ ] **Step 2: Declare the helper in `src/mic_vad.h`**

After the `mic_vad_block_rms` declaration, add:
```cpp
/* Pure: sum of energy bins e[lo..hi] inclusive, indices clamped to [0,n).
 * Returns 0 if lo>hi or n==0. Exposed for host unit test. */
float mic_vad_band_sum(const float *e, size_t n, int lo, int hi);
```

- [ ] **Step 3: Run the host test — verify it fails (helper undefined)**

Run: `g++ -std=c++11 -Isrc tests/test_mic_vad.cpp src/mic_vad_rms.cpp -lm -o /tmp/mv 2>&1 | tail -3`
Expected: link/compile error — `mic_vad_band_sum` undefined.

- [ ] **Step 4: Implement the pure helper in `src/mic_vad_rms.cpp`**

Append (keep it Zephyr-free):
```cpp
float mic_vad_band_sum(const float *e, size_t n, int lo, int hi)
{
    if (n == 0 || lo > hi) return 0.0f;
    if (lo < 0) lo = 0;
    if (hi >= (int)n) hi = (int)n - 1;
    float acc = 0.0f;
    for (int i = lo; i <= hi; i++) acc += e[i];
    return acc;
}
```

- [ ] **Step 5: Run the host test — expect PASS**

Run: `g++ -std=c++11 -Isrc tests/test_mic_vad.cpp src/mic_vad_rms.cpp -lm -o /tmp/mv && /tmp/mv`
Expected: `ALL PASS`

- [ ] **Step 6: Add the FFT voiced-band feature to `src/mic_vad.cpp`**

After the existing `#include` lines add:
```cpp
#include <arm_math.h>
```
After the `MIC_SLAB_BLOCKS` defines add the FFT constants + buffers (file-scope statics — NOT on the 2 KB thread stack):
```cpp
/* Voiced-band spectral feature. 512-pt real FFT over the 320-sample block
 * (Hann-windowed, zero-padded). Bin width = 16000/512 = 31.25 Hz.
 * Voiced band 300-3000 Hz => bins 10..96 (300/31.25=9.6->10, 3000/31.25=96). */
#define MIC_FFT_N        512
#define MIC_VOICED_LO    10
#define MIC_VOICED_HI    96

static arm_rfft_fast_instance_f32 mic_fft;
static float mic_hann[MIC_BLOCK_SAMPLES];       /* 320-pt Hann window */
static float mic_fft_in[MIC_FFT_N];             /* windowed + zero-padded */
static float mic_fft_out[MIC_FFT_N];            /* packed real FFT output */
static float mic_bin_e[MIC_FFT_N / 2];          /* per-bin energy (re^2+im^2) */

/* Voiced-band energy of one int16 block: Hann-window, zero-pad to 512, rFFT,
 * sum |X|^2 over bins MIC_VOICED_LO..MIC_VOICED_HI. */
static float mic_voiced_energy(const int16_t *s, size_t n)
{
    size_t m = (n < MIC_BLOCK_SAMPLES) ? n : MIC_BLOCK_SAMPLES;
    for (size_t i = 0; i < m; i++)        mic_fft_in[i] = (float)s[i] * mic_hann[i];
    for (size_t i = m; i < MIC_FFT_N; i++) mic_fft_in[i] = 0.0f;

    arm_rfft_fast_f32(&mic_fft, mic_fft_in, mic_fft_out, 0 /* forward */);

    /* Packed format: out[0]=DC real, out[1]=Nyquist real, then re,im pairs. */
    mic_bin_e[0] = mic_fft_out[0] * mic_fft_out[0];
    for (int k = 1; k < MIC_FFT_N / 2; k++) {
        float re = mic_fft_out[2 * k];
        float im = mic_fft_out[2 * k + 1];
        mic_bin_e[k] = re * re + im * im;
    }
    return mic_vad_band_sum(mic_bin_e, MIC_FFT_N / 2, MIC_VOICED_LO, MIC_VOICED_HI);
}
```
In `mic_vad_init()`, after the `LOG_INF("[MIC] init ...")` line, initialize the FFT + window once:
```cpp
    arm_rfft_fast_init_f32(&mic_fft, MIC_FFT_N);
    for (int i = 0; i < (int)MIC_BLOCK_SAMPLES; i++) {
        mic_hann[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * i / (MIC_BLOCK_SAMPLES - 1)));
    }
```
(Add `#include <math.h>` if not already present — it is needed for `cosf`.)

- [ ] **Step 7: Extend the capture loop to compute + log the voiced-band feature**

In `mic_thread()`, inside the `while (atomic_get(&mic_running))` loop, replace the existing RMS/floor/log block with one that also computes voiced energy. Find:
```cpp
            float rms = mic_vad_block_rms((const int16_t *)buf, size / 2);
            k_mem_slab_free(&mic_slab, buf);   /* return the block */

            /* Slow floor tracker (ambient). */
            floor_rms = (floor_rms == 0.0f) ? rms : (0.98f * floor_rms + 0.02f * rms);
            float ratio = (floor_rms > 1.0f) ? (rms / floor_rms) : 0.0f;

            if ((++log_ctr % 5) == 0) {   /* ~10 Hz */
                LOG_INF("[MIC] rms=%d floor=%d ratio=%d.%02d",
                        (int)rms, (int)floor_rms,
                        (int)ratio, (int)((ratio - (int)ratio) * 100));
            }
```
and replace with:
```cpp
            const int16_t *pcm = (const int16_t *)buf;
            size_t nsamp = size / 2;
            float rms = mic_vad_block_rms(pcm, nsamp);
            float ve  = mic_voiced_energy(pcm, nsamp);
            k_mem_slab_free(&mic_slab, buf);   /* return the block */

            /* Measurement floors (live EMA -- A.1 Task 4 replaces with a latch). */
            floor_rms   = (floor_rms == 0.0f)   ? rms : (0.98f * floor_rms + 0.02f * rms);
            voiced_floor = (voiced_floor == 0.0f) ? ve : (0.98f * voiced_floor + 0.02f * ve);
            float vratio = (voiced_floor > 1.0f) ? (ve / voiced_floor) : 0.0f;

            if ((++log_ctr % 5) == 0) {   /* ~10 Hz */
                LOG_INF("[MIC] rms=%d voiced=%d vfloor=%d vratio=%d.%02d",
                        (int)rms, (int)ve, (int)voiced_floor,
                        (int)vratio, (int)((vratio - (int)vratio) * 100));
            }
```
Add the new accumulator next to `float floor_rms = 0.0f;` near the top of `mic_thread()`:
```cpp
    float voiced_floor = 0.0f;
```

- [ ] **Step 8: Build firmware**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors. (If `arm_rfft_fast_init_f32` is unresolved, confirm `CONFIG_CMSIS_DSP_TRANSFORM=y` in `prj.conf` — it is — and that `#include <arm_math.h>` matches `bio_acoustic.cpp`'s include.)

- [ ] **Step 9: Commit**
```bash
git add src/mic_vad.h src/mic_vad.cpp src/mic_vad_rms.cpp tests/test_mic_vad.cpp
git commit -m "$(printf 'feat(mic): voiced-band (300-3000Hz) spectral energy + [MIC] log\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 2: HW spectral measurement (user) — sets the VAD thresholds

**Files:** none (hardware-in-the-loop; user flashes + pastes logs).

- [ ] **Step 1: Flash + smoke**

`cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/` (double-tap RESET first). Press `m`; expect `[MIC] rms=… voiced=… vfloor=… vratio=…` at ~10 Hz.

- [ ] **Step 2: Re-run the three captures, watching `voiced`/`vratio`**

(a) Speak at the ear (held) vs silent; (b) speak hand-down; (c) still ambient. Paste each.

- [ ] **Step 3: Decide thresholds (recorded into Task 3's `gesture_thresholds.h`)**

From the pasted `voiced` numbers, choose: `VAD_VOICED_ABS_MIN` (absolute voiced-energy floor that speak-at-ear clears but still-ambient + hand-down do not), `VAD_VOICED_RATIO_K` (multiple over the latched floor, e.g. ~3–4), `VAD_ONSET_DWELL_MS` (~150 ms; how long sustained), `VAD_FLOOR_SAMPLE_MS` (~400 ms silent-window to latch the floor). **Gate:** if speak-at-ear voiced energy does not clearly separate from still-ambient + hand-down, stop and report — do not hardcode a guess; the spec's Tier-2 (Cobra) decision is triggered here.

---

### Task 3: Pose-model cleanup + `POSE_EAR`

**Files:** Modify `src/gesture_thresholds.h`, `src/gesture_poses.h`, `src/gesture_poses.cpp`, `src/gesture_mode.cpp`

- [ ] **Step 1: `gesture_thresholds.h` — replace the AIR_MOUSE + DICTATION pose constants**

Delete these four lines:
```c
#define POSE_AIRMOUSE_GX        1.0f
#define POSE_AIRMOUSE_GY        0.0f
#define POSE_AIRMOUSE_GZ        0.0f
#define POSE_AIRMOUSE_TOL       0.45f
```
Replace the `POSE_DICTATION_*` block:
```c
#define POSE_DICTATION_GX       0.92f
#define POSE_DICTATION_GY       0.39f
#define POSE_DICTATION_GZ       0.03f
#define POSE_DICTATION_TOL      2.0f
```
with the measured-and-normalized ear canonical (`g=(8.2,−4.6,2.6)`, |g|=9.755 → unit
`(0.841, −0.472, 0.266)`):
```c
/* POSE_EAR [USER][HOUSING]: phone-call/raise-to-ear pose. Measured 2026-06-17
 * (20 s held trace, g=(8.2,-4.6,2.6) m/s^2; normalized below). Dominant gx
 * separates it from a generic raise. TOL is PROVISIONAL -- tighten/loosen in the
 * Task 6 HW check so it arms ONLY when settled at the ear, not mid-raise. */
#define POSE_EAR_GX             0.841f
#define POSE_EAR_GY             (-0.472f)
#define POSE_EAR_GZ             0.266f
#define POSE_EAR_TOL            0.906f   /* cos(25 deg); PROVISIONAL, verify on HW */
```
Leave `POSE_SURFACE_*`, `POSE_ARM_WINDOW_MS`, `POSE_MATCH_THRESH` unchanged.

- [ ] **Step 2: `gesture_poses.h` — update the enum**

Replace:
```c
typedef enum {
    POSE_NONE = 0,
    POSE_AIR_MOUSE,     /* forearm raised forward, band volar facing
                         * screen (away from user's face) */
    POSE_DICTATION,     /* forearm raised + rotated, band volar facing
                         * user's mouth */
    POSE_SURFACE,       /* wrist horizontal, band volar facing up
                         * (palm-down rest) */
    POSE_COUNT
} pose_id_t;
```
with (air-mouse removed; dictation pose renamed to the posture it is):
```c
typedef enum {
    POSE_NONE = 0,
    POSE_EAR,           /* raise-to-ear (phone-call) pose; enters MODE_DICTATION
                         * via voice-onset (see gesture_mode). */
    POSE_SURFACE,       /* wrist horizontal, band volar facing up
                         * (palm-down rest). Dormant scaffolding. */
    POSE_COUNT
} pose_id_t;
```

- [ ] **Step 3: `gesture_poses.cpp` — update the canonical table**

Replace the `POSE_AIR_MOUSE` and `POSE_DICTATION` table entries (the two long comment blocks + their two struct rows, lines ~25–61) with a single `POSE_EAR` entry:
```cpp
    /* POSE_EAR: raise-to-ear (phone-call) pose. Canonical = the 2026-06-17
     * measured held-ear gravity, normalized (POSE_EAR_* in gesture_thresholds.h).
     * Tight tolerance so it does NOT match a generic forward raise (the old broad
     * AIR_MOUSE cone was removed). The MODE it enters (DICTATION) is gated by
     * voice-onset in gesture_mode, not by the pose alone. */
    { POSE_EAR,     POSE_EAR_GX, POSE_EAR_GY, POSE_EAR_GZ, POSE_EAR_TOL, "EAR" },
```
Leave the `POSE_SURFACE` entry unchanged. (The table is indexed by enum order; `POSE_EAR`=1, `POSE_SURFACE`=2 — order matches the enum.)

- [ ] **Step 4: `gesture_mode.cpp` — fix the commit switch (keep the tap machinery)**

In `multi_tap_commit_handler()` replace the `switch (armed)` body's `POSE_AIR_MOUSE` case
(the `POSE_AIR_MOUSE` enum no longer exists). Find:
```cpp
    switch (armed) {
    case POSE_AIR_MOUSE:
        LOG_INF("GESTURE: raised-pose + cadenced double-tap (no mode bound)");
        break;

    case POSE_SURFACE:
```
and replace the `case POSE_AIR_MOUSE: ... break;` with nothing (delete those 3 lines), leaving
`case POSE_SURFACE:` as the first case. Then change the `default:` to a benign, explanatory log
(it now also covers a stray double-tap while `POSE_EAR` is armed — `POSE_EAR` is voice-gated, not
tap-bound):
```cpp
    default:
        /* No tap-bound mode for this pose (POSE_EAR enters via voice, not taps).
         * The tap counter/cadence/commit machinery is kept intact + exercised,
         * unbound, ready to wire a future tap-based mode here. */
        LOG_INF("GESTURE: %s + cadenced double-tap (no tap-bound mode)",
                pose_name(armed));
        break;
```
Do NOT touch the multi-tap counter, cadence checks, `k_work` scheduling, or `on_chip_single_tap`.

- [ ] **Step 5: Build firmware**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors. (If the compiler flags an unhandled enum in a
`switch`, confirm only `POSE_NONE`/`POSE_EAR`/`POSE_SURFACE`/`POSE_COUNT` remain and that no other
file references `POSE_AIR_MOUSE` or `POSE_DICTATION`: `grep -rn "POSE_AIR_MOUSE\|POSE_DICTATION" src/` must return nothing.)

- [ ] **Step 6: Commit**
```bash
git add src/gesture_thresholds.h src/gesture_poses.h src/gesture_poses.cpp src/gesture_mode.cpp
git commit -m "$(printf 'refactor(pose): remove vestigial POSE_AIR_MOUSE, add measured POSE_EAR\n\nGeneric raised is the orientation layer; only POSE_EAR arms. Tap machinery\nkept intact + unbound (benign default log). POSE_SURFACE stays dormant.\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 4: `mic_vad` latched floor + voice-onset query

**Files:** Modify `src/mic_vad.h`, `src/mic_vad.cpp`, `src/gesture_thresholds.h`

- [ ] **Step 1: Add VAD thresholds to `gesture_thresholds.h`**

Provisional values from the Task 2 skin-mount measurement (2026-06-17). All work in `veM` units
(voiced energy ÷ 1e6, matching the `[MIC]` log). Ambient `veM` measured ≤ ~1030; speech syllable
peaks 3000–21000.
```c
/* --- Voice-onset VAD (dictation A.1) [HOUSING] -------------------------------
 * Mount-dependent (prototype skin-mount); EXPECTED TO MOVE when the housing is
 * built -- re-tune then. Discriminator (from T2): absolute voiced energy veM,
 * against a floor latched per pose-entry (adaptive across environments). Onset
 * is M-of-N (speech veM is bursty, so a consecutive-block dwell fails). */
#define VAD_VEM_ABS_MIN      1200.0f  /* veM quiet-room backstop threshold */
#define VAD_K                8.0f     /* x over the latched ambient floor (adaptive) */
#define VAD_ONSET_HITS       3        /* hot blocks needed... */
#define VAD_ONSET_WINDOW_MS  700      /* ...within this window -> onset */
#define VAD_FLOOR_SAMPLE_MS  500      /* silent window to latch the ambient floor */
```

- [ ] **Step 2: Declare the onset query in `src/mic_vad.h`**

After `mic_vad_stop()`:
```cpp
/* True once a sustained voiced-onset has been detected since the last
 * mic_vad_start(). Reading it CLEARS the latch (one-shot per session). */
bool mic_vad_voice_onset(void);
```

- [ ] **Step 3: Implement the latched-floor + onset state machine in `src/mic_vad.cpp`**

`mic_vad.cpp` does NOT include the thresholds yet — add near the top includes:
```cpp
#include "gesture_thresholds.h"
```
Add state + the onset flag near `static atomic_t mic_running`:
```cpp
static atomic_t mic_onset = ATOMIC_INIT(0);   /* latched voiced-onset, read-and-clear */

/* Floor-latch + M-of-N onset state, owned by the single mic thread. */
static bool    mic_floor_latched;
static float   mic_floor_vem;          /* frozen ambient floor (veM units) */
static float   mic_floor_sum;          /* accumulator during the latch window */
static int     mic_floor_n;
static int64_t mic_listen_start_ms;
static int64_t mic_hot_ts[VAD_ONSET_HITS];   /* ring of recent hot-block times */
static int     mic_hot_idx;
static int     mic_hot_count;
```
In `mic_vad_start()` reset the session:
```cpp
void mic_vad_start(void)
{
    if (atomic_get(&mic_running)) return;   /* idempotent: no re-init under the running thread */
    atomic_set(&mic_onset, 0);
    mic_floor_latched = false;
    mic_floor_vem = 0.0f;
    mic_floor_sum = 0.0f;
    mic_floor_n   = 0;
    mic_listen_start_ms = k_uptime_get();
    mic_hot_idx   = 0;
    mic_hot_count = 0;
    atomic_set(&mic_running, 1);
}
```
Add the query:
```cpp
bool mic_vad_voice_onset(void)
{
    return atomic_set(&mic_onset, 0) != 0;   /* read-and-clear */
}
```
In `mic_thread()`, REPLACE the current `veM`/`frac` compute-and-log block (added in Task 1's
instrument fix) — i.e. from `int veM ...` (or `float veM ...`) down through the `LOG_INF("[MIC] ...")`
— with the latch + M-of-N onset logic (keep the `[MIC]` log, extended with floor/latched):
```cpp
            float veM = ve / 1.0e6f;                       /* voiced energy, millions */
            int   frac = (te > 1.0f) ? (int)(1000.0f * ve / te) : 0;  /* diagnostic only */
            int64_t now = k_uptime_get();

            if (!mic_floor_latched) {
                /* Mean ambient veM over the silent window, then freeze it. */
                mic_floor_sum += veM;
                mic_floor_n++;
                if ((now - mic_listen_start_ms) >= VAD_FLOOR_SAMPLE_MS) {
                    mic_floor_vem = (mic_floor_n > 0) ? (mic_floor_sum / mic_floor_n) : veM;
                    mic_floor_latched = true;
                    LOG_INF("[MIC] floor latched veM=%d", (int)mic_floor_vem);
                }
            } else {
                float thresh = VAD_K * mic_floor_vem;
                if (thresh < VAD_VEM_ABS_MIN) thresh = VAD_VEM_ABS_MIN;  /* quiet-room backstop */
                if (veM >= thresh) {                       /* "hot" block */
                    mic_hot_ts[mic_hot_idx] = now;
                    mic_hot_idx = (mic_hot_idx + 1) % VAD_ONSET_HITS;
                    if (mic_hot_count < VAD_ONSET_HITS) mic_hot_count++;
                    /* After advancing, mic_hot_ts[mic_hot_idx] is the oldest of the
                     * last VAD_ONSET_HITS hot blocks. >= HITS hot within the window
                     * (gaps allowed -- speech veM is bursty) => onset. */
                    if (mic_hot_count == VAD_ONSET_HITS &&
                        (now - mic_hot_ts[mic_hot_idx]) <= VAD_ONSET_WINDOW_MS) {
                        atomic_set(&mic_onset, 1);
                        mic_hot_count = 0;                 /* re-arm; don't spam */
                    }
                }
            }

            if ((++log_ctr % 5) == 0) {   /* ~10 Hz */
                LOG_INF("[MIC] rms=%d veM=%d frac=%d floor=%d lat=%d",
                        (int)rms, (int)veM, frac, (int)mic_floor_vem,
                        (int)mic_floor_latched);
            }
```
Keep `mic_vad_block_rms`/`rms` (still public API + host test, and `rms` is still logged). Do not
re-introduce the removed `floor_rms`/`voiced_floor` accumulators.

- [ ] **Step 4: Build firmware**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors.

- [ ] **Step 5: Commit**
```bash
git add src/mic_vad.h src/mic_vad.cpp src/gesture_thresholds.h
git commit -m "$(printf 'feat(mic): latched-floor sustained voiced-onset + mic_vad_voice_onset()\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 5: `gesture_mode` — `MODE_DICTATION` + POSE_EAR-held mic gate + entry/exit

**Files:** Modify `src/gesture_mode.h`, `src/gesture_mode.cpp`

- [ ] **Step 1: Add `MODE_DICTATION` to the enum (`gesture_mode.h`)**

In the `GestureMode` enum, after `MODE_GESTURE_AMBIENT,` add:
```c
    MODE_DICTATION,     /* raise-to-ear pose + voice-onset; detect + log only
                         * (no audio stream / HID -- that is sub-project B). */
```

- [ ] **Step 2: Add the mode string (`gesture_mode.cpp`)**

In `_mode_str()` (the `switch` near line 299, `case MODE_IDLE: return "IDLE";`), add:
```cpp
    case MODE_DICTATION:        return "DICTATION";
```
(Keep any existing `MODE_GESTURE_AMBIENT` case; if the switch has a `default`, the new case still
makes the log explicit.)

- [ ] **Step 3: Add `mic_vad.h` include + the ear-gate state (`gesture_mode.cpp`)**

Add near the other includes (after `#include "bio_acoustic.h"`):
```cpp
#include "mic_vad.h"
```
Add file-scope state near the other statics (e.g. after `static atomic_t mode_atomic`):
```cpp
/* POSE_EAR-held mic gate (dictation). Independent of the tap pose-arm: the mic
 * stays on while the ear pose is held (not a one-shot arm window). */
static bool    ear_mic_on;
static int64_t ear_last_match_ms;   /* last time best pose == POSE_EAR */
#define EAR_EXIT_DWELL_MS  400      /* pose must be gone this long to exit */
```

- [ ] **Step 4: Implement the ear gate, called from `pose_fsm_update`**

`pose_fsm_update(gx,gy,gz)` already computes `best = pose_classify_best(...)` and reads
orientation elsewhere. Add a helper above `pose_fsm_update` and call it with `best`:
```cpp
/* Mic gate + dictation entry/exit, driven every accel sample. POSE_EAR held +
 * at_rest turns the mic on (and latches its floor); voice-onset while held enters
 * MODE_DICTATION (detect + log); the pose going away for EAR_EXIT_DWELL_MS turns
 * the mic off and exits. */
static void ear_gate_update(pose_id_t best)
{
    orientation_state_t ori;
    orientation_get(&ori);
    int64_t now = k_uptime_get();

    /* HOLD keys on the gravity pose alone (NOT at_rest): speaking is motion, so
     * at_rest flickers false while talking -- gating the hold on it would drop the
     * mic mid-speech. Exit = pose dropping (gravity leaves the cone), per spec §7.
     * at_rest is required only on the START edge (clean floor latch, spec §6). */
    bool ear_pose = (best == POSE_EAR);
    if (ear_pose) ear_last_match_ms = now;

    if (!ear_mic_on) {
        if (ear_pose && ori.at_rest) {
            ear_mic_on = true;
            mic_vad_start();          /* begins the floor-latch window */
            (void)mic_vad_voice_onset();  /* discard any stale onset (e.g. left by an 'm' probe) */
            LOG_INF("POSE_EAR held + still -> mic ON (listening)");
        }
        return;
    }

    /* Mic is on. Enter dictation on a sustained voiced-onset. */
    if (mic_vad_voice_onset() &&
        (GestureMode)atomic_get(&mode_atomic) != MODE_DICTATION) {
        _transition_to(MODE_DICTATION);   /* logs "Mode transition: ..." */
        LOG_INF("DICTATION entry: ear-pose + voice");
    }

    /* Exit when the ear pose has been gone long enough. */
    if ((now - ear_last_match_ms) > EAR_EXIT_DWELL_MS) {
        ear_mic_on = false;
        mic_vad_stop();
        if ((GestureMode)atomic_get(&mode_atomic) == MODE_DICTATION) {
            _transition_to(MODE_IDLE);
            LOG_INF("DICTATION exit: pose dropped");
        } else {
            LOG_INF("POSE_EAR released -> mic OFF");
        }
    }
}
```
Then in `pose_fsm_update`, immediately after the `best = pose_classify_best(...)` line (and after the
existing `if (best == POSE_SURFACE) best = POSE_NONE;` demotion), add:
```cpp
    ear_gate_update(best);
```
Names are exact for this file: the mode setter is `static void _transition_to(GestureMode)` (it
already logs `Mode transition: X -> Y` and re-evaluates the acq request — call it, don't
`atomic_set` directly), and the orientation type is `orientation_state_t` with a `bool at_rest`
field, read via `orientation_get(&ori)` (per `orientation.h`, same call `pose_fsm_update` already
uses for its trace logging).

- [ ] **Step 5: Reset the gate in `gesture_mode_init`**

In `gesture_mode_init()` (near `atomic_set(&mode_atomic, MODE_IDLE);`), add:
```cpp
    ear_mic_on = false;
    ear_last_match_ms = 0;
```

- [ ] **Step 6: Build firmware**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors.

- [ ] **Step 7: Commit**
```bash
git add src/gesture_mode.h src/gesture_mode.cpp
git commit -m "$(printf 'feat(gesture): MODE_DICTATION via POSE_EAR-held mic gate + voice-onset\n\nTwo-factor (pose AND voice) entry, detect + log only. Mic on while POSE_EAR\nheld + at_rest; voiced-onset enters; pose drop > exit dwell exits.\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 6: HW verification (user) — pose isolation + two-factor entry

**Files:** none (hardware-in-the-loop).

- [ ] **Step 1: Flash** — `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`.

- [ ] **Step 2: Pose isolation.** Raise the arm *forward* (old air-mouse posture) and hold — expect NO
`POSE_EAR held -> mic ON`. Raise toward the ear and settle — expect `POSE_EAR held -> mic ON` only
once settled (not mid-raise). If it arms on a generic raise, tighten `POSE_EAR_TOL` / add a `gx`
floor and rebuild (spec §4 verification).

- [ ] **Step 3: Two-factor entry.** In the ear pose, speak → expect `DICTATION entry: ear-pose + voice`.
Drop the pose → expect `DICTATION exit: pose dropped` + mic OFF.

- [ ] **Step 4: Negative checks (must NOT enter).** (a) Hold ear pose silently → mic ON but no entry;
(b) speak with hand down → no `POSE_EAR`, no entry; (c) raise through the ear position without
settling → no arm. Tune `VAD_*` (Task 4) / `EAR_EXIT_DWELL_MS` if any check fails.

---

## Done when
- `./build.sh` clean at each task; host test `ALL PASS`; `grep -rn "POSE_AIR_MOUSE\|POSE_DICTATION" src/` empty.
- Task 2 spectral numbers captured + `VAD_*` set from them (or Tier-2 triggered).
- Task 6: `POSE_EAR` arms only when settled at the ear; speaking-while-held logs `DICTATION entry`;
  pose drop logs exit; the three negative checks do not trigger; the tap counter/commit machinery is
  intact (a double-tap in the ear pose logs the benign "no tap-bound mode" line, not an ABORT).
- Out of scope confirmed absent: no audio stream, no HID, no regulator power-gate, no KWS/Cobra.
