# Dictation Entry A.0 — PDM-Mic Feasibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`). Firmware is hardware-in-the-loop: most "verify" steps are a clean `./build.sh`; the actual feasibility answer comes from the Task 6 hardware measurement (the user runs it). Spec: `docs/superpowers/specs/2026-06-17-dictation-entry-firmware-design.md` §3. Build/flash: `CLAUDE.md`.

**Goal:** Bring up the XIAO nRF52840 Sense onboard PDM mic in our firmware and log short-term RMS energy, so we can measure whether the wrist mic detects voice-onset at the ear-pose distinctly from ambient (the SNR make-or-break) — before building any VAD threshold or FSM.

**Architecture:** Enable the board's existing `&pdm0` node + its mic-power regulator via `app.overlay`; capture 16 kHz mono via Zephyr's DMIC API in a new isolated `mic_vad` module (mirrors how `bio_acoustic` is isolated — no dependency into the gesture FSM); log per-block RMS as `[MIC]`. A serial command toggles the probe so the mic isn't always-on.

**Tech Stack:** Zephyr DMIC API (`<zephyr/audio/dmic.h>`, `CONFIG_AUDIO_DMIC_NRFX_PDM`), nRF52840 PDM peripheral, MSM261D3526H1CPM mic (GPIO-enable on P1.10).

---

## File structure
- `app.overlay` — enable `&pdm0` + the `regulator-fixed` mic-power node (P1.10).
- `prj.conf` — DMIC Kconfig.
- `src/mic_vad.{h,cpp}` — NEW module: DMIC bring-up + capture thread + RMS + `[MIC]` log + `mic_vad_start()/stop()`. The pure `mic_vad_block_rms()` is host-testable.
- `tests/test_mic_vad.cpp` — host test for the pure RMS helper.
- `src/main.cpp` — `mic_vad_init()` at boot + a serial `m` command (free since mouse-test was removed) to toggle the probe.
- `CMakeLists.txt` — add `src/mic_vad.cpp`.

---

### Task 1: Enable the PDM mic (devicetree + Kconfig)

**Files:** Modify `app.overlay`, `prj.conf`

- [ ] **Step 1: Add the mic-power regulator + enable `&pdm0` to `app.overlay`**

Append (the `&pinctrl`/`&i2c1` blocks already there stay unchanged):
```dts
/* Onboard PDM mic (MSM261D3526H1CPM): power regulator on P1.10 + enable pdm0.
 * Board DTS already provides the pdm0 node + pdm0_default/sleep pinctrl. */
/ {
    msm261d3526hicpm-c-en {
        compatible = "regulator-fixed";
        regulator-name = "MSM261D3526HICPM-C-EN";
        enable-gpios = <&gpio1 10 (NRF_GPIO_DRIVE_S0H1 | GPIO_ACTIVE_HIGH)>;
        regulator-boot-on;
    };
};

&pdm0 {
    status = "okay";
    clock-source = "PCLK32M";
};
```
(`regulator-boot-on` powers the mic at boot — fine for A.0; A.1 will gate it dynamically. The board's `xiao_ble_nrf52840_sense.dts` already sets `pinctrl-0/1` for `&pdm0`, so we don't repeat pinctrl.)

- [ ] **Step 2: Add DMIC Kconfig to `prj.conf`**

Append:
```
# PDM microphone (dictation entry A.0)
CONFIG_AUDIO=y
CONFIG_AUDIO_DMIC=y
CONFIG_AUDIO_DMIC_NRFX_PDM=y
```

- [ ] **Step 3: Verify the build picks up the DMIC driver + enabled node**

Run: `./build.sh -p 2>&1 | tail -5`
Expected: `==> Flashable artifact:` line, no errors. (Pristine `-p` because devicetree/Kconfig changed.) If you see "Cannot find suitable PDM clock configuration," the rate/clock pairing is wrong — we use 16 kHz in Task 2, which pairs with the driver's ~1.28 MHz PDM clock; do not change the rate.

- [ ] **Step 4: Commit**
```bash
git add app.overlay prj.conf
git commit -m "$(printf 'feat(mic): enable onboard PDM mic (pdm0 + P1.10 regulator) + DMIC Kconfig\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 2: `mic_vad` module — pure RMS helper first (host-tested)

**Files:** Create `src/mic_vad.h`, `src/mic_vad.cpp`, `tests/test_mic_vad.cpp`

- [ ] **Step 1: Write the failing host test for the RMS helper**

Create `tests/test_mic_vad.cpp`:
```cpp
#include "mic_vad.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

static int failures = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while(0)

int main(void) {
    /* All-zero block -> RMS 0. */
    int16_t z[64] = {0};
    CHECK(mic_vad_block_rms(z, 64) == 0.0f);

    /* Constant +1000 -> RMS 1000. */
    int16_t c[100];
    for (int i = 0; i < 100; i++) c[i] = 1000;
    CHECK(fabsf(mic_vad_block_rms(c, 100) - 1000.0f) < 1e-3f);

    /* Full-scale square wave +/-10000 -> RMS 10000. */
    int16_t sq[100];
    for (int i = 0; i < 100; i++) sq[i] = (i % 2) ? 10000 : -10000;
    CHECK(fabsf(mic_vad_block_rms(sq, 100) - 10000.0f) < 1e-3f);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 2: Create `src/mic_vad.h` with the public interface**

```cpp
#ifndef MIC_VAD_H
#define MIC_VAD_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A.0: PDM mic feasibility probe. mic_vad_init() at boot; start/stop toggled
 * from the serial console. While started, the capture thread logs per-block
 * RMS energy as [MIC]. (A.1 will add voice-onset + the pose-gated power enable.) */
void  mic_vad_init(void);
void  mic_vad_start(void);
void  mic_vad_stop(void);

/* Pure: RMS of a 16-bit PCM block. Exposed for host unit test. */
float mic_vad_block_rms(const int16_t *samples, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* MIC_VAD_H */
```

- [ ] **Step 3: Run the host test to verify it fails (no implementation yet)**

Run: `g++ -std=c++11 -Isrc tests/test_mic_vad.cpp src/mic_vad_rms.cpp -lm -o /tmp/mv 2>&1 | tail -3`
Expected: error — `src/mic_vad_rms.cpp` does not exist yet (cannot open input file).

- [ ] **Step 4: Create the pure RMS helper `src/mic_vad_rms.cpp` (makes the test pass)**

Keep this file free of Zephyr headers so it compiles on the host. The Zephyr capture TU (Step 6) will call this same symbol:
```cpp
#include "mic_vad.h"
#include <math.h>

float mic_vad_block_rms(const int16_t *s, size_t n)
{
    if (n == 0) return 0.0f;
    double acc = 0.0;
    for (size_t i = 0; i < n; i++) { double v = (double)s[i]; acc += v * v; }
    return (float)sqrt(acc / (double)n);
}
```

- [ ] **Step 5: Run the host test — expect PASS**

Run: `g++ -std=c++11 -Isrc tests/test_mic_vad.cpp src/mic_vad_rms.cpp -lm -o /tmp/mv && /tmp/mv`
Expected: `ALL PASS`

- [ ] **Step 6: Create `src/mic_vad.cpp` (DMIC capture + `[MIC]` log; calls the helper)**

`mic_vad_block_rms` is defined in `src/mic_vad_rms.cpp` — do NOT redefine it here, just call it.
```cpp
#include "mic_vad.h"

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(mic_vad, LOG_LEVEL_INF);

/* 16 kHz mono, ~20 ms blocks (320 samples, 640 bytes). Known-good rate (pairs
 * with the driver's ~1.28 MHz PDM clock). */
#define MIC_PCM_RATE_HZ    16000
#define MIC_BLOCK_SAMPLES  (MIC_PCM_RATE_HZ * 20 / 1000)   /* 320 */
#define MIC_BLOCK_BYTES    (MIC_BLOCK_SAMPLES * 2)          /* 640 */
#define MIC_SLAB_BLOCKS    4

K_MEM_SLAB_DEFINE(mic_slab, MIC_BLOCK_BYTES, MIC_SLAB_BLOCKS, 4);

static const struct device *mic_dev;
static atomic_t mic_running = ATOMIC_INIT(0);

static struct dmic_cfg make_cfg(struct pcm_stream_cfg *stream)
{
    stream->pcm_width = 16;
    stream->pcm_rate  = MIC_PCM_RATE_HZ;
    stream->block_size = MIC_BLOCK_BYTES;
    stream->mem_slab   = &mic_slab;

    struct dmic_cfg cfg = {};
    cfg.io.min_pdm_clk_freq = 1000000;
    cfg.io.max_pdm_clk_freq = 3500000;
    cfg.io.min_pdm_clk_dc   = 40;
    cfg.io.max_pdm_clk_dc   = 60;
    cfg.streams = stream;
    cfg.channel.req_num_streams = 1;
    cfg.channel.req_num_chan    = 1;
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
    return cfg;
}

static void mic_thread(void *, void *, void *)
{
    if (!device_is_ready(mic_dev)) {
        LOG_ERR("[MIC] PDM device not ready");
        return;
    }
    struct pcm_stream_cfg stream;
    struct dmic_cfg cfg = make_cfg(&stream);
    if (dmic_configure(mic_dev, &cfg) < 0) {
        LOG_ERR("[MIC] dmic_configure failed");
        return;
    }

    float floor_rms = 0.0f;        /* slow ambient-floor estimate */
    int   log_ctr = 0;

    for (;;) {
        if (!atomic_get(&mic_running)) { k_msleep(50); continue; }

        if (dmic_trigger(mic_dev, DMIC_TRIGGER_START) < 0) {
            LOG_ERR("[MIC] trigger START failed"); k_msleep(200); continue;
        }
        LOG_INF("[MIC] capture started (16 kHz mono)");

        while (atomic_get(&mic_running)) {
            void *buf; size_t size;
            int rc = dmic_read(mic_dev, 0, &buf, &size, 1000);
            if (rc < 0) { LOG_ERR("[MIC] read err %d", rc); break; }

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
        }
        dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
        LOG_INF("[MIC] capture stopped");
    }
}

K_THREAD_DEFINE(mic_thread_id, 2048, mic_thread, NULL, NULL, NULL, 6, 0, 0);

void mic_vad_init(void)
{
    mic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0));
    LOG_INF("[MIC] init (pdm0 %s)", device_is_ready(mic_dev) ? "ready" : "NOT READY");
}

void mic_vad_start(void) { atomic_set(&mic_running, 1); }
void mic_vad_stop(void)  { atomic_set(&mic_running, 0); }
```

(The host test never links this TU — it pulls in Zephyr headers and won't compile on the host. The host test links only `mic_vad_rms.cpp`, the pure helper from Step 4.)

- [ ] **Step 7: Add both sources to `CMakeLists.txt`**

After the existing `src/...cpp` lines, add:
```
    src/mic_vad.cpp
    src/mic_vad_rms.cpp
```

- [ ] **Step 8: Verify firmware build**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors.

- [ ] **Step 9: Commit**
```bash
git add src/mic_vad.h src/mic_vad.cpp src/mic_vad_rms.cpp tests/test_mic_vad.cpp CMakeLists.txt
git commit -m "$(printf 'feat(mic): mic_vad module -- DMIC 16kHz capture + RMS energy [MIC] log\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 3: Wire the serial probe toggle in `main.cpp`

**Files:** Modify `src/main.cpp`

- [ ] **Step 1: Include + init at boot**

Add near the other includes in `src/main.cpp`:
```cpp
#include "mic_vad.h"
```
In the boot/init path (where other subsystems init, e.g. near `gesture_mode_init()` / `bio_acoustic_init()`), add:
```cpp
    mic_vad_init();
```

- [ ] **Step 2: Add the `m` serial command (input-arm + dispatch)**

`'m'` is free (the mouse-test mode that used it was removed in the air-mouse extraction). In the serial input-arm `if/else if` chain (where `c == 'u'`, `']'` etc. set `pending_cmd`), add:
```cpp
        } else if (c == 'm') {
            // Toggle the PDM mic feasibility probe ([MIC] RMS log).
            pending_cmd = 'm';
```
In the dispatch block (where `cmd == 'u'` etc. are handled), add:
```cpp
        } else if (cmd == 'm') {
            pending_cmd = 0;
            static bool mic_on = false;
            mic_on = !mic_on;
            if (mic_on) mic_vad_start(); else mic_vad_stop();
            LOG_INF("Mic probe %s", mic_on ? "ON ([MIC] rms logging)" : "OFF");
```
Add a help line near the other `LOG_INF("  '…'=…")` lines:
```cpp
    LOG_INF("  'm'=toggle PDM mic feasibility probe ([MIC] rms log)");
```

- [ ] **Step 3: Verify build**

Run: `./build.sh 2>&1 | tail -4`
Expected: `==> Flashable artifact:` line, no errors.

- [ ] **Step 4: Commit**
```bash
git add src/main.cpp
git commit -m "$(printf 'feat(mic): serial m command toggles the PDM mic feasibility probe\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>')"
```

---

### Task 4: Hardware smoke (the build actually captures audio)

**Files:** none (hardware-in-the-loop; the user flashes + reads serial).

- [ ] **Step 1: Flash**
```bash
cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/
```
(double-tap RESET first.)

- [ ] **Step 2: Confirm bring-up**

At boot expect `[MIC] init (pdm0 ready)`. Press `m` → expect `Mic probe ON` then `[MIC] capture started` and a stream of `[MIC] rms=… floor=… ratio=…` lines at ~10 Hz. Press `m` again → `capture stopped`. If you see `pdm0 NOT READY` or `dmic_configure failed`, stop — the devicetree/Kconfig (Task 1) is wrong; do not proceed to Task 5.

---

### Task 5: The feasibility measurement (the point of A.0)

**Files:** none (user runs; paste logs back for analysis).

- [ ] **Step 1: Voice-at-ear vs ambient.** `m` to start. Hold the phone-call/ear pose and speak normally for ~5 s; then stay silent ~5 s. Capture the `[MIC] rms`/`ratio` for speaking vs silent.
- [ ] **Step 2: Ear vs hand-down.** Speak the same way with the hand at the ear, then with the hand at your side. Compare `rms`.
- [ ] **Step 3: Ear-pose gravity signature.** `m` to stop the mic; send `v` (pose trace) and hold the ear pose ~10 s to capture gravity/pitch/roll for the future `POSE_EAR` canonical.
- [ ] **Step 4: Report.** Paste the three captures. **Decision gate:** if speaking-at-ear `rms` sits clearly above the ambient `floor` (a clear `ratio` gap), A.1 proceeds and these numbers seed the VAD threshold + `POSE_EAR` canonical. If it does NOT separate, stop and revisit the trigger design (spec §7) before building the FSM.

---

## Done when
- `./build.sh` clean; host test `ALL PASS`; `m` toggles a sane `[MIC]` RMS stream on hardware.
- Task 5 captures pasted, and the speak-vs-ambient separability is judged → A.1 thresholds decided OR the trigger is revisited.
- A.1 (the `POSE_EAR` + voice-onset FSM) is a SEPARATE plan, written after this measurement.
