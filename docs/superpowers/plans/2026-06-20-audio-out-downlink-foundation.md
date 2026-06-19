# Audio-Out (Speaker Downlink Foundation) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reusable `audio_out` speaker-output engine to the real firmware (I2S → MAX98357A), verifiable today with a serial test tone, designed so the future BLE downlink just calls `audio_out_write()`.

**Architecture:** Producer/consumer module mirroring the uplink (`mic_vad`/`audio_stream`/`ble_audio`). Callers push mono 16-bit PCM via `audio_out_write()` into a `ring_buf`; a dedicated feeder thread drains it, expands mono→stereo, and writes to the stock Zephyr `i2s_nrfx` driver. The amp's SD pin is held in shutdown except during a session.

**Tech Stack:** Zephyr (NCS v3.2.3), C/C++, stock `i2s_nrfx` driver, `ring_buf`, `k_mem_slab`, MAX98357A I2S amp.

**Verification model:** No unit-test harness (hardware I2S/threads). Each code task's gate is a **clean build** (`./build.sh`, expect `Flashable artifact: build/zephyr/zephyr.uf2`). Final task is hardware-in-the-loop.

**Commits:** This repo commits **only when the user explicitly asks** (standing rule). Do NOT auto-commit. The commit command at the end is provided for when the user requests it.

---

## File Structure

- **Modify** `app.overlay` — add `i2s0` pinctrl + node (D1/D0/D2), alongside existing i2c1/pdm0.
- **Modify** `prj.conf` — add `CONFIG_I2S=y`.
- **Create** `src/audio_out.h` — module API (4 functions).
- **Create** `src/audio_out.cpp` — I2S engine + feeder thread + ring + amp control.
- **Modify** `CMakeLists.txt` — add `src/audio_out.cpp`.
- **Modify** `src/main.cpp` — call `audio_out_init()` at boot; add `'p'` test-tone console command.

---

## Task 1: I2S pins + Kconfig

**Files:**
- Modify: `app.overlay`
- Modify: `prj.conf`

- [ ] **Step 1: Add the i2s0 pinctrl + node to `app.overlay`**

Append to the `&pinctrl { ... }` block (inside it, after the existing `i2c1_*` groups):

```dts
	i2s0_default_alt: i2s0_default_alt {
		group1 {
			psels = <NRF_PSEL(I2S_SCK_M, 0, 3)>,    /* BCLK -> D1/P0.03 */
				<NRF_PSEL(I2S_LRCK_M, 0, 2)>,   /* LRCK -> D0/P0.02 */
				<NRF_PSEL(I2S_SDOUT, 0, 28)>;   /* DIN  -> D2/P0.28 */
		};
	};
```

Then add this node at top level (e.g. after the `&i2c1 { ... };` block):

```dts
&i2s0 {
	status = "okay";
	pinctrl-0 = <&i2s0_default_alt>;
	pinctrl-names = "default";
};
```

- [ ] **Step 2: Enable I2S in `prj.conf`**

Add this line (group it with the other driver configs):

```
CONFIG_I2S=y
```

- [ ] **Step 3: Build**

Run: `./build.sh`
Expected: ends with `==> Flashable artifact: build/zephyr/zephyr.uf2`. (No new code references I2S yet; this just confirms the overlay + Kconfig are valid and the i2s0 node binds.)

---

## Task 2: The `audio_out` module

**Files:**
- Create: `src/audio_out.h`
- Create: `src/audio_out.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create `src/audio_out.h`**

```c
/*
 * audio_out -- speaker output engine (downlink path). I2S TX -> MAX98357A.
 *
 * Producer/consumer, mirroring the uplink split (mic_vad/audio_stream/ble_audio).
 * Callers push mono 16-bit PCM via audio_out_write(); a dedicated feeder thread
 * drains it to I2S, duplicating mono to L/R for the stereo frame. The amp's SD
 * pin is held in shutdown except between start() and stop(). The future BLE
 * downlink (LC3 decode) feeds the same audio_out_write() seam -- no API change.
 */
#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time boot init: bind I2S + amp GPIO, set amp muted, ready the feeder.
 * Returns 0 on success, -ENODEV if a device isn't ready. */
int audio_out_init(void);

/* Begin a playback session at sample_rate (Hz): enables the amp, configures and
 * starts I2S. Returns 0, or -EBUSY if a session is already active. */
int audio_out_start(uint32_t sample_rate);

/* Enqueue mono 16-bit PCM. Non-blocking; drops (logged) on ring overflow; no-op
 * if no session is active. This is the seam the BLE downlink feeds. */
void audio_out_write(const int16_t *mono_pcm, size_t nsamp);

/* End the session: the feeder stops I2S and mutes the amp. */
void audio_out_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_OUT_H */
```

- [ ] **Step 2: Create `src/audio_out.cpp`**

```cpp
/*
 * audio_out -- speaker output engine. See audio_out.h.
 *
 * All I2S/amp hardware access lives in the feeder thread; start()/stop() only
 * flip an atomic flag + signal a semaphore, and write() only fills the ring.
 * This keeps every i2s_* call on one thread (no cross-thread driver races) and
 * keeps the producer (BLE/test) decoupled from I2S timing -- the same discipline
 * that fixed the sub-project-B slab starvation.
 */
#include "audio_out.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_out, LOG_LEVEL_INF);

/* --- tunables (tag for housing/production retune) --- */
#define FRAMES_PER_BLOCK   256                          /* [STRUCTURAL] I2S block: latency vs jitter */
#define I2S_BLOCK_BYTES    (FRAMES_PER_BLOCK * 2 * 2)   /* stereo 16-bit */
#define NUM_BLOCKS         4                            /* [STRUCTURAL] mem-slab depth */
#define RING_BYTES         16384                        /* [STRUCTURAL] ~0.5 s @ 16 kHz mono jitter buffer */
#define IDLE_STOP_MS       400                          /* [STRUCTURAL] silence before auto-stop */
#define FEEDER_STACK       2048
#define FEEDER_PRIO        7                            /* same tier as the uplink audio thread */
#define AMP_SD_GPIO_PIN    12                           /* D7 / P1.12 (gpio1) */

K_MEM_SLAB_DEFINE(tx_slab, I2S_BLOCK_BYTES, NUM_BLOCKS, 4);
RING_BUF_DECLARE(pcm_ring, RING_BYTES);
K_MUTEX_DEFINE(ring_mutex);
K_SEM_DEFINE(run_sem, 0, 1);

static const struct device *i2s_dev;
static const struct device *amp_gpio;
static atomic_t active = ATOMIC_INIT(0);
static uint32_t session_rate = 16000;
static int16_t  mono_scratch[FRAMES_PER_BLOCK];

static int i2s_setup(uint32_t rate)
{
	struct i2s_config cfg = {
		.word_size      = 16,
		.channels       = 2,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = rate,
		.mem_slab       = &tx_slab,
		.block_size     = I2S_BLOCK_BYTES,
		.timeout        = 2000,
	};
	return i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
}

static inline void amp_enable(bool on)
{
	gpio_pin_set(amp_gpio, AMP_SD_GPIO_PIN, on ? 1 : 0);
}

/* Pull up to one block of mono samples from the ring, expand to stereo into
 * `block`, zero-padding any shortfall. Returns true if the block was ENTIRELY
 * silence (ring empty). */
static bool fill_block(int16_t *block)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	uint32_t got_bytes = ring_buf_get(&pcm_ring, (uint8_t *)mono_scratch,
					  FRAMES_PER_BLOCK * 2);
	k_mutex_unlock(&ring_mutex);

	size_t got = got_bytes / 2;
	for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
		int16_t s = (i < got) ? mono_scratch[i] : 0;
		block[2 * i]     = s;
		block[2 * i + 1] = s;
	}
	return got == 0;
}

static void feeder(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (1) {
		k_sem_take(&run_sem, K_FOREVER);    /* wait for a session */

		if (i2s_setup(session_rate) != 0) {
			LOG_ERR("i2s_configure failed");
			atomic_set(&active, 0);
			continue;
		}
		amp_enable(true);

		uint32_t block_ms = (FRAMES_PER_BLOCK * 1000u) / session_rate;
		if (block_ms == 0) {
			block_ms = 1;
		}
		uint32_t silence_ms = 0;
		int primed = 0;

		while (atomic_get(&active)) {
			void *block;

			if (k_mem_slab_alloc(&tx_slab, &block, K_MSEC(100)) != 0) {
				continue;
			}
			bool all_silence = fill_block((int16_t *)block);

			if (i2s_write(i2s_dev, block, I2S_BLOCK_BYTES) != 0) {
				k_mem_slab_free(&tx_slab, block);
				continue;
			}
			if (primed < 2 && ++primed == 2) {
				i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			}

			silence_ms = all_silence ? silence_ms + block_ms : 0;
			if (silence_ms >= IDLE_STOP_MS) {
				atomic_set(&active, 0);   /* end after a gap of silence */
			}
		}

		/* session end: stop + purge queued blocks + mute amp */
		i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
		amp_enable(false);
	}
}

K_THREAD_DEFINE(audio_out_tid, FEEDER_STACK, feeder, NULL, NULL, NULL,
		FEEDER_PRIO, 0, 0);

int audio_out_init(void)
{
	i2s_dev  = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	amp_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(i2s_dev) || !device_is_ready(amp_gpio)) {
		LOG_ERR("audio_out: device not ready");
		return -ENODEV;
	}
	/* amp SD low = MAX98357A shutdown (muted) until a session starts */
	gpio_pin_configure(amp_gpio, AMP_SD_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
	LOG_INF("audio_out ready");
	return 0;
}

int audio_out_start(uint32_t sample_rate)
{
	if (atomic_get(&active)) {
		return -EBUSY;
	}
	session_rate = sample_rate;

	k_mutex_lock(&ring_mutex, K_FOREVER);
	ring_buf_reset(&pcm_ring);
	k_mutex_unlock(&ring_mutex);

	atomic_set(&active, 1);
	k_sem_give(&run_sem);
	return 0;
}

void audio_out_write(const int16_t *mono_pcm, size_t nsamp)
{
	if (!atomic_get(&active) || mono_pcm == NULL || nsamp == 0) {
		return;
	}
	k_mutex_lock(&ring_mutex, K_FOREVER);
	uint32_t put = ring_buf_put(&pcm_ring, (const uint8_t *)mono_pcm,
				    nsamp * 2);
	k_mutex_unlock(&ring_mutex);

	if (put < nsamp * 2) {
		LOG_WRN("audio_out: ring overflow, dropped %u bytes",
			(unsigned)(nsamp * 2 - put));
	}
}

void audio_out_stop(void)
{
	atomic_set(&active, 0);   /* feeder stops I2S + mutes amp */
}
```

- [ ] **Step 3: Add the source to `CMakeLists.txt`**

In the `target_sources(app PRIVATE ...)` list, add `src/audio_out.cpp` (e.g. after `src/audio_stream.cpp`):

```cmake
    src/audio_out.cpp
```

- [ ] **Step 4: Build**

Run: `./build.sh`
Expected: `==> Flashable artifact: build/zephyr/zephyr.uf2`. The module compiles + links (it's compiled because it's in the source list, even though nothing calls it yet). Fix any compile errors before moving on.

---

## Task 3: Boot init + `'p'` test-tone console command

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 1: Include the header**

Near the other `#include` lines at the top of `src/main.cpp`, add:

```cpp
#include "audio_out.h"
```

Also ensure `#include <math.h>` is present (needed for `sinf` in Step 4); add it if it isn't.

- [ ] **Step 2: Init the module at boot**

Find where the uplink audio is initialised — search for `audio_stream_init(`. Immediately after that call, add:

```cpp
    audio_out_init();
```

(If `audio_stream_init()` is not called in `main()`, place `audio_out_init();` alongside the other subsystem init calls in `main()`, before the console loop.)

- [ ] **Step 3: Register the `'p'` key in the UART RX callback**

In `uart_rx_cb` (the `while (uart_fifo_read(...))` chain that sets `pending_cmd`), add a branch alongside the existing `'r'`/`'b'` handling:

```cpp
            } else if (c == 'p' || c == 'P') {
                pending_cmd = 'p';
```

- [ ] **Step 4: Handle `'p'` in the console command loop**

In the console command `while (1)` loop (the `if (cmd == 'r') { ... } else if (cmd == 'b') { ... }` chain), add another branch:

```cpp
        } else if (cmd == 'p') {
            pending_cmd = 0;
            LOG_INF("playing 440 Hz test tone via audio_out");
            audio_out_start(16000);
            const float two_pi = 6.2831853f;
            const float inc = two_pi * 440.0f / 16000.0f;
            float phase = 0.0f;
            for (int chunk = 0; chunk < 100; chunk++) {   // 100 * 10 ms = 1 s
                int16_t buf[160];
                for (int i = 0; i < 160; i++) {
                    buf[i] = (int16_t)(sinf(phase) * 8000.0f);
                    phase += inc;
                    if (phase >= two_pi) {
                        phase -= two_pi;
                    }
                }
                audio_out_write(buf, 160);
                k_msleep(10);   // pace ~realtime so the ring doesn't overflow
            }
            k_msleep(100);
            audio_out_stop();
```

- [ ] **Step 5: (Optional) advertise the command in the console banner**

If `main.cpp` logs a list of console commands at startup (search for `'b'=UF2 bootloader`), add a line like:

```cpp
    LOG_INF("  'p'=play 440 Hz test tone through the speaker");
```

- [ ] **Step 6: Build**

Run: `./build.sh`
Expected: `==> Flashable artifact: build/zephyr/zephyr.uf2`.

---

## Task 4: Hardware verification

**Files:** none (manual HW test).

- [ ] **Step 1: Flash**

Double-tap RESET on the Xiao, then:

```bash
cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/
```

- [ ] **Step 2: Confirm boot + amp muted**

Open the serial monitor. Expect `audio_out ready` in the boot log, no I2S errors. (Optional: the amp SD pin / D7 sits low at idle — multimeter ~0 V.)

- [ ] **Step 3: Play the tone**

Press `p` in the serial console. Expected: log `playing 440 Hz test tone via audio_out`, and a clean ~1 s 440 Hz tone from the speaker, then silence (amp returns to shutdown).

- [ ] **Step 4: Confirm full-duplex coexistence**

Trigger `MODE_DICTATION` (raise-to-ear + speak, so the mic/PDM capture is running), and while it's active press `p`. Expected: the tone plays AND the mic uplink + HR keep working — no `dmic_nrfx_pdm` slab errors, no gesture/HR disruption. This proves I2S output and PDM mic coexist (separate peripherals, dedicated feeder thread).

- [ ] **Step 5: Report results** to the user (tone clean? coexistence OK? any log warnings like `ring overflow`?). If `ring overflow` appears, the test-tone pacing or `RING_BYTES` needs a bump — note it but it's benign for the test.

---

## Done / Next

`audio_out` is in the real firmware, verified. **Step B** (BLE downlink: a receive characteristic + LC3 decode → `audio_out_write()`) is a separate plan and needs no change to this module's API.

**Commit (only when the user asks):**

```bash
git add app.overlay prj.conf src/audio_out.h src/audio_out.cpp CMakeLists.txt src/main.cpp docs/superpowers/specs/2026-06-20-audio-out-downlink-foundation-design.md docs/superpowers/plans/2026-06-20-audio-out-downlink-foundation.md
git commit -m "feat(audio): audio_out speaker engine + 'p' test tone (downlink foundation)"
```
