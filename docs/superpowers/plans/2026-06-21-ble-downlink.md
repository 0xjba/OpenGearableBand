# BLE Audio Downlink (phone → speaker) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Receive LC3 audio over BLE from the phone, decode it, and play it through the proven `audio_out` speaker engine — plus an instant barge-in flush — closing the two-way voice loop.

**Architecture:** Mirror of the uplink. A new Write-Without-Response BLE characteristic carries `[seq16][LC3]` frames; a new `audio_downlink` bridge (k_msgq + dedicated decode thread, decoupled from the BLE RX context) LC3-decodes and feeds `audio_out_write()`. A 1-byte control characteristic flushes on interruption. The device does only the light decode + play; the phone does all heavy lifting (codec/rate bridging, VAD, turn/interrupt logic).

**Tech Stack:** Zephyr/NCS, C/C++, liblc3 (`CONFIG_LIBLC3`), Zephyr GATT, the existing `audio_out` module. Host test sender in Python (bleak + liblc3 ctypes).

**Verification model:** No unit-test harness for the BLE/codec/thread firmware — each code task's gate is a **clean build** (`./build.sh`, expect `Flashable artifact: build/zephyr/zephyr.uf2`). Final task is hardware-in-the-loop via `tools/audio_tx.py`.

**Commits:** This repo commits **only when the user explicitly asks** (standing rule). Do NOT auto-commit; the commit command at the end is for when the user requests it.

---

## File Structure
- **Modify** `src/lc3_codec.h` / `src/lc3_codec.c` — add the LC3 decoder.
- **Modify** `src/audio_out.h` / `src/audio_out.cpp` — add `audio_out_flush()` (barge-in).
- **Create** `src/audio_downlink.h` / `src/audio_downlink.cpp` — the decode bridge.
- **Modify** `src/ble_audio.h` / `src/ble_audio.cpp` — add the downlink audio + control characteristics.
- **Modify** `CMakeLists.txt` — add `src/audio_downlink.cpp`.
- **Modify** `src/main.cpp` — `audio_downlink_init()` at boot.
- **Create** `tools/audio_tx.py` — host BLE sender (test harness).

---

## Task 1: LC3 decoder

**Files:** Modify `src/lc3_codec.h`, `src/lc3_codec.c`

- [ ] **Step 1: Declare the decode API** — in `src/lc3_codec.h`, after the encode declarations (near `lc3_codec_encode_pcm_buffer`):
```c
/* Decode one 10 ms frame: 40-byte LC3 in -> 160 int16 PCM out (16 kHz).
 * Returns 0 on success (incl. liblc3 PLC), negative errno on failure. */
int lc3_codec_decode_frame(const uint8_t *in, int16_t *pcm_out);
```

- [ ] **Step 2: Add the decoder to `src/lc3_codec.c`.** Read the file first to match its include + encoder pattern. Add the decoder static memory near the encoder's:
```c
/* LC3 decoder static memory (16 kHz, 10 ms) -- mirrors the encoder. */
static lc3_decoder_mem_16k_t lc3_decoder_mem;
static lc3_decoder_t          lc3_decoder;
```
Inside `lc3_codec_init()`, after the encoder is set up (and before the `return 0;`), add:
```c
	lc3_decoder = lc3_setup_decoder(LC3_FRAME_DURATION_US,
					LC3_SAMPLE_RATE_HZ, 0, &lc3_decoder_mem);
	if (lc3_decoder == NULL) {
		LOG_ERR("LC3 decoder setup failed");
		return -EIO;
	}
	LOG_INF("LC3 decoder ready (16 kHz, 10 ms, 40 B/frame -> 160 samples)");
```
Then add the decode function (next to the encode functions):
```c
int lc3_codec_decode_frame(const uint8_t *in, int16_t *pcm_out)
{
	if (lc3_decoder == NULL || in == NULL || pcm_out == NULL) {
		return -EINVAL;
	}
	/* lc3_decode: 0 = ok, 1 = PLC applied (still valid PCM), <0 = error. */
	int rc = lc3_decode(lc3_decoder, in, LC3_FRAME_BYTES,
			    LC3_PCM_FORMAT_S16, pcm_out, 1);
	return (rc < 0) ? -EIO : 0;
}
```
(`lc3_decoder_mem_16k_t`, `lc3_setup_decoder`, `lc3_decode`, `LC3_PCM_FORMAT_S16` are all from `<lc3.h>`, already included for the encoder.)

- [ ] **Step 3: Build.** Run `./build.sh`. Expected: `==> Flashable artifact: build/zephyr/zephyr.uf2`. (The decoder compiles + links; nothing calls it yet.) Fix compile errors against the actual liblc3 API in-tree if any (e.g. the `lc3_setup_decoder` arg list) before continuing.

---

## Task 2: audio_out flush (barge-in)

**Files:** Modify `src/audio_out.h`, `src/audio_out.cpp`

- [ ] **Step 1: Declare it** — in `src/audio_out.h`, after `audio_out_stop`:
```c
/* Barge-in flush: clear the ring AND stop the session immediately (drops both
 * the buffered ring audio and the queued I2S blocks -> instant silence). Use
 * this for interruption; plain stop() leaves the ring to be reset on next start. */
void audio_out_flush(void);
```

- [ ] **Step 2: Implement it** — in `src/audio_out.cpp`, after `audio_out_stop()`:
```c
void audio_out_flush(void)
{
	k_mutex_lock(&ring_mutex, K_FOREVER);
	ring_buf_reset(&pcm_ring);
	k_mutex_unlock(&ring_mutex);
	atomic_set(&active, 0);   /* feeder DROPs queued I2S + mutes amp */
}
```

- [ ] **Step 3: Build.** Run `./build.sh`. Expected: the artifact line. Fix any compile error.

---

## Task 3: `audio_downlink` bridge

**Files:** Create `src/audio_downlink.h`, `src/audio_downlink.cpp`; Modify `CMakeLists.txt`

- [ ] **Step 1: Create `src/audio_downlink.h`:**
```c
/*
 * audio_downlink -- BLE downlink bridge: raw LC3 from the phone -> decode -> audio_out.
 *
 * Mirror of audio_stream (uplink). The ble_audio downlink write callback hands LC3
 * payloads to audio_downlink_feed() (BLE RX context, must stay light); a dedicated
 * decode thread drains them, LC3-decodes, and calls audio_out_write(). The phone
 * does all heavy lifting; the device only does the <1 ms/frame decode + play.
 */
#ifndef AUDIO_DOWNLINK_H
#define AUDIO_DOWNLINK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot init (the decode thread auto-starts; LC3 decoder is inited by
 * lc3_codec_init via the uplink). Returns 0. */
int audio_downlink_init(void);

/* Feed one raw LC3 payload (1+ contiguous 40-byte frames) from the BLE write
 * callback. Non-blocking: copies into a queue, drops + counts on overflow. */
void audio_downlink_feed(const uint8_t *lc3, size_t len);

/* Barge-in: purge queued frames + flush audio_out (instant silence). */
void audio_downlink_flush(void);

/* Count of payloads dropped (queue full / oversized) since boot -- bring-up only. */
uint32_t audio_downlink_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_DOWNLINK_H */
```

- [ ] **Step 2: Create `src/audio_downlink.cpp`:**
```cpp
/*
 * audio_downlink -- see audio_downlink.h. Decouples the BLE RX context from the
 * LC3 decode + I2S play (the discipline that fixed sub-project B's slab starvation).
 */
#include "audio_downlink.h"
#include "audio_out.h"
#include "lc3_codec.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(audio_downlink, LOG_LEVEL_INF);

#define DL_RATE_HZ      16000                       /* [UNIT] matches LC3 16 kHz */
#define DL_MAX_PAYLOAD  240                         /* [STRUCTURAL] up to 6 x 40 B frames per write */
#define DL_MSGQ_DEPTH   16                          /* [STRUCTURAL] downlink jitter queue */
#define DL_THREAD_PRIO  7
#define DL_THREAD_STACK 2048

struct dl_item {
	uint16_t len;
	uint8_t  data[DL_MAX_PAYLOAD];
};

K_MSGQ_DEFINE(dl_msgq, sizeof(struct dl_item), DL_MSGQ_DEPTH, 4);

static uint32_t dl_drops;
static int16_t  dl_pcm[LC3_FRAME_SAMPLES];          /* 160 samples, decode scratch */

static void dl_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct dl_item item;

	while (1) {
		k_msgq_get(&dl_msgq, &item, K_FOREVER);

		/* Auto-start a session on the first frame after idle. */
		if (!audio_out_active()) {
			audio_out_start(DL_RATE_HZ);
		}

		for (uint16_t off = 0; off + LC3_FRAME_BYTES <= item.len;
		     off += LC3_FRAME_BYTES) {
			if (lc3_codec_decode_frame(item.data + off, dl_pcm) == 0) {
				audio_out_write(dl_pcm, LC3_FRAME_SAMPLES);
			}
		}
	}
}

K_THREAD_DEFINE(dl_thread_id, DL_THREAD_STACK, dl_thread, NULL, NULL, NULL,
		DL_THREAD_PRIO, 0, 0);

int audio_downlink_init(void)
{
	LOG_INF("audio_downlink ready");
	return 0;
}

void audio_downlink_feed(const uint8_t *lc3, size_t len)
{
	if (lc3 == NULL || len == 0 || len > DL_MAX_PAYLOAD) {
		dl_drops++;
		return;
	}
	struct dl_item item;
	item.len = (uint16_t)len;
	memcpy(item.data, lc3, len);
	if (k_msgq_put(&dl_msgq, &item, K_NO_WAIT) != 0) {
		dl_drops++;   /* queue full -- never block the BLE RX context */
	}
}

void audio_downlink_flush(void)
{
	k_msgq_purge(&dl_msgq);
	audio_out_flush();
	LOG_INF("downlink flush (barge-in)");
}

uint32_t audio_downlink_drop_count(void)
{
	return dl_drops;
}
```

- [ ] **Step 3: Add to `CMakeLists.txt`** — in the `target_sources(app PRIVATE ...)` list, after `src/audio_out.cpp`:
```cmake
    src/audio_downlink.cpp
```

- [ ] **Step 4: Build.** Run `./build.sh`. Expected: the artifact line. Fix compile/link errors.

---

## Task 4: BLE downlink + control characteristics

**Files:** Modify `src/ble_audio.h`, `src/ble_audio.cpp`

- [ ] **Step 1: Declare the control command** — in `src/ble_audio.h`, near the other defines:
```c
/* Downlink control characteristic commands (1 byte). */
#define BLE_AUDIO_CTRL_FLUSH 0x01   /* barge-in: stop + clear buffered audio */
```

- [ ] **Step 2: Add the characteristics to `src/ble_audio.cpp`.** Read the file to find the `BT_GATT_SERVICE_DEFINE(...)` block and its existing UUID defs. Add `#include "audio_downlink.h"` at the top. Add two new 128-bit UUIDs next to the existing ones:
```c
/* Downlink audio (phone -> device): 47A10003-9B70-4C2E-8A1D-2F6B9E4A77C1 */
static struct bt_uuid_128 ble_audio_dl_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10003, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));
/* Downlink control: 47A10004-9B70-4C2E-8A1D-2F6B9E4A77C1 */
static struct bt_uuid_128 ble_audio_ctrl_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10004, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));
```
Add the two write callbacks (above the `BT_GATT_SERVICE_DEFINE`):
```c
static ssize_t dl_audio_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset,
			      uint8_t flags)
{
	ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	const uint8_t *p = (const uint8_t *)buf;
	/* Wire format: [seq16 LE][LC3 frames]. Skip the seq, feed the LC3. */
	if (len > BLE_AUDIO_SEQ_HDR_SIZE) {
		audio_downlink_feed(p + BLE_AUDIO_SEQ_HDR_SIZE,
				    len - BLE_AUDIO_SEQ_HDR_SIZE);
	}
	return len;
}

static ssize_t dl_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset,
			     uint8_t flags)
{
	ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(offset); ARG_UNUSED(flags);
	if (len >= 1 && ((const uint8_t *)buf)[0] == BLE_AUDIO_CTRL_FLUSH) {
		audio_downlink_flush();
	}
	return len;
}
```
Inside the existing `BT_GATT_SERVICE_DEFINE(...)` list (after the uplink notify char + its CCC), add two characteristic entries:
```c
	/* Downlink audio: Write-Without-Response (unacked, high-throughput). */
	BT_GATT_CHARACTERISTIC(&ble_audio_dl_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL, dl_audio_write, NULL),
	/* Downlink control (FLUSH/barge-in). */
	BT_GATT_CHARACTERISTIC(&ble_audio_ctrl_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, dl_ctrl_write, NULL),
```
(NOTE: `BT_GATT_PERM_WRITE` is unencrypted for dev/test so `audio_tx.py` can write without bonding. PRODUCTION should use `BT_GATT_PERM_WRITE_ENCRYPT` — flag this `[HOUSING]`.)

- [ ] **Step 3: Build.** Run `./build.sh`. Expected: the artifact line. Fix compile/link errors.

---

## Task 5: Boot integration

**Files:** Modify `src/main.cpp`

- [ ] **Step 1: Include + init.** In `src/main.cpp`, add near the other audio includes:
```cpp
#include "audio_downlink.h"
```
Find the `audio_out_init()` call (added by the previous feature) and add right after it:
```cpp
    audio_downlink_init();
```

- [ ] **Step 2: Build.** Run `./build.sh`. Expected: the artifact line.

---

## Task 6: Host BLE test sender

**Files:** Create `tools/audio_tx.py`

- [ ] **Step 1: Write `tools/audio_tx.py`.** Read `tools/audio_rx.py` first — it already has the **bleak** connection boilerplate and the **liblc3 ctypes** binding (it *decodes*); this script *encodes* and *writes*, the inverse. Reuse its liblc3 setup (`lc3_setup_encoder`, `lc3_encode`, `lc3_encoder_size`) and device-discovery code. The script:
  1. Scans/connects to the device (same name/UUID filter as `audio_rx.py`).
  2. Reads a 16 kHz mono 16-bit WAV (arg; default `music.wav`), resampling/erroring if not 16 kHz.
  3. Encodes each 10 ms / 160-sample frame → 40 B LC3; packs two frames into an 80 B block, prepends a 2-byte LE sequence number → 82 B.
  4. Writes each block to the downlink char `47A10003-…` with **`write_gatt_char(..., response=False)`** (Write-Without-Response), paced at ~20 ms/block (`asyncio.sleep`).
  5. CLI flags: `--flush-after N` writes the 1-byte `0x01` FLUSH to the control char `47A10004-…` N seconds in (to test barge-in mid-stream).
  Mirror `audio_rx.py`'s structure/style; keep the LC3 params identical (16 kHz / 10 ms / 32 kbps / 40 B).

- [ ] **Step 2: Smoke-check it parses** (no device needed): `python3 tools/audio_tx.py --help` — expected: prints usage, no import/syntax error. (Requires `pip install bleak` + the local liblc3 as `audio_rx.py` documents.)

---

## Task 7: Hardware verification

**Files:** none (manual HW test).

- [ ] **Step 1: Build + flash.** `./build.sh` then double-tap RESET and `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`. Boot log should show `LC3 decoder ready` and `audio_downlink ready`, no errors.

- [ ] **Step 2: Stream from the Mac.** With the speaker wired and the device advertising, run `python3 tools/audio_tx.py music.wav`. Expected: it connects and you **hear the WAV on the speaker**, clean (the `audio_out` pipeline is proven). The serial log shows playback (and `audio_out session:` counters at the end).

- [ ] **Step 3: Test barge-in.** Run `python3 tools/audio_tx.py music.wav --flush-after 2`. Expected: playback **stops within a BLE round-trip** at ~2 s (the `downlink flush (barge-in)` log fires, the amp mutes), with no ghost audio.

- [ ] **Step 4: Coexistence.** Trigger `MODE_DICTATION` (mic uplink active) while streaming the downlink. Expected: both directions work; no `dmic_nrfx_pdm` slab errors, no underruns. Full-duplex confirmed.

- [ ] **Step 5: Report** results (audio clean? flush instant? coexistence OK? `audio_downlink_drop_count`/underruns?). If conversational latency feels high, note it — the `[STRUCTURAL]` jitter-buffer/prebuffer is the lever to tune (smaller = lower latency).

---

## Done / Next
The two-way voice loop is closed (device side). The phone app (codec/rate bridging, VAD, turn/interrupt logic — all per the "bare minimum on device" rule) is a separate project; its contract is in the spec. Latency tuning (prebuffer depth) is the likely follow-up after hearing it on hardware.

**Commit (only when the user asks):**
```bash
git add src/lc3_codec.h src/lc3_codec.c src/audio_out.h src/audio_out.cpp \
        src/audio_downlink.h src/audio_downlink.cpp src/ble_audio.h src/ble_audio.cpp \
        CMakeLists.txt src/main.cpp tools/audio_tx.py \
        docs/superpowers/specs/2026-06-21-ble-downlink-design.md \
        docs/superpowers/plans/2026-06-21-ble-downlink.md
git commit -m "feat(audio): BLE downlink (phone->speaker) -- LC3 decode + barge-in flush"
```
