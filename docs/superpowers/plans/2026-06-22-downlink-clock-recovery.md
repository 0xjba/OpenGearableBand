# Downlink Clock Recovery (adaptive playout) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate gapless-stream crackle on the BLE downlink by adding a device buffer-level feedback channel and a host-side adaptive-playout control loop, so the sender keeps the `audio_out` ring near a setpoint instead of drifting into overflow/underrun.

**Architecture:** The device (nRF52840) only *reports* — a new NOTIFY characteristic pushes `[ring used][ring capacity][event flags]` every 100 ms while a downlink session plays. The host runs a slow PI controller on that feedback and nudges its send rate by ≤±1.5 % (must exceed the ~0.8 % I2S drift). This is the USB-Audio-async-feedback pattern. All control math is host-side ("bare minimum on device"). See `docs/superpowers/specs/2026-06-22-downlink-clock-recovery-design.md`.

**Tech Stack:** Zephyr v4.2.99 / NCS v3.2.3, C/C++ firmware (board `xiao_ble/nrf52840/sense`, build `./build.sh`); Python 3 + bleak host tool (`tools/audio_tx.py`). Firmware is hardware-in-the-loop (build clean + read serial); the host control law is pure-Python unit-tested.

**Build:** `./build.sh` → `build/zephyr/zephyr.uf2`. Flash: double-tap RESET, `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`. File layout + serial console in `CLAUDE.md`.

---

## File Structure

- **`src/audio_out.h` / `src/audio_out.cpp`** (modify) — add `audio_out_ring_capacity()`, `audio_out_take_event_flags()`, overflow/underrun event latches, and a `prebuf_ms` parameter on `audio_out_start()`.
- **`src/sd_card.cpp`, `src/main.cpp`, `src/audio_downlink.cpp`** (modify) — update the three `audio_out_start()` call sites for the new signature.
- **`src/ble_audio.h` / `src/ble_audio.cpp`** (modify) — add the status NOTIFY characteristic (`47A10005`), `ble_audio_status_subscribed()`, `ble_audio_notify_status()`.
- **`src/audio_downlink.cpp`** (modify) — own the periodic status reporter (`k_work_delayable`) that reads `audio_out` and calls `ble_audio_notify_status()` while a session plays; use the downlink prebuffer.
- **`tools/audio_tx.py`** (modify) — module-level pure control function `clock_recovery_step()`, subscribe to the status char, adaptive pacing (default on), `--no-adaptive` flag.
- **`tools/test_audio_tx.py`** (create) — pure-Python unit test for `clock_recovery_step()`.

Constants/signatures fixed once here, used consistently across tasks:
- `audio_out.h`: `#define AUDIO_OUT_EV_OVERFLOW 0x01`, `#define AUDIO_OUT_EV_UNDERRUN 0x02`; `int audio_out_start(uint32_t sample_rate, uint32_t prebuf_ms);`
- `ble_audio.h`: `#define BLE_AUDIO_STATUS_LEN 5`; `bool ble_audio_status_subscribed(void);`; `int ble_audio_notify_status(const uint8_t *payload, uint16_t len);`
- `audio_downlink.cpp`: `#define DL_PREBUF_MS 120`, `#define DL_STATUS_PERIOD_MS 100`
- Status payload (little-endian, 5 bytes): `[used_bytes u16][capacity_bytes u16][flags u8]`; `flags` bit0=active, bit1=overflowed-since-last, bit2=underran-since-last.
- `audio_tx.py`: `AUDIO_STATUS_UUID = "47a10005-9b70-4c2e-8a1d-2f6b9e4a77c1"`, `SETPOINT_MS = 120`.

---

## Task 1: `audio_out` event latches + capacity getter

**Files:**
- Modify: `src/audio_out.h` (after line 58, before the `#ifdef __cplusplus` close)
- Modify: `src/audio_out.cpp` (declares near line 57; overflow path line 269-273; feeder underrun path ~line 150)

- [ ] **Step 1: Declare the new public API in `audio_out.h`**

Insert after the `audio_out_get_volume()` declaration (current line 58):

```c
/* Total capacity of the playback ring, in bytes (the high-water for ring_used).
 * The downlink status reporter sends this so the host needs no hard-coded size. */
size_t audio_out_ring_capacity(void);

/* Buffer-event latch bits, set by audio_out and cleared on read. Lets the downlink
 * status reporter hand the host a hard over/underflow signal, not just inference. */
#define AUDIO_OUT_EV_OVERFLOW 0x01   /* a write was dropped (ring full) */
#define AUDIO_OUT_EV_UNDERRUN 0x02   /* the feeder pulled < a full block (ring low) */

/* Return the events latched since the last call, then clear them (read-and-clear). */
uint8_t audio_out_take_event_flags(void);
```

- [ ] **Step 2: Add the event-flag latch storage in `audio_out.cpp`**

After the diagnostic counters (current line 57, the `dbg_play_start_ms` line), add:

```c
/* Buffer-event latches for the downlink status reporter (read-and-clear). Set from
 * the writer (overflow) and feeder (underrun) threads -> atomic. */
static atomic_t event_flags = ATOMIC_INIT(0);
```

- [ ] **Step 3: Latch overflow in `audio_out_write()`**

In the overflow branch (current lines 269-273), add the `atomic_or` next to the existing counter:

```c
	if (put < nsamp * 2) {
		atomic_inc(&dbg_overflow);
		atomic_or(&event_flags, AUDIO_OUT_EV_OVERFLOW);
		LOG_WRN("audio_out: ring overflow, dropped %u bytes",
			(unsigned)(nsamp * 2 - put));
	}
```

- [ ] **Step 4: Latch underrun in the feeder**

In the feeder loop, the underrun branch currently reads:

```c
			if (got < FRAMES_PER_BLOCK) {
				dbg_underruns++;
				/* 0 < got < block = ring not yet empty but producer is
				 * behind -> this is the audible mid-stream crackle; got==0
				 * is a full gap / the post-stream drain tail (inaudible). */
				if (got > 0) {
					dbg_underruns_partial++;
				}
			}
```

Add the latch as the first line inside the `if`:

```c
			if (got < FRAMES_PER_BLOCK) {
				atomic_or(&event_flags, AUDIO_OUT_EV_UNDERRUN);
				dbg_underruns++;
				if (got > 0) {
					dbg_underruns_partial++;
				}
			}
```

- [ ] **Step 5: Implement the two getters in `audio_out.cpp`**

Add after `audio_out_ring_used()` (which ends at current line 267, just before `audio_out_active()`):

```c
size_t audio_out_ring_capacity(void)
{
	return RING_BYTES;
}

uint8_t audio_out_take_event_flags(void)
{
	return (uint8_t)atomic_clear(&event_flags);   /* returns prior value, zeroes it */
}
```

- [ ] **Step 6: Build and verify clean**

Run: `./build.sh`
Expected: ends with `Flashable artifact: build/zephyr/zephyr.uf2`, no errors/warnings about `audio_out`. FLASH/RAM % roughly unchanged from the prior build (~63 % / ~64 %).

- [ ] **Step 7: Commit**

```bash
git add src/audio_out.h src/audio_out.cpp
git commit -m "feat(audio_out): ring-capacity getter + over/underflow event latches"
```

---

## Task 2: Parameterize the playback prebuffer

The feeder prebuffers to `RING_BYTES/2` (≈256 ms) regardless of caller. The downlink wants ≈120 ms (= the control setpoint) for lower latency; SD/tone playback keeps ≈256 ms. Make it a per-session argument.

**Files:**
- Modify: `src/audio_out.h` (line 27 declaration)
- Modify: `src/audio_out.cpp` (define line 28; feeder prebuf check line 124; `audio_out_start` line 235)
- Modify: `src/sd_card.cpp:157`, `src/main.cpp:807`, `src/audio_downlink.cpp:58` (call sites)

- [ ] **Step 1: Change the `audio_out_start` declaration in `audio_out.h`**

Current line 25-27 comment + declaration:

```c
/* Begin a playback session at sample_rate (Hz): enables the amp, configures and
 * starts I2S. Returns 0, or -EBUSY if a session is already active. */
int audio_out_start(uint32_t sample_rate);
```

Replace with:

```c
/* Begin a playback session. sample_rate in Hz. prebuf_ms = how much audio to
 * buffer before playback starts (latency vs opening-underrun); the feeder yields
 * until the ring holds ~that much, capped by an internal time budget. Returns 0,
 * or -EBUSY if a session is already active. */
int audio_out_start(uint32_t sample_rate, uint32_t prebuf_ms);
```

- [ ] **Step 2: Add session prebuffer state + compute it in `audio_out.cpp`**

Replace the `PREBUF_BYTES` define (current line 28):

```c
#define PREBUF_BYTES       (RING_BYTES / 2)             /* [STRUCTURAL] fill ring ~half before starting playback */
```

with a floor constant (the static `RING_BUF_DECLARE` keeps `RING_BYTES`):

```c
#define PREBUF_FLOOR_BYTES (FRAMES_PER_BLOCK * 2)       /* [STRUCTURAL] min cushion: >=1 mono block */
```

Add session state near `session_rate` (current line 43):

```c
static uint32_t session_prebuf_bytes = RING_BYTES / 2;   /* set per session by audio_out_start */
```

In `audio_out_start` (current line 235), after `session_rate = sample_rate;` (line 244), add:

```c
	/* prebuf_ms -> bytes (mono 16-bit), clamped to [floor, ring - one block]. */
	uint32_t pb = (uint32_t)(((uint64_t)prebuf_ms * sample_rate * 2) / 1000);
	if (pb < PREBUF_FLOOR_BYTES) {
		pb = PREBUF_FLOOR_BYTES;
	}
	if (pb > RING_BYTES - I2S_BLOCK_BYTES / 2) {
		pb = RING_BYTES - I2S_BLOCK_BYTES / 2;
	}
	session_prebuf_bytes = pb;
```

- [ ] **Step 3: Use the session prebuffer in the feeder**

The feeder prebuffer check is currently (line 124):

```c
			if (ring_buf_space_get(&pcm_ring) <= (RING_BYTES - PREBUF_BYTES)) {
				break;
			}
```

Replace with:

```c
			if (ring_buf_space_get(&pcm_ring) <= (RING_BYTES - session_prebuf_bytes)) {
				break;
			}
```

- [ ] **Step 4: Update the three call sites**

`src/sd_card.cpp:157` — currently `rc = audio_out_start(w.rate);`. Replace with (preserve the old ~256 ms behaviour):

```c
    rc = audio_out_start(w.rate, 256);
```

`src/main.cpp:807` — currently `audio_out_start(SPK_TEST_RATE_HZ);`. Replace with:

```c
            audio_out_start(SPK_TEST_RATE_HZ, 256);
```

`src/audio_downlink.cpp:58` — currently `audio_out_start(DL_RATE_HZ);`. Replace with (the downlink low-latency value, defined in Task 4 but used here; add the define now at the top of the file with the other `DL_` constants):

```c
			audio_out_start(DL_RATE_HZ, DL_PREBUF_MS);
```

And add near the other `DL_` defines (after line 19 `DL_THREAD_PRIO`):

```c
#define DL_PREBUF_MS    120                         /* [STRUCTURAL] downlink jitter buffer = control setpoint */
```

- [ ] **Step 5: Build and verify clean**

Run: `./build.sh`
Expected: `Flashable artifact: build/zephyr/zephyr.uf2`, no errors (all three call sites updated, no "too few arguments to function 'audio_out_start'").

- [ ] **Step 6: Commit**

```bash
git add src/audio_out.h src/audio_out.cpp src/sd_card.cpp src/main.cpp src/audio_downlink.cpp
git commit -m "feat(audio_out): per-session prebuffer (downlink 120ms, SD/tone 256ms)"
```

---

## Task 3: `ble_audio` downlink status characteristic

**Files:**
- Modify: `src/ble_audio.h` (after line 56, the `ble_audio_drop_count` declaration)
- Modify: `src/ble_audio.cpp` (UUIDs near line 34; state line 38; CCC cb near line 50; service def lines 82-98; public API near line 143)

- [ ] **Step 1: Declare the status API in `ble_audio.h`**

Insert after the `ble_audio_drop_count()` declaration (current line 56):

```c
/* Downlink status report length: [ring_used u16 LE][ring_capacity u16 LE][flags u8].
 * flags bit0=session active, bit1=overflowed-since-last, bit2=underran-since-last. */
#define BLE_AUDIO_STATUS_LEN 5

/* True when a central has subscribed (CCC) to the downlink status characteristic.
 * The downlink reporter gates its notifications on this. */
bool ble_audio_status_subscribed(void);

/* Notify the downlink status characteristic. payload must be BLE_AUDIO_STATUS_LEN
 * bytes. Non-blocking; returns 0, -ENOTCONN if not subscribed, other negative on
 * error. Distinct from ble_audio_notify() (the uplink audio char). */
int ble_audio_notify_status(const uint8_t *payload, uint16_t len);
```

- [ ] **Step 2: Add the status UUID + subscription state in `ble_audio.cpp`**

After the control UUID (current lines 33-34, `ble_audio_ctrl_uuid`), add:

```c
/* Downlink status (device -> host buffer feedback): 47A10005-...77C1 */
static struct bt_uuid_128 ble_audio_status_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x47A10005, 0x9B70, 0x4C2E, 0x8A1D, 0x2F6B9E4A77C1));
```

After `static bool data_notifications_enabled;` (current line 38), add:

```c
static bool status_notifications_enabled;
```

- [ ] **Step 3: Add the status CCC-changed callback**

After `data_ccc_changed` (ends current line 50), add:

```c
static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	status_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("downlink status notifications %s",
		status_notifications_enabled ? "enabled" : "disabled");
}
```

- [ ] **Step 4: Append the status char + CCC to the service definition**

The service currently ends (lines 94-98):

```c
	/* Downlink control (FLUSH/barge-in). */
	BT_GATT_CHARACTERISTIC(&ble_audio_ctrl_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, dl_ctrl_write, NULL),
);
```

Replace with (adds the status NOTIFY char + CCC; updates the attr-index comment):

```c
	/* Downlink control (FLUSH/barge-in). */
	BT_GATT_CHARACTERISTIC(&ble_audio_ctrl_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE, NULL, dl_ctrl_write, NULL),
	/* Downlink status: device->host buffer feedback. attrs[8]=decl, attrs[9]=value
	 * (notify target), attrs[10]=CCC. */
	BT_GATT_CHARACTERISTIC(&ble_audio_status_uuid.uuid,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(status_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
```

- [ ] **Step 5: Implement the public API**

After `ble_audio_subscribed()` (ends current line 146), add:

```c
bool ble_audio_status_subscribed(void)
{
	return (audio_conn != NULL) && status_notifications_enabled;
}

int ble_audio_notify_status(const uint8_t *payload, uint16_t len)
{
	if (audio_conn == NULL || !status_notifications_enabled) {
		return -ENOTCONN;
	}
	if (payload == NULL || len != BLE_AUDIO_STATUS_LEN) {
		return -EINVAL;
	}
	/* attrs[9] = status characteristic value (see the service definition). */
	return bt_gatt_notify(audio_conn, &ble_audio_svc.attrs[9], payload, len);
}
```

Also clear the flag on disconnect: in `on_disconnected` (it currently sets `data_notifications_enabled = false;`), add alongside it:

```c
	status_notifications_enabled = false;
```

- [ ] **Step 6: Build and verify clean**

Run: `./build.sh`
Expected: `Flashable artifact: build/zephyr/zephyr.uf2`, no errors. (If the linker complains about `attrs[9]` being out of range, the characteristic order in the service definition differs from this plan — recount: service=0, audio decl=1/value=2, audio CCC=3, dl-audio decl=4/value=5, ctrl decl=6/value=7, status decl=8/value=9, status CCC=10.)

- [ ] **Step 7: Commit**

```bash
git add src/ble_audio.h src/ble_audio.cpp
git commit -m "feat(ble_audio): downlink status NOTIFY char (47A10005) + buffer-feedback API"
```

---

## Task 4: `audio_downlink` status reporter

Drives a periodic `k_work_delayable` that, while a session plays and the host is subscribed, builds the 5-byte status payload and notifies it. Lives here because `audio_downlink` already bridges `audio_out`↔`ble_audio` and owns session start/stop.

**Files:**
- Modify: `src/audio_downlink.cpp` (defines near line 16; new work + handler; arm it in `dl_thread` start path)

- [ ] **Step 1: Add the status-period define**

After the `DL_PREBUF_MS` define added in Task 2 (near line 19), add:

```c
#define DL_STATUS_PERIOD_MS 100                     /* [STRUCTURAL] buffer-feedback cadence */
```

- [ ] **Step 2: Add the reporter work + handler**

After the `dl_pcm` scratch declaration (current line 35), add the forward-declared work + handler:

```c
static void dl_status_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dl_status_work, dl_status_fn);

/* Periodic downlink buffer-feedback reporter. Runs only while a session plays
 * (self-stops when audio_out goes idle); notifies only when the host is subscribed
 * to the status char. Payload: [used u16 LE][capacity u16 LE][flags u8]. */
static void dl_status_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!audio_out_active()) {
		return;   /* session ended -> stop the periodic chain */
	}
	if (ble_audio_status_subscribed()) {
		uint16_t used = (uint16_t)audio_out_ring_used();
		uint16_t cap  = (uint16_t)audio_out_ring_capacity();
		uint8_t  ev   = audio_out_take_event_flags();   /* bits: overflow|underrun */
		uint8_t  flags = (uint8_t)(0x01 |               /* bit0 = active */
			((ev & AUDIO_OUT_EV_OVERFLOW) ? 0x02 : 0) |
			((ev & AUDIO_OUT_EV_UNDERRUN) ? 0x04 : 0));
		uint8_t pkt[BLE_AUDIO_STATUS_LEN] = {
			(uint8_t)(used & 0xFF), (uint8_t)(used >> 8),
			(uint8_t)(cap & 0xFF),  (uint8_t)(cap >> 8),
			flags,
		};
		ble_audio_notify_status(pkt, sizeof(pkt));
	}
	k_work_reschedule(&dl_status_work, K_MSEC(DL_STATUS_PERIOD_MS));
}
```

- [ ] **Step 3: Arm the reporter when a session starts**

In `dl_thread`, the session-start block currently reads (lines 57-59):

```c
		if (!audio_out_active()) {
			audio_out_start(DL_RATE_HZ, DL_PREBUF_MS);
		}
```

Replace with (schedule the reporter when we actually start a session; `k_work_schedule` is a no-op if already scheduled):

```c
		if (!audio_out_active()) {
			audio_out_start(DL_RATE_HZ, DL_PREBUF_MS);
			k_work_schedule(&dl_status_work, K_MSEC(DL_STATUS_PERIOD_MS));
		}
```

(No explicit stop needed: `dl_status_fn` self-terminates the chain on the first tick where `audio_out_active()` is false.)

- [ ] **Step 4: Build and verify clean**

Run: `./build.sh`
Expected: `Flashable artifact: build/zephyr/zephyr.uf2`, no errors. `audio_downlink.cpp` references `audio_out_ring_used/capacity/take_event_flags` (Task 1), `ble_audio_status_subscribed/notify_status` (Task 3) — all in scope via `audio_out.h` + `ble_audio.h` already included at the top of the file.

- [ ] **Step 5: HW smoke (optional but recommended)**

Flash; connect with nRF Connect; confirm the service now lists characteristic `47A10005` with the NOTIFY property and a CCC descriptor. (Full loop verification is Task 6.)

- [ ] **Step 6: Commit**

```bash
git add src/audio_downlink.cpp
git commit -m "feat(audio_downlink): periodic ring buffer-level status reporter"
```

---

## Task 5: Host adaptive control loop in `audio_tx.py` (TDD)

The control law is the one genuinely unit-testable unit — write the test first.

**Files:**
- Create: `tools/test_audio_tx.py`
- Modify: `tools/audio_tx.py` (constants near line 71; new pure function; argparse near line 130; subscribe + adaptive pacing in the stream loop lines 254-276)

- [ ] **Step 1: Write the failing unit test**

Create `tools/test_audio_tx.py`:

```python
#!/usr/bin/env python3
"""Unit tests for the audio_tx clock-recovery control law (pure, no BLE)."""
import audio_tx as tx


def test_clamps_to_pm_1_5_percent():
    # Huge positive error (ring nearly full) -> clamp at +1.5% (send slower).
    integ, scale = tx.clock_recovery_step(0.0, used=16000, capacity=16384,
                                          setpoint_bytes=3840, ev_overflow=0, ev_underrun=0)
    assert scale <= 1.015 + 1e-9
    # Huge negative error (ring empty) -> clamp at -1.5% (send faster).
    integ, scale = tx.clock_recovery_step(0.0, used=0, capacity=16384,
                                          setpoint_bytes=3840, ev_overflow=0, ev_underrun=0)
    assert scale >= 0.985 - 1e-9


def test_cancels_0_8pct_drift_without_overflow():
    # Simulate: consumer drains 0.8% faster than nominal; the loop must converge to
    # sending ~0.8% slower so the buffer stops growing, staying off the rails.
    cap, setp = 16384, 3840
    used = float(setp)
    integ = 0.0
    rate = 16000 * 2  # nominal bytes/s the sender would push at scale=1.0
    drain = rate * (1.0 - 0.008)  # I2S consumes 0.8% slower than nominal -> ring fills
    dt = 0.1
    max_used = used
    for _ in range(3000):  # 300 s simulated
        integ, scale = tx.clock_recovery_step(integ, int(used), cap, setp, 0, 0)
        produced = rate / scale * dt   # slower pace (scale>1) -> fewer bytes pushed
        used += produced - drain * dt
        used = max(0.0, min(used, cap))
        max_used = max(max_used, used)
    assert max_used < cap - 256, f"ring hit the rail: max_used={max_used}"
    assert abs(used - setp) < cap * 0.25, f"did not settle near setpoint: used={used}"


def test_flags_bias_direction():
    _, scale_of = tx.clock_recovery_step(0.0, used=3840, capacity=16384,
                                         setpoint_bytes=3840, ev_overflow=1, ev_underrun=0)
    assert scale_of > 1.0   # overflow -> send slower
    _, scale_uf = tx.clock_recovery_step(0.0, used=3840, capacity=16384,
                                         setpoint_bytes=3840, ev_overflow=0, ev_underrun=1)
    assert scale_uf < 1.0   # underrun -> send faster


def test_reset_zeroes_integral():
    assert tx.clock_recovery_reset() == 0.0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    print("all passed")
```

- [ ] **Step 2: Run it to confirm it fails (function not defined)**

Run: `cd tools && python3 test_audio_tx.py`
Expected: `AttributeError: module 'audio_tx' has no attribute 'clock_recovery_step'`.

- [ ] **Step 3: Add the pure control law + constants to `audio_tx.py`**

After the existing constants block (after `CTRL_FLUSH = 0x01`, current line 71), add:

```python
AUDIO_STATUS_UUID = "47a10005-9b70-4c2e-8a1d-2f6b9e4a77c1"
SETPOINT_MS = 120                       # target buffered audio (latency vs jitter)
SETPOINT_BYTES = SETPOINT_MS * SR_HZ * 2 // 1000   # 120ms @ 16k mono 16-bit = 3840
# Control-law gains (host clock recovery). Slow loop -> no oscillation; clamp must
# EXCEED worst-case I2S drift (~0.8%), so +/-1.5%. See the design doc.
CR_KP = 0.5            # proportional, on normalized error (used-setp)/capacity
CR_KI = 0.02           # integral (slow)
CR_CLAMP = 0.015       # +/-1.5% correction authority
CR_INTEG_MAX = CR_CLAMP / CR_KI        # anti-windup: integral alone can't exceed clamp
CR_FLAG_KICK = 0.10    # integral nudge on a hard over/underrun event


def clock_recovery_reset():
    """Reset controller state; call on barge-in flush / session restart. Returns the
    fresh integral (0.0) so callers can do `integ = clock_recovery_reset()`."""
    return 0.0


def clock_recovery_step(integ, used, capacity, setpoint_bytes, ev_overflow, ev_underrun):
    """Pure PI step. Returns (new_integ, pace_scale). pace_scale multiplies the
    inter-block send interval: >1 = send slower (ring too full), <1 = faster.
    The real app applies the same scale to its resample ratio instead."""
    err = (used - setpoint_bytes) / float(capacity)     # >0 = too full -> slow down
    integ += err
    if ev_overflow:
        integ += CR_FLAG_KICK       # too full -> bias slower
    if ev_underrun:
        integ -= CR_FLAG_KICK       # too empty -> bias faster
    integ = max(-CR_INTEG_MAX, min(CR_INTEG_MAX, integ))   # anti-windup
    corr = CR_KP * err + CR_KI * integ
    corr = max(-CR_CLAMP, min(CR_CLAMP, corr))
    return integ, 1.0 + corr
```

- [ ] **Step 4: Run the test to confirm it passes**

Run: `cd tools && python3 test_audio_tx.py`
Expected: four `PASS test_...` lines, then `all passed`.

- [ ] **Step 5: Wire the loop into `main()` — argparse flag**

In `main()` add the flag (after the `--address` argument, current line ~130 region):

```python
    ap.add_argument("--no-adaptive", action="store_true",
                    help="disable clock-recovery (don't subscribe, fixed pacing) -- "
                         "only to reproduce the drift baseline")
```

- [ ] **Step 6: Subscribe to status + run adaptive pacing in the stream loop**

The stream loop currently (lines 254-276) opens with fixed pacing. Replace the block from `t0 = time.time()` (line 254) through the end of the `for pkt in packets:` loop (line 276) with the adaptive version:

```python
        # Clock-recovery state. adaptive ON by default; --no-adaptive => fixed pace.
        adaptive = not args.no_adaptive
        cr_integ = clock_recovery_reset()
        pace_scale = 1.0

        if adaptive:
            def on_status(_char, data: bytearray):
                nonlocal cr_integ, pace_scale
                if len(data) != 5:
                    return
                used = data[0] | (data[1] << 8)
                cap = data[2] | (data[3] << 8)
                flags = data[4]
                cr_integ, pace_scale = clock_recovery_step(
                    cr_integ, used, cap or 1, SETPOINT_BYTES,
                    1 if flags & 0x02 else 0, 1 if flags & 0x04 else 0)
            await client.start_notify(AUDIO_STATUS_UUID, on_status)
            print("adaptive playout ON (subscribed to status 47A10005)")
        else:
            print("adaptive playout OFF (--no-adaptive baseline)")

        t0 = time.time()
        flush_at = args.flush_after if args.flush_after > 0 else None
        flushed = False
        sent = 0

        for pkt in packets:
            now = time.time()
            if flush_at is not None and not flushed and (now - t0) >= flush_at:
                await client.write_gatt_char(
                    ctrl_char, bytes([CTRL_FLUSH]), response=True)
                print(f"FLUSH sent at {now - t0:.2f}s (barge-in) -- stopping stream.")
                flushed = True
                break
            await client.write_gatt_char(dl_char, pkt, response=False)
            sent += 1
            # Pace to real time, scaled by the controller (>1 = slower).
            block_s = (BLOCK_MS / 1000.0) * pace_scale
            target = t0 + sent * block_s
            delay = target - time.time()
            if delay > 0:
                await asyncio.sleep(delay)
```

Note: `target = t0 + sent * block_s` uses the *current* `pace_scale`; since `pace_scale` changes slowly, recomputing the running target each iteration tracks it without jumps.

- [ ] **Step 7: Syntax-check + re-run the unit test**

Run: `python3 -m py_compile tools/audio_tx.py && cd tools && python3 test_audio_tx.py`
Expected: compiles silently; `all passed`.

- [ ] **Step 8: Commit**

```bash
git add tools/audio_tx.py tools/test_audio_tx.py
git commit -m "feat(audio_tx): host clock-recovery control loop (adaptive by default) + unit test"
```

---

## Task 6: Hardware acceptance test

No code — the headline verification that the loop actually removes the drift on hardware.

**Files:** none (uses `music16k_long.wav`; regenerate via the header in `tools/audio_tx.py` if absent).

- [ ] **Step 1: Flash the full build**

Run: `./build.sh` then double-tap RESET and `cp build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/`.

- [ ] **Step 2: Baseline (drift present) — control**

With a serial monitor open, run:
`python3 tools/audio_tx.py music16k_long.wav --lib tools/lib/liblc3.dylib --no-adaptive`
Expected serial at end: `audio_out session: ... NN overflow` with NN in the tens (the drift, as before ~39).

- [ ] **Step 3: Adaptive (the fix)**

Run:
`python3 tools/audio_tx.py music16k_long.wav --lib tools/lib/liblc3.dylib`
Expected:
- Host prints `adaptive playout ON (subscribed to status 47A10005)`.
- Audio plays ~60 s with no audible end-crackle.
- Serial `audio_out session:` shows **~0 overflow and ~0 partial/audible underrun** (vs the baseline's tens). `0 write-err`.

- [ ] **Step 4: Barge-in still works under adaptive**

Run:
`python3 tools/audio_tx.py music16k_long.wav --lib tools/lib/liblc3.dylib --flush-after 3`
Expected: audio cuts to silence at ~3 s and stays silent; serial shows `downlink flush (barge-in)`; no overflow storm.

- [ ] **Step 5: Record the result**

If pass, note the before/after `overflow` counts in the commit/PR description. If overflow is still non-trivial under adaptive, the `[STRUCTURAL]` gains (`CR_KP`/`CR_KI`) or `CR_CLAMP` need HW tuning per the design doc's Tunables — adjust in `audio_tx.py`, re-run Step 3.

---

## Notes for the implementer
- Build/flash/serial commands and the file map are in `CLAUDE.md`. Firmware is hardware-in-the-loop: the "tests" for Tasks 1–4 are a clean `./build.sh` plus the Task 6 serial check; only the Task 5 control law is unit-tested.
- Do not change the uplink (`audio_stream`/`ble_audio_notify`), gesture, or HR code.
- Keep every tuned constant tagged (`[STRUCTURAL]`/`[USER]`/`[UNIT]`) per the repo convention.
