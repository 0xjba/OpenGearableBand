"""Bidirectional BLE transport for the band's full-duplex audio (Phase C1).

Merges the two throwaway bring-up tools into one reusable client:
  * UPLINK  (device -> host, notify 47A10002): mic audio as [seq16][ts32][LC3];
    parsed to UplinkFrame and handed to the orchestrator's callback. (from audio_rx.py)
  * DOWNLINK(host -> device, write-no-resp 47A10003): AI audio as [seq16][LC3];
    sent by an internal PACED task that applies clock recovery so we feed the device
    at its true 15873 Hz consumption rate, not our nominal 16 kHz. (from audio_tx.py)
  * STATUS  (device -> host, notify 47A10005): [used16][cap16][flags8] drives the
    clock-recovery PI -> pace_scale. (from audio_tx.py)
  * CONTROL (host -> device, write 47A10004): 0x01 = FLUSH (barge-in stop). (from audio_tx.py)

THE TIMING AUTHORITY owns the playout reference: send_downlink() takes BOTH the LC3
block and its source PCM; the paced sender appends that PCM to a `played_ref` ring
ONLY WHEN THE BLOCK IS ACTUALLY TRANSMITTED. So the ring advances at playout rate
(not at the rate the AI produces audio, which is faster than real-time) -- the AEC
reference therefore stays time-aligned with the mic. The orchestrator reads it via
played_reference().

Real-time discipline: this class never blocks the event loop on DSP; it only does BLE
I/O + light byte-packing. AEC/VAD live in the orchestrator's DSP task.
"""
import asyncio
import struct
import sys

import numpy as np

from voiceio import clock_recovery as cr
from voiceio.frame import parse_uplink

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - environment guard
    BleakClient = BleakScanner = None

DEVICE_NAME = "Xiao_Pulse"
SVC_UUID = "47a10001-9b70-4c2e-8a1d-2f6b9e4a77c1"
UPLINK_UUID = "47a10002-9b70-4c2e-8a1d-2f6b9e4a77c1"   # device -> host mic notify
DL_UUID = "47a10003-9b70-4c2e-8a1d-2f6b9e4a77c1"       # host -> device audio
CTRL_UUID = "47a10004-9b70-4c2e-8a1d-2f6b9e4a77c1"     # host -> device control (FLUSH)
STATUS_UUID = "47a10005-9b70-4c2e-8a1d-2f6b9e4a77c1"   # device -> host buffer status

# LC3 framing (must match firmware lc3_codec.h). One downlink block = 2 frames = 20 ms.
FRAME_BYTES = 40
FRAME_SAMPLES = 160
FRAMES_PER_BLOCK = 2
BLOCK_SAMPLES = FRAMES_PER_BLOCK * FRAME_SAMPLES       # 320 samples = 20 ms
BLOCK_MS = BLOCK_SAMPLES * 1000 // cr.SR_HZ            # 20 ms
CTRL_FLUSH = 0x01


class BleLink:
    """Bidirectional BLE audio transport with a paced, clock-recovered downlink."""

    def __init__(self, device_name=DEVICE_NAME, address=None, setpoint_ms=cr.SETPOINT_MS):
        if BleakClient is None:
            sys.exit("Missing dependency: pip install bleak")
        self.device_name = device_name
        self.address = address
        self.setpoint = cr.setpoint_bytes(setpoint_ms)

        self._client = None
        self._dl_char = None
        self._ctrl_char = None

        # Downlink: queue of (lc3_block_bytes, pcm_float32) pairs; paced sender drains it.
        self._dl_q = asyncio.Queue()
        self._dl_seq = 0
        self._sender_task = None

        # Clock recovery (updated from STATUS notifications).
        self._cr_integ = cr.reset()
        self._pace_scale = 1.0

        # Played-reference ring: host-16k PCM appended as blocks are actually sent.
        # played_samples = cumulative count == index of the next sample to be played.
        self._played_ref = np.zeros(0, dtype=np.float32)
        self._played_samples = 0

    # --- connection ---

    async def connect(self):
        """Scan (by name or --address), connect, discover + validate chars, probe."""
        dev = await self._find_device()
        self._client = BleakClient(dev)
        await self._client.__aenter__()
        if not self._client.is_connected:
            sys.exit("Device dropped right after connecting (link not held).")
        # Service discovery can lag the connect (or land before GATT is resolved). Retry a
        # few times, re-reading the collection each pass, before deciding it's missing.
        for attempt in range(8):
            await asyncio.sleep(0.6)
            self._dl_char = self._client.services.get_characteristic(DL_UUID)
            self._ctrl_char = self._client.services.get_characteristic(CTRL_UUID)
            chars = list(self._client.services.characteristics.values())
            if self._dl_char is not None and self._ctrl_char is not None:
                break
            print(f"[ble] discovery attempt {attempt + 1}/8: {len(chars)} chars, "
                  f"downlink {'found' if self._dl_char else 'missing'} ...")
        if self._dl_char is None or self._ctrl_char is None:
            print("[ble] discovered characteristics:")
            for ch in list(self._client.services.characteristics.values()):
                print(f"      {ch.uuid}  ({','.join(ch.properties)})")
            sys.exit(
                "\n[ble] Downlink chars (47a10003/47a10004) NOT discovered -- macOS is serving\n"
                "a STALE GATT cache. You do NOT need to toggle Mac Bluetooth:\n"
                "  1. On the band's SERIAL console, press 'r' to REBOOT it (fresh GATT), then\n"
                "     re-run this. If it still fails, press 'u' (clear bonds) then 'r'.\n"
                "  2. Make sure no other app (nRF Connect, a prior voice_loop) holds the\n"
                "     connection -- the firmware accepts ONE connection at a time.")
        # Escalating write probe (localizes an L2CAP size failure; settles DLE).
        await self._probe()
        print(f"[ble] connected {dev.address}, MTU={self._client.mtu_size}, "
              f"{len(list(self._client.services.characteristics.values()))} chars")

    async def _find_device(self):
        if self.address:
            dev = await BleakScanner.find_device_by_address(self.address, timeout=15.0)
            if not dev:
                sys.exit(f"No device at address '{self.address}'.")
            return dev
        print(f"[ble] scanning for '{self.device_name}' ...")
        found = await BleakScanner.discover(timeout=15.0, return_adv=True)
        for d, adv in found.values():
            if self.device_name in (adv.local_name, d.name):
                return d
        sys.exit(f"Device '{self.device_name}' not found. If connected elsewhere, the "
                 "firmware accepts ONE connection; disconnect others and retry. If it "
                 "shows under a stale name, pass address=<addr>.")

    async def _probe(self):
        block0 = struct.pack("<H", 0) + bytes(FRAME_BYTES * FRAMES_PER_BLOCK)
        for name, char, payload in (
            ("ctrl-1B", self._ctrl_char, bytes([0x00])),       # no-op (handler ignores != 0x01)
            ("dl-1frame", self._dl_char, block0[:FRAME_BYTES + 2]),
            ("dl-fullblock", self._dl_char, block0),
        ):
            try:
                await self._client.write_gatt_char(char, payload, response=True)
            except Exception as e:  # noqa: BLE001
                sys.exit(f"[ble] probe {name} ({len(payload)}B) FAILED -> {e!r}")
            await asyncio.sleep(0.25)

    # --- start streaming ---

    async def start(self, on_uplink):
        """Subscribe uplink + status, start the paced downlink sender.

        on_uplink(frame: UplinkFrame) is called for each mic notification (on the
        event loop -- keep it light; offload DSP to a task)."""
        def _uplink_cb(_char, data: bytearray):
            frame = parse_uplink(bytes(data))
            if frame is not None:
                on_uplink(frame)

        def _status_cb(_char, data: bytearray):
            if len(data) != 5:
                return
            used = data[0] | (data[1] << 8)
            cap = (data[2] | (data[3] << 8)) or 1
            flags = data[4]
            self._cr_integ, self._pace_scale = cr.step(
                self._cr_integ, used, cap, self.setpoint,
                1 if flags & cr.STATUS_FL_OVERFLOW else 0,
                1 if flags & cr.STATUS_FL_UNDERRUN else 0)

        await self._client.start_notify(UPLINK_UUID, _uplink_cb)
        await self._client.start_notify(STATUS_UUID, _status_cb)
        self._sender_task = asyncio.create_task(self._downlink_sender())
        print("[ble] streaming (uplink + status subscribed, downlink sender up)")

    # --- downlink ---

    def send_downlink(self, lc3_block: bytes, pcm_block: np.ndarray):
        """Enqueue one 20 ms downlink block + its source PCM (for the AEC reference).
        Non-blocking; the paced sender transmits it at the device's consumption rate."""
        self._dl_q.put_nowait((lc3_block, np.asarray(pcm_block, dtype=np.float32)))

    def downlink_pending(self):
        """Blocks still queued to transmit. >0 means the AI is still being played out
        (the producer bursts a full reply in faster than real-time; the paced sender
        drains it over real time). The orchestrator uses this -- not the producer's
        per-iteration output -- to know whether the AI is currently 'playing'."""
        return self._dl_q.qsize()

    async def _downlink_sender(self):
        """Drain the downlink queue, paced to real-time * clock-recovery scale.

        Running-sum accumulator (NOT target=t0+n*block): a pace_scale change never
        retroactively reprices already-sent blocks. Appends each block's PCM to the
        played_ref ring at transmit time so the reference advances at playout rate."""
        loop = asyncio.get_running_loop()
        planned_t = None
        while True:
            lc3_block, pcm_block = await self._dl_q.get()
            if planned_t is None:
                planned_t = loop.time()
            try:
                await self._client.write_gatt_char(self._dl_char,
                                                   struct.pack("<H", self._dl_seq & 0xFFFF) + lc3_block,
                                                   response=False)
            except Exception as e:  # noqa: BLE001
                print(f"[ble] downlink write failed: {e!r}")
                return
            self._dl_seq += 1
            # Record what is now playing (timing authority owns the reference timeline).
            self._played_ref = np.concatenate([self._played_ref, pcm_block])
            self._played_samples += len(pcm_block)
            # Pace: advance the planned clock by one block * current scale, then sleep.
            planned_t += (BLOCK_MS / 1000.0) * self._pace_scale
            delay = planned_t - loop.time()
            if delay > 0:
                await asyncio.sleep(delay)
            # If we fell behind (queue built up while AI produced fast), reset the
            # accumulator so we don't burst -- pace from now.
            elif delay < -(BLOCK_MS / 1000.0):
                planned_t = loop.time()

    async def flush(self):
        """Barge-in: stop playback NOW. Clear the queue, FLUSH the device buffer,
        reset clock recovery. (The played_ref ring is intentionally kept -- the mic
        may still capture the tail echo of already-played audio.)"""
        while not self._dl_q.empty():
            try:
                self._dl_q.get_nowait()
            except asyncio.QueueEmpty:
                break
        self._cr_integ = cr.reset()
        self._pace_scale = 1.0
        if self._ctrl_char is not None and self._client and self._client.is_connected:
            try:
                await self._client.write_gatt_char(self._ctrl_char, bytes([CTRL_FLUSH]), response=True)
            except Exception as e:  # noqa: BLE001
                print(f"[ble] flush write failed: {e!r}")

    # --- reference for the AEC ---

    @property
    def played_samples(self):
        """Cumulative host-16k samples actually transmitted (== playout sample index)."""
        return self._played_samples

    def played_reference(self):
        """The full played-reference ring (host-16k float32). The orchestrator slices +
        drift-resamples this to align with the mic stream for the AEC."""
        return self._played_ref

    # --- teardown ---

    async def aclose(self):
        if self._sender_task:
            self._sender_task.cancel()
        if self._client:
            try:
                await self._client.__aexit__(None, None, None)
            except Exception:  # noqa: BLE001
                pass
