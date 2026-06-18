#!/usr/bin/env python3
"""
audio_rx.py -- throwaway Mac receiver for the gestureband dictation audio stream
(sub-project B). NOT a product; a bring-up/validation tool.

What it does:
  1. Scans for the band (by name "Xiao_Pulse") and connects.
  2. Subscribes to the audio data characteristic (notifications).
  3. Parses each notification: [seq:2 LE][payload], where payload is one 20 ms
     block = two 40-byte LC3 frames (80 bytes).
  4. Tracks sequence continuity (so you SEE dropped/reordered notifications),
     prints throughput stats.
  5. Always writes the raw LC3 stream to <out>.lc3 (concatenated 40-byte frames).
  6. If a liblc3 shared library is available (--lib, or auto-found), decodes to a
     16 kHz mono <out>.wav so you can listen and confirm the audio is real.

Stream parameters (must match firmware lc3_codec.h):
  16 kHz, 10 ms frames (160 samples), 32 kbps -> 40 bytes/frame.

Requirements:
  pip install bleak
  (decode) a liblc3 shared lib. Build one from the NCS copy, e.g.:
     git clone https://github.com/google/liblc3 /tmp/liblc3
     cd /tmp/liblc3 && meson setup build && meson compile -C build
     # -> build/liblc3.dylib  (do NOT build with -ffast-math; it breaks LC3 MDCT)
  then run with:  --lib /tmp/liblc3/build/liblc3.dylib

Usage:
  python3 audio_rx.py                       # capture + validate + raw dump
  python3 audio_rx.py --out dictation       # custom output basename
  python3 audio_rx.py --lib /path/liblc3.dylib   # also decode to WAV
"""

import argparse
import asyncio
import ctypes
import struct
import sys
import time
import wave

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("Missing dependency: pip install bleak")

DEVICE_NAME = "Xiao_Pulse"
AUDIO_SVC_UUID = "47a10001-9b70-4c2e-8a1d-2f6b9e4a77c1"
AUDIO_DATA_UUID = "47a10002-9b70-4c2e-8a1d-2f6b9e4a77c1"

# Must match firmware lc3_codec.h
DT_US = 10000
SR_HZ = 16000
FRAME_BYTES = 40
FRAME_SAMPLES = 160
SEQ_HDR = 2


class Lc3Decoder:
    """Minimal ctypes wrapper over liblc3's decoder. None if no lib given."""

    def __init__(self, lib_path):
        self.ok = False
        if not lib_path:
            return
        try:
            lib = ctypes.CDLL(lib_path)
            lib.lc3_decoder_size.restype = ctypes.c_uint
            lib.lc3_decoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
            lib.lc3_setup_decoder.restype = ctypes.c_void_p
            lib.lc3_setup_decoder.argtypes = [
                ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
            lib.lc3_decode.restype = ctypes.c_int
            lib.lc3_decode.argtypes = [
                ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
            size = lib.lc3_decoder_size(DT_US, SR_HZ)
            self._mem = ctypes.create_string_buffer(size)
            self._dec = lib.lc3_setup_decoder(DT_US, SR_HZ, 0, self._mem)
            self._lib = lib
            self._pcm = (ctypes.c_int16 * FRAME_SAMPLES)()
            # LC3_PCM_FORMAT_S16 == 0
            self.ok = self._dec is not None
        except Exception as e:  # noqa: BLE001
            print(f"[decode disabled] {e}")
            self.ok = False

    def decode_frame(self, frame40: bytes) -> bytes:
        buf = (ctypes.c_uint8 * len(frame40)).from_buffer_copy(frame40)
        self._lib.lc3_decode(self._dec, buf, len(frame40), 0,
                             self._pcm, 1)
        return bytes(self._pcm)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="dictation", help="output basename")
    ap.add_argument("--lib", default=None, help="path to liblc3 shared lib")
    ap.add_argument("--seconds", type=float, default=0,
                    help="auto-stop after N seconds (0 = until Ctrl-C)")
    args = ap.parse_args()

    dec = Lc3Decoder(args.lib)
    raw = open(f"{args.out}.lc3", "wb")
    pcm_chunks = []

    state = {"count": 0, "drops": 0, "last_seq": None,
             "t0": None, "bytes": 0}

    def on_notify(_char, data: bytearray):
        if len(data) < SEQ_HDR + FRAME_BYTES:
            return
        seq = data[0] | (data[1] << 8)
        payload = bytes(data[SEQ_HDR:])

        if state["t0"] is None:
            state["t0"] = time.time()
            print("First audio notification received -- streaming.")
        state["count"] += 1
        state["bytes"] += len(payload)

        if state["last_seq"] is not None:
            gap = (seq - (state["last_seq"] + 1)) & 0xFFFF
            if gap:
                state["drops"] += gap
        state["last_seq"] = seq

        # payload = N*40-byte LC3 frames (B sends 2)
        for off in range(0, len(payload) - FRAME_BYTES + 1, FRAME_BYTES):
            frame = payload[off:off + FRAME_BYTES]
            raw.write(frame)
            if dec.ok:
                pcm_chunks.append(dec.decode_frame(frame))

    print(f"Scanning for '{DEVICE_NAME}' ...")
    dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0)
    if not dev:
        sys.exit(f"Device '{DEVICE_NAME}' not found.")

    async with BleakClient(dev) as client:
        print(f"Connected: {dev.address}. MTU={client.mtu_size}")
        print("Subscribed. Raise to ear and speak (MODE_DICTATION). Ctrl-C to stop.")
        await client.start_notify(AUDIO_DATA_UUID, on_notify)
        try:
            if args.seconds > 0:
                await asyncio.sleep(args.seconds)
            else:
                while True:
                    await asyncio.sleep(1.0)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            try:
                await client.stop_notify(AUDIO_DATA_UUID)
            except Exception:  # noqa: BLE001
                pass

    raw.close()
    dur = (time.time() - state["t0"]) if state["t0"] else 0
    print("\n--- stats ---")
    print(f"notifications: {state['count']}  est. dropped: {state['drops']}")
    if dur > 0:
        print(f"duration: {dur:.1f}s  rate: {state['count']/dur:.1f} notif/s  "
              f"payload: {state['bytes']/dur/1024:.1f} KB/s")
    print(f"raw LC3 -> {args.out}.lc3")

    if dec.ok and pcm_chunks:
        with wave.open(f"{args.out}.wav", "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SR_HZ)
            w.writeframes(b"".join(pcm_chunks))
        print(f"decoded WAV -> {args.out}.wav  (open it / play to verify audio)")
    elif not dec.ok:
        print("no decode (pass --lib /path/liblc3.dylib to also produce a WAV)")


if __name__ == "__main__":
    asyncio.run(main())
