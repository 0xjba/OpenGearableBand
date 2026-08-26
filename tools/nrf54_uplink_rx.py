#!/usr/bin/env python3
"""
M3c.2 uplink receiver: connect to the nRF54 OLED-variant band, subscribe to the
uplink audio characteristic, decode the LC3 stream to a WAV, and report stats.

Run from YOUR Terminal (macOS grants Bluetooth to Terminal, not to sandboxed
processes):

    tools/dtln-venv/bin/python tools/nrf54_uplink_rx.py --seconds 15 --out /tmp/uplink.wav

On the band: connect happens automatically; once connected + subscribed, press
'd' in the serial console (tio) to open the dictation gate, then speak.

Verifies the whole chain: mic -> mic_vad -> audio_stream -> LC3 encode ->
ble_audio NOTIFY -> host -> LC3 decode -> WAV.
"""
import argparse
import asyncio
import os
import sys
import wave

import numpy as np

# Reuse the shared wire-format parser + LC3 decoder from tools/voiceio.
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from voiceio.frame import parse_uplink          # noqa: E402
from voiceio.codec import Lc3Codec              # noqa: E402

from bleak import BleakScanner, BleakClient      # noqa: E402

DEV_NAME = "gband-uplink"
UPLINK_UUID = "47a10002-9b70-4c2e-8a1d-2f6b9e4a77c1"
FRAME_BYTES = 40          # one LC3 frame
SR_HZ = 16000


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--out", default="/tmp/uplink.wav")
    ap.add_argument("--lib", default=os.path.join(os.path.dirname(__file__), "lib", "liblc3.dylib"))
    args = ap.parse_args()

    codec = Lc3Codec(lib_path=args.lib)

    pcm_chunks = []
    stats = {"notifs": 0, "frames": 0, "bytes": 0, "last_seq": None, "gaps": 0}

    def on_notify(_char, data: bytearray):
        f = parse_uplink(bytes(data))
        stats["notifs"] += 1
        stats["bytes"] += len(f.lc3)
        if stats["last_seq"] is not None:
            expected = (stats["last_seq"] + 1) & 0xFFFF
            if f.seq != expected:
                stats["gaps"] += 1
        stats["last_seq"] = f.seq
        # 80-byte payload = 2 x 40-byte LC3 frames -> decode each.
        for i in range(0, len(f.lc3) - FRAME_BYTES + 1, FRAME_BYTES):
            pcm_chunks.append(codec.decode(f.lc3[i:i + FRAME_BYTES]))
            stats["frames"] += 1

    print(f"scanning for {DEV_NAME} ...")
    dev = await BleakScanner.find_device_by_name(DEV_NAME, timeout=15.0)
    if not dev:
        print("NOT FOUND -- is the band advertising? (reflash / power-cycle)")
        return 2

    print(f"found {dev.address}, connecting ...")
    async with BleakClient(dev) as c:
        print("connected; subscribing to uplink audio char")
        await c.start_notify(UPLINK_UUID, on_notify)
        print(f">>> NOW: press 'd' in tio (dictation ON), then speak for ~{args.seconds:.0f}s <<<")
        await asyncio.sleep(args.seconds)
        await c.stop_notify(UPLINK_UUID)

    # Report + save.
    dur = stats["frames"] * 160 / SR_HZ
    print("\n=== uplink stats ===")
    print(f"  notifications : {stats['notifs']}")
    print(f"  LC3 frames    : {stats['frames']}  ({dur:.1f} s of audio)")
    print(f"  seq gaps      : {stats['gaps']} (dropped notifications)")
    if pcm_chunks:
        pcm = np.concatenate(pcm_chunks)
        rms = float(np.sqrt(np.mean((pcm.astype(np.float32)) ** 2)))
        peak = int(np.max(np.abs(pcm)))
        print(f"  decoded RMS   : {rms:.0f}  peak={peak}  (silence ~<50, speech >>500)")
        with wave.open(args.out, "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR_HZ)
            w.writeframes(pcm.astype(np.int16).tobytes())
        print(f"  wrote WAV     : {args.out}  (play it to confirm your voice)")
        return 0
    print("  NO AUDIO DECODED -- did you press 'd' + speak? gate = dictation AND subscribed")
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
