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
    ap.add_argument("--out", default="uplink.wav")   # cwd (repo dir), openable
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

    # Match by name SUBSTRING (case-insensitive) not exact name: macOS caches
    # the device name per address, so an old advertised name (gband-OLED /
    # gband-uplink) can shadow the current one. Any "gband*" peripheral exposing
    # the uplink char is the target.
    print("scanning for a gband* device ...")
    found = {}

    def _cb(d, adv):
        name = (adv.local_name or d.name or "")
        found[d.address] = (d, name)

    scanner = BleakScanner(detection_callback=_cb)
    await scanner.start()
    await asyncio.sleep(8.0)
    await scanner.stop()

    dev = None
    for addr, (d, name) in found.items():
        tag = "  MATCH" if "gband" in name.lower() else "  seen "
        print(f"{tag}: {name!r} [{addr}]")
        if dev is None and "gband" in name.lower():
            dev = d
    if dev is None:
        print("NOT FOUND -- no gband* device seen. (In nRF Connect, note the exact "
              "name; if it's cached-different, forget the device in macOS BT settings.)")
        return 2

    print(f"connecting to {dev.address} ...")
    async with BleakClient(dev) as c:
        print("connected; subscribing to uplink audio char")
        await c.start_notify(UPLINK_UUID, on_notify)
        print(f">>> NOW: hold the ear pose (or press 'd' in tio), then speak for ~{args.seconds:.0f}s <<<")
        await asyncio.sleep(args.seconds)
        await c.stop_notify(UPLINK_UUID)

    # Report + save.
    dur = stats["frames"] * 160 / SR_HZ
    print("\n=== uplink stats ===")
    print(f"  notifications : {stats['notifs']}")
    print(f"  LC3 frames    : {stats['frames']}  ({dur:.1f} s of audio)")
    print(f"  seq gaps      : {stats['gaps']} (dropped notifications)")
    if pcm_chunks:
        pcm = np.concatenate(pcm_chunks).astype(np.float32)
        rms = float(np.sqrt(np.mean(pcm ** 2)))
        peak = float(np.max(np.abs(pcm)))
        print(f"  decoded RMS   : {rms:.0f}  peak={int(peak)}  (silence ~<50, speech >>500)")
        # Normalize so faint captures (e.g. mic pointed away in the ear pose) are
        # audible: scale the peak up to ~90% full scale (capped so we don't blow
        # up pure silence). This also lifts the noise floor -- it's for confirming
        # the voice is there, not for fidelity.
        gain = min(60.0, 0.9 * 32767.0 / peak) if peak > 1.0 else 1.0
        out = np.clip(pcm * gain, -32768, 32767).astype(np.int16)
        with wave.open(args.out, "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR_HZ)
            w.writeframes(out.tobytes())
        print(f"  wrote WAV     : {args.out}  (normalized x{gain:.0f})")
        # Auto-play so you don't have to hunt for the file.
        import subprocess
        try:
            subprocess.run(["afplay", args.out], timeout=40)
        except Exception:
            print(f"  play manually: afplay {args.out}")
        return 0
    print("  NO AUDIO DECODED -- is dictation on (real POSE_EAR or 'd') AND host subscribed?")
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
