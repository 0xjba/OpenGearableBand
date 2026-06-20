#!/usr/bin/env python3
"""
audio_tx.py -- throwaway Mac sender for the gestureband BLE audio downlink
(phone -> speaker). NOT a product; a bring-up/validation tool. The inverse of
audio_rx.py: that one subscribes and DECODES; this one ENCODES and WRITES.

What it does:
  1. Scans for the band (by name "Xiao_Pulse") and connects.
  2. Reads a 16 kHz mono 16-bit WAV (positional arg; default "music.wav").
  3. Encodes each 10 ms / 160-sample frame -> 40 B LC3; packs two frames into an
     80 B block, prepends a 2-byte LE sequence number -> 82 B.
  4. Writes each block to the downlink characteristic 47A10003-... with
     Write-Without-Response (response=False), paced at ~20 ms/block.
  5. --flush-after N writes the 1-byte 0x01 FLUSH to the control char 47A10004-...
     N seconds into the stream (to test barge-in mid-stream).

Stream parameters (must match firmware lc3_codec.h):
  16 kHz, 10 ms frames (160 samples), 32 kbps -> 40 bytes/frame.

Requirements:
  pip install bleak
  a liblc3 shared lib. Build one from the NCS copy, e.g.:
     git clone https://github.com/google/liblc3 /tmp/liblc3
     cd /tmp/liblc3 && meson setup build && meson compile -C build
     # -> build/liblc3.dylib  (do NOT build with -ffast-math; it breaks LC3 MDCT)
  then run with:  --lib /tmp/liblc3/build/liblc3.dylib

Usage:
  python3 audio_tx.py music.wav --lib /path/liblc3.dylib
  python3 audio_tx.py music.wav --lib /path/liblc3.dylib --flush-after 2

Test clips (gitignored -- regenerate locally from any source audio):
  # 16 kHz mono 16-bit, the input this tool expects:
  afconvert -f WAVE -d LEI16@16000 -c 1 <source>.wav music16k.wav
  # ~60 s gapless loop (long-stream / clock-drift test), tiles the 6 s clip 10x:
  python3 - <<'PY'
  import wave
  with wave.open("music16k.wav","rb") as w: p=w.getparams(); f=w.readframes(w.getnframes())
  with wave.open("music16k_long.wav","wb") as o:
      o.setparams(p); [o.writeframes(f) for _ in range(10)]
  PY
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
AUDIO_DL_UUID = "47a10003-9b70-4c2e-8a1d-2f6b9e4a77c1"
AUDIO_CTRL_UUID = "47a10004-9b70-4c2e-8a1d-2f6b9e4a77c1"

# Must match firmware lc3_codec.h
DT_US = 10000
SR_HZ = 16000
BITRATE_BPS = 32000
FRAME_BYTES = 40
FRAME_SAMPLES = 160
FRAMES_PER_BLOCK = 2          # 80 B LC3 block = 20 ms
SEQ_HDR = 2
BLOCK_MS = (FRAMES_PER_BLOCK * FRAME_SAMPLES * 1000) // SR_HZ  # 20 ms
CTRL_FLUSH = 0x01             # BLE_AUDIO_CTRL_FLUSH


class Lc3Encoder:
    """Minimal ctypes wrapper over liblc3's encoder (inverse of Lc3Decoder)."""

    def __init__(self, lib_path):
        self.ok = False
        if not lib_path:
            return
        try:
            lib = ctypes.CDLL(lib_path)
            lib.lc3_encoder_size.restype = ctypes.c_uint
            lib.lc3_encoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
            lib.lc3_setup_encoder.restype = ctypes.c_void_p
            lib.lc3_setup_encoder.argtypes = [
                ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
            lib.lc3_encode.restype = ctypes.c_int
            lib.lc3_encode.argtypes = [
                ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
                ctypes.c_int, ctypes.c_void_p]
            size = lib.lc3_encoder_size(DT_US, SR_HZ)
            self._mem = ctypes.create_string_buffer(size)
            self._enc = lib.lc3_setup_encoder(DT_US, SR_HZ, 0, self._mem)
            self._lib = lib
            self._out = (ctypes.c_uint8 * FRAME_BYTES)()
            # LC3_PCM_FORMAT_S16 == 0
            self.ok = self._enc is not None
        except Exception as e:  # noqa: BLE001
            print(f"[encode disabled] {e}")
            self.ok = False

    def encode_frame(self, pcm160: bytes) -> bytes:
        """pcm160 = 160 int16 samples (320 bytes) -> 40-byte LC3 frame."""
        buf = (ctypes.c_uint8 * len(pcm160)).from_buffer_copy(pcm160)
        # lc3_encode(enc, fmt=0(S16), pcm, stride=1(mono), nbytes=40, out)
        self._lib.lc3_encode(self._enc, 0, buf, 1, FRAME_BYTES, self._out)
        return bytes(self._out)


def read_wav_frames(path):
    """Yield 160-sample (320-byte) mono 16-bit PCM frames from a 16 kHz WAV."""
    with wave.open(path, "rb") as w:
        if w.getnchannels() != 1:
            sys.exit(f"{path}: need mono, got {w.getnchannels()} channels.")
        if w.getsampwidth() != 2:
            sys.exit(f"{path}: need 16-bit samples, got {w.getsampwidth()*8}-bit.")
        if w.getframerate() != SR_HZ:
            sys.exit(f"{path}: need {SR_HZ} Hz, got {w.getframerate()} Hz "
                     "(resample first, e.g. ffmpeg -ar 16000).")
        pcm = w.readframes(w.getnframes())
    fbytes = FRAME_SAMPLES * 2
    for off in range(0, len(pcm) - fbytes + 1, fbytes):
        yield pcm[off:off + fbytes]


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav", nargs="?", default="music.wav",
                    help="16 kHz mono 16-bit WAV to stream (default music.wav)")
    ap.add_argument("--lib", default=None, help="path to liblc3 shared lib")
    ap.add_argument("--flush-after", type=float, default=0,
                    help="write a FLUSH (barge-in) N seconds into the stream "
                         "(0 = never)")
    ap.add_argument("--address", default=None,
                    help="connect directly to this BLE address/UUID (skips the "
                         "name scan; use the address printed by the scan)")
    args = ap.parse_args()

    enc = Lc3Encoder(args.lib)
    if not enc.ok:
        sys.exit("LC3 encoder unavailable -- pass --lib /path/liblc3.dylib")

    # Build the [seq16 LE][80 B LC3 block] packets up front.
    frames = list(read_wav_frames(args.wav))
    if not frames:
        sys.exit(f"{args.wav}: no audio frames decoded.")
    packets = []
    seq = 0
    for i in range(0, len(frames), FRAMES_PER_BLOCK):
        block = enc.encode_frame(frames[i])
        if i + 1 < len(frames):
            block += enc.encode_frame(frames[i + 1])
        packets.append(struct.pack("<H", seq & 0xFFFF) + block)
        seq += 1
    print(f"{args.wav}: {len(frames)} frames -> {len(packets)} blocks "
          f"(~{len(packets) * BLOCK_MS / 1000:.1f}s @ {BLOCK_MS} ms/block)")

    dev = None
    if args.address:
        # Direct-address bypass: skip the name scan entirely (macOS name caching
        # can make a name match miss even when the device is advertising fine).
        dev = await BleakScanner.find_device_by_address(args.address, timeout=15.0)
        if not dev:
            sys.exit(f"No device at address '{args.address}'. Is it connected "
                     "elsewhere? (firmware accepts ONE connection.)")
    else:
        # Match on the LIVE advertised local name, not find_device_by_name (which
        # uses the macOS-cached GAP name -- this board may still be cached under an
        # old name like "oneDiary", making a name match silently miss). Scan all,
        # print what we see (diagnostic), and match adv.local_name OR the cached name.
        print(f"Scanning for '{DEVICE_NAME}' (advertised name) ...")
        found = await BleakScanner.discover(timeout=15.0, return_adv=True)
        for d, adv in found.values():
            live_name = adv.local_name or "?"
            print(f"  {d.address}  adv={live_name!r}  cached={d.name!r}  rssi={adv.rssi}")
            if DEVICE_NAME in (adv.local_name, d.name):
                dev = d
        if not dev:
            sys.exit(
                f"\nDevice advertising '{DEVICE_NAME}' not found in the scan above.\n"
                "  - If you DON'T see it at all: it's likely connected elsewhere (the\n"
                "    firmware accepts ONE connection and stops advertising while\n"
                "    connected). Disconnect nRF Connect, close it, and turn the phone's\n"
                "    Bluetooth OFF, then retry.\n"
                "  - If you see it under a DIFFERENT name (e.g. 'oneDiary'): macOS has a\n"
                "    stale cached name. Pass --address <addr from the list> to skip the\n"
                "    name match.")

    conn_t0 = time.time()

    def on_disconnect(_client):
        # Timestamps the drop relative to connect -> tells immediate-reject from a
        # ~supervision-timeout drop. Pair with the firmware's "disconnected,
        # reason 0xNN" log to get the HCI cause.
        print(f"\n!! DISCONNECTED by device at t+{time.time() - conn_t0:.2f}s "
              "-- check the serial log for 'disconnected, reason 0xNN'.")

    async with BleakClient(dev, disconnected_callback=on_disconnect) as client:
        conn_t0 = time.time()
        print(f"Connected: {dev.address}. MTU={client.mtu_size}")

        # Resolve the characteristics up front so we (a) confirm the link held +
        # GATT was discovered, (b) write to the resolved objects (no per-write
        # lookup). If the link dropped, or macOS served a STALE cached GATT (the
        # downlink UUIDs are new, added when this firmware was flashed), the
        # lookups fail here with a clear message instead of a cryptic write error.
        if not client.is_connected:
            sys.exit("Device dropped right after connecting (link not held).")
        try:
            discovered = list(client.services.characteristics.values())
        except Exception as e:  # noqa: BLE001
            sys.exit(f"GATT not available ({e}) -- link dropped or no discovery.")
        # Diagnostic: show exactly what CoreBluetooth handed us.
        print(f"Discovered {len(discovered)} characteristics:")
        for ch in discovered:
            print(f"  {ch.uuid}  ({','.join(ch.properties)})")
        dl_char = client.services.get_characteristic(AUDIO_DL_UUID)
        ctrl_char = client.services.get_characteristic(AUDIO_CTRL_UUID)
        if dl_char is None or ctrl_char is None:
            sys.exit(
                "Downlink characteristics NOT in the discovered set above.\n"
                "  If the list looks like OLD firmware (missing 47a10003/47a10004),\n"
                "  macOS is serving a STALE cached GATT: toggle Mac Bluetooth OFF then\n"
                "  ON (flushes the CoreBluetooth cache) and re-run. Also clear the\n"
                "  device's bonds: serial 'u' then 'r'.")

        # --- Escalating write probe (localizes the L2CAP "unknown channel ID"
        # failure). Each step is a Write-WITH-response so the device must ATT-ack
        # it; the step that fails/disconnects tells us the size threshold:
        #   1B ctrl  -> ATT/handle path OK
        #   42B (1 LC3 frame) -> small single-fragment write OK
        #   82B (full block)  -> the real downlink packet
        # Also gives DLE a moment to settle (sleep between steps). ---
        async def probe(name, char, payload):
            if not client.is_connected:
                sys.exit(f"  PROBE {name}: link already dropped before this write.")
            try:
                await client.write_gatt_char(char, payload, response=True)
                print(f"  PROBE {name} ({len(payload)}B): OK")
            except Exception as e:  # noqa: BLE001
                sys.exit(f"  PROBE {name} ({len(payload)}B): FAILED -> {e!r}\n"
                         "  (note the serial 'reason 0xNN' / l2cap line at this point)")
            await asyncio.sleep(0.25)

        print("Probing write path (escalating size, with response) ...")
        await probe("ctrl-1B", ctrl_char, bytes([0x00]))   # no-op (handler ignores != 0x01)
        await probe("dl-1frame", dl_char, packets[0][:FRAME_BYTES + 2])
        await probe("dl-fullblock", dl_char, packets[0])
        print("Probe passed -- full-size writes accepted. Proceeding to stream.")

        print(f"Streaming {len(packets)} blocks to downlink ...")

        t0 = time.time()
        flush_at = args.flush_after if args.flush_after > 0 else None
        flushed = False
        sent = 0

        for pkt in packets:
            now = time.time()
            if flush_at is not None and not flushed and (now - t0) >= flush_at:
                await client.write_gatt_char(
                    ctrl_char, bytes([CTRL_FLUSH]), response=True)
                # A real barge-in = FLUSH *and stop sending* (else the next frame
                # auto-restarts playback on the device). Stop the stream here.
                print(f"FLUSH sent at {now - t0:.2f}s (barge-in) -- stopping stream.")
                flushed = True
                break
            # Write-Without-Response (unacked, high-throughput).
            await client.write_gatt_char(dl_char, pkt, response=False)
            sent += 1
            # Pace to real time (~20 ms/block). Compensate for elapsed work.
            target = t0 + sent * (BLOCK_MS / 1000.0)
            delay = target - time.time()
            if delay > 0:
                await asyncio.sleep(delay)

        dur = time.time() - t0
        print(f"\n--- done ---")
        print(f"sent {sent} blocks in {dur:.1f}s "
              f"({sent / dur:.1f} block/s)" if dur > 0 else f"sent {sent} blocks")


if __name__ == "__main__":
    asyncio.run(main())
