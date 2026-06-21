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

AUDIO_STATUS_UUID = "47a10005-9b70-4c2e-8a1d-2f6b9e4a77c1"
AUDIO_UPLINK_UUID = "47a10002-9b70-4c2e-8a1d-2f6b9e4a77c1"   # device->host mic notify
SETPOINT_MS = 140                       # target buffered audio (latency vs jitter);
                                        # HW-tuned 2026-06-21: 120 was jitter-marginal
                                        # (occasional underrun), 160 robust -> 140 split
SETPOINT_BYTES = SETPOINT_MS * SR_HZ * 2 // 1000   # 140ms @ 16k mono 16-bit = 4480
# Control-law gains (host clock recovery). Slow loop -> no oscillation; clamp must
# EXCEED worst-case I2S drift (~0.8%), so +/-1.5%. See the design doc.
CR_KP = 0.5            # proportional, on normalized error (used-setp)/capacity
CR_KI = 0.02           # integral (slow)
CR_CLAMP = 0.015       # +/-1.5% correction authority
CR_INTEG_MAX = CR_CLAMP / CR_KI        # anti-windup: integral alone can't exceed clamp
CR_FLAG_KICK = 0.10    # integral nudge on a hard over/underrun event
# Status flags byte bits (mirror firmware ble_audio.h BLE_AUDIO_STATUS_FL_*).
STATUS_FL_OVERFLOW = 0x02
STATUS_FL_UNDERRUN = 0x04


def clock_recovery_reset():
    """Return a fresh controller integral (0.0). Call on barge-in flush / session
    restart: `integ = clock_recovery_reset()`. Side-effect-free by design (the
    integral is threaded through clock_recovery_step, not stored globally)."""
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


class Lc3Decoder:
    """Minimal ctypes wrapper over liblc3's decoder (mirror of Lc3Encoder)."""

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
            self.ok = self._dec is not None
        except Exception as e:  # noqa: BLE001
            print(f"[decode disabled] {e}")
            self.ok = False

    def decode_frame(self, frame40: bytes) -> bytes:
        """40-byte LC3 frame -> 160 int16 samples (320 bytes)."""
        buf = (ctypes.c_uint8 * len(frame40)).from_buffer_copy(frame40)
        self._lib.lc3_decode(self._dec, buf, len(frame40), 0, self._pcm, 1)
        return bytes(self._pcm)


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
    ap.add_argument("--no-adaptive", action="store_true",
                    help="disable clock-recovery (don't subscribe, fixed pacing) -- "
                         "only to reproduce the drift baseline")
    ap.add_argument("--setpoint-ms", type=int, default=SETPOINT_MS,
                    help=f"target buffered audio in ms (default {SETPOINT_MS}); higher "
                         "= more jitter cushion (fewer underruns) but more latency")
    ap.add_argument("--duplex", action="store_true",
                    help="full-duplex smoke test: also subscribe to the uplink mic "
                         "notify (47A10002) and count frames/drops while streaming down "
                         "(raise band to ear + speak to trigger the uplink)")
    ap.add_argument("--record", default=None,
                    help="with --duplex: save <REC>_ref.wav (streamed source) + "
                         "<REC>_mic.wav (decoded uplink mic w/ echo) for offline AEC")
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

    # For --record: the reference = the exact source PCM we will stream down.
    ref_pcm = b"".join(frames) if args.record else b""

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

        # Clock-recovery state. adaptive ON by default; --no-adaptive => fixed pace.
        adaptive = not args.no_adaptive
        cr_integ = clock_recovery_reset()
        pace_scale = 1.0
        setpoint_bytes = args.setpoint_ms * SR_HZ * 2 // 1000   # A/B knob

        if adaptive:
            def on_status(_char, data: bytearray):
                nonlocal cr_integ, pace_scale
                if len(data) != 5:
                    return
                used = data[0] | (data[1] << 8)
                cap = data[2] | (data[3] << 8)
                flags = data[4]
                cr_integ, pace_scale = clock_recovery_step(
                    cr_integ, used, cap or 1, setpoint_bytes,
                    1 if flags & STATUS_FL_OVERFLOW else 0,
                    1 if flags & STATUS_FL_UNDERRUN else 0)
            await client.start_notify(AUDIO_STATUS_UUID, on_status)
            print(f"adaptive playout ON (status 47A10005, setpoint={args.setpoint_ms}ms)")
        else:
            print("adaptive playout OFF (--no-adaptive baseline)")

        # Full-duplex smoke test: subscribe to the uplink mic notify and count
        # frames + sequence gaps WHILE we stream downlink, proving the band runs
        # both directions on one connection. The uplink only flows in MODE_DICTATION
        # (raise to ear + speak), so frames arrive once dictation is triggered.
        uplink = {"count": 0, "drops": 0, "last_seq": None}
        mic_pcm = bytearray()
        dec = Lc3Decoder(args.lib) if args.record else None
        if args.record and not args.duplex:
            sys.exit("--record requires --duplex (the uplink mic comes from the duplex subscription).")
        if args.record and not (dec and dec.ok):
            sys.exit("--record needs --lib <liblc3> to decode the uplink mic.")
        if args.duplex:
            def on_uplink(_char, data: bytearray):
                if len(data) < 2:
                    return
                seq = data[0] | (data[1] << 8)
                if uplink["last_seq"] is not None:
                    uplink["drops"] += (seq - (uplink["last_seq"] + 1)) & 0xFFFF
                uplink["last_seq"] = seq
                uplink["count"] += 1
                if dec:  # [seq16][80B LC3 = 2x40B] -> decode both frames
                    payload = bytes(data[2:])
                    for off in range(0, len(payload) - FRAME_BYTES + 1, FRAME_BYTES):
                        mic_pcm.extend(dec.decode_frame(payload[off:off + FRAME_BYTES]))
            await client.start_notify(AUDIO_UPLINK_UUID, on_uplink)
            print("duplex ON (subscribed to uplink mic 47A10002 -- raise to ear + speak)")

        t0 = time.time()
        flush_at = args.flush_after if args.flush_after > 0 else None
        flushed = False
        sent = 0
        planned_t = t0

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
            await client.write_gatt_char(dl_char, pkt, response=False)
            sent += 1
            # Pace to real time, scaled by the controller (>1 = slower). Running-sum
            # accumulator: each block advances the planned clock by the CURRENT scale,
            # so a scale change never retroactively reprices already-sent blocks. (Real
            # Mac/phone apps should use this accumulator form, not target=t0+sent*block.)
            planned_t += (BLOCK_MS / 1000.0) * pace_scale
            delay = planned_t - time.time()
            if delay > 0:
                await asyncio.sleep(delay)

        dur = time.time() - t0
        print(f"\n--- done ---")
        print(f"sent {sent} blocks in {dur:.1f}s "
              f"({sent / dur:.1f} block/s)" if dur > 0 else f"sent {sent} blocks")
        if args.duplex:
            print(f"uplink (full-duplex): {uplink['count']} mic frames received, "
                  f"{uplink['drops']} seq-gap drops "
                  f"({'NO uplink -- did you raise to ear + speak?' if uplink['count'] == 0 else 'concurrent up+down OK'})")
            if args.record:
                def _save_wav(path, pcm_bytes):
                    with wave.open(path, "wb") as w:
                        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR_HZ)
                        w.writeframes(pcm_bytes)
                _save_wav(f"{args.record}_ref.wav", ref_pcm)
                _save_wav(f"{args.record}_mic.wav", bytes(mic_pcm))
                print(f"recorded: {args.record}_ref.wav ({len(ref_pcm)//320} frames), "
                      f"{args.record}_mic.wav ({len(mic_pcm)//320} frames)")


if __name__ == "__main__":
    asyncio.run(main())
