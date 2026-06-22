#!/usr/bin/env python3
"""Live full-duplex voice loop on the band (Phase C entrypoint).

Wires BleLink + a VoiceBackend + the Orchestrator into a runnable loop. Two backends:

  --backend loopback   No AI. Streams a canned WAV (or a macOS `say` line) to the
                       speaker as the "AI reply", so you can verify the loop end-to-end:
                       AI plays -> you talk over it -> AEC+VAD detect the barge-in ->
                       downlink FLUSHES + playback stops. Needs no API key.

  --backend gemini     Real conversation via Gemini Live (reads GEMINI_API_KEY from
                       .env). You talk -> mic -> AEC -> Gemini -> reply plays -> barge-in.

The render->capture loop delay is estimated ONLINE (voiceio/delay_estimator.py, AEC3-style)
-- nothing acoustic is hardcoded, so it self-calibrates per unit/session. --init-delay-ms is
an OPTIONAL warm-start (e.g. a measure_delay value) so the first response isn't un-cancelled
while the estimator acquires; omit it for pure online acquisition.

Usage:
  tools/dtln-venv/bin/python tools/voice_loop.py --backend loopback --say "Counting: one, two, three, four, five, six, seven, eight."
  tools/dtln-venv/bin/python tools/voice_loop.py --backend gemini
  tools/dtln-venv/bin/python tools/voice_loop.py --backend loopback --canned reply.wav --init-delay-ms 200
"""
import argparse
import asyncio
import os
import subprocess
import sys

import numpy as np
import soundfile as sf
import soxr

sys.path.insert(0, os.path.dirname(__file__))
from voiceio.ble_link import BleLink                       # noqa: E402
from voiceio.orchestrator import Orchestrator              # noqa: E402
from voiceio.backend import EchoLoopbackBackend            # noqa: E402

HERE = os.path.dirname(__file__)
DEFAULT_MODEL = os.path.join(HERE, "DTLN-aec/pretrained_models/dtln_aec_512")
DEFAULT_LIB = os.path.join(HERE, "lib/liblc3.dylib")
SR = 16000


def load_16k_mono(path):
    a, sr = sf.read(path, dtype="float32")
    if a.ndim > 1:
        a = a.mean(axis=1)
    if sr != SR:
        a = soxr.resample(a, sr, SR).astype(np.float32)
    return a.astype(np.float32)


def synth_16k(text):
    aiff = "/tmp/voice_loop_say.aiff"
    subprocess.run(["say", "-o", aiff, text], check=True)
    return load_16k_mono(aiff)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["loopback", "gemini"], default="loopback")
    ap.add_argument("--canned", help="loopback: WAV to play as the AI reply")
    ap.add_argument("--say", default="Counting now: one, two, three, four, five, six, seven, eight, nine, ten.",
                    help="loopback: text to synthesize if --canned is not given")
    ap.add_argument("--model-dir", default=DEFAULT_MODEL, help="DTLN model path prefix")
    ap.add_argument("--lib", default=DEFAULT_LIB, help="liblc3 shared lib")
    ap.add_argument("--init-delay-ms", type=float, default=None,
                    help="optional warm-start for the ONLINE delay estimator (e.g. a "
                         "measure_delay value); omit for pure online acquisition")
    ap.add_argument("--address", default=None, help="connect directly to this BLE address")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s (0 = Ctrl-C)")
    args = ap.parse_args()

    # --- backend ---
    if args.backend == "gemini":
        from dotenv import load_dotenv
        from voiceio.gemini_backend import GeminiLiveBackend
        load_dotenv()
        if not os.environ.get("GEMINI_API_KEY"):
            sys.exit("GEMINI_API_KEY not set (put it in a git-ignored .env).")
        backend = GeminiLiveBackend(
            system_instruction="You are a concise voice assistant. Keep replies short.")
        backend.wait_ready(timeout=15.0)
        print("[gemini] connected")
    else:
        reply = load_16k_mono(args.canned) if args.canned else synth_16k(args.say)
        print(f"[loopback] AI reply = {len(reply)/SR:.1f}s canned audio")
        backend = EchoLoopbackBackend(reply)

    # --- BLE + orchestrator ---
    ble = BleLink(address=args.address)
    await ble.connect()
    orch = Orchestrator(ble, backend, model_dir=args.model_dir, lib_path=args.lib,
                        init_delay_ms=args.init_delay_ms)

    run_task = asyncio.create_task(orch.run())
    print("[voice_loop] running. Raise to ear + speak (force mic with serial 'j'). Ctrl-C to stop.")
    try:
        if args.seconds > 0:
            await asyncio.sleep(args.seconds)
        else:
            await run_task
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        orch.stop()
        run_task.cancel()
        await asyncio.gather(run_task, return_exceptions=True)
        if args.backend == "gemini":
            backend.close()
        await ble.aclose()
        print("[voice_loop] stopped.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
