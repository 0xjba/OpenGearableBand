#!/usr/bin/env python3
"""Standalone Gemini Live backend de-risk -- NO band/hardware needed.

Proves the real AI backend end-to-end on the Mac: open a Live session, stream a
recorded spoken question as if it were the user's clean (echo-cancelled) mic
audio, capture the model's spoken reply, and save it to a WAV you can listen to.
This validates the API contract (16 kHz in / 24 kHz out, async->sync bridge,
model id, key handling) before wiring it into the live BLE loop.

Key handling: reads GEMINI_API_KEY from a git-ignored .env (or the environment).
Never pass the key on the command line or paste it into a chat.

Usage:
    # make a .env first:  echo 'GEMINI_API_KEY=your_key' > .env
    tools/dtln-venv/bin/python tools/gemini_test.py
    tools/dtln-venv/bin/python tools/gemini_test.py --in my_question.wav
    tools/dtln-venv/bin/python tools/gemini_test.py --say "What is 17 times 4?"
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import soundfile as sf
from dotenv import load_dotenv

# Allow `python tools/gemini_test.py` from the repo root.
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from voiceio.gemini_backend import GeminiLiveBackend  # noqa: E402

INTERNAL_HZ = 16000
CHUNK_MS = 20  # feed mic in 20 ms chunks, paced ~real-time so server VAD behaves


def load_utterance(path):
    """Load a WAV as 16 kHz mono float32."""
    audio, sr = sf.read(path, dtype="float32")
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if sr != INTERNAL_HZ:
        import soxr
        audio = soxr.resample(audio, sr, INTERNAL_HZ).astype(np.float32)
    return audio


def synth_question(text):
    """Use macOS `say` to synthesize a spoken question -> 16 kHz mono float32."""
    aiff = "/tmp/gemini_test_q.aiff"
    subprocess.run(["say", "-o", aiff, text], check=True)
    audio = load_utterance(aiff)
    return audio


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="infile", help="WAV of the spoken question (any rate)")
    ap.add_argument("--say", default="What is the capital of France? Answer in one short sentence.",
                    help="text to synthesize via macOS `say` if --in is not given")
    ap.add_argument("--out", default="/tmp/gemini_reply.wav", help="where to save the AI reply WAV")
    ap.add_argument("--quiet-s", type=float, default=2.0,
                    help="stop after this many seconds of no new reply audio")
    ap.add_argument("--timeout-s", type=float, default=30.0, help="overall receive timeout")
    args = ap.parse_args()

    load_dotenv()  # pull GEMINI_API_KEY from .env
    if not os.environ.get("GEMINI_API_KEY"):
        sys.exit("GEMINI_API_KEY not set. Create a git-ignored .env:\n"
                 "  echo 'GEMINI_API_KEY=your_key_here' > .env")

    if args.infile:
        question = load_utterance(args.infile)
        print(f"[in] loaded {args.infile}: {len(question)/INTERNAL_HZ:.2f}s")
    else:
        question = synth_question(args.say)
        print(f"[in] synthesized: {args.say!r} ({len(question)/INTERNAL_HZ:.2f}s)")

    print("[gemini] connecting...")
    be = GeminiLiveBackend(
        system_instruction="You are a concise voice assistant. Keep replies short.")
    be.wait_ready(timeout=15.0)
    print("[gemini] connected.")

    # Stream the question as the user's mic audio, paced ~real-time, + trailing
    # silence so the server's VAD sees end-of-turn.
    chunk = int(INTERNAL_HZ * CHUNK_MS / 1000)
    for i in range(0, len(question), chunk):
        be.feed_mic(question[i:i + chunk])
        time.sleep(CHUNK_MS / 1000.0)
    be.end_turn()  # one-shot: explicit end-of-turn (digital silence won't trip auto-VAD)
    print("[in] question streamed; waiting for reply...")

    # Collect the reply until `quiet_s` of silence or the overall timeout.
    reply = []
    last_audio = time.monotonic()
    start = time.monotonic()
    while True:
        now = time.monotonic()
        if now - start > args.timeout_s:
            print("[reply] overall timeout reached.")
            break
        got = be.next_audio(INTERNAL_HZ)  # up to 1 s at a time
        if len(got) > 0:
            reply.append(got)
            last_audio = now
        elif reply and (now - last_audio) > args.quiet_s:
            print(f"[reply] {args.quiet_s}s quiet -> reply complete.")
            break
        else:
            time.sleep(0.02)

    be.close()

    if not reply:
        sys.exit("[reply] no audio received -- check the model id / key / network.")
    out = np.concatenate(reply)
    sf.write(args.out, out, INTERNAL_HZ)
    peak = float(np.max(np.abs(out))) if len(out) else 0.0
    print(f"[reply] {len(out)/INTERNAL_HZ:.2f}s, peak={peak:.3f} -> {args.out}")
    print(f"[ok] play it:  afplay {args.out}")


if __name__ == "__main__":
    main()
