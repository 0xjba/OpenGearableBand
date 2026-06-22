#!/usr/bin/env python3
"""Gemini Live diagnosis: (A) text-in->audio-out path, (B) audio-in w/ transcription."""
import asyncio, subprocess
import numpy as np, soundfile as sf, soxr
from dotenv import load_dotenv
from google import genai
from google.genai import types

load_dotenv()
MODEL = "gemini-3.1-flash-live-preview"


def synth(text="What is the capital of France?"):
    subprocess.run(["say", "-o", "/tmp/q.aiff", text], check=True)
    a, sr = sf.read("/tmp/q.aiff", dtype="float32")
    if a.ndim > 1: a = a.mean(axis=1)
    if sr != 16000: a = soxr.resample(a, sr, 16000).astype(np.float32)
    return a


def pcm(a):
    return (np.clip(a, -1, 1) * 32767).astype("<i2").tobytes()


async def drain(session, label, timeout=15):
    n, total, texts = 0, 0, []
    async def rd():
        nonlocal n, total
        async for r in session.receive():
            sc = r.server_content
            if getattr(r, "data", None):
                n += 1; total += len(r.data)
            if sc is not None:
                it = getattr(sc, "input_transcription", None)
                ot = getattr(sc, "output_transcription", None)
                if it and getattr(it, "text", None): texts.append(f"IN<{it.text!r}>")
                if ot and getattr(ot, "text", None): texts.append(f"OUT<{ot.text!r}>")
                if getattr(sc, "turn_complete", None):
                    print(f"[{label}] turn_complete"); return
    try:
        await asyncio.wait_for(rd(), timeout=timeout)
    except asyncio.TimeoutError:
        print(f"[{label}] (timeout)")
    print(f"[{label}] audio_chunks={n} bytes={total} transcripts={texts}")
    return total


async def main():
    cfg = {
        "response_modalities": ["AUDIO"],
        "system_instruction": "You are concise. One short sentence.",
        "input_audio_transcription": {},
        "output_audio_transcription": {},
    }
    client = genai.Client()

    # --- Probe A: text input -> audio output ---
    async with client.aio.live.connect(model=MODEL, config=cfg) as s:
        print("[A] text-in: 'What is the capital of France?'")
        await s.send_client_content(
            turns=types.Content(role="user", parts=[types.Part(text="What is the capital of France?")]),
            turn_complete=True)
        await drain(s, "A")

    # --- Probe B: audio input + explicit stream-end ---
    q = synth()
    print(f"[B] audio-in: {len(q)/16000:.2f}s, peak={np.max(np.abs(q)):.3f}")
    async with client.aio.live.connect(model=MODEL, config=cfg) as s:
        chunk = 320
        for i in range(0, len(q), chunk):
            await s.send_realtime_input(audio=types.Blob(data=pcm(q[i:i+chunk]), mime_type="audio/pcm;rate=16000"))
            await asyncio.sleep(0.02)
        await s.send_realtime_input(audio_stream_end=True)  # explicit end-of-turn
        print("[B] audio sent + stream_end")
        await drain(s, "B")


asyncio.run(main())
