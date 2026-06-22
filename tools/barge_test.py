#!/usr/bin/env python3
"""De-risk full-duplex barge-in: does a VAD on the neural-AEC'd mic fire when the
user talks over the AI, but stay quiet on echo alone (no false barge-ins)?

Protocol for the recording <base>: hold band to ear, start the AI stream, stay
SILENT for the first <silent> seconds (AI plays alone = echo only), THEN talk in
short bursts. We DTLN-clean the mic, run webrtcvad on the cleaned stream AND the raw
mic, and report:
  * false-barge-ins in the silent prefix (want ~0 on cleaned; raw mic shows what the
    echo alone would trigger without AEC)
  * detection in the talk region (want high on cleaned)

Run:  tools/dtln-venv/bin/python tools/barge_test.py <base> <silent_seconds>
"""
import sys, os, wave
import numpy as np
import soundfile as sf
import webrtcvad

ROOT = "/Users/0xjba/Projects/gestureband"
HERE = os.path.join(ROOT, "tools")
sys.path.insert(0, os.path.join(HERE, "DTLN-aec"))
import tensorflow.lite as tflite
from run_aec import process_file

SR = 16000
MODEL = os.path.join(HERE, "DTLN-aec/pretrained_models/dtln_aec_512")
FRAME_MS = 30
ONSET_FRAMES = 5          # 5x30ms = 150ms of speech => declare a barge-in (debounce)


def read_i16(path):
    with wave.open(path, "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float64)


def dtln_clean(ref, mic, drift=-0.0075, off=0):   # -0.75% = correct sign (device plays slow)
    i1 = tflite.Interpreter(model_path=MODEL + "_1.tflite"); i1.allocate_tensors()
    i2 = tflite.Interpreter(model_path=MODEL + "_2.tflite"); i2.allocate_tensors()
    idx = np.arange(len(mic), dtype=np.float64) * (1.0 + drift) + off
    lpb = np.interp(idx, np.arange(len(ref), dtype=np.float64), ref) / 32768.0
    os.makedirs(os.path.join(HERE, "dtln_in"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "dtln_out"), exist_ok=True)
    sf.write(os.path.join(HERE, "dtln_in/b_mic.wav"), mic / 32768.0, SR)
    sf.write(os.path.join(HERE, "dtln_in/b_lpb.wav"), lpb, SR)
    process_file(i1, i2, os.path.join(HERE, "dtln_in/b_mic.wav"),
                 os.path.join(HERE, "dtln_out/b_proc.wav"))
    proc, _ = sf.read(os.path.join(HERE, "dtln_out/b_proc.wav"))
    return proc


def vad_flags(x, level):
    """Per-30ms speech/no-speech flags from webrtcvad."""
    pcm = (np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes()
    vad = webrtcvad.Vad(level)
    fb = int(SR * FRAME_MS / 1000) * 2          # bytes per 30ms frame
    return np.array([1 if vad.is_speech(pcm[i:i + fb], SR) else 0
                     for i in range(0, len(pcm) - fb + 1, fb)])


def frame_rms(x):
    fl = int(SR * FRAME_MS / 1000)
    m = len(x) // fl
    return np.sqrt((np.clip(x[:m * fl], -1, 1).reshape(m, fl) ** 2).mean(1) + 1e-12)


def onsets(barge):
    """Count rising edges where >=ONSET_FRAMES consecutive barge frames begin."""
    n, c, active = 0, 0, False
    for f in barge:
        c = c + 1 if f else 0
        if c >= ONSET_FRAMES and not active:
            n += 1; active = True
        if not f:
            active = False
    return n


def analyze(name, x, fps, silent_s, level, margin_db, floor=None):
    """Barge = webrtcvad-speech AND rms > (echo floor + margin). Floor learned from
    the silent-AI prefix. Returns the learned floor for reuse."""
    flags = vad_flags(x, level)
    rms = frame_rms(x)
    k = min(len(flags), len(rms)); flags, rms = flags[:k], rms[:k]
    s = int(silent_s * fps)
    if floor is None:
        floor = np.percentile(rms[:s], 75)          # residual-echo level (AI alone)
    gate = floor * (10 ** (margin_db / 20.0))
    barge = (flags == 1) & (rms > gate)
    sil, talk = barge[:s], barge[s:]
    talk_rms = np.percentile(rms[s:], 90)
    print(f"  {name:8s}: echo-floor={20*np.log10(floor+1e-9):6.1f}dB  "
          f"talk-level={20*np.log10(talk_rms+1e-9):6.1f}dB  margin={20*np.log10(talk_rms/floor):4.1f}dB"
          f"  |  FALSE barge-ins(silent)={onsets(sil)}  detections(talk)={onsets(talk)}")
    return floor


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "barge1"
    silent_s = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    ref = read_i16(os.path.join(ROOT, f"{base}_ref.wav"))
    mic = read_i16(os.path.join(ROOT, f"{base}_mic.wav"))
    print(f"{base}: {len(mic)/SR:.1f}s, silent-AI prefix {silent_s:.0f}s")
    proc = dtln_clean(ref, mic)
    fps = 1000.0 / FRAME_MS
    print("barge = speech(webrtcvad L2) AND >6dB over the echo floor:")
    analyze("RAW mic", mic / 32768.0, fps, silent_s, 2, 6.0)
    analyze("AEC'd", proc, fps, silent_s, 2, 6.0)
    print("\nWANT (AEC'd): margin >~10dB, FALSE barge-ins=0, detections track your bursts.")
    print("The RAW-vs-AEC'd margin gap = what the neural AEC buys: it drops the echo floor")
    print("so your voice clears it. Needs a real silent-AI prefix to measure the floor.")


if __name__ == "__main__":
    main()
