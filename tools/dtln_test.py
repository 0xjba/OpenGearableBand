#!/usr/bin/env python3
"""Run DTLN-aec (mask-based NEURAL residual-echo suppressor) on our real recording,
to test the hypothesis the deep-research surfaced: a SUPPRESSOR may cancel echo where
the SUBTRACTIVE linear AECs (Speex/AEC3, ~2-4 dB) failed, because it doesn't need a
linearly-predictive reference.

DTLN-aec feeds mic + loopback in lockstep and handles the echo DELAY internally, but
NOT clock drift -- so we drift-resample our reference onto the mic clock first and
sweep (drift, offset), scoring by ERLE. Run with the venv that has tensorflow:
  tools/dtln-venv/bin/python tools/dtln_test.py <base>     # e.g. speech2
"""
import sys, os, wave
import numpy as np
import soundfile as sf

ROOT = "/Users/0xjba/Projects/gestureband"
HERE = os.path.join(ROOT, "tools")
sys.path.insert(0, os.path.join(HERE, "DTLN-aec"))
import tensorflow.lite as tflite
from run_aec import process_file

SR = 16000
MODEL = os.path.join(HERE, "DTLN-aec/pretrained_models/dtln_aec_512")
IN_DIR = os.path.join(HERE, "dtln_in")
OUT_DIR = os.path.join(HERE, "dtln_out")
# Drift sign matters: device I2S plays slower (15881 vs 16000), the CORRECT comp is
# NEGATIVE (~-0.75%) -- verified on barge1 (echo removed start-to-end vs leaking back
# at +0/+0.75%). Sweep both signs; do not assume positive.
DRIFTS = [-0.0075, -0.005, 0.0, 0.0075]
OFFSETS_MS = [0, 170, 340]             # skip from ref start (anchor-trim lead ~340 ms)


def read_i16(path):
    with wave.open(path, "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float64)


def drift_resample(ref, n, drift, off):
    """refc[i] = ref[(1+drift)*i + off], length n (off in samples)."""
    idx = np.arange(n, dtype=np.float64) * (1.0 + drift) + off
    return np.interp(idx, np.arange(len(ref), dtype=np.float64), ref)


def erle(mic, proc, skip=2 * SR):
    n = min(len(mic), len(proc))
    a, b = mic[skip:n], proc[skip:n]
    return 10.0 * np.log10((np.mean(a ** 2) + 1e-9) / (np.mean(b ** 2) + 1e-9))


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "speech2"
    os.makedirs(IN_DIR, exist_ok=True); os.makedirs(OUT_DIR, exist_ok=True)
    ref = read_i16(os.path.join(ROOT, f"{base}_ref.wav"))
    mic = read_i16(os.path.join(ROOT, f"{base}_mic.wav"))
    print(f"{base}: ref {len(ref)/SR:.1f}s  mic {len(mic)/SR:.1f}s")
    micf = mic / 32768.0

    i1 = tflite.Interpreter(model_path=MODEL + "_1.tflite"); i1.allocate_tensors()
    i2 = tflite.Interpreter(model_path=MODEL + "_2.tflite"); i2.allocate_tensors()

    sf.write(os.path.join(IN_DIR, "s_mic.wav"), micf, SR)
    best = None
    for d in DRIFTS:
        for off_ms in OFFSETS_MS:
            off = off_ms * SR // 1000
            lpb = drift_resample(ref, len(mic), d, off) / 32768.0
            sf.write(os.path.join(IN_DIR, "s_lpb.wav"), lpb, SR)
            outp = os.path.join(OUT_DIR, "s_processed.wav")
            process_file(i1, i2, os.path.join(IN_DIR, "s_mic.wav"), outp)
            proc, _ = sf.read(outp)
            e = erle(micf, proc)
            print(f"  drift {d*100:.2f}%  offset {off_ms:3d}ms  ->  ERLE {e:5.1f} dB")
            if best is None or e > best[0]:
                best = (e, d, off_ms, proc.copy())
    e, d, off_ms, proc = best
    out = os.path.join(ROOT, f"{base}_dtln_clean.wav")
    sf.write(out, proc, SR)
    nn = min(len(micf), len(proc))
    win = 8 * SR
    segs = [f"{s//SR:>3}-{(s+win)//SR}s:{erle(micf[s:s+win], proc[s:s+win], 0):4.1f}"
            for s in range(2 * SR, nn - win, win)]
    print(f"\nBEST: drift {d*100:.2f}%, offset {off_ms}ms -> ERLE {e:.1f} dB "
          f"(Speex/AEC3 got 2-4 dB here)")
    if segs:
        print("hold over time (dB): " + " | ".join(segs))
    print(f"wrote {base}_dtln_clean.wav -- LISTEN: is the AI speech gone, your voice kept?")


if __name__ == "__main__":
    main()
