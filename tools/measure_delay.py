#!/usr/bin/env python3
"""Measure the real render->capture loop delay on the actual band (no guessing).

Answers "what is ref_delay_ms on THIS unit?" empirically: cross-correlates the
recorded far-end reference against the recorded mic to find the lag at which the
speaker echo lines up. Run it on several captures / units / reconnects to see how
STABLE the delay is -- that decides whether a measured constant is acceptable or we
need an online delay estimator (WebRTC AEC3 style), exactly the measure-first method
we used for the clock-drift ratio.

Input = the sidecar WAVs that `audio_tx.py --duplex --record <base>` already writes:
  <base>_ref.wav  (the streamed reference)   <base>_mic.wav  (the decoded uplink mic w/ echo)

Usage:
  tools/dtln-venv/bin/python tools/measure_delay.py <base>            # e.g. delaytest
  tools/dtln-venv/bin/python tools/measure_delay.py <base> --max-ms 400
"""
import argparse
import sys

import numpy as np
import soundfile as sf

sys.path.insert(0, "tools")
from voiceio.clocks import AEC_DRIFT_RATIO          # noqa: E402
from voiceio.resample import resample_ref           # noqa: E402

SR = 16000


def load(path):
    a, sr = sf.read(path, dtype="float32")
    if a.ndim > 1:
        a = a.mean(axis=1)
    if sr != SR:
        sys.exit(f"{path}: expected {SR} Hz, got {sr}")
    return a.astype(np.float32)


def best_lag(ref, mic, max_lag):
    """Normalized cross-correlation peak lag (in samples) of mic vs ref, searching
    delays 0..max_lag. Returns (lag, peak_corr in [-1,1])."""
    n = min(len(ref), len(mic))
    ref = ref[:n] - ref[:n].mean()
    mic = mic[:n] - mic[:n].mean()
    rstd = ref.std() + 1e-9
    # Slide mic back by `lag`: corr(lag) = <ref[:n-lag], mic[lag:n]> normalized.
    best = (-1.0, 0)
    # Coarse-to-fine would be faster; n is short enough for a direct search step.
    step = max(1, max_lag // 800)         # ~sub-ms resolution, bounded work
    for lag in range(0, max_lag, step):
        a = ref[: n - lag]
        b = mic[lag:n]
        if len(a) < SR:                    # need >=1 s overlap for a stable estimate
            break
        c = float(np.dot(a, b) / (len(a) * rstd * (b.std() + 1e-9)))
        if c > best[0]:
            best = (c, lag)
    return best[1], best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base", help="recording basename (expects <base>_ref.wav / _mic.wav)")
    ap.add_argument("--max-ms", type=float, default=400.0, help="max delay to search (ms)")
    args = ap.parse_args()

    ref = load(f"{args.base}_ref.wav")
    mic = load(f"{args.base}_mic.wav")
    # Put the reference on the mic clock first (so drift doesn't smear the correlation).
    lpb = resample_ref(ref, AEC_DRIFT_RATIO, len(mic))

    max_lag = int(args.max_ms * SR / 1000.0)
    lag, corr = best_lag(lpb, mic, max_lag)
    print(f"ref={len(ref)/SR:.1f}s  mic={len(mic)/SR:.1f}s")
    print(f"measured loop delay = {lag} samples = {lag*1000.0/SR:.1f} ms   (xcorr peak={corr:.3f})")
    if corr < 0.15:
        print("  ! weak correlation -- ensure the mic actually captured the speaker echo "
              "(force mic with serial 'j', keep the band near the speaker, make sound).")
    else:
        print(f"  -> try:  voice_loop.py --ref-delay-ms {lag*1000.0/SR:.0f}")
        print("  Run on 2-3 reconnects/units: if it's stable (+/- a few ms) a measured")
        print("  constant is fine; if it wanders, we build the online delay estimator.")


if __name__ == "__main__":
    main()
