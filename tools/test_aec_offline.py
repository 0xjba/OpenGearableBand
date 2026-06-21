#!/usr/bin/env python3
"""Synthetic unit test for the offline AEC (no hardware, no real recording).

Models the REAL case: a WIDEBAND far/echo source (like the music we stream) that is
loud, plus a quieter near-end "voice". The AEC must cut the dominant echo (high
broadband ERLE) while leaving the near-end mostly intact. Wideband (not pure tones)
is deliberate -- it matches real speech/music and avoids the periodicity/frequency
pathologies that pure tones create for delay estimation and ERLE."""
import numpy as np
import aec_offline as aec

SR = 16000


def _noise(n, amp, seed):
    """Wideband (white-ish) signal, like real echo content."""
    rng = np.random.default_rng(seed)
    return np.clip(rng.standard_normal(n) * amp * 32767, -32768, 32767).astype(np.int16)


def test_aec_cuts_wideband_echo():
    # Real geometry: the mic STARTS `offset` samples into the reference (it only begins
    # once dictation triggers), so mic[t] = 0.6*ref[t+offset] + near[t]  -- the mic LAGS
    # the ref by a positive offset. ref is made longer so the mic can sample ahead.
    n = SR * 4
    offset = 1200
    ref = _noise(n + offset, 0.5, seed=1)           # loud wideband far = speaker output
    echo = (ref[offset:offset + n].astype(np.float64) * 0.6).astype(np.int16)  # loud echo
    near = _noise(n, 0.06, seed=2)                  # quieter, uncorrelated near-end "voice"
    mic = np.clip(echo.astype(np.int32) + near.astype(np.int32), -32768, 32767).astype(np.int16)

    clean, (a, b), erle = aec.run_aec(ref, mic, sr=SR)

    # bulk offset found within a frame or so; no drift in the synthetic signal
    assert abs(a - offset) <= 256, f"bulk offset est off: {a} vs {offset}"
    assert abs(b) < 1e-3, f"spurious drift detected: {b}"
    # the dominant echo is removed -> big drop in energy (broadband ERLE)
    assert erle >= 10.0, f"ERLE too low ({erle:.1f} dB) -- echo not cancelled"
    # the near-end survives (uncorrelated with ref, so AEC shouldn't remove it).
    # Compare over the same span clean covers (nn may be < n if not a frame multiple).
    nn = len(clean)
    near_energy = float(np.mean(near[:nn].astype(np.float64) ** 2))
    clean_energy = float(np.mean(clean.astype(np.float64) ** 2))
    assert clean_energy >= 0.2 * near_energy, "near-end was destroyed"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"PASS {name}")
    print("all passed")
