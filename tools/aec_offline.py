#!/usr/bin/env python3
"""Offline acoustic echo cancellation proof for the gestureband voice-loop.

Loads <base>_ref.wav (reference = what the speaker played) and <base>_mic.wav
(mic = echo + the user's voice), aligns the reference onto the mic timeline, runs
Speex EchoCanceller, writes <base>_clean.wav, and prints ERLE.

Alignment is the hard part for our setup and is done in two parts:
  * BULK OFFSET -- the mic only starts streaming once dictation triggers (seconds
    into the reference), so the echo lags the reference by a large, content-dependent
    amount.
  * CLOCK DRIFT -- the band's I2S plays at a slightly different rate than the 16 kHz
    reference (~0.8% measured), so the lag changes *linearly* across the recording.
We probe the local echo lag at several points, linear-fit lag(t), and resample the
reference onto the mic timeline (far[i] = ref[(1+b)*i + a]) -- handling both at once.

IMPORTANT: use a NON-REPETITIVE reference (speech, or the provided pink-noise clip).
A looped/repetitive clip makes the alignment ambiguous (the same waveform recurs) and
the AEC cannot lock on.

Usage:  python3 aec_offline.py echo1        # uses echo1_ref.wav + echo1_mic.wav
"""
import sys
import wave
import numpy as np

try:
    from speexdsp import EchoCanceller
except ImportError:
    EchoCanceller = None

SR = 16000
FRAME = 256              # Speex process() frame, samples
FILTER = 4096            # echo-tail filter length, samples (~256 ms)


def _read_wav(path):
    with wave.open(path, "rb") as w:
        if not (w.getnchannels() == 1 and w.getsampwidth() == 2 and w.getframerate() == SR):
            raise ValueError(f"{path}: need 16 kHz mono 16-bit")
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


def _write_wav(path, pcm_i16):
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes(pcm_i16.astype(np.int16).tobytes())


def _local_lag(ref_f, mic_f, s, win, max_delay):
    """Lag (samples) by which ref leads the mic window [s, s+win), via FFT xcorr.
    Needs ref to extend to s + max_delay + win. Returns None if out of range."""
    if s + win > len(mic_f) or s + win + max_delay > len(ref_f):
        return None
    m = mic_f[s:s + win] - mic_f[s:s + win].mean()
    seg = ref_f[s:s + win + max_delay].copy()
    seg -= seg.mean()
    nfft = 1 << int(np.ceil(np.log2(len(seg) + win)))
    c = np.fft.irfft(np.fft.rfft(seg, nfft) * np.conj(np.fft.rfft(m, nfft)), nfft)
    return int(np.argmax(c[:max_delay + 1]))


def estimate_alignment(ref, mic):
    """Return (a, b): reference index for mic sample i is (1+b)*i + a.
    a = bulk offset (samples), b = per-sample drift slope. Probes the local lag at
    several points and linear-fits; for short clips with one usable probe, b=0."""
    ref_f = ref.astype(np.float64); mic_f = mic.astype(np.float64)
    max_delay = int(min(6 * SR, max(SR, len(ref) // 3)))
    win = int(min(4 * SR, max(SR // 2, len(mic) // 5)))
    s_hi = min(len(mic) - win, len(ref) - max_delay - win)
    pts = []
    if s_hi > 0:
        for s in np.linspace(0, s_hi, 6).astype(int):
            lag = _local_lag(ref_f, mic_f, int(s), win, max_delay)
            if lag is not None:
                pts.append((int(s), lag))
    if len(pts) >= 2:
        ss = np.array([p[0] for p in pts], float)
        ll = np.array([p[1] for p in pts], float)
        b, a = np.polyfit(ss, ll, 1)
        return float(a), float(b), pts
    # fallback: single bulk delay over the first usable span, no drift fit
    lag = _local_lag(ref_f, mic_f, 0, min(win, len(mic) // 2), max_delay)
    return float(lag or 0), 0.0, pts


def build_far(ref, mic, a, b):
    """Reference resampled onto the mic timeline: far[i] = ref[(1+b)*i + a]."""
    i = np.arange(len(mic))
    idx = np.clip((1.0 + b) * i + a, 0, len(ref) - 1)
    return np.interp(idx, np.arange(len(ref)), ref.astype(np.float64)).astype(np.int16)


def erle_db(mic_in, residual):
    """ERLE = 10 log10( power(mic_in) / power(residual) ), broadband, over the aligned
    region. High = the (loud, dominant) echo was removed; the residual is mostly the
    quieter near-end. VALID ONLY WHEN THE ECHO DOMINATES the mic (true for our band:
    the speaker is inches from the mic >> the user's voice). If the near-end were the
    loud one, a high ERLE could instead mean the AEC *destroyed the near-end* -- so
    always also LISTEN to *_clean.wav, never trust ERLE alone."""
    e = float(np.mean(mic_in.astype(np.float64) ** 2)) + 1e-9
    r = float(np.mean(residual.astype(np.float64) ** 2)) + 1e-9
    return 10.0 * np.log10(e / r)


def run_aec(ref, mic, sr=SR):
    """Returns (clean_i16, (a, b), erle_db). (a, b) = drift-aware alignment."""
    if EchoCanceller is None:
        raise RuntimeError("speexdsp not installed (see plan Task 2 Step 1)")
    a, b, _pts = estimate_alignment(ref, mic)
    far = build_far(ref, mic, a, b)
    ec = EchoCanceller.create(FRAME, FILTER, sr)
    out = bytearray()
    for i in range(0, len(mic) - FRAME + 1, FRAME):
        out.extend(ec.process(mic[i:i + FRAME].tobytes(), far[i:i + FRAME].tobytes()))
    clean = np.frombuffer(bytes(out), dtype=np.int16)
    nn = len(clean)
    skip = min(FILTER, max(0, nn - FRAME))   # skip Speex convergence (~256 ms)
    erle = erle_db(mic[skip:nn], clean[skip:])
    return clean, (a, b), erle


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: python3 aec_offline.py <basename>  (uses <base>_ref.wav + <base>_mic.wav)")
    base = sys.argv[1]
    try:
        ref = _read_wav(f"{base}_ref.wav")
        mic = _read_wav(f"{base}_mic.wav")
    except FileNotFoundError as e:
        sys.exit(f"missing recording ({e}) -- capture first with: "
                 f"python3 audio_tx.py aectest16k.wav --lib <liblc3> --duplex --record {base}")
    if len(mic) < SR // 10 or len(ref) < SR // 10:
        sys.exit(f"recording too short (ref={len(ref)}, mic={len(mic)} samples). The mic "
                 f"uplink only streams in MODE_DICTATION -- re-capture and hold the band to "
                 f"your ear + speak so it enters dictation; the capture should report "
                 f"'uplink (full-duplex): N mic frames received' with N>0.")
    clean, (a, b), erle = run_aec(ref, mic)
    _write_wav(f"{base}_clean.wav", clean)
    print(f"alignment: bulk offset {a / SR * 1000:.0f} ms, clock drift {-b * 100:+.2f}%")
    print(f"ERLE: {erle:.1f} dB   (higher = more echo removed; >~12 dB is good)")
    print(f"wrote {base}_clean.wav -- listen: echo should be down, your voice should remain")


if __name__ == "__main__":
    main()
