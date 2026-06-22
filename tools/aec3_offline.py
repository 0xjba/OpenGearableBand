#!/usr/bin/env python3
"""Offline WebRTC AEC3 echo-cancellation proof for the gestureband voice-loop.

Speex (linear, fixed-delay, no drift compensation) could not cancel the band's echo
(ERLE ~0 dB across every recording). AEC3 -- the WebRTC Audio Processing Module's
echo canceller -- has a real adaptive delay estimator, continuous clock-drift
tracking, and nonlinear residual suppression, which is exactly what our path needs
(remote BLE speaker, ~0.78% I2S clock drift, lossy LC3 codec, cheap nonlinear speaker).

We use the AEC3 that ships prebuilt inside LiveKit's `livekit.rtc.apm` (no libwebrtc
build). It wants 10 ms frames (160 samples @ 16 kHz) fed as two interleaved streams:
  * process_reverse_stream(far)  -- the speaker reference (what was played)
  * process_stream(near)         -- the mic (echo + your voice), cancelled in place

ALIGNMENT + DRIFT (the crux): audio_tx.py --record trims <base>_ref.wav to start a few
hundred ms before dictation began, so ref and mic are coarse-aligned. But the band's
I2S plays at ~0.78% off 16 kHz, so the echo delay GROWS ~850 ms over a 60 s clip --
more than AEC3's adaptive filter can chase if fed naively (measured: AEC3 alone decays
14 dB -> 2 dB across a long monologue). So we first RESAMPLE the reference onto the mic
clock to cancel the bulk drift (leaving AEC3 a near-constant delay it handles easily),
then feed it a small LEAD ahead of the mic. We sweep (drift, lead) with AEC3's own ERLE
as the objective -- no fragile cross-correlation, just maximize cancellation. With
drift compensation AEC3 holds ~20 dB FLAT across the whole clip (validated synthetic).

This proves the live pipeline must continuously drift-correct the reference (via the
ts32 mic-capture timestamp + buffer-status feedback), not assume a fixed delay.

Usage:  python3 aec3_offline.py speech1     # uses speech1_ref.wav + speech1_mic.wav
"""
import sys
import wave
import numpy as np

try:
    from livekit.rtc.apm import AudioProcessingModule
    from livekit.rtc import AudioFrame
except ImportError:
    AudioProcessingModule = None

SR = 16000
F = 160                 # 10 ms frame @ 16 kHz (AEC3's required block size)
DRIFT_GRID = np.linspace(0.0, 0.012, 7)   # candidate clock drift, 0..1.2% (ours ~0.78%)
LEAD_GRID = range(0, 41, 8)               # reference lead ahead of mic, 0..400 ms


def _read_wav(path):
    with wave.open(path, "rb") as w:
        if not (w.getnchannels() == 1 and w.getsampwidth() == 2 and w.getframerate() == SR):
            raise ValueError(f"{path}: need 16 kHz mono 16-bit")
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


def _write_wav(path, pcm_i16):
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes(pcm_i16.astype(np.int16).tobytes())


def drift_resample(ref, n, drift):
    """Resample `ref` onto the mic clock: refc[i] = ref[(1+drift)*i], length n. Cancels
    the bulk clock drift so AEC3 sees a near-constant (not growing) echo delay."""
    idx = np.arange(n, dtype=np.float64) * (1.0 + drift)
    return np.interp(idx, np.arange(len(ref), dtype=np.float64), ref.astype(np.float64)).astype(np.int16)


def run_aec3(far, near, stream_delay_ms=20):
    """Feed far (reference) and near (mic) through AEC3 in lockstep. far and near are
    already relatively shifted by the caller; stream_delay_ms is the small residual
    acoustic delay hint. Returns the cancelled near (int16), same length as the overlap."""
    apm = AudioProcessingModule(
        echo_cancellation=True,     # AEC3
        noise_suppression=False,    # off: keep ERLE a pure echo-removal measure
        high_pass_filter=True,      # removes DC / sub-100 Hz rumble
        auto_gain_control=False,    # off: don't rescale the residual
    )
    apm.set_stream_delay_ms(stream_delay_ms)
    n = (min(len(far), len(near)) // F) * F
    out = np.empty(n, dtype=np.int16)
    for i in range(0, n, F):
        rf = AudioFrame(far[i:i + F].tobytes(), SR, 1, F)
        apm.process_reverse_stream(rf)
        nf = AudioFrame(near[i:i + F].tobytes(), SR, 1, F)
        apm.process_stream(nf)              # cancels echo in place
        out[i:i + F] = np.frombuffer(nf.data, dtype=np.int16)
    return out


def erle_db(mic_in, residual):
    """ERLE = 10 log10( power(mic_in) / power(residual) ) over the converged region.
    High = echo removed. Caveat (same as Speex tool): if the near-end voice were the
    loud part, a high ERLE could mean the AEC ate the voice -- always also LISTEN to
    *_clean.wav and confirm your voice survives."""
    e = float(np.mean(mic_in.astype(np.float64) ** 2)) + 1.0
    r = float(np.mean(residual.astype(np.float64) ** 2)) + 1.0
    return 10.0 * np.log10(e / r)


def main():
    if AudioProcessingModule is None:
        sys.exit("livekit not installed -- run: pip3 install livekit")
    if len(sys.argv) != 2:
        sys.exit("usage: python3 aec3_offline.py <basename>  (uses <base>_ref.wav + <base>_mic.wav)")
    base = sys.argv[1]
    try:
        ref = _read_wav(f"{base}_ref.wav")
        mic = _read_wav(f"{base}_mic.wav")
    except FileNotFoundError as e:
        sys.exit(f"missing recording ({e}) -- capture first with: python3 audio_tx.py "
                 f"aecspeech16k.wav --lib tools/lib/liblc3.dylib --duplex --record {base}")
    if len(mic) < SR or len(ref) < SR:
        sys.exit(f"recording too short (ref={len(ref)}, mic={len(mic)} samples). The mic "
                 f"uplink only streams in MODE_DICTATION -- re-capture holding the band to "
                 f"your ear + speaking so it enters dictation (expect 'N mic frames received', N>0).")

    skip = min(3 * SR, len(mic) // 3)       # let AEC3 converge before scoring (~3 s)

    def score(drift, lead):
        refc = drift_resample(ref, len(mic) + lead * F, drift)
        out = run_aec3(refc[lead * F:], mic)
        nn = len(out)
        if nn <= skip:
            return -99.0, None
        return erle_db(mic[skip:nn], out[skip:nn]), out

    # Stage A: find drift at a mid lead (drift governs long-term hold).
    mid_lead = 16
    drift = max(DRIFT_GRID, key=lambda d: score(d, mid_lead)[0])
    # Stage B: find lead at the chosen drift (lead governs initial lock).
    best = max(((score(drift, l)[0], drift, l) for l in LEAD_GRID), key=lambda t: t[0])
    erle, drift, lead = best
    _, out = score(drift, lead)

    if out is None:
        sys.exit("no usable overlap between ref and mic")
    _write_wav(f"{base}_clean.wav", out)
    nn = len(out)
    print(f"AEC3 best lock: clock drift {drift * 100:.2f}%, reference lead {lead * F * 1000 // SR} ms")
    print(f"ERLE: {erle:.1f} dB   (Speex got ~0 dB here; >~12 dB is good, >20 dB is strong)")
    # per-window ERLE -- watch whether cancellation HOLDS across the long clip (drift test)
    win = 8 * SR
    segs = [f"{s // SR:>3}-{(s + win) // SR}s: {erle_db(mic[s:s + win], out[s:s + win]):4.1f}"
            for s in range(skip, nn - win, win)]
    if segs:
        print("hold over time (dB): " + " | ".join(segs))
    print(f"wrote {base}_clean.wav -- LISTEN: echo (the speech) should be gone, your voice should remain")


if __name__ == "__main__":
    main()
