import numpy as np, soundfile as sf, glob, os
from voiceio.aec import DtlnAec

ROOT = "/Users/0xjba/Projects/gestureband/tools/DTLN-aec"


def test_cancels_bundled_far_end_echo():
    mic_path = sorted(glob.glob(os.path.join(ROOT, "audio_samples", "*_farend_singletalk_mic.wav")))[0]
    lpb_path = mic_path.replace("_mic.wav", "_lpb.wav")
    mic, sr = sf.read(mic_path)
    lpb, _ = sf.read(lpb_path)
    n = (min(len(mic), len(lpb)) // 128) * 128
    aec = DtlnAec(os.path.join(ROOT, "pretrained_models", "dtln_aec_512"))
    clean = aec.process(mic[:n].astype("float32"), lpb[:n].astype("float32"))
    skip = 2 * sr   # let it converge
    erle = 10 * np.log10((np.mean(mic[skip:n] ** 2) + 1e-12) / (np.mean(clean[skip:] ** 2) + 1e-12))
    assert erle > 30.0, f"ERLE too low: {erle:.1f} dB (wrapper not cancelling like run_aec.py)"


def test_reset_gives_repeatable_output():
    # Same input twice (with reset between) -> identical output (deterministic, state cleared).
    rng = np.random.default_rng(0)
    mic = rng.standard_normal(128 * 20).astype("float32") * 0.1
    ref = rng.standard_normal(128 * 20).astype("float32") * 0.1
    aec = DtlnAec(os.path.join(ROOT, "pretrained_models", "dtln_aec_512"))
    a = aec.process(mic, ref)
    aec.reset()
    b = aec.process(mic, ref)
    assert np.allclose(a, b, atol=1e-6)
