import glob, os
import numpy as np
import soundfile as sf
from voiceio.pipeline import run_session

ROOT = "/Users/0xjba/Projects/gestureband/tools/DTLN-aec"
MODEL = os.path.join(ROOT, "pretrained_models", "dtln_aec_512")

def test_end_to_end_cancels_echo_and_returns_onsets():
    mic_path = sorted(glob.glob(os.path.join(ROOT, "audio_samples", "*_farend_singletalk_mic.wav")))[0]
    lpb_path = mic_path.replace("_mic.wav", "_lpb.wav")
    mic, sr = sf.read(mic_path)
    lpb, _ = sf.read(lpb_path)
    clean, onsets = run_session(lpb.astype("float32"), mic.astype("float32"), MODEL, drift_ratio=1.0, sr=sr)
    n = len(clean)
    erle = 10 * np.log10((np.mean(mic[:n] ** 2) + 1e-12) / (np.mean(clean ** 2) + 1e-12))
    assert erle > 25.0, f"pipeline did not cancel echo end-to-end: ERLE {erle:.1f} dB"
    assert isinstance(onsets, list)            # far-end-only clip -> the list type is returned
    # resample_ref re-clocks the reference to len(mic), so the AEC's 128-multiple
    # truncation is driven by the mic length alone.
    assert len(clean) == (len(mic) // 128) * 128
