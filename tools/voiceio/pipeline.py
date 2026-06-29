"""Offline end-to-end AEC on a recorded session: drift-resample the reference, then
DTLN-cancel the echo from the mic. (Barge-in is NOT done here -- in the live loop the backend's
own VAD detects barge-ins on the forwarded clean signal; there is no local detector. This helper
is for offline AEC/ERLE verification.)"""
import numpy as np
from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.resample import resample_ref
from voiceio.aec import DtlnAec


def run_session(ref, mic, model_dir, drift_ratio=AEC_DRIFT_RATIO, sr=16000):
    """ref, mic: float32 numpy arrays in [-1,1] (the recorded far-end reference + mic PCM).
    Returns clean_mic: the echo-cancelled mic (float32)."""
    ref = np.asarray(ref, dtype=np.float32)
    mic = np.asarray(mic, dtype=np.float32)
    lpb = resample_ref(ref, drift_ratio, len(mic))         # reference on the mic clock
    n = (min(len(mic), len(lpb)) // 128) * 128             # DtlnAec needs a multiple of 128
    return DtlnAec(model_dir).process(mic[:n], lpb[:n])
