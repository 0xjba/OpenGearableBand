"""Offline end-to-end voice-I/O on a recorded session: drift-resample the reference,
DTLN-cancel the echo from the mic, detect barge-ins on the clean output."""
import numpy as np
from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.resample import resample_ref
from voiceio.aec import DtlnAec
from voiceio.vad import BargeInVad

def run_session(ref, mic, model_dir, drift_ratio=AEC_DRIFT_RATIO, sr=16000):
    """ref, mic: float32 numpy arrays in [-1,1] (the recorded far-end reference + mic PCM).
    Returns (clean_mic, barge_in_onsets): the echo-cancelled mic (float32) and a list of
    barge-in onset times in seconds."""
    ref = np.asarray(ref, dtype=np.float32)
    mic = np.asarray(mic, dtype=np.float32)
    lpb = resample_ref(ref, drift_ratio, len(mic))         # reference on the mic clock
    n = (min(len(mic), len(lpb)) // 128) * 128             # DtlnAec needs a multiple of 128
    clean = DtlnAec(model_dir).process(mic[:n], lpb[:n])
    onsets = BargeInVad(sr=sr).process(clean)
    return clean, onsets
