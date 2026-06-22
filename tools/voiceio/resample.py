"""Resample the far-end AEC reference onto the device (mic) clock: out[i] = ref[ratio*i],
length n. ratio = device_rate/16000 (from voiceio.drift); ratio<1 (device plays slow)
stretches the reference. Linear interpolation -- at a ~0.75% rate change the error is
negligible, and this matches the validated offline drift-resampler. (A polyphase soxr
upgrade is a future quality lever; unnecessary for v1.)"""
import numpy as np

def resample_ref(ref, ratio, n):
    idx = np.arange(n, dtype=np.float64) * ratio
    return np.interp(idx, np.arange(len(ref), dtype=np.float64), ref).astype(np.float32)
