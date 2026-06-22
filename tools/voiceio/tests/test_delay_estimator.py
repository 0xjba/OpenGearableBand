"""Online alignment estimator recovers the true ref_base_mic (pure, no hardware)."""
import numpy as np

from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.delay_estimator import DelayEstimator

SR = 16000


def _make_ref(n, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.standard_normal(n).astype(np.float32)
    return np.convolve(x, np.ones(4, dtype=np.float32) / 4, mode="same").astype(np.float32)


def _make_mic(played_ref, ref_base, mic_start, w, ratio=AEC_DRIFT_RATIO, noise=0.0, seed=1):
    """Mic echo per the model: mic[m] = played_ref[(m - ref_base) * ratio]."""
    m = np.arange(mic_start, mic_start + w)
    echo = np.interp((m - ref_base) * ratio, np.arange(len(played_ref)),
                     played_ref, left=0.0, right=0.0).astype(np.float32)
    if noise:
        echo = echo + np.random.default_rng(seed).standard_normal(w).astype(np.float32) * noise
    return echo


def test_recovers_alignment_from_offset_center():
    ref = _make_ref(120000)
    ref_base = 40000                       # true mic index where played_ref[0] is heard
    mic_start = ref_base + SR              # 1 s into the echo
    w = SR // 2
    mic = _make_mic(ref, ref_base, mic_start, w)
    est = DelayEstimator(search_ms=250.0)
    center = ref_base - int(0.080 * SR)    # playout-derived seed is 80 ms off
    out = est.estimate(mic, mic_start, ref, center)
    assert out is not None
    assert abs(out - ref_base) < 0.005 * SR            # recovered within 5 ms
    assert est.confidence > 0.5
    assert est.locked


def test_tracks_and_smooths_over_calls():
    ref = _make_ref(200000)
    ref_base = 33333
    w = SR // 2
    est = DelayEstimator(search_ms=250.0)
    out = None
    for j in range(6):
        mic_start = ref_base + SR + j * w
        mic = _make_mic(ref, ref_base, mic_start, w, noise=0.01, seed=j)
        center = ref_base + int(0.050 * SR)            # center 50 ms off (the other way)
        out = est.estimate(mic, mic_start, ref, center)
    assert out is not None
    assert abs(out - ref_base) < 0.006 * SR


def test_rejects_uncorrelated_mic_no_false_lock():
    ref = _make_ref(120000)
    w = SR // 2
    mic_start = 50000
    mic = np.random.default_rng(99).standard_normal(w).astype(np.float32)
    est = DelayEstimator(conf_threshold=0.30)
    out = est.estimate(mic, mic_start, ref, search_center=48000)
    assert out is None
    assert not est.locked
    assert est.confidence < 0.30


def test_silent_reference_holds():
    est = DelayEstimator()
    assert est.estimate(np.zeros(8000, np.float32), 10000, np.zeros(0, np.float32), 0) is None
