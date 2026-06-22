"""Online delay estimator recovers a known loop delay (pure, no hardware)."""
import numpy as np

from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.delay_estimator import DelayEstimator

SR = 16000


def _make_ref(n, seed=0):
    """Broadband, speech-ish reference (white noise band-limited a touch)."""
    rng = np.random.default_rng(seed)
    x = rng.standard_normal(n).astype(np.float32)
    # light smoothing -> a sharper, more realistic correlation peak than pure white
    k = np.ones(4, dtype=np.float32) / 4
    return np.convolve(x, k, mode="same").astype(np.float32)


def _make_mic(played_ref, ref_base_mic, mic_start_idx, w, ratio=AEC_DRIFT_RATIO, noise=0.0, seed=1):
    """Mic echo per the model: mic[m] = played_ref[(m - ref_base_mic) * ratio]."""
    m = np.arange(mic_start_idx, mic_start_idx + w)
    echo = np.interp((m - ref_base_mic) * ratio, np.arange(len(played_ref)),
                     played_ref, left=0.0, right=0.0).astype(np.float32)
    if noise:
        echo = echo + np.random.default_rng(seed).standard_normal(w).astype(np.float32) * noise
    return echo


def test_recovers_known_loop_delay():
    ref = _make_ref(80000)
    seed_base = 8000                 # mic active before downlink (origin offset)
    D = int(0.120 * SR)              # true loop delay = 120 ms
    ref_base = seed_base + D
    mic_start = ref_base + SR        # 1 s into the echo
    w = SR // 2                      # 0.5 s window
    mic = _make_mic(ref, ref_base, mic_start, w)

    est = DelayEstimator()
    out = est.estimate(mic, mic_start, ref, seed_base)
    assert out is not None
    assert abs(out - ref_base) < 0.005 * SR          # within 5 ms
    assert abs(est.loop_delay_ms - 120.0) < 5.0
    assert est.confidence > 0.5


def test_tracks_and_smooths_over_calls():
    ref = _make_ref(120000)
    seed_base = 5000
    D = int(0.090 * SR)              # 90 ms
    ref_base = seed_base + D
    w = SR // 2
    est = DelayEstimator()
    out = None
    for j in range(6):               # several windows marching forward in time
        mic_start = ref_base + SR + j * w
        mic = _make_mic(ref, ref_base, mic_start, w, noise=0.01, seed=j)
        out = est.estimate(mic, mic_start, ref, seed_base)
    assert out is not None
    assert abs(out - ref_base) < 0.006 * SR          # converged within ~6 ms
    assert abs(est.loop_delay_ms - 90.0) < 6.0


def test_rejects_uncorrelated_mic_no_false_lock():
    ref = _make_ref(80000)
    seed_base = 8000
    w = SR // 2
    mic_start = 30000
    # Mic is independent noise -> no echo -> must NOT acquire a delay.
    mic = np.random.default_rng(99).standard_normal(w).astype(np.float32)
    est = DelayEstimator(conf_threshold=0.30)
    out = est.estimate(mic, mic_start, ref, seed_base)
    assert out is None                                # never locked
    assert est.confidence < 0.30
    assert est.loop_delay_ms is None


def test_silent_reference_holds():
    est = DelayEstimator()
    out = est.estimate(np.zeros(8000, np.float32), 10000, np.zeros(0, np.float32), 0)
    assert out is None
