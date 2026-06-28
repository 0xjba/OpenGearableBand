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


def _lock(est, ref, ref_base, mic_start, w=SR // 2):
    """Acquire a lock by seeding the center at the true ref_base, return the locked value."""
    est.estimate(_make_mic(ref, ref_base, mic_start, w), mic_start, ref, ref_base)
    assert est.locked
    return est.current()


def test_locked_tracking_around_held_center_is_stable():
    # Once locked, the orchestrator centers the search on the HELD estimate (current()).
    # Repeated in-place tracking on matching echo must NOT drift.
    ref = _make_ref(300000)
    ref_base, w = 40000, SR // 2
    est = DelayEstimator()
    rb0 = _lock(est, ref, ref_base, ref_base + SR)
    for j in range(8):
        ms = ref_base + SR + j * w
        est.estimate(_make_mic(ref, ref_base, ms, w, noise=0.01, seed=j), ms, ref,
                     int(est.current()))           # center on the held estimate
    assert abs(est.current() - rb0) < 0.004 * SR   # stayed put (< 4 ms)


def test_garbage_gap_center_cannot_move_a_held_lock():
    # The inter-turn-silence bug: a playout-derived center that drifted by the gap (here a
    # wildly wrong center). Searching ±span around it misses the true echo -> low conf ->
    # the held lock is unchanged (this is what centering-on-held-estimate guarantees).
    ref = _make_ref(300000)
    ref_base, w = 40000, SR // 2
    est = DelayEstimator()
    rb0 = _lock(est, ref, ref_base, ref_base + SR)
    ms = ref_base + 2 * SR
    bad_center = ref_base - 15 * SR                 # ~15 s off, like the observed res -15753 ms
    est.estimate(_make_mic(ref, ref_base, ms, w), ms, ref, bad_center)
    assert est.confidence < est.conf_threshold      # window missed the echo
    assert est.current() == rb0                      # lock untouched


def test_hysteresis_clamps_per_update_step():
    # A confident estimate that implies a big jump (echo genuinely shifted) may move the lock
    # by at most max_step per update -- a single noisy frame can't yank it.
    ref = _make_ref(300000)
    ref_base, w = 40000, SR // 2
    est = DelayEstimator(max_step_ms=40.0)
    rb0 = _lock(est, ref, ref_base, ref_base + SR)
    shifted = ref_base + int(0.20 * SR)             # true alignment jumps 200 ms
    ms = ref_base + SR
    est.estimate(_make_mic(ref, shifted, ms, w), ms, ref, int(est.current()))
    assert abs(est.current() - rb0) <= est.max_step + 1   # moved at most max_step toward it
