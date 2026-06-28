"""Pure reference-alignment math for the orchestrator (no hardware/model needed)."""
import numpy as np

from voiceio.orchestrator import aligned_reference


def test_identity_when_ratio_one_no_delay():
    ref = np.arange(1000, dtype=np.float32)
    out = aligned_reference(ref, ref_base_mic=0, mic_count_start=100, n=50, ratio=1.0)
    assert np.allclose(out, ref[100:150])


def test_zeros_outside_the_played_ring():
    ref = np.arange(100, dtype=np.float32)
    # mic block starts past the end of what's been played -> no echo -> zeros.
    out = aligned_reference(ref, ref_base_mic=0, mic_count_start=500, n=32, ratio=1.0)
    assert np.all(out == 0.0)


def test_empty_ring_returns_zeros():
    out = aligned_reference(np.zeros(0, np.float32), 0, 0, 64, ratio=0.99206)
    assert out.shape == (64,) and np.all(out == 0.0)


def test_recovers_drifted_delayed_echo():
    # Far-end reference (a tone burst), and a simulated mic echo = the reference heard
    # on the mic clock with a loop delay: mic-sample m hears ref[(m - base)*ratio].
    # aligned_reference must reconstruct exactly that, aligning at lag 0.
    sr = 16000
    t = np.arange(4 * sr) / sr
    ref = (0.5 * np.sin(2 * np.pi * 440 * t)).astype(np.float32)
    ratio = 15873.0 / 16000.0          # device plays slow (AEC_DRIFT_RATIO)
    base = 1200                         # loop delay in mic samples (dl_origin + delay)
    m0, n = 30000, 4096
    m = np.arange(m0, m0 + n)
    echo = np.interp((m - base) * ratio, np.arange(len(ref)), ref).astype(np.float32)

    out = aligned_reference(ref, ref_base_mic=base, mic_count_start=m0, n=n, ratio=ratio)
    # Reconstruction matches the true echo (this is the alignment the AEC relies on).
    assert np.corrcoef(out, echo)[0, 1] > 0.999
    # And it is meaningfully BETTER aligned than ignoring the drift (ratio=1 wrong model):
    wrong = aligned_reference(ref, ref_base_mic=base, mic_count_start=m0, n=n, ratio=1.0)
    assert np.corrcoef(out, echo)[0, 1] > np.corrcoef(wrong, echo)[0, 1]
