import numpy as np
from voiceio.vad import BargeInVad


def test_fires_once_per_loud_burst_not_on_floor():
    sr = 16000
    n = 10 * sr
    rng = np.random.default_rng(0)
    x = rng.standard_normal(n).astype("float32") * 0.001  # residual-echo floor
    for t in (2, 5, 8):  # 3 loud bursts (~34 dB over floor)
        s = t * sr
        x[s : s + sr // 2] += rng.standard_normal(sr // 2).astype("float32") * 0.05
    onsets = BargeInVad(sr=sr, margin_db=12.0).process(x)
    assert len(onsets) == 3, f"got {len(onsets)} onsets: {onsets}"
    for got, want in zip(sorted(onsets), (2, 5, 8)):
        assert abs(got - want) < 0.3


def test_no_onsets_on_pure_floor():
    sr = 16000
    rng = np.random.default_rng(1)
    x = rng.standard_normal(8 * sr).astype("float32") * 0.001
    assert BargeInVad(sr=sr, margin_db=12.0).process(x) == []


def test_absolute_floor_rejects_quiet_relatively_loud_blips():
    # Near-silent stream (~-80 dBFS floor) with blips that are >12 dB OVER the floor but
    # still below the absolute floor (-60 dBFS) -- the residual-echo case that false-fired.
    # The relative gate alone would fire; the absolute floor must reject them.
    sr = 16000
    rng = np.random.default_rng(2)
    x = rng.standard_normal(8 * sr).astype("float32") * 0.0001          # ~-80 dBFS floor
    for t in (2, 4, 6):
        s = t * sr
        x[s : s + sr // 2] += rng.standard_normal(sr // 2).astype("float32") * 0.0008  # ~-62 dBFS
    assert BargeInVad(sr=sr, margin_db=12.0, abs_floor_db=-60.0).process(x) == []
