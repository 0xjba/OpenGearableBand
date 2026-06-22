import numpy as np
from voiceio.resample import resample_ref

def test_output_index_maps_to_ratio_times_i():
    ref = np.arange(20000, dtype=np.float32)        # ramp: value == index
    out = resample_ref(ref, ratio=0.9925, n=16000)
    assert len(out) == 16000
    for i in (0, 4000, 8000, 15000):
        assert abs(out[i] - 0.9925 * i) < 1.0       # out[i] ≈ ref[ratio*i] == ratio*i

def test_identity_when_ratio_is_one():
    ref = np.arange(1000, dtype=np.float32)
    out = resample_ref(ref, ratio=1.0, n=1000)
    assert np.allclose(out, ref)
