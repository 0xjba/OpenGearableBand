from voiceio.clocks import AEC_DRIFT_RATIO, I2S_HZ, PDM_HZ

def test_ratio_is_the_divider_ratio():
    assert AEC_DRIFT_RATIO == I2S_HZ / PDM_HZ

def test_ratio_in_validated_band():
    # -0.79..-0.80% is what the blind sweep + the audible end-to-end test confirmed.
    assert -0.0085 < (AEC_DRIFT_RATIO - 1.0) < -0.0070

def test_i2s_is_the_nrf_divider_value():
    # 32 MHz / 21 / 96 = 15873.0 -- the closest nRF I2S divider to 16000.
    assert abs(I2S_HZ - 32_000_000 / 21 / 96) < 1.0
