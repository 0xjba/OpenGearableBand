import os, sys
import numpy as np
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from oled_bridge import agc

def test_agc_boosts_quiet_toward_target():
    quiet = (np.sin(np.linspace(0, 200, 16000)) * 0.023).astype(np.float32)
    out = agc(quiet, target_peak=0.25, max_gain=40.0)
    assert 0.2 < float(np.max(np.abs(out))) <= 0.26
    assert float(np.max(np.abs(out))) <= 1.0

def test_agc_caps_gain_on_silence():
    silence = np.zeros(16000, dtype=np.float32)
    out = agc(silence, target_peak=0.25, max_gain=40.0)
    assert float(np.max(np.abs(out))) == 0.0

def test_agc_leaves_loud_audio_roughly_alone():
    loud = (np.sin(np.linspace(0, 200, 16000)) * 0.5).astype(np.float32)
    out = agc(loud, target_peak=0.25, max_gain=40.0)
    assert 0.49 < float(np.max(np.abs(out))) <= 0.51   # boost-only: unchanged

from oled_bridge import wrap_pages

def test_wrap_short_fits_one_page_one_line():
    assert wrap_pages("Hi there", cols=12, rows=2) == [["Hi there"]]

def test_wrap_two_lines_one_page():
    assert wrap_pages("Response is okay", cols=12, rows=2) == [["Response is", "okay"]]

def test_wrap_paginates_long_text():
    pages = wrap_pages("one two three four five six seven eight", cols=12, rows=2)
    assert len(pages) >= 2
    assert all(len(p) <= 2 for p in pages)
    assert all(len(line) <= 12 for p in pages for line in p)

def test_agc_rejects_non_float32():
    with pytest.raises(AssertionError):
        agc(np.zeros(100, dtype=np.int16))

def test_agc_nan_input_returns_unchanged():
    x = np.array([np.nan, 0.1, -0.1], dtype=np.float32)
    out = agc(x)
    assert out is x   # NaN peak -> guard returns input unchanged

def test_wrap_word_exactly_cols_not_split():
    assert wrap_pages("a" * 12, cols=12) == [["a" * 12]]

def test_wrap_cols_zero_raises():
    with pytest.raises(ValueError):
        wrap_pages("hi", cols=0)
