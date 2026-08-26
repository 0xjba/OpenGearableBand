import os, sys
import numpy as np
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
    assert float(np.max(np.abs(out))) <= 0.51

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
