"""OLED-variant voice->text bridge: band mic -> LC3 uplink -> Gemini -> OLED text.
Run from YOUR Terminal (macOS BT): tools/dtln-venv/bin/python tools/oled_bridge.py
See docs/superpowers/specs/2026-08-26-oled-gemini-bridge-design.md."""
import numpy as np

def agc(pcm, target_peak=0.25, max_gain=40.0):
    """Scale float32 PCM so its peak approaches target_peak (full-scale=1.0),
    capped at max_gain so silence isn't amplified into noise. Only boosts."""
    peak = float(np.max(np.abs(pcm))) if pcm.size else 0.0
    if peak < 1e-6:
        return pcm
    gain = min(max_gain, target_peak / peak)
    gain = max(gain, 1.0)
    return np.clip(pcm * gain, -1.0, 1.0).astype(np.float32)

def wrap_pages(text, cols=12, rows=2):
    """Word-wrap text into lines <= cols chars, grouped into pages of <= rows lines.
    Long words are hard-split. Returns list[list[str]]."""
    words = text.split()
    lines, cur = [], ""
    for w in words:
        while len(w) > cols:
            if cur:
                lines.append(cur); cur = ""
            lines.append(w[:cols]); w = w[cols:]
        if not cur:
            cur = w
        elif len(cur) + 1 + len(w) <= cols:
            cur += " " + w
        else:
            lines.append(cur); cur = w
    if cur:
        lines.append(cur)
    if not lines:
        lines = [""]
    return [lines[i:i + rows] for i in range(0, len(lines), rows)]
