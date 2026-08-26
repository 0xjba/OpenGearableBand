"""OLED-variant voice->text bridge: band mic -> LC3 uplink -> Gemini -> OLED text.
Run from YOUR Terminal (macOS BT): tools/dtln-venv/bin/python tools/oled_bridge.py
See docs/superpowers/specs/2026-08-26-oled-gemini-bridge-design.md."""
import io, os, wave
import numpy as np

def agc(pcm, target_peak=0.25, max_gain=40.0):
    """Scale float32 PCM so its peak approaches target_peak (full-scale=1.0),
    capped at max_gain so silence isn't amplified into noise. Only boosts."""
    assert pcm.dtype == np.float32, f"agc expects float32 PCM, got {pcm.dtype}"
    peak = float(np.max(np.abs(pcm))) if pcm.size else 0.0
    if not np.isfinite(peak) or peak < 1e-6:
        return pcm
    gain = min(max_gain, target_peak / peak)
    gain = max(gain, 1.0)
    return np.clip(pcm * gain, -1.0, 1.0).astype(np.float32)

def wrap_pages(text, cols=12, rows=2):
    """Word-wrap text into lines <= cols chars, grouped into pages of <= rows lines.
    Long words are hard-split. Returns list[list[str]]."""
    if cols < 1:
        raise ValueError("cols must be >= 1")
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


def pcm_to_wav(pcm_f32, rate=16000):
    """float32 [-1,1] mono -> 16-bit PCM WAV bytes (Gemini inline audio wants a
    container, not raw PCM)."""
    i16 = np.clip(pcm_f32 * 32767.0, -32768, 32767).astype("<i2")
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(i16.tobytes())
    return buf.getvalue()


SYSTEM_PROMPT = (
    "You are a wrist display assistant. Answer the spoken question in ONE short, "
    "glanceable sentence, ideally under 40 characters. No preamble, no markdown."
)


class GeminiBrain:
    """v1 all-in-one brain: one generate_content call does STT+LLM. Swappable seam --
    a future PipelineBrain (local STT -> gateway -> agent) implements the same process()."""

    def __init__(self, model="gemini-3.6-flash", api_key=None):
        from google import genai
        key = api_key or os.environ.get("GEMINI_API_KEY")
        if not key:
            raise RuntimeError("GEMINI_API_KEY not set (put it in a git-ignored .env)")
        self._client = genai.Client(api_key=key)
        self._model = model

    def process(self, pcm_f32_16k):
        """utterance audio (float32 16 kHz) -> reply text."""
        from google.genai import types
        wav = pcm_to_wav(pcm_f32_16k, 16000)
        resp = self._client.models.generate_content(
            model=self._model,
            contents=[types.Content(
                role="user",
                parts=[types.Part.from_bytes(data=wav, mime_type="audio/wav")],
            )],
            config=types.GenerateContentConfig(system_instruction=SYSTEM_PROMPT),
        )
        text = (resp.text or "").strip()
        return text if text else "[no reply]"
