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


import argparse, asyncio, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from voiceio.frame import parse_uplink
from voiceio.codec import Lc3Codec
from bleak import BleakScanner, BleakClient

UPLINK_UUID  = "47a10002-9b70-4c2e-8a1d-2f6b9e4a77c1"
DISPLAY_UUID = "e9a10002-4b2c-4d3e-9f5a-0123456789ab"
NAME_SUBSTR  = "gband"
FRAME_BYTES  = 40
SR_HZ        = 16000
GAP_MS       = 500      # no uplink frames for this long => utterance done
MIN_MS       = 300      # ignore shorter blips
PAGE_SECS    = 2.5      # host-driven scroll pace

async def find_band(timeout=15.0):
    found = {}
    def _cb(d, adv):
        found[d.address] = (d, (adv.local_name or d.name or ""))
    scanner = BleakScanner(detection_callback=_cb)
    await scanner.start(); await asyncio.sleep(8.0); await scanner.stop()
    for _, (d, name) in found.items():
        if NAME_SUBSTR in name.lower():
            return d
    return None

async def show(client, text):
    """Word-wrap + paginate text to the OLED, scrolling pages over time."""
    for page in wrap_pages(text, cols=12, rows=2):
        payload = "\n".join(page).encode("utf-8")
        await client.write_gatt_char(DISPLAY_UUID, payload, response=True)
        await asyncio.sleep(PAGE_SECS)

async def run(args):
    brain = GeminiBrain(model=args.model)
    codec = Lc3Codec(lib_path=os.path.join(os.path.dirname(__file__), "lib", "liblc3.dylib"))
    dev = await find_band()
    if not dev:
        print("band not found (advertising as gband-oled?)"); return 2
    print("connecting", dev.address)
    async with BleakClient(dev) as client:
        buf = []
        last_frame = [0.0]
        def on_notify(_c, data: bytearray):
            f = parse_uplink(bytes(data))
            for i in range(0, len(f.lc3) - FRAME_BYTES + 1, FRAME_BYTES):
                pcm = codec.decode(f.lc3[i:i+FRAME_BYTES]).astype(np.float32) / 32768.0
                buf.append(pcm)
            last_frame[0] = time.monotonic()
        await client.start_notify(UPLINK_UUID, on_notify)
        await show(client, "ready")
        print("ready: trigger dictation (d / pose) + speak")
        while True:
            await asyncio.sleep(0.1)
            if not buf:
                continue
            if (time.monotonic() - last_frame[0]) * 1000 < GAP_MS:
                continue
            pcm = np.concatenate(buf); buf.clear()
            if pcm.size / SR_HZ * 1000 < MIN_MS:
                continue
            pcm = agc(pcm)
            await show(client, "thinking...")
            try:
                reply = brain.process(pcm) or "(no reply)"
            except Exception as e:
                print("brain error:", e); reply = "error"
            print("reply:", reply)
            if args.dump:
                print(f"  ({pcm.size/SR_HZ:.1f}s utterance)")
            await show(client, reply)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="gemini-3.6-flash")
    ap.add_argument("--dump", action="store_true", help="print replies + timing")
    ap.add_argument("--wav", help="offline: run GeminiBrain on a WAV, skip BLE")
    args = ap.parse_args()
    try:
        from dotenv import load_dotenv; load_dotenv()
    except Exception:
        pass
    if args.wav:
        import wave
        w = wave.open(args.wav, "rb")
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32)/32768.0
        print("reply:", GeminiBrain(model=args.model).process(agc(pcm)))
        return 0
    return asyncio.run(run(args))

if __name__ == "__main__":
    sys.exit(main())
