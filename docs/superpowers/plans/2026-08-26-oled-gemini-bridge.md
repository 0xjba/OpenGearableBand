# OLED voice→text bridge (Gemini v1) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement task-by-task. Steps use
> checkbox (`- [ ]`) syntax. Pure helpers are TDD (host pytest); BLE + Gemini stages are
> verified by running live against the band (hardware-in-the-loop, matching this project).

**Goal:** A host script `tools/oled_bridge.py` that closes the OLED loop — you speak into
the band, a Gemini text reply appears on its OLED.

**Architecture:** A stable harness (bleak BLE I/O, LC3 decode, AGC, turn-bounding by the
uplink frame gap, host pagination) wrapping a single swappable `Brain.process(pcm)->text`.
v1 brain = `GeminiBrain` (one `generate_content` call: utterance audio → concise reply,
Gemini doing STT+LLM). Later `PipelineBrain` (local STT → gateway → agent) drops into the
same seam.

**Tech Stack:** Python 3.11 (`tools/dtln-venv`), bleak (BLE), `google-genai` 2.9.0 (Gemini),
numpy, `tools/voiceio/codec.py` (LC3 decode), `tools/voiceio/frame.py` (uplink parse).

**Spec:** `docs/superpowers/specs/2026-08-26-oled-gemini-bridge-design.md`.
**Run/test env:** `tools/dtln-venv/bin/python`. Device chars: uplink `47A10002`, display-text
`e9a10002`, name `gband-oled`. Trigger dictation with `d` (force) or the pose.

---

## File Structure

- **Create `tools/oled_bridge.py`** — the whole bridge. Sections: pure helpers (`agc`,
  `wrap_pages`, `pcm_to_wav`), `GeminiBrain`, the async BLE harness + turn loop + display
  sender, and the CLI (`--dump`, `--wav`, `--model`).
- **Create `tools/tests/test_oled_bridge.py`** — pytest for the pure helpers (`agc`,
  `wrap_pages`, `pcm_to_wav`).
- **Reuse** `tools/voiceio/codec.py` (`Lc3Codec`), `tools/voiceio/frame.py` (`parse_uplink`),
  `tools/lib/liblc3.dylib`, `.env` (`GEMINI_API_KEY`).

Keep it ONE file for v1 (small, cohesive); the `Brain` class is the extract-later seam.

---

## Task 1: AGC (pure helper, TDD)

**Files:**
- Create: `tools/oled_bridge.py`
- Create: `tools/tests/test_oled_bridge.py`

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_oled_bridge.py
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from oled_bridge import agc

def test_agc_boosts_quiet_toward_target():
    # peak ~745/32768 ~= 0.023 full-scale, like the real quiet ear-pose capture
    quiet = (np.sin(np.linspace(0, 200, 16000)) * 0.023).astype(np.float32)
    out = agc(quiet, target_peak=0.25, max_gain=40.0)
    assert 0.2 < float(np.max(np.abs(out))) <= 0.26   # lifted to ~target
    assert float(np.max(np.abs(out))) <= 1.0           # never clips past full-scale

def test_agc_caps_gain_on_silence():
    silence = np.zeros(16000, dtype=np.float32)
    out = agc(silence, target_peak=0.25, max_gain=40.0)
    assert float(np.max(np.abs(out))) == 0.0           # no NaN/inf, no blow-up

def test_agc_leaves_loud_audio_roughly_alone():
    loud = (np.sin(np.linspace(0, 200, 16000)) * 0.5).astype(np.float32)
    out = agc(loud, target_peak=0.25, max_gain=40.0)
    assert float(np.max(np.abs(out))) <= 0.51          # not amplified above itself
```

- [ ] **Step 2: Run it, verify it fails**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: FAIL — `ImportError` / `agc` not defined.

- [ ] **Step 3: Implement `agc`**

```python
# tools/oled_bridge.py  (top of file)
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
    gain = max(gain, 1.0)                      # never attenuate; only lift quiet audio
    return np.clip(pcm * gain, -1.0, 1.0).astype(np.float32)
```

- [ ] **Step 4: Run it, verify pass**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add tools/oled_bridge.py tools/tests/test_oled_bridge.py
git commit -m "oled_bridge: AGC helper (adaptive gain toward target peak)"
```

---

## Task 2: Pagination + word-wrap (pure helper, TDD)

**Files:**
- Modify: `tools/oled_bridge.py`
- Modify: `tools/tests/test_oled_bridge.py`

- [ ] **Step 1: Write the failing test**

```python
# append to tools/tests/test_oled_bridge.py
from oled_bridge import wrap_pages

def test_wrap_short_fits_one_page_one_line():
    assert wrap_pages("Hi there", cols=12, rows=2) == [["Hi there"]]

def test_wrap_two_lines_one_page():
    # 16 chars -> wraps to 2 lines, still one page
    assert wrap_pages("Response is okay", cols=12, rows=2) == [["Response is", "okay"]]

def test_wrap_paginates_long_text():
    pages = wrap_pages("one two three four five six seven eight", cols=12, rows=2)
    assert len(pages) >= 2
    assert all(len(p) <= 2 for p in pages)             # never more than rows lines/page
    assert all(len(line) <= 12 for p in pages for line in p)   # never over cols
```

- [ ] **Step 2: Run it, verify it fails**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: FAIL — `wrap_pages` not defined.

- [ ] **Step 3: Implement `wrap_pages`**

```python
# tools/oled_bridge.py
def wrap_pages(text, cols=12, rows=2):
    """Word-wrap text into lines <= cols chars, grouped into pages of <= rows lines.
    Long words are hard-split. Returns list[list[str]]."""
    words = text.split()
    lines, cur = [], ""
    for w in words:
        while len(w) > cols:                    # hard-split over-long words
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
```

- [ ] **Step 4: Run it, verify pass**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: all passed.

- [ ] **Step 5: Commit**

```bash
git add tools/oled_bridge.py tools/tests/test_oled_bridge.py
git commit -m "oled_bridge: word-wrap + pagination helper"
```

---

## Task 3: PCM→WAV + GeminiBrain (audio → text)

**Files:**
- Modify: `tools/oled_bridge.py`
- Modify: `tools/tests/test_oled_bridge.py`

- [ ] **Step 1: Write the failing test for `pcm_to_wav`**

```python
# append to tools/tests/test_oled_bridge.py
from oled_bridge import pcm_to_wav

def test_pcm_to_wav_has_riff_header_and_correct_length():
    pcm = np.zeros(160, dtype=np.float32)
    wav = pcm_to_wav(pcm, rate=16000)
    assert wav[:4] == b"RIFF" and wav[8:12] == b"WAVE"
    # 44-byte header + 160 samples * 2 bytes
    assert len(wav) == 44 + 160 * 2
```

- [ ] **Step 2: Run it, verify it fails**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: FAIL — `pcm_to_wav` not defined.

- [ ] **Step 3: Implement `pcm_to_wav` + `GeminiBrain`**

```python
# tools/oled_bridge.py
import io, os, wave

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
    def __init__(self, model="gemini-2.5-flash", api_key=None):
        from google import genai
        key = api_key or os.environ.get("GEMINI_API_KEY")
        if not key:
            raise RuntimeError("GEMINI_API_KEY not set (put it in a git-ignored .env)")
        self._genai = genai
        self._client = genai.Client(api_key=key)
        self._model = model

    def process(self, pcm_f32_16k):
        """utterance audio (float32 16 kHz) -> reply text."""
        from google.genai import types
        wav = pcm_to_wav(pcm_f32_16k, 16000)
        resp = self._client.models.generate_content(
            model=self._model,
            contents=[types.Part.from_bytes(data=wav, mime_type="audio/wav")],
            config=types.GenerateContentConfig(system_instruction=SYSTEM_PROMPT),
        )
        return (resp.text or "").strip()
```

- [ ] **Step 4: Run it, verify `pcm_to_wav` passes**

Run: `cd tools && ../tools/dtln-venv/bin/python -m pytest tests/test_oled_bridge.py -q`
Expected: all passed. (GeminiBrain is exercised live in Step 5.)

- [ ] **Step 5: Verify GeminiBrain live with a real question WAV**

Record a short question to a WAV (or reuse a prior `uplink.wav`), then:

```bash
cd /Users/0xjba/Projects/gestureband
tools/dtln-venv/bin/python - <<'PY'
import sys, wave; import numpy as np
sys.path.insert(0, "tools")
from oled_bridge import GeminiBrain
w = wave.open("uplink.wav","rb")   # any 16 kHz mono wav with a spoken question
pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32)/32768.0
print("reply:", GeminiBrain().process(pcm))
PY
```
Expected: prints a short text reply relevant to the spoken question (confirms the model
id + audio path + key work). If the model id errors, set `--model` to a current
audio-capable Gemini flash model and note it.

- [ ] **Step 6: Commit**

```bash
git add tools/oled_bridge.py tools/tests/test_oled_bridge.py
git commit -m "oled_bridge: pcm_to_wav + GeminiBrain (audio -> concise text)"
```

---

## Task 4: BLE harness + turn-bounding + display + main loop

**Files:**
- Modify: `tools/oled_bridge.py`

- [ ] **Step 1: Implement the async harness**

```python
# tools/oled_bridge.py
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
```

- [ ] **Step 2: Implement the main turn loop**

```python
# tools/oled_bridge.py
async def run(args):
    brain = GeminiBrain(model=args.model)
    codec = Lc3Codec(lib_path=os.path.join(os.path.dirname(__file__), "lib", "liblc3.dylib"))

    dev = await find_band()
    if not dev:
        print("band not found (advertising as gband-oled?)"); return 2
    print("connecting", dev.address)

    async with BleakClient(dev) as client:
        buf = []                      # float32 PCM of the in-progress utterance
        last_frame = [0.0]            # monotonic time of the last uplink frame

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
            # utterance complete when frames have stopped for GAP_MS
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
```

- [ ] **Step 3: (no separate test — verified end-to-end in Task 5)**

- [ ] **Step 4: Commit**

```bash
git add tools/oled_bridge.py
git commit -m "oled_bridge: BLE harness + turn-bounding + display scroll + main loop"
```

---

## Task 5: CLI + end-to-end live test

**Files:**
- Modify: `tools/oled_bridge.py`

- [ ] **Step 1: Add the CLI entrypoint**

```python
# tools/oled_bridge.py  (bottom of file)
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="gemini-2.5-flash")
    ap.add_argument("--dump", action="store_true", help="print replies + timing")
    ap.add_argument("--wav", help="offline: run GeminiBrain on a WAV, skip BLE")
    args = ap.parse_args()

    # load .env if present (GEMINI_API_KEY)
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
```

- [ ] **Step 2: Offline sanity (no band)**

Run: `tools/dtln-venv/bin/python tools/oled_bridge.py --wav uplink.wav`
Expected: prints a text reply to the spoken question in `uplink.wav`.

- [ ] **Step 3: End-to-end live (user, with the band)**

1. Flash the M4 firmware if not current: `./build_nrf54.sh apps/nrf54_oled && ./flash_nrf54.sh`.
2. Terminal 1: `tio /dev/cu.usbmodem8B571AEC3`.
3. Terminal 2: `tools/dtln-venv/bin/python tools/oled_bridge.py --dump`.
4. When it prints "ready", press `d` in tio (or hold the pose) + ask a question, then stop.
5. Expected: `thinking...` then the reply scrolls on the OLED; `--dump` prints it.

- [ ] **Step 4: Commit**

```bash
git add tools/oled_bridge.py
git commit -m "oled_bridge: CLI (--wav/--dump) + end-to-end voice->text->OLED"
```

---

## Self-review notes
- **Spec coverage:** BLE I/O (T4), decode+AGC (T1,T4), turn-bound gap (T4), GeminiBrain
  all-in-one (T3), pagination/scroll (T2,T4), `--dump`/`--wav` (T3,T5), error handling (T4),
  missing-key (T3). The `Brain` seam is the extract point for the future PipelineBrain.
- **Model id:** `gemini-2.5-flash` is the default; Task 3 Step 5 verifies it live and says to
  swap to a current audio-capable flash id if it errors (SDK 2.9.0).
- **Out of scope (v1):** staged pipeline (local STT/gateway/agent), barge-in — behind the seam.
