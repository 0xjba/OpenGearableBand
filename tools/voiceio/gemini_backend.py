"""Gemini Live API backend for the voice loop (implements VoiceBackend).

This is the first real 'brain' behind the backend-agnostic interface. The
orchestrator owns BLE/AEC/VAD and the device's 16 kHz clock; this backend turns
the user's clean mic audio into the AI's spoken reply via Google's Gemini Live
API (a stateful audio-to-audio WebSocket).

KEY BRIDGE: Gemini Live is ASYNC (an awaited WebSocket session) but our
VoiceBackend interface is SYNCHRONOUS (feed_mic / next_audio / barge_in, called
from the orchestrator's audio thread). So the whole async session runs on its own
background thread with an asyncio event loop, and we hand audio across the thread
boundary through queues:
  * feed_mic(pcm)  -> push to an asyncio.Queue (via call_soon_threadsafe); the
                      async sender drains it and calls send_realtime_input.
  * received audio -> resampled 24 kHz->16 kHz, pushed to a thread-safe queue
                      that next_audio() drains.
  * barge_in()     -> drop everything queued/buffered for playback NOW (our local
                      AEC+VAD is the barge-in authority; the model also detects
                      turn-taking server-side from the continuous mic feed).

AUDIO CONTRACT (verified against ai.google.dev Live API docs, 2026-06-23):
  * Input : raw 16-bit PCM, 16 kHz, little-endian, mime "audio/pcm;rate=16000".
            == our device mic clock, so NO inbound resample.
  * Output: raw 16-bit PCM, 24 kHz, little-endian -> we resample 24->16 kHz here
            so the rest of the pipeline (LC3 -> speaker) stays at 16 kHz.
  * Model id: "gemini-3.1-flash-live-preview" (Live-API native-audio model).

The API key is read from GEMINI_API_KEY (env or a git-ignored .env loaded by the
caller). The key is NEVER hardcoded here.
"""
import asyncio
import os
import queue
import threading

import numpy as np

from .backend import VoiceBackend
from .codec import resample

# [STRUCTURAL] Gemini Live audio contract -- fixed by the API, not tunable.
GEMINI_IN_HZ = 16000          # Live API input rate (matches our device mic clock)
GEMINI_OUT_HZ = 24000         # Live API output rate (native-audio TTS)
INTERNAL_HZ = 16000           # our pipeline rate (device mic + speaker)
DEFAULT_MODEL = "gemini-3.1-flash-live-preview"
GEMINI_IN_MIME = "audio/pcm;rate=16000"


def _f32_to_pcm16le(x: np.ndarray) -> bytes:
    """float32 [-1,1] -> raw 16-bit little-endian PCM bytes."""
    x = np.clip(np.asarray(x, dtype=np.float32), -1.0, 1.0)
    return (x * 32767.0).astype("<i2").tobytes()


def _pcm16le_to_f32(b: bytes) -> np.ndarray:
    """raw 16-bit little-endian PCM bytes -> float32 [-1,1]."""
    return np.frombuffer(b, dtype="<i2").astype(np.float32) / 32768.0


class GeminiLiveBackend(VoiceBackend):
    """VoiceBackend backed by the Gemini Live API (async session on a bg thread)."""

    def __init__(self, api_key=None, model=DEFAULT_MODEL, system_instruction=None):
        """
        Args:
            api_key: Gemini key. Defaults to env GEMINI_API_KEY (never hardcode).
            model: Live-API model id.
            system_instruction: optional system prompt (e.g. to shape reply style).

        Barge-in uses Gemini's AUTOMATIC VAD: the orchestrator forwards the AEC-cleaned mic while the
        AI plays, the server hears the user talk over it, cancels + discards the in-flight response
        (retaining only what it already sent to us), and signals `server_content.interrupted` -- which
        we surface via server_barge_count(). (The manual activity_start/end A/B leg lost on HW and was
        removed 2026-06-30.)"""
        from google import genai
        from google.genai import types
        self._types = types

        key = api_key or os.environ.get("GEMINI_API_KEY")
        if not key:
            raise RuntimeError(
                "GEMINI_API_KEY not set -- put it in a git-ignored .env "
                "(GEMINI_API_KEY=...) or the environment; never hardcode it."
            )
        self._client = genai.Client(api_key=key)
        self._model = model
        self._config = {"response_modalities": ["AUDIO"]}
        if system_instruction:
            self._config["system_instruction"] = system_instruction
        # Detune the server VAD. HW log 2026-07-01 showed Gemini firing `interrupted` (self-barge) on
        # NEAR-SILENT post-AEC input (clean_rms 0.0001-0.0019, ~-70 dBFS) -- its default sensitivity is
        # too hot for our unusually-quiet, AEC-cleaned stream. START_SENSITIVITY_LOW makes it require
        # real, sustained, louder speech before declaring a user interruption, so the ultra-quiet
        # residual can't trip it while a genuine barge (~0.02-0.09) still does. END_SENSITIVITY_LOW
        # likewise tolerates natural pauses. [USER/HOUSING: re-tune per host + production speaker/coupling.]
        self._config["realtime_input_config"] = types.RealtimeInputConfig(
            automatic_activity_detection=types.AutomaticActivityDetection(
                start_of_speech_sensitivity=types.StartSensitivity.START_SENSITIVITY_LOW,
                end_of_speech_sensitivity=types.EndSensitivity.END_SENSITIVITY_LOW,
            ))

        # Per-session plumbing -- (re)created by start_session(). The WebSocket is opened
        # per CONVERSATION (arm raised -> start_session, arm lowered -> end_session), NOT in
        # __init__, so an idle band isn't holding an open Gemini session (cost/battery/privacy).
        self._thread = None
        self._init_session_state()

    def _init_session_state(self):
        self._play_q = queue.Queue()           # inbound 16 kHz float32 chunks to play
        self._play_buf = np.zeros(0, np.float32)  # leftover from a partial next_audio()
        self._loop = None                      # the bg event loop (set when ready)
        self._aio_mic_q = None                 # asyncio.Queue of outbound PCM16 bytes
        self._ready = threading.Event()        # session connected (or _err set)
        self._stop = threading.Event()
        self._err = None
        self._n_turns = 0                      # completed model turns (diagnostic)
        self._n_audio_recv = 0                 # audio chunks received (diagnostic)
        self._n_interrupts = 0                 # server-side `interrupted` events (Gemini's VAD fired)

    # --- VoiceBackend interface (called from the orchestrator's audio thread) ---

    def feed_mic(self, pcm):
        """Push a chunk of the user's clean 16 kHz float32 mic audio to the model."""
        if not self._ready.is_set() or self._loop is None or self._err is not None:
            return
        data = _f32_to_pcm16le(pcm)
        # Hop onto the bg loop to enqueue; asyncio.Queue isn't thread-safe.
        self._loop.call_soon_threadsafe(self._aio_mic_q.put_nowait, data)

    def next_audio(self, max_samples):
        """Return up to max_samples of the AI's 16 kHz float32 reply (empty if none)."""
        # Drain any whole chunks sitting in the thread-safe queue into our buffer.
        while True:
            try:
                self._play_buf = np.concatenate([self._play_buf, self._play_q.get_nowait()])
            except queue.Empty:
                break
        if len(self._play_buf) == 0:
            return np.zeros(0, np.float32)
        out = self._play_buf[:max_samples]
        self._play_buf = self._play_buf[max_samples:]
        return out.astype(np.float32)

    def barge_in(self):
        """User interrupted: drop all pending/buffered AI audio locally. (Gemini cancels + discards
        its in-flight generation server-side on its own interrupt, retaining only what it already
        sent, so there's no separate cancel/truncate call to make here.)"""
        self._play_buf = np.zeros(0, np.float32)
        while True:
            try:
                self._play_q.get_nowait()
            except queue.Empty:
                break

    def server_barge_count(self):
        """How many times Gemini's server VAD has fired `interrupted` this object's lifetime. The
        orchestrator polls this to mirror each barge with a device-playout flush + alignment reset."""
        return self._n_interrupts

    def wait_ready(self, timeout=10.0):
        """Block until the session is connected. Raises if it failed to connect."""
        if not self._ready.wait(timeout):
            raise TimeoutError("Gemini Live session did not connect in time")
        if self._err is not None:
            raise self._err

    def start_session(self, timeout=15.0):
        """Open a FRESH Gemini Live session (WebSocket) and block until ready. Reusable:
        call again after end_session() for the next conversation."""
        if self._thread is not None and self._thread.is_alive():
            return                              # a session is already open
        self._init_session_state()              # fresh queues/events/counters
        self._thread = threading.Thread(target=self._run, name="gemini-live", daemon=True)
        self._thread.start()
        self.wait_ready(timeout)
        print("[gemini] session opened")

    def end_session(self):
        """Close the current Gemini session (WebSocket) + join its thread. The object stays
        reusable -- a later start_session() opens a new one. Invalidates the loop so feed_mic
        / next_audio become no-ops until the next session."""
        if self._thread is None:
            return
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)  # wake the loop to see _stop
        self._thread.join(timeout=5.0)
        self._thread = None
        self._loop = None                       # feed_mic/next_audio guard on this -> no-op
        self._play_buf = np.zeros(0, np.float32)  # drop any unplayed tail
        while not self._play_q.empty():
            try:
                self._play_q.get_nowait()
            except queue.Empty:
                break
        print("[gemini] session closed")

    def close(self):
        """Final teardown (app exit). Just ends the session."""
        self.end_session()

    # --- background asyncio session ---

    def _run(self):
        try:
            asyncio.run(self._session_main())
        except Exception as e:  # surface to wait_ready()/feed_mic()
            self._err = e
            self._ready.set()

    async def _session_main(self):
        self._loop = asyncio.get_running_loop()
        self._aio_mic_q = asyncio.Queue()
        async with self._client.aio.live.connect(model=self._model, config=self._config) as session:
            self._ready.set()
            sender = asyncio.create_task(self._sender(session))
            receiver = asyncio.create_task(self._receiver(session))
            # Idle until close() is requested, then cancel the pumps.
            while not self._stop.is_set():
                await asyncio.sleep(0.05)
            sender.cancel()
            receiver.cancel()
            await asyncio.gather(sender, receiver, return_exceptions=True)

    async def _sender(self, session):
        """Drain outbound mic PCM and stream it to the model."""
        import traceback
        try:
            while True:
                item = await self._aio_mic_q.get()
                await session.send_realtime_input(
                    audio=self._types.Blob(data=item, mime_type=GEMINI_IN_MIME)
                )
        except asyncio.CancelledError:
            raise
        except Exception:
            print("[gemini] sender task died:\n" + traceback.format_exc())

    async def _receiver(self, session):
        """Receive the model's audio across MULTIPLE turns.

        CRITICAL: session.receive() is a PER-TURN generator -- the SDK breaks the
        loop on turn_complete (verified in the SDK source). So a single `async for`
        ends after the first reply and never sees turn 2. We must re-enter receive()
        for each turn, hence the outer while loop. Audio arrives on the convenience
        attribute response.data (the model_turn.parts path was empty in practice)."""
        import traceback
        try:
            while not self._stop.is_set():
                async for response in session.receive():
                    sc = response.server_content
                    # Server-side interruption: flush, mirroring our local barge_in().
                    if sc is not None and getattr(sc, "interrupted", False):
                        self._n_interrupts += 1
                        print(f"[gemini] <- interrupted #{self._n_interrupts} "
                              f"(server VAD; {self._n_audio_recv} chunks so far)")
                        self.barge_in()
                        continue
                    data = getattr(response, "data", None)
                    if data:
                        self._n_audio_recv += 1
                        pcm = _pcm16le_to_f32(data)
                        pcm16k = resample(pcm, GEMINI_OUT_HZ, INTERNAL_HZ)
                        self._play_q.put(pcm16k)
                    if sc is not None and getattr(sc, "turn_complete", False):
                        self._n_turns += 1
                        print(f"[gemini] <- turn_complete (#{self._n_turns}, "
                              f"{self._n_audio_recv} chunks total)")
                # turn generator ended -> loop back into receive() for the next turn
        except asyncio.CancelledError:
            raise
        except Exception:
            print("[gemini] receiver task died:\n" + traceback.format_exc())
        finally:
            print("[gemini] receiver loop exited")
