"""Pluggable AI-backend contract for the voice loop + a no-API test backend.

THE BOUNDARY (keep it clean): the core (orchestrator + ble_link + codec + aec + vad +
delay_estimator) is backend-agnostic and knows NOTHING about any specific API. A "brain"
plugs in behind this one interface: it receives the user's clean (echo-cancelled) mic
audio and produces audio to play on the speaker, and stops on barge-in. Gemini Live is
just the first impl (voiceio/gemini_backend.py); an STT->LLM->TTS stack, a different
realtime API, or a local model all plug in the same way, with NO changes to the core.

To add a new backend:
  1. Subclass VoiceBackend; implement feed_mic / next_audio / barge_in (required).
  2. Override any of wait_ready / end_turn / reset / close that apply (all default to
     safe no-ops, so a simple backend needs only the three required methods).
  3. Construct it in the entry point (tools/voice_loop.py) -- the ONLY place that names a
     concrete backend. Nothing in voiceio/ (except this file's test backend) should.

AUDIO CONTRACT: every audio array crossing this interface is 16 kHz mono float32 in
[-1, 1]. Any rate conversion a backend needs (e.g. Gemini's 24 kHz output) happens INSIDE
that backend, never in the core.
"""
import numpy as np


class VoiceBackend:
    """The pluggable 'brain' contract. All audio is 16 kHz mono float32 in [-1, 1]."""

    # --- required: every backend must implement these three ---

    def feed_mic(self, pcm):
        """Push a chunk of the user's clean (echo-cancelled) mic audio to the backend."""
        raise NotImplementedError

    def next_audio(self, max_samples):
        """Return up to max_samples of AI audio to play now (float32 array; empty
        np.zeros(0, np.float32) if there's nothing to play yet)."""
        raise NotImplementedError

    def barge_in(self):
        """The user interrupted -- STOP/flush the current output immediately. Drops any audio the
        backend still has queued for playback (the orchestrator separately flushes the device
        playout). A backend whose own server cancels/truncates its generation on interrupt (e.g.
        Gemini Live retains only what it already sent) does that here too. Called either by the
        orchestrator or self-triggered when the backend's own VAD reports an interrupt."""
        raise NotImplementedError

    # --- optional lifecycle: safe no-op defaults so the core treats all backends alike ---

    def server_barge_count(self):
        """How many barge-ins the backend's OWN detector (server-side VAD) has signalled so far.
        The orchestrator polls this and mirrors each new one with a device-playout flush + reset
        (see Orchestrator._server_barge_loop). A backend with no server-side VAD returns 0 (the
        default) and so never self-barges. This is the model-agnostic barge boundary -- the core
        never names a specific API."""
        return 0

    def start_session(self, timeout=15.0):
        """Begin a NEW conversation session (e.g. open the AI WebSocket) and block until
        ready. Called by the orchestrator when the user starts a conversation (arm raised
        -> uplink audio begins). A backend may be start/ended many times over its life.
        Default: ready immediately. Raise on failure."""
        return

    def end_session(self):
        """End the current conversation session (e.g. close the AI WebSocket to stop
        billing/listening). The backend must be reusable -- a later start_session() opens a
        fresh one. Called when the user ends the conversation (arm lowered -> uplink stops).
        Default: no-op."""
        return

    def wait_ready(self, timeout=15.0):
        """Block until ready. Deprecated alias kept for callers; prefer start_session().
        Default: ready immediately."""
        return

    def reset(self):
        """Clear per-session state. Default: no-op."""
        return

    def close(self):
        """Release resources (network sessions, threads). Default: no-op."""
        return


class EchoLoopbackBackend(VoiceBackend):
    """No-API test backend: streams a fixed canned response (the pretend 'AI reply').
    Lets us exercise the full loop -- speaker plays the response, the user barges in,
    AEC+VAD detect it, the orchestrator calls barge_in() and the response stops -- with
    no AI service. feed_mic is ignored (the canned reply is fixed)."""

    def __init__(self, response_pcm):
        self._resp = np.asarray(response_pcm, dtype=np.float32)
        self._pos = 0
        self._open = False   # only plays during an active session

    def feed_mic(self, pcm):
        pass  # test backend ignores the mic; its reply is canned

    def next_audio(self, max_samples):
        if not self._open:
            return np.zeros(0, np.float32)
        chunk = self._resp[self._pos:self._pos + max_samples]
        self._pos += len(chunk)
        return chunk.astype(np.float32)

    def barge_in(self):
        self._pos = len(self._resp)  # stop: no more audio to hand out

    def reset(self):
        self._pos = 0

    def start_session(self, timeout=15.0):
        self._pos = 0        # replay the canned reply for each new session
        self._open = True

    def end_session(self):
        self._open = False
