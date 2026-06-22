"""Pluggable AI-backend interface for the voice loop + a no-API test backend.

The orchestrator owns BLE/AEC/VAD; the backend is the 'brain': it receives the
user's clean mic audio and produces audio to play on the speaker, and stops on
barge-in. Gemini Live (or any STT->LLM->TTS) plugs in behind this same interface."""
import numpy as np


class VoiceBackend:
    """Interface. All audio is 16 kHz mono float32 in [-1, 1]."""

    def feed_mic(self, pcm):
        """Push a chunk of the user's clean (echo-cancelled) mic audio to the backend."""
        raise NotImplementedError

    def next_audio(self, max_samples):
        """Return up to max_samples of AI audio to play now (a float32 array; empty array
        np.zeros(0, np.float32) if there's nothing to play yet)."""
        raise NotImplementedError

    def barge_in(self):
        """The user interrupted -- stop/flush the current output immediately."""
        raise NotImplementedError


class EchoLoopbackBackend(VoiceBackend):
    """No-API test backend: streams a fixed canned response (the pretend 'AI reply').
    Lets us exercise the full loop -- speaker plays the response, the user barges in,
    AEC+VAD detect it, the orchestrator calls barge_in() and the response stops -- with
    no AI service. feed_mic is ignored (the canned reply is fixed)."""

    def __init__(self, response_pcm):
        self._resp = np.asarray(response_pcm, dtype=np.float32)
        self._pos = 0

    def feed_mic(self, pcm):
        pass  # test backend ignores the mic; its reply is canned

    def next_audio(self, max_samples):
        chunk = self._resp[self._pos:self._pos + max_samples]
        self._pos += len(chunk)
        return chunk.astype(np.float32)

    def barge_in(self):
        self._pos = len(self._resp)  # stop: no more audio to hand out

    def reset(self):
        self._pos = 0
