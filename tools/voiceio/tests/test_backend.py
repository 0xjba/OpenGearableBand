import numpy as np
from voiceio.backend import EchoLoopbackBackend, VoiceBackend


def test_no_audio_until_session_started():
    b = EchoLoopbackBackend(np.ones(1000, dtype=np.float32))
    assert len(b.next_audio(256)) == 0                    # idle (no session) -> silent
    b.start_session()
    assert len(b.next_audio(256)) == 256                  # session open -> plays
    b.end_session()
    assert len(b.next_audio(256)) == 0                    # session closed -> silent again


def test_streams_response_in_chunks_then_empties():
    resp = np.arange(1000, dtype=np.float32)
    b = EchoLoopbackBackend(resp)
    b.start_session()
    out = []
    while True:
        c = b.next_audio(256)
        if len(c) == 0:
            break
        out.append(c)
    assert np.array_equal(np.concatenate(out), resp)      # full response streamed back, in order
    assert len(b.next_audio(256)) == 0                    # exhausted -> empty


def test_barge_in_stops_output():
    b = EchoLoopbackBackend(np.ones(1000, dtype=np.float32))
    b.start_session()
    assert len(b.next_audio(100)) == 100                  # playing
    b.barge_in()
    assert len(b.next_audio(100)) == 0                    # stopped immediately


def test_start_session_replays():
    b = EchoLoopbackBackend(np.ones(500, dtype=np.float32))
    b.start_session()
    b.next_audio(500)
    assert len(b.next_audio(10)) == 0
    b.start_session()                                     # new session replays from the top
    assert len(b.next_audio(10)) == 10


def test_feed_mic_is_a_noop_for_loopback():
    b = EchoLoopbackBackend(np.ones(10, dtype=np.float32))
    b.start_session()
    b.feed_mic(np.zeros(160, dtype=np.float32))           # must not raise / not consume output
    assert len(b.next_audio(10)) == 10


def test_minimal_backend_runs_via_lifecycle_defaults():
    # A new backend implementing only the 3 required methods must work through the
    # default no-op lifecycle (start_session/end_session/wait_ready/end_turn/reset/close).
    class MyAPI(VoiceBackend):
        def feed_mic(self, pcm): pass
        def next_audio(self, n): return np.zeros(0, np.float32)
        def barge_in(self): pass
    b = MyAPI()
    b.start_session(); b.wait_ready(); b.end_turn(); b.reset(); b.end_session(); b.close()
