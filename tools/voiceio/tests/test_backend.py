import numpy as np
from voiceio.backend import EchoLoopbackBackend

def test_streams_response_in_chunks_then_empties():
    resp = np.arange(1000, dtype=np.float32)
    b = EchoLoopbackBackend(resp)
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
    assert len(b.next_audio(100)) == 100                  # playing
    b.barge_in()
    assert len(b.next_audio(100)) == 0                    # stopped immediately

def test_reset_replays():
    b = EchoLoopbackBackend(np.ones(500, dtype=np.float32))
    b.next_audio(500)
    assert len(b.next_audio(10)) == 0
    b.reset()
    assert len(b.next_audio(10)) == 10                    # replays after reset

def test_feed_mic_is_a_noop_for_loopback():
    b = EchoLoopbackBackend(np.ones(10, dtype=np.float32))
    b.feed_mic(np.zeros(160, dtype=np.float32))           # must not raise / not consume output
    assert len(b.next_audio(10)) == 10
