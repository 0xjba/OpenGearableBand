import numpy as np
from voiceio.codec import Lc3Codec, resample

LIB = "/Users/0xjba/Projects/gestureband/tools/lib/liblc3.dylib"

def test_lc3_roundtrip_preserves_a_tone():
    sr = 16000
    codec = Lc3Codec(lib_path=LIB)

    # Single 160-sample frame encodes to exactly 40 bytes and decodes to 160 samples.
    t1 = np.arange(160) / sr
    pcm1 = (np.sin(2*np.pi*1000*t1) * 0.3 * 32767).astype(np.int16)
    frame = codec.encode(pcm1)
    assert len(frame) == 40                       # 40-byte LC3 frame
    out1 = codec.decode(frame)
    assert len(out1) == 160

    # LC3 is lossy but faithful. It has ~184 samples of encoder+decoder
    # algorithmic delay, so fidelity must be measured over a stream and at the
    # matching lag (a single isolated frame's output is mostly codec transient).
    nframes = 8
    t = np.arange(160 * nframes) / sr
    pcm = (np.sin(2*np.pi*1000*t) * 0.3 * 32767).astype(np.int16)
    out = np.concatenate(
        [codec.decode(codec.encode(pcm[i*160:(i+1)*160])) for i in range(nframes)]
    )
    inp = pcm.astype(float)
    out = out.astype(float)
    best = -1.0
    for lag in range(0, 200):                      # search the codec delay
        a = inp[:len(inp) - lag] - inp[:len(inp) - lag].mean()
        b = out[lag:] - out[lag:].mean()
        corr = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
        best = max(best, corr)
    assert best > 0.9, f"LC3 roundtrip corr too low: {best:.2f}"

def test_resample_16k_to_24k_length_and_tone():
    sr_in, sr_out = 16000, 24000
    t = np.arange(sr_in) / sr_in
    x = np.sin(2*np.pi*1000*t).astype(np.float32)
    y = resample(x, sr_in, sr_out)
    assert abs(len(y) - sr_out) <= 2            # 1 s in -> ~1 s out at the new rate
    f = np.fft.rfftfreq(len(y), 1/sr_out)
    peak = f[np.argmax(np.abs(np.fft.rfft(y)))]
    assert abs(peak - 1000) < 10                 # 1 kHz tone preserved through resample
