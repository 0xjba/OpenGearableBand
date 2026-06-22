"""LC3 codec + sample-rate resampling for the Mac voice-I/O core.

LC3 wrappers are ported from tools/audio_tx.py (Lc3Encoder / Lc3Decoder), which
drive Google liblc3 over ctypes. One frame = 160 int16 samples (10 ms @ 16 kHz)
<-> 40-byte LC3 frame -- matching the device's BLE audio framing.

The resampler bridges the device's fixed 16 kHz to whatever rate a backend wants
(e.g. 24 kHz), using soxr's high-quality polyphase resampler.
"""
import ctypes

import numpy as np
import soxr

# LC3 framing (same as audio_tx.py: 16 kHz / 10 ms / 32 kbps -> 40 B/frame).
DT_US = 10000          # 10 ms frame duration
SR_HZ = 16000          # 16 kHz sample rate
FRAME_BYTES = 40       # encoded LC3 frame size
FRAME_SAMPLES = 160    # PCM samples per frame (10 ms @ 16 kHz)


class Lc3Codec:
    """Wraps a liblc3 encoder + decoder for 16 kHz / 10 ms / 40 B frames.

    Ported from audio_tx.py's Lc3Encoder/Lc3Decoder -- same ctypes setup and
    encode/decode calling convention.
    """

    def __init__(self, lib_path="tools/lib/liblc3.dylib"):
        lib = ctypes.CDLL(lib_path)

        # --- encoder ---
        lib.lc3_encoder_size.restype = ctypes.c_uint
        lib.lc3_encoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.lc3_setup_encoder.restype = ctypes.c_void_p
        lib.lc3_setup_encoder.argtypes = [
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        lib.lc3_encode.restype = ctypes.c_int
        lib.lc3_encode.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
            ctypes.c_int, ctypes.c_void_p]

        # --- decoder ---
        lib.lc3_decoder_size.restype = ctypes.c_uint
        lib.lc3_decoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.lc3_setup_decoder.restype = ctypes.c_void_p
        lib.lc3_setup_decoder.argtypes = [
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        lib.lc3_decode.restype = ctypes.c_int
        lib.lc3_decode.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
            ctypes.c_int, ctypes.c_void_p, ctypes.c_int]

        self._lib = lib

        enc_size = lib.lc3_encoder_size(DT_US, SR_HZ)
        self._enc_mem = ctypes.create_string_buffer(enc_size)
        self._enc = lib.lc3_setup_encoder(DT_US, SR_HZ, 0, self._enc_mem)
        self._enc_out = (ctypes.c_uint8 * FRAME_BYTES)()

        dec_size = lib.lc3_decoder_size(DT_US, SR_HZ)
        self._dec_mem = ctypes.create_string_buffer(dec_size)
        self._dec = lib.lc3_setup_decoder(DT_US, SR_HZ, 0, self._dec_mem)
        self._dec_pcm = (ctypes.c_int16 * FRAME_SAMPLES)()

        if self._enc is None or self._dec is None:
            raise RuntimeError(f"liblc3 setup failed for {lib_path}")

    def encode(self, pcm160):
        """160 int16 samples (bytes or np.int16 array) -> 40-byte LC3 frame."""
        if isinstance(pcm160, np.ndarray):
            pcm160 = pcm160.astype(np.int16).tobytes()
        buf = (ctypes.c_uint8 * len(pcm160)).from_buffer_copy(pcm160)
        # lc3_encode(enc, fmt=0(S16), pcm, stride=1(mono), nbytes=40, out)
        self._lib.lc3_encode(self._enc, 0, buf, 1, FRAME_BYTES, self._enc_out)
        return bytes(self._enc_out)

    def decode(self, lc3_40):
        """40-byte LC3 frame -> 160 int16 samples (np.int16 array)."""
        buf = (ctypes.c_uint8 * len(lc3_40)).from_buffer_copy(lc3_40)
        # lc3_decode(dec, frame, nbytes, fmt=0(S16), pcm, stride=1(mono))
        self._lib.lc3_decode(self._dec, buf, len(lc3_40), 0, self._dec_pcm, 1)
        return np.frombuffer(bytes(self._dec_pcm), dtype=np.int16)


def resample(audio, in_rate, out_rate):
    """Resample a float32 array from in_rate to out_rate (high-quality polyphase)."""
    audio = np.asarray(audio, dtype=np.float32)
    if in_rate == out_rate:
        return audio
    return soxr.resample(audio, in_rate, out_rate).astype(np.float32)
