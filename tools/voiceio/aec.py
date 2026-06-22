# -*- coding: utf-8 -*-
"""Streaming wrapper around the DTLN-aec two-stage TF-Lite echo canceller.

Ports the exact per-block math from tools/DTLN-aec/run_aec.py into a stateful
class so the voice-I/O pipeline can feed mic + far-end reference hops and get
back the echo-cancelled ("clean") mic. State persists across hops and across
calls; reset() clears it at the start of each session.
"""

import os

import numpy as np
import tensorflow.lite as tflite

# make GPUs invisible (matches run_aec.py)
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")

BLOCK_LEN = 512
BLOCK_SHIFT = 128


class DtlnAec:
    def __init__(self, model_dir):
        """model_dir is the path prefix, e.g. ".../dtln_aec_512"; loads
        model_dir + "_1.tflite" and model_dir + "_2.tflite"."""
        self.interpreter_1 = tflite.Interpreter(model_path=model_dir + "_1.tflite")
        self.interpreter_1.allocate_tensors()
        self.interpreter_2 = tflite.Interpreter(model_path=model_dir + "_2.tflite")
        self.interpreter_2.allocate_tensors()

        self.input_details_1 = self.interpreter_1.get_input_details()
        self.output_details_1 = self.interpreter_1.get_output_details()
        self.input_details_2 = self.interpreter_2.get_input_details()
        self.output_details_2 = self.interpreter_2.get_output_details()

        self.reset()

    def reset(self):
        """(Re)initialise per-session state. Call at the start of each session."""
        self.states_1 = np.zeros(self.input_details_1[1]["shape"]).astype("float32")
        self.states_2 = np.zeros(self.input_details_2[1]["shape"]).astype("float32")
        self.in_buffer = np.zeros(BLOCK_LEN).astype("float32")       # mic
        self.in_buffer_lpb = np.zeros(BLOCK_LEN).astype("float32")   # far-end reference
        self.out_buffer = np.zeros(BLOCK_LEN).astype("float32")

    def process(self, mic, ref):
        """Echo-cancel mic against far-end reference ref.

        mic and ref are equal-length float32 arrays whose length is a multiple
        of BLOCK_SHIFT (truncated to the largest multiple if not). State is
        persisted across hops and across calls. Returns the clean output, same
        length as the (truncated) input.
        """
        mic = np.asarray(mic, dtype="float32")
        ref = np.asarray(ref, dtype="float32")
        n = (min(len(mic), len(ref)) // BLOCK_SHIFT) * BLOCK_SHIFT
        out = np.zeros(n, dtype="float32")

        for idx in range(0, n, BLOCK_SHIFT):
            mic_hop = mic[idx:idx + BLOCK_SHIFT]
            lpb_hop = ref[idx:idx + BLOCK_SHIFT]

            # shift and write the input buffers
            self.in_buffer[:-BLOCK_SHIFT] = self.in_buffer[BLOCK_SHIFT:]
            self.in_buffer[-BLOCK_SHIFT:] = mic_hop
            self.in_buffer_lpb[:-BLOCK_SHIFT] = self.in_buffer_lpb[BLOCK_SHIFT:]
            self.in_buffer_lpb[-BLOCK_SHIFT:] = lpb_hop

            # FFTs + magnitudes
            in_block_fft = np.fft.rfft(self.in_buffer).astype("complex64")
            in_mag = np.reshape(np.abs(in_block_fft), (1, 1, -1)).astype("float32")
            lpb_block_fft = np.fft.rfft(self.in_buffer_lpb).astype("complex64")
            lpb_mag = np.reshape(np.abs(lpb_block_fft), (1, 1, -1)).astype("float32")

            # stage 1: mask estimation
            self.interpreter_1.set_tensor(self.input_details_1[0]["index"], in_mag)
            self.interpreter_1.set_tensor(self.input_details_1[2]["index"], lpb_mag)
            self.interpreter_1.set_tensor(self.input_details_1[1]["index"], self.states_1)
            self.interpreter_1.invoke()
            out_mask = self.interpreter_1.get_tensor(self.output_details_1[0]["index"])
            self.states_1 = self.interpreter_1.get_tensor(self.output_details_1[1]["index"])

            # apply mask, ifft back to time domain
            estimated_block = np.fft.irfft(in_block_fft * out_mask)
            estimated_block = np.reshape(estimated_block, (1, 1, -1)).astype("float32")
            in_lpb = np.reshape(self.in_buffer_lpb, (1, 1, -1)).astype("float32")

            # stage 2: time-domain refinement
            self.interpreter_2.set_tensor(self.input_details_2[1]["index"], self.states_2)
            self.interpreter_2.set_tensor(self.input_details_2[0]["index"], estimated_block)
            self.interpreter_2.set_tensor(self.input_details_2[2]["index"], in_lpb)
            self.interpreter_2.invoke()
            out_block = self.interpreter_2.get_tensor(self.output_details_2[0]["index"])
            self.states_2 = self.interpreter_2.get_tensor(self.output_details_2[1]["index"])

            # overlap-add into the output buffer
            self.out_buffer[:-BLOCK_SHIFT] = self.out_buffer[BLOCK_SHIFT:]
            self.out_buffer[-BLOCK_SHIFT:] = np.zeros(BLOCK_SHIFT)
            self.out_buffer += np.squeeze(out_block)

            out[idx:idx + BLOCK_SHIFT] = self.out_buffer[:BLOCK_SHIFT]

        return out
