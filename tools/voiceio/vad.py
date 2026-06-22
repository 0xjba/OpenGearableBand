"""Barge-in detector via energy gate above echo floor.

After AEC removes the AI's echo, the user's voice stands ~33 dB above residual
floor. This module detects barge-in (user starting to talk over AI) by an
ENERGY gate: fire when a frame is loud relative to a continuously-tracked echo
floor. No speech-VAD (residual echo is itself speech, causing false-fires);
loudness-over-floor is the discriminator.
"""

import numpy as np


class BargeInVad:
    """Detects barge-in (user voice over AI) via energy gate above tracked floor."""

    def __init__(
        self,
        sr: int = 16000,
        frame_ms: int = 30,
        margin_db: float = 12.0,
        abs_floor_db: float = -60.0,
        onset_frames: int = 5,
        down_alpha: float = 0.3,
        up_alpha: float = 0.001,
    ):
        """
        Args:
            sr: sample rate (Hz).
            frame_ms: frame duration (milliseconds).
            margin_db: dB above the tracked floor to fire the relative gate (e.g. 12 dB).
            abs_floor_db: absolute dBFS floor -- a frame below this NEVER fires, even if
                it is >margin_db over the tracked floor. Stops false barge-ins on tiny
                residual-echo blips when the cleaned signal is near-silent (the relative
                gate alone trips on them). [USER: depends on mic gain + how loud the user
                talks; -60 dBFS rejects residual while passing real voice in our captures.]
            onset_frames: consecutive gated frames required to emit an onset.
            down_alpha: floor smoothing factor when energy drops (fast).
            up_alpha: floor smoothing factor when energy rises (very slow).
        """
        self.sr = sr
        self.frame_ms = frame_ms
        self.margin_db = margin_db
        self.abs_floor_linear = 10.0 ** (abs_floor_db / 20.0)
        self.onset_frames = onset_frames
        self.down_alpha = down_alpha
        self.up_alpha = up_alpha

        # State: carried across process() calls.
        self.floor = None  # Initialized on first frame.
        self.consec = 0  # Consecutive gated frames.
        self.armed_fired = False  # Has this onset been emitted?
        self._carry = np.zeros(0, dtype=np.float32)  # sub-frame remainder between calls

    def reset(self) -> None:
        """Clear state (floor, re-arm flag, frame counter)."""
        self.floor = None
        self.consec = 0
        self.armed_fired = False
        self._carry = np.zeros(0, dtype=np.float32)

    def process(self, audio: np.ndarray) -> list:
        """
        Process audio and return list of barge-in onset times (in seconds).

        Args:
            audio: float32 numpy array.

        Returns:
            List of float onset times (seconds from start of this audio segment).
        """
        # Prepend any sub-frame remainder from the previous call so arbitrary chunk sizes
        # (e.g. the orchestrator's 128/256/384-sample AEC outputs) are handled without
        # dropping samples -- otherwise a chunk shorter than one frame yields zero frames
        # and the detector never runs.
        audio = np.concatenate([self._carry, np.asarray(audio, dtype=np.float32)])

        # Split into non-overlapping frames.
        frame_samples = int(self.sr * self.frame_ms / 1000.0)
        n_frames = len(audio) // frame_samples
        self._carry = audio[n_frames * frame_samples:]  # keep the remainder for next call
        onsets = []

        # Conversion factor for dB to linear.
        margin_linear = 10.0 ** (self.margin_db / 20.0)

        for i in range(n_frames):
            # Extract frame.
            frame_start = i * frame_samples
            frame_end = frame_start + frame_samples
            frame = audio[frame_start:frame_end]

            # Compute RMS.
            rms = np.sqrt(np.mean(frame**2) + 1e-12)

            # Initialize floor on first frame.
            if self.floor is None:
                self.floor = rms

            # Online echo-floor tracker: fast down, very slow up.
            if rms < self.floor:
                self.floor = self.floor + (rms - self.floor) * self.down_alpha
            else:
                self.floor = self.floor + (rms - self.floor) * self.up_alpha

            # Gate: loud enough above the tracked floor AND above the absolute floor
            # (the latter rejects quiet residual-echo blips on a near-silent stream).
            gate = (rms > self.floor * margin_linear) and (rms > self.abs_floor_linear)

            # Onset logic: `onset_frames` consecutive gated frames; re-arm after a non-gated frame.
            if gate:
                self.consec += 1
                if self.consec >= self.onset_frames and not self.armed_fired:
                    # Emit onset at this frame's time.
                    onset_time = i * self.frame_ms / 1000.0
                    onsets.append(onset_time)
                    self.armed_fired = True
            else:
                self.consec = 0
                self.armed_fired = False

        return onsets
