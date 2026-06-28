"""Online render->capture alignment estimator (AEC3-style), so nothing acoustic is hardcoded.

The loop delay (downlink jitter buffer + BLE scheduling + I2S/PDM pipeline + uplink) is NOT
a universal constant -- it varies per unit/session/BLE conditions -- and production AEC
(WebRTC AEC3's echo_path_delay_estimator) estimates it ONLINE by cross-correlating the
far-end reference against the mic. This module does the same.

It tracks `ref_base_mic` -- the mic-sample index at which played_ref[0] is heard -- which is
exactly what orchestrator.aligned_reference needs. The orchestrator gives it a coarse SEARCH
CENTER each call (derived live from the playout position: played_samples minus the in-flight
buffer), and the estimator finds the precise alignment by cross-correlation within +/- a
search window around that center. The center is robust to who-started-first (it comes from the
playout position, not a once-captured origin), which the prior seed-at-first-block was not.

Robustness: updates only when the normalized correlation peak clears a confidence floor
(rejects silence / double-talk / no-echo); smooths the tracked alignment; holds the last good
value otherwise. Drift is removed first (reference put on the mic clock via AEC_DRIFT_RATIO).

[STRUCTURAL] This REPLACES the hardcoded ref_delay_ms. The only remaining fixed audio
constant is AEC_DRIFT_RATIO (exact clock-divider physics, placement-independent).
"""
import numpy as np

from voiceio.clocks import AEC_DRIFT_RATIO

SR = 16000


def _ncc_valid(mic, ref_win):
    """Normalized cross-correlation of `mic` (length W) slid over `ref_win` (length W+L).
    Returns length L+1; out[k] = cosine similarity of mic vs ref_win[k:k+W]. Vectorized."""
    w = len(mic)
    L = len(ref_win) - w
    if L < 0 or w == 0:
        return np.zeros(0, dtype=np.float64)
    mic = mic.astype(np.float64)
    ref_win = ref_win.astype(np.float64)
    mic_z = mic - mic.mean()
    mic_norm = np.linalg.norm(mic_z) + 1e-12
    full = np.correlate(ref_win, mic_z, mode="valid")            # length L+1 (C-speed)
    cs = np.concatenate([[0.0], np.cumsum(ref_win)])
    cs2 = np.concatenate([[0.0], np.cumsum(ref_win * ref_win)])
    s = cs[w:] - cs[:-w]                                          # sliding sums, length L+1
    s2 = cs2[w:] - cs2[:-w]
    var = np.maximum(s2 - s * s / w, 1e-12)                      # mean-removed window energy
    return full / (mic_norm * np.sqrt(var))


class DelayEstimator:
    def __init__(self, sr=SR, ratio=AEC_DRIFT_RATIO, search_ms=250.0,
                 conf_threshold=0.30, smooth=0.4, max_step_ms=40.0):
        """
        Args:
            search_ms: +/- window (ms) searched around the live center. [STRUCTURAL: widen if
                a buffering change pushes the true alignment outside this band.]
            conf_threshold: min normalized-correlation peak to accept an update (rejects
                silence/double-talk). [USER/HOUSING: depends on echo strength of the final
                acoustic design; re-check when the speaker/placement is finalized.]
            smooth: EMA factor toward a new accepted alignment (0..1; lower = steadier).
            max_step_ms: hysteresis -- max distance ref_base may move per accepted update. A
                single noisy/spurious correlation peak (e.g. at a reply onset, when the playout
                center momentarily jumps) cannot yank the held alignment; it can only nudge it by
                this much, so the AEC stays aligned through the transition and `clean` does not
                spike. WebRTC AEC3's render_delay_controller applies an equivalent limit. Drift
                (~8 ms/s) is far below 40 ms/update, so true tracking is unaffected. [STRUCTURAL]
        """
        self.sr = sr
        self.ratio = ratio
        self.span = int(search_ms * sr / 1000.0)
        self.conf_threshold = conf_threshold
        self.smooth = smooth
        self.max_step = max_step_ms * sr / 1000.0
        self._ref_base = None        # tracked absolute alignment (mic index of played_ref[0])
        self._last_center = None
        self.confidence = 0.0

    def reset(self):
        self._ref_base = None
        self._last_center = None
        self.confidence = 0.0

    def estimate(self, mic, mic_start_idx, played_ref, search_center):
        """Refine the alignment from a recent mic window + the played reference.

        Args:
            mic: recent mic samples (float32); mic[0] is at absolute mic index mic_start_idx.
            mic_start_idx: absolute mic-sample index (ts32) of mic[0].
            played_ref: full host-clock reference ring (index 0 = first downlink sample).
            search_center: live estimate of ref_base_mic (mic index where played_ref[0] is
                heard), from the playout position. The search runs +/- search_ms around it.

        Returns:
            ref_base_mic (float) if locked (new or held), else None.
        """
        mic = np.asarray(mic, dtype=np.float32)
        played_ref = np.asarray(played_ref, dtype=np.float32)
        w = len(mic)
        self._last_center = search_center
        if w == 0 or len(played_ref) == 0:
            return self._ref_base
        # Reference-on-mic span covering every candidate B in [center-span, center+span]:
        #   mic[m] ~ ref_on_mic_abs[m - B], ref_on_mic_abs[j] = played_ref[j*ratio].
        j0 = mic_start_idx - search_center - self.span
        j1 = mic_start_idx + w - search_center + self.span
        if j1 - j0 <= w:
            return self._ref_base
        j = np.arange(j0, j1, dtype=np.float64)
        ref_on_mic = np.interp(j * self.ratio, np.arange(len(played_ref), dtype=np.float64),
                               played_ref, left=0.0, right=0.0).astype(np.float32)
        if np.linalg.norm(ref_on_mic) < 1e-6:
            return self._ref_base                                # reference silent here

        ncc = _ncc_valid(mic, ref_on_mic)                        # k in [0, 2*span]
        if len(ncc) == 0:
            return self._ref_base
        kstar = int(np.argmax(ncc))
        peak = float(ncc[kstar])
        # Built so candidate B = center + span - k (see module/orchestrator alignment math).
        B = search_center + self.span - kstar

        self.confidence = peak
        if peak >= self.conf_threshold:
            if self._ref_base is None:
                self._ref_base = float(B)                        # initial (coarse) lock
            else:
                step = self.smooth * (B - self._ref_base)        # EMA toward the new estimate
                step = max(-self.max_step, min(self.max_step, step))   # hysteresis clamp
                self._ref_base += step
        return self._ref_base

    def current(self):
        """The tracked absolute alignment (mic index where played_ref[0] is heard), or None."""
        return self._ref_base

    @property
    def locked(self):
        return self._ref_base is not None

    @property
    def residual_ms(self):
        """Tracked alignment minus the last search center, in ms (diagnostic: how far the
        true alignment sat from the playout-derived seed). None until locked."""
        if self._ref_base is None or self._last_center is None:
            return None
        return (self._ref_base - self._last_center) * 1000.0 / self.sr
