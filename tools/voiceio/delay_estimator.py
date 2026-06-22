"""Online render->capture delay estimator (AEC3-style), so nothing acoustic is hardcoded.

The loop delay (downlink jitter buffer + BLE scheduling + I2S/PDM pipeline + uplink) is
NOT a universal constant -- it varies per unit, per session, and with BLE conditions, and
production AEC (WebRTC AEC3's echo_path_delay_estimator) estimates it ONLINE by cross-
correlating the far-end reference against the mic. This module does the same: it finds the
lag at which the played reference best matches the mic echo, and tracks it continuously.

It produces `ref_base_mic` -- the mic-sample index at which played_ref[0] is heard -- which
is exactly what orchestrator.aligned_reference needs. The orchestrator seeds it coarsely
from the downlink origin (the mic ts when the AI started playing); this estimator refines
the residual loop delay on top of that seed and keeps tracking it.

Robustness: only updates when the normalized correlation peak clears a confidence floor
(rejects silence / double-talk / no-echo); smooths the estimate so a single noisy frame
can't yank the alignment; holds the last good value otherwise. Drift between the two clocks
is removed first (the reference is put on the mic clock via AEC_DRIFT_RATIO) so it doesn't
smear the correlation peak.

[STRUCTURAL] This REPLACES the hardcoded ref_delay_ms. The only remaining fixed audio
constant is AEC_DRIFT_RATIO (exact clock-divider physics, placement-independent).
"""
import numpy as np

from voiceio.clocks import AEC_DRIFT_RATIO

SR = 16000


def _ncc_valid(mic, ref_win):
    """Normalized cross-correlation of `mic` (length W) slid over `ref_win`
    (length W + L). Returns an array of length L+1 where out[k] is the cosine
    similarity of mic vs ref_win[k:k+W]. Peak picks the best alignment."""
    w = len(mic)
    L = len(ref_win) - w
    if L < 0 or w == 0:
        return np.zeros(0, dtype=np.float64)
    mic = mic.astype(np.float64)
    ref_win = ref_win.astype(np.float64)
    mic_z = mic - mic.mean()
    mic_norm = np.linalg.norm(mic_z) + 1e-12
    # Raw correlation at every lag via FFT (np.correlate is O(W*L); FFT is cheaper here).
    full = np.correlate(ref_win, mic_z, mode="valid")        # length L+1
    # Per-lag reference-window energy for normalization (sliding, mean-removed).
    cs = np.concatenate([[0.0], np.cumsum(ref_win)])
    cs2 = np.concatenate([[0.0], np.cumsum(ref_win * ref_win)])
    out = np.zeros(L + 1, dtype=np.float64)
    for k in range(L + 1):
        s = cs[k + w] - cs[k]
        s2 = cs2[k + w] - cs2[k]
        var = s2 - s * s / w                                  # mean-removed energy
        denom = mic_norm * (np.sqrt(var) + 1e-12)
        out[k] = full[k] / denom
    return out


class DelayEstimator:
    def __init__(self, sr=SR, ratio=AEC_DRIFT_RATIO, search_lo_ms=-50.0,
                 search_hi_ms=500.0, conf_threshold=0.30, smooth=0.4):
        """
        Args:
            search_lo_ms / search_hi_ms: loop-delay range to search around the seed
                (negative allows the seed to be slightly late). [STRUCTURAL: widen if a
                future buffering change pushes the loop delay outside this window.]
            conf_threshold: min normalized-correlation peak to accept an update (rejects
                silence/double-talk). [USER/HOUSING: depends on echo strength on the final
                acoustic design; re-check when the speaker/placement is finalized.]
            smooth: EMA factor toward a new accepted estimate (0..1; lower = steadier).
        """
        self.sr = sr
        self.ratio = ratio
        self.lo = int(search_lo_ms * sr / 1000.0)
        self.hi = int(search_hi_ms * sr / 1000.0)
        self.conf_threshold = conf_threshold
        self.smooth = smooth
        self._loop_delay = None      # tracked residual delay (mic samples) on top of seed
        self.confidence = 0.0

    def reset(self):
        self._loop_delay = None
        self.confidence = 0.0

    def estimate(self, mic, mic_start_idx, played_ref, seed_base_mic):
        """Refine the loop delay from a recent mic window and the played reference.

        Args:
            mic: recent mic samples (float32); mic[0] is at absolute mic index mic_start_idx.
            mic_start_idx: absolute mic-sample index (ts32) of mic[0].
            played_ref: full host-clock reference ring (index 0 = first downlink sample).
            seed_base_mic: coarse mic index where played_ref[0] is heard (downlink origin;
                zero loop delay). True ref_base_mic = seed_base_mic + loop_delay.

        Returns:
            ref_base_mic (float) if a usable estimate exists (new or held), else None.
        """
        mic = np.asarray(mic, dtype=np.float32)
        played_ref = np.asarray(played_ref, dtype=np.float32)
        w = len(mic)
        # Reference span (on the mic clock) covering all candidate delays for this window:
        #   for delay d in [lo, hi], mic[i] (abs idx mic_start_idx+i) aligns to
        #   ref_on_mic index (mic_start_idx + i - seed_base_mic - d).
        k0 = mic_start_idx - seed_base_mic - self.hi          # smallest ref-on-mic index needed
        k1 = mic_start_idx + w - seed_base_mic - self.lo      # largest (exclusive)
        if w == 0 or k1 <= k0 or len(played_ref) == 0:
            return self._held(seed_base_mic)
        k_idx = np.arange(k0, k1, dtype=np.float64)
        host_idx = k_idx * self.ratio                          # map mic clock -> host ref clock
        ref_on_mic = np.interp(host_idx, np.arange(len(played_ref), dtype=np.float64),
                               played_ref, left=0.0, right=0.0).astype(np.float32)
        if np.linalg.norm(ref_on_mic) < 1e-6:
            return self._held(seed_base_mic)                   # reference silent here -> no echo

        ncc = _ncc_valid(mic, ref_on_mic)                      # out[k] for k in [0, hi-lo]
        if len(ncc) == 0:
            return self._held(seed_base_mic)
        kstar = int(np.argmax(ncc))
        peak = float(ncc[kstar])
        # Map the correlation lag back to a loop delay: we built ref_on_mic so that
        # ref_on_mic[k + i] aligns mic[i] at delay d = hi - k (see module/orchestrator math).
        d = self.hi - kstar

        self.confidence = peak
        if peak >= self.conf_threshold:
            if self._loop_delay is None:
                self._loop_delay = float(d)
            else:
                self._loop_delay += self.smooth * (d - self._loop_delay)
        return self._held(seed_base_mic)

    def current(self, seed_base_mic):
        """ref_base_mic from the currently-tracked loop delay + this turn's seed, without
        re-estimating. The loop delay PERSISTS across turns (same hardware) -- only the
        seed changes per response -- so this gives a valid alignment every block once
        locked, between the periodic estimate() refinements."""
        return self._held(seed_base_mic)

    def warm_start(self, loop_delay_ms):
        """Optionally pre-seed the tracked loop delay (e.g. from a prior measure_delay run)
        so the very first response doesn't pass through un-cancelled while acquiring."""
        self._loop_delay = float(loop_delay_ms) * self.sr / 1000.0

    def _held(self, seed_base_mic):
        if self._loop_delay is None:
            return None
        return seed_base_mic + self._loop_delay

    @property
    def loop_delay_ms(self):
        return None if self._loop_delay is None else self._loop_delay * 1000.0 / self.sr
