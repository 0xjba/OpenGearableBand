"""Real-time full-duplex voice orchestrator (Phase C3).

Ties the pieces into the live conversational loop, backend-agnostic:

  UPLINK   mic notify -> LC3 decode -> [DSP task] drift-aligned reference + DTLN AEC
           -> energy-gated barge-in VAD -> backend.feed_mic(clean)
  DOWNLINK backend.next_audio -> LC3 encode -> ble_link paced/clock-recovered send
  BARGE-IN VAD onset while AI is playing -> ble_link.flush() + backend.barge_in()

Threading/real-time discipline:
  * The BLE uplink callback is LIGHT (decode + enqueue only) -- never runs DSP on the
    event loop.
  * A single DSP task drains the mic queue SEQUENTIALLY (DtlnAec holds LSTM state and
    must see blocks in order) and runs the blocking AEC in a thread executor so the
    event loop stays responsive.
  * The downlink producer assembles fixed 320-sample (20 ms) blocks and hands them to
    ble_link, which paces transmission at the device's true consumption rate.

REFERENCE ALIGNMENT (the load-bearing, hardware-tuned part):
  The AEC needs the far-end reference time-aligned to the mic. The played-reference ring
  (ble_link) is on the host 16 kHz clock; the mic is on the device PDM clock. The echo of
  host-ref-sample k is heard by the mic at mic-sample  dl_origin_mic + delay + k/ratio
  (ratio = I2S/PDM = AEC_DRIFT_RATIO; the device plays slow). Inverting: for mic-sample m,
  the aligned reference is  played_ref[(m - dl_origin_mic - delay) * ratio]. `aligned_reference`
  is that pure mapping; `ref_base_mic` (= dl_origin_mic + delay) and `ref_delay_samples`
  are HW-tuned. Until the AI starts playing, the ring is empty -> reference is zeros ->
  AEC passes the mic through unchanged (correct: nothing to cancel).
"""
import asyncio
import time

import numpy as np

from voiceio import clock_recovery as cr
from voiceio.aec import DtlnAec
from voiceio.ble_link import BLOCK_SAMPLES, FRAME_BYTES, FRAME_SAMPLES, FRAMES_PER_BLOCK
from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.codec import Lc3Codec
from voiceio.delay_estimator import DelayEstimator

SR = 16000
# [USER/HOUSING] AEC-convergence FORWARDING gate (2026-07-01, HW-measured). We forward the cleaned mic
# to the backend only AFTER the AEC has DEMONSTRABLY cancelled this reply: clean_rms * AEC_CONV_RATIO <
# mic_rms (~14 dB ERLE) with real echo present, for AEC_CONV_BLOCKS consecutive blocks. Before that, at
# each reply's onset the alignment is still (re)acquiring and `clean` carries un-cancelled echo (the AI's
# OWN voice); forwarding it tripped the backend's VAD into a SELF-barge -- which flushed + reset the
# alignment, so the next reply's onset was un-cancelled again => a self-perpetuating barge loop. HW data:
# every self-barge fired in the first ~1-2 s with clean ~= mic (ratio 1-3x); the converged remainder
# (clean ~0.0001 = 20-200x below mic) NEVER barged. Latched per reply: once converged we forward
# everything (so a real LOUD barge still gets through); re-proven each reply (reset in the not-playing
# branch). Replaces the old fixed 1 s onset timer, which keyed off `locked` -- true even on a garbage lock.
AEC_CONV_RATIO    = 5.0     # clean must be < mic / this (~14 dB ERLE) to count as "cancelling"
AEC_CONV_MIN_MIC  = 0.003   # only judge convergence when there's real echo present to cancel
AEC_CONV_BLOCKS   = 3       # consecutive cancelling blocks before we start forwarding. (Briefly tried
                            # 2 on 2026-07-01 to shorten the first-reply onset hold, but it REGRESSED:
                            # latching "converged" after only 2 blocks flips the gate on earlier, so
                            # more of each reply is exposed to LATER AEC divergence that then gets
                            # forwarded -> self-barge. Dump analysis showed ~32 divergence-forward
                            # events. Reverted to 3. The real fix is a divergence-aware forward gate,
                            # not a shorter convergence proof -- see the latch vulnerability below.)
# Absolute inaudibility floor (2026-07-01): the ratio gate alone opened while clean was still ~0.002 --
# faint but COHERENT residual AI speech the user could hear AND the backend's speech-VAD self-barged on
# (a speech detector fires on low-level coherent speech, not just loud energy). So also require clean to
# be genuinely INAUDIBLE before forwarding. HW: audible self-barges sat at clean 0.0011-0.0019; truly
# silent stretches 0.0001-0.0002 -- so 0.0005 sits between, and the AEC's steady state clears it. [USER]
AEC_CONV_MAX_CLEAN = 0.0005
# ADAPTIVE FLOOR-RELATIVE barge gate (2026-07-02) -- replaces the old fixed BARGE_ABS_THRESH = 0.008.
# That was an ABSOLUTE amplitude gate ("clean_rms > 0.008 == the user"): the textbook Geigel method,
# hand-tuned for ONE mount / mic-gain / room, and fragile everywhere else. Production systems decide
# near-end speech RELATIVE to a tracked floor, not against a baked number. We now use the MCRA principle
# (Rangachari & Loizou 2006 / Cohen): decide "this is the user" when clean_rms rises BARGE_FLOOR_RATIO x
# above its own CONTINUOUSLY-TRACKED residual floor. The ratio is dimensionless -> level/gain-invariant
# -> portable across mount/gain/room with no re-tuning. Same minimum-controlled-averaging idea the
# firmware `mic_vad` already uses for its adaptive noise floor. (The floor tracker lives in the DSP loop.)
# Grounding: deep-research workflow wo2yue0su. [STRUCTURAL] -- these are dimensionless ratios / time
# constants, portable; only BARGE_FLOOR_RATIO is a real behavioural knob, tuned LIVE (validation is live).
# SCOPE: this is a PORTABILITY fix. It does NOT cure the volume-boost self-barge -- loud residual and a
# real barge both raise the ratio (the proven energy ceiling: no energy signal separates them). That
# remains a hardware (5V amp + decoupling) / identity (Personal VAD) problem, per docs/PROJECT-LOG.md.
BARGE_FLOOR_RATIO   = 12.0      # [STRUCTURAL] dimensionless -> portable. User when clean_rms > this x the
                               #   tracked residual floor (~+21 dB; above residual spikes, below a real
                               #   barge). The one behavioural knob -- tune ONCE live, then holds across rigs.
BARGE_FLOOR_RECOVER = 1.003     # [STRUCTURAL] per-block up-creep of the floor so it recovers when the
                               #   residual genuinely rises (minimum-statistics slow release); slightly > 1.
BARGE_FLOOR_MIN     = 1e-4      # [UNIT] floor guard ~ mic self-noise amplitude (hardware-dependent), so a
                               #   near-silent residual can't make the ratio hypersensitive (div-by-~0).
BARGE_HANGOVER_BLOCKS = 25     # ~0.5 s: keep forwarding after a loud block, to bridge word gaps
CAL_SECONDS = 0.8          # length of the startup calibration chirp
SESSION_END_S = 2.5           # uplink silent this long (arm lowered) -> end the conversation
HIST_SAMPLES = SR // 2        # 0.5 s mic window fed to the delay estimator
EST_EVERY_BLOCKS = 12         # refine the alignment ~every 240 ms while the AI plays
# In-flight buffer between "sample transmitted" and "sample heard by the mic": the device
# playout jitter buffer (~setpoint) plus whatever ble_link still has queued. Used to derive
# the delay estimator's search center from the playout position.
SETPOINT_SAMPLES = cr.SETPOINT_MS * SR // 1000


def make_cal_chirp(seconds=CAL_SECONDS, sr=SR):
    """A short rising chirp (300->3000 Hz) used as a startup calibration earcon.

    A chirp has a sharp autocorrelation peak (good for delay estimation) and reads as
    a brief 'ready' swoop. The estimator locks on its echo so the alignment is acquired
    BEFORE the AI's first reply -- the user never has to wait in silence. Fades in/out
    to avoid clicks."""
    n = int(seconds * sr)
    t = np.arange(n, dtype=np.float64) / sr
    f0, f1 = 300.0, 3000.0
    chirp = 0.2 * np.sin(2 * np.pi * (f0 * t + (f1 - f0) / (2 * seconds) * t * t))
    fade = np.clip(np.minimum(t / 0.05, (seconds - t) / 0.05), 0.0, 1.0)  # 50 ms fades
    return (chirp * fade).astype(np.float32)


def aligned_reference(played_ref, ref_base_mic, mic_count_start, n, ratio=AEC_DRIFT_RATIO):
    """Far-end reference aligned to mic samples [mic_count_start, mic_count_start+n).

    out[i] = played_ref[(mic_count_start + i - ref_base_mic) * ratio], with samples
    that fall outside the played-reference ring set to 0 (no echo there yet).

    Args:
        played_ref: host-16k float32 reference ring (everything transmitted so far).
        ref_base_mic: mic-sample index at which played_ref[0] is heard by the mic
            (= dl_origin_mic + loop-delay; hardware-tuned).
        mic_count_start: ts32 of the first mic sample in this block.
        n: number of samples to produce (== mic block length).
        ratio: I2S/PDM clock ratio (AEC_DRIFT_RATIO).
    """
    played_ref = np.asarray(played_ref, dtype=np.float32)
    i = np.arange(n, dtype=np.float64)
    k = (mic_count_start + i - ref_base_mic) * ratio          # fractional ref indices
    out = np.zeros(n, dtype=np.float32)
    if len(played_ref) == 0:
        return out
    valid = (k >= 0) & (k <= len(played_ref) - 1)
    if np.any(valid):
        out[valid] = np.interp(k[valid], np.arange(len(played_ref), dtype=np.float64),
                               played_ref).astype(np.float32)
    return out


class Orchestrator:
    """Full-duplex real-time voice loop over a BleLink + a VoiceBackend."""

    def __init__(self, ble_link, backend, model_dir, lib_path="tools/lib/liblc3.dylib",
                 calibrate_on_start=True, dump_path=None):
        """
        Args:
            ble_link: a connected BleLink (call .connect() first).
            backend: a VoiceBackend (EchoLoopbackBackend / GeminiLiveBackend).
            model_dir: DTLN model path prefix (e.g. ".../dtln_aec_512").
            lib_path: liblc3 shared lib.

        BARGE-IN MODEL (finalized 2026-06-30, HW-validated): there is NO local barge detector. While
        the AI plays, the orchestrator forwards the AEC-cleaned mic to the backend (gated off during
        the pre-lock onset); the backend's own VAD hears the user talk over the AI and fires its
        interrupt, which _server_barge_loop mirrors with a local flush+reset. This "clean signal +
        backend VAD" design beat a local energy detector in the A/B (the energy detector false-fired
        on ambient + residual echo; the backend VAD did not), so the energy detector, abs-floor, and
        auto/manual harness modes were removed. It needs no Silero/DTD, and is viable because the AEC
        residual is low in steady state (clean ~0.0000-0.001). See docs/research/barge-in-aec-2026-research.md.

        The render->capture alignment is tracked live by DelayEstimator (AEC3-style) -- no
        hardcoded delay -- so it self-calibrates per unit/session and carries to production
        hardware unchanged.
        """
        self.ble = ble_link
        self.backend = backend
        self.codec = Lc3Codec(lib_path)
        self.aec = DtlnAec(model_dir)
        self.delay_est = DelayEstimator()

        self._mic_q = asyncio.Queue()
        self._last_mic_ts = 0
        self._dl_origin_mic = None     # coarse seed: mic ts32 when this response started playing
        self._playing = False          # is the AI currently being streamed out?
        self._aec_converged = False    # has the AEC proven it's cancelling THIS reply? (forward gate)
        self._conv_blocks = 0          # consecutive cancelling blocks toward AEC_CONV_BLOCKS
        self._fwd_hangover = 0         # blocks left to keep forwarding after a loud (user) block
        self._dl_residual = np.zeros(0, dtype=np.float32)  # sub-block carry for encoding
        self._mic_hist = np.zeros(0, dtype=np.float32)     # recent mic for the delay estimator
        self._mic_hist_start = 0       # abs mic index of _mic_hist[0]
        self._since_est = 0
        self._aec_mic = np.zeros(0, dtype=np.float32)      # carry to feed AEC in 128-multiples
        self._aec_ref = np.zeros(0, dtype=np.float32)
        self._stop = asyncio.Event()
        # Startup calibration: play a brief chirp the moment the mic activates (when the
        # backend isn't already playing) so the alignment locks before the AI's first reply.
        self.calibrate_on_start = calibrate_on_start
        self._cal_audio = np.zeros(0, dtype=np.float32)    # queued calibration samples to play
        self._calibrating = False                          # suppress barge-in during the chirp
        # Conversation session lifecycle: a session = one raise-to-ear conversation. It opens
        # when uplink audio starts (arm raised / mic forced) and closes when uplink goes silent
        # for SESSION_END_S (arm lowered), which opens/closes the backend's AI socket so an idle
        # band isn't holding one open (cost/battery/privacy).
        self._session_active = False
        # --- bring-up instrumentation (printed by _status_loop) ---
        self._n_uplink = 0             # mic frames received from the band
        self._n_blocks_sent = 0        # downlink blocks transmitted
        self._n_onsets = 0             # barge-ins handled (backend-VAD interrupts mirrored locally)
        self._mic_rms = 0.0            # last raw mic level
        self._clean_rms = 0.0          # last echo-cancelled level
        self._clean_floor = 0.0        # adaptive residual-echo floor (MCRA min-tracker) for the barge
                                       # gate; 0 = unseeded, self-seeds on the first block, reset per session
        # Debug (--dump-clean): record the EXACT signal forwarded to the backend (the "cleaned" mic
        # Gemini's VAD judges) and the raw mic, to two WAVs, so residual echo can be heard directly.
        self._dump_path = dump_path
        self._dump_fed = [] if dump_path else None
        self._dump_mic = [] if dump_path else None
        self._dump_aec = [] if dump_path else None   # un-gated AEC `clean` output (during playback)

    # --- uplink: light callback (decode + enqueue only) ---

    def _on_uplink(self, frame):
        self._last_mic_ts = frame.ts32
        self._n_uplink += 1
        pcm = self._decode_block(frame.lc3)
        if len(pcm):
            self._mic_q.put_nowait((frame.ts32, pcm))

    def _decode_block(self, lc3_payload):
        """[N*40B LC3] -> float32 [-1,1] (concatenated decoded frames)."""
        chunks = []
        for off in range(0, len(lc3_payload) - FRAME_BYTES + 1, FRAME_BYTES):
            pcm16 = self.codec.decode(lc3_payload[off:off + FRAME_BYTES])  # int16 (160,)
            chunks.append(pcm16.astype(np.float32) / 32768.0)
        return np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.float32)

    def _forward(self, pcm):
        """Feed the backend the user-side audio, and (debug) record exactly what we forwarded
        so it can be played back to check for residual echo (what Gemini's VAD actually judges)."""
        self.backend.feed_mic(pcm)
        if self._dump_fed is not None:
            self._dump_fed.append(np.asarray(pcm, dtype=np.float32).copy())

    # --- DSP task: sequential AEC, then forward the cleaned mic to the backend ---

    async def _dsp_loop(self):
        import traceback
        loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            try:
                try:
                    mic_ts, mic = await asyncio.wait_for(self._mic_q.get(), timeout=0.2)
                except asyncio.TimeoutError:
                    continue
                n = len(mic)
                self._mic_rms = float(np.sqrt(np.mean(mic ** 2) + 1e-12))
                if self._dump_mic is not None:
                    self._dump_mic.append(np.asarray(mic, dtype=np.float32).copy())

                if not self._playing:
                    # AI silent -> no echo to cancel. Pass the user's mic STRAIGHT to the
                    # backend: running DTLN with a zero reference distorts clean speech (seen
                    # as clean_rms > mic_rms in the play=0 stat lines), which would corrupt
                    # the user's turn audio sent to the AI. Drop any AEC carry so the next
                    # playing turn starts clean.
                    self._clean_rms = self._mic_rms
                    self._aec_mic = np.zeros(0, dtype=np.float32)
                    self._aec_ref = np.zeros(0, dtype=np.float32)
                    self._aec_converged = False    # next reply must re-prove cancellation
                    self._conv_blocks = 0
                    self._fwd_hangover = 0
                    self._forward(mic)
                    continue

                # --- AI is playing: drift-aligned reference + DTLN echo cancellation ---
                played = self.ble.played_reference()
                self._push_mic_hist(mic_ts, mic)
                # Refine the alignment online (~every 240 ms) -- no hardcoded constant.
                self._since_est += 1
                if self._since_est >= EST_EVERY_BLOCKS and len(self._mic_hist) >= HIST_SAMPLES:
                    self._since_est = 0
                    # Live search center from the playout position. played_samples counts only
                    # TRANSMITTED samples (ble_link increments it as it paces sends), so the
                    # sample HEARD now is played_samples minus only the DEVICE buffer
                    # (~setpoint) -- NOT the host send queue (downlink_pending), which sits
                    # BEFORE transmission and would wrongly add seconds when a backend bursts a
                    # whole reply in. Heard at the most recent mic sample: center =
                    # mic_now - heard_idx/ratio (~ ref_base_mic).
                    mic_now = self._mic_hist_start + len(self._mic_hist)
                    heard_idx = max(0, self.ble.played_samples - SETPOINT_SAMPLES)
                    center = int(mic_now - heard_idx / AEC_DRIFT_RATIO)
                    self.delay_est.estimate(self._mic_hist, self._mic_hist_start, played, center)
                ref = np.zeros(n, dtype=np.float32)
                ref_base_mic = self.delay_est.current()
                if ref_base_mic is not None:
                    ref = aligned_reference(played, ref_base_mic, mic_ts, n)
                # else: estimator not locked yet -> pass mic through (brief, first turn)

                # DTLN consumes whole 128-sample hops; a 320-sample block is NOT a multiple of
                # 128, so feeding it directly would DROP 64 samples/block (20% loss). Accumulate
                # mic+ref and feed the largest 128-multiple, carrying the remainder. Aligned.
                self._aec_mic = np.concatenate([self._aec_mic, mic])
                self._aec_ref = np.concatenate([self._aec_ref, ref])
                m = (len(self._aec_mic) // 128) * 128
                if m == 0:
                    continue
                mic_chunk, self._aec_mic = self._aec_mic[:m], self._aec_mic[m:]
                ref_chunk, self._aec_ref = self._aec_ref[:m], self._aec_ref[m:]
                # AEC is blocking -> run in a thread; sequential (await) preserves order.
                clean = await loop.run_in_executor(None, self.aec.process, mic_chunk, ref_chunk)
                if len(clean) == 0:
                    continue
                self._clean_rms = float(np.sqrt(np.mean(clean ** 2) + 1e-12))
                # AEC INSTABILITY CLAMP (2026-07-01): a canceller can only REMOVE echo, never ADD
                # energy -- clean <= mic is a structural property of echo cancellation. But DTLN
                # occasionally goes unstable and emits MORE energy than its mic input (HW-measured
                # ratio up to 2.4x: clean 0.032 from mic 0.013). That hallucinated output is pure
                # divergence -- if forwarded it self-barges. Proven NOT caused by playback underruns
                # (0/201 divergence blocks coincided with a device UNDERRUN, buffer healthy), so it's
                # internal to the model. A genuine near-end barge is always <= mic (the user's voice
                # IS in the mic), so scaling clean down to mic level can only ever cancel divergence,
                # never attenuate a real barge. [STRUCTURAL: holds for any AEC, not a tuned constant.]
                # Compare against the SAME chunk's mic level (not self._mic_rms, the raw 320-sample
                # block -- `clean` is the AEC output of `mic_chunk`, a 128-multiple that spans
                # different samples due to the carry, so the block-RMS is mis-windowed and let a few
                # marginal clean>mic blocks slip the clamp).
                mic_chunk_rms = float(np.sqrt(np.mean(mic_chunk ** 2) + 1e-12))
                if self._clean_rms > mic_chunk_rms and mic_chunk_rms > 1e-6:
                    over = self._clean_rms / mic_chunk_rms
                    clean = clean * (mic_chunk_rms / self._clean_rms)
                    self._clean_rms = mic_chunk_rms
                    if mic_chunk_rms > 0.005:   # log-verbosity only: skip near-silent clamp events
                        print(f"[aec-clamp] DTLN divergence caught: {over:.2f}x over mic "
                              f"-> clamped to mic={mic_chunk_rms:.4f}")
                if self._dump_aec is not None:
                    self._dump_aec.append(np.asarray(clean, dtype=np.float32).copy())
                # AEC-convergence FORWARDING gate: prove the AEC is cancelling THIS reply (clean well
                # below mic, real echo present, for a few blocks) BEFORE we forward the cleaned mic to
                # the backend's VAD -- otherwise the un-cancelled onset echo (the AI's own voice) self-
                # barges it. Latch once converged so a real LOUD barge (clean spikes with the user's
                # voice) still gets forwarded; the latch is reset per reply in the not-playing branch.
                if not self._aec_converged and not self._calibrating:
                    if (self._mic_rms > AEC_CONV_MIN_MIC and
                            self._clean_rms * AEC_CONV_RATIO < self._mic_rms and
                            self._clean_rms < AEC_CONV_MAX_CLEAN):
                        self._conv_blocks += 1
                        if self._conv_blocks >= AEC_CONV_BLOCKS:
                            self._aec_converged = True
                    else:
                        self._conv_blocks = 0
                # --- Adaptive floor-relative barge gate (MCRA ratio; replaces the fixed 0.008) ---
                # Track the RESIDUAL-ECHO FLOOR of `clean` (the quiet level between the user's words)
                # with a minimum-statistics tracker: snap DOWN to any new minimum instantly, and RECOVER
                # up only SLOWLY -- and only while NOT barging (fwd_hangover==0), so the user's own voice
                # can't inflate the floor. Then "this is the USER" = clean rises BARGE_FLOOR_RATIO x above
                # that tracked floor. This is the Rangachari&Loizou / Cohen MCRA principle (ratio of the
                # signal to a tracked local minimum) -> level/gain-invariant -> portable, no rig-specific
                # absolute. Reset per session (below); self-seeds on the first block.
                # [The forward decision downstream is unchanged: locked + conv=1 latch + hangover.]
                # SCOPE reminder: PORTABILITY only -- a loud residual (volume boost) also crosses the
                # ratio, so this does NOT cure the boost self-barge (energy ceiling); that stays HW/identity.
                if self._clean_floor <= 0.0 or self._clean_rms < self._clean_floor:
                    self._clean_floor = self._clean_rms          # instant attack: snap to a new minimum
                elif self._fwd_hangover == 0:
                    self._clean_floor *= BARGE_FLOOR_RECOVER      # slow release UP (only when not barging)
                self._clean_floor = max(self._clean_floor, BARGE_FLOOR_MIN)   # guard near-silence
                if self._aec_converged and self._clean_rms > BARGE_FLOOR_RATIO * self._clean_floor:
                    self._fwd_hangover = BARGE_HANGOVER_BLOCKS
                forward = (self.delay_est.locked and not self._calibrating
                           and self._aec_converged and self._fwd_hangover > 0)
                if self._fwd_hangover > 0:
                    self._fwd_hangover -= 1
                self._forward(clean if forward
                              else np.zeros(len(clean), dtype=np.float32))
            except Exception:  # surface a crash instead of silently killing the task
                print("[orch] DSP loop error:\n" + traceback.format_exc())

    def _push_mic_hist(self, mic_ts, mic):
        """Append the latest mic block to the recent-history window the delay estimator
        correlates against. Resets on a sequence discontinuity so absolute indexing stays
        correct, and trims to HIST_SAMPLES."""
        expected = self._mic_hist_start + len(self._mic_hist)
        if len(self._mic_hist) == 0 or mic_ts != expected:
            self._mic_hist = np.asarray(mic, dtype=np.float32)
            self._mic_hist_start = mic_ts
            return
        self._mic_hist = np.concatenate([self._mic_hist, mic])
        if len(self._mic_hist) > HIST_SAMPLES:
            drop = len(self._mic_hist) - HIST_SAMPLES
            self._mic_hist = self._mic_hist[drop:]
            self._mic_hist_start += drop

    def _reset_mic_hist(self):
        self._mic_hist = np.zeros(0, dtype=np.float32)
        self._mic_hist_start = 0
        self._since_est = 0
        self._aec_mic = np.zeros(0, dtype=np.float32)
        self._aec_ref = np.zeros(0, dtype=np.float32)

    async def _local_barge_cleanup(self):
        """Orchestrator-side of a barge (no backend call): flush the device playout + reset the
        per-turn playing/alignment state so the next reply re-acquires cleanly. Shared by the
        local-detector path (_barge_in) and the server-initiated path (_server_barge_loop)."""
        await self.ble.flush()
        self._dl_residual = np.zeros(0, dtype=np.float32)
        self._playing = False
        # Clear the per-turn origin: the downlink loop re-arms _playing only when _dl_origin_mic is
        # None (a fresh response). Leaving it set keeps _playing False on every later reply -> no AEC.
        self._dl_origin_mic = None
        # AEC-DIVERGENCE FIX (2026-07-01): do NOT reset the delay estimator here. The physical loop
        # delay (speaker->air->mic + BLE/buffers) is HARDWARE-STABLE across replies, so the converged
        # lock should PERSIST. Resetting forced a full re-acquisition whose first lock is un-clamped and,
        # in the post-barge double-talk/discontinuity, grabbed a SPURIOUS peak -> mis-aligned AEC ->
        # residual SPIKE -> self-barge -> reset -> repeat (the vicious cycle behind the res -12000ms
        # blow-ups). Keeping the lock lets the hysteresis clamp hold it steady; only the per-turn DSP
        # carry/history is cleared (a playout discontinuity, not a delay change). delay_est is still
        # reset between CONVERSATIONS in _reset_session_state.
        self._reset_mic_hist()

    async def _server_barge_loop(self):
        """The barge-in path: the backend's own VAD detects the user talking over the AI and fires
        its interrupt (the backend drops its local playout queue + bumps its barge counter). We poll
        that counter and mirror each interrupt with the orchestrator-side cleanup (flush the device
        playout + reset alignment). Backend-agnostic via server_barge_count() -- 0 for backends with
        no server VAD (e.g. the loopback test backend), which therefore never barge."""
        last = self.backend.server_barge_count()
        while not self._stop.is_set():
            await asyncio.sleep(0.04)
            n = self.backend.server_barge_count()
            if n > last:
                last = n
                self._n_onsets += 1
                print(f"[barge] {time.monotonic():.3f} backend interrupt #{n} "
                      f"-> flush device playout + reset alignment")
                await self._local_barge_cleanup()

    # --- downlink producer: assemble 20 ms blocks from backend audio ---

    async def _downlink_loop(self):
        import traceback
        while not self._stop.is_set():
            try:
                # Calibration chirp (if queued) plays first; otherwise the backend's audio,
                # but ONLY during an active conversation -- between sessions a backend may
                # still hold a stale tail, and an idle band must play nothing.
                if len(self._cal_audio) > 0:
                    take = min(BLOCK_SAMPLES, len(self._cal_audio))
                    got = self._cal_audio[:take]
                    self._cal_audio = self._cal_audio[take:]
                elif self._session_active:
                    got = self.backend.next_audio(BLOCK_SAMPLES)
                else:
                    got = np.zeros(0, dtype=np.float32)
                if len(got):
                    self._dl_residual = np.concatenate([self._dl_residual, got])
                # Emit as many whole 320-sample blocks as we have.
                while len(self._dl_residual) >= BLOCK_SAMPLES:
                    if self._dl_origin_mic is None:
                        # First block of a fresh response -> anchor the reference origin to
                        # the mic timeline (the mic ts as this response starts playing). The
                        # delay estimator keeps its tracked loop delay across turns (same
                        # hardware); only this per-turn seed changes.
                        self._dl_origin_mic = self._last_mic_ts
                        self._playing = True
                        self._reset_mic_hist()
                    block = self._dl_residual[:BLOCK_SAMPLES]
                    self._dl_residual = self._dl_residual[BLOCK_SAMPLES:]
                    self._send_block(block)
                # _playing reflects audio still QUEUED/PLAYING (ble_link drains over real
                # time), NOT the producer's per-iteration output -- a burst-produced reply
                # (Gemini) is enqueued in one shot but plays out over seconds, and barge-in
                # must stay armed for that whole window.
                drained = (not len(got)
                           and len(self._dl_residual) < BLOCK_SAMPLES
                           and self.ble.downlink_pending() == 0)
                if drained and self._playing:
                    self._playing = False
                    self._dl_origin_mic = None
                if not len(got):
                    await asyncio.sleep(0.01)
            except Exception:
                print("[orch] downlink loop error:\n" + traceback.format_exc())
                await asyncio.sleep(0.05)

    def _send_block(self, pcm_block):
        """Encode 320 float32 samples -> 2x40B LC3 and hand to ble_link with the PCM ref."""
        pcm16 = np.clip(pcm_block, -1.0, 1.0)
        pcm16 = (pcm16 * 32767.0).astype(np.int16)
        lc3 = b"".join(
            self.codec.encode(pcm16[f * FRAME_SAMPLES:(f + 1) * FRAME_SAMPLES])
            for f in range(FRAMES_PER_BLOCK))
        self.ble.send_downlink(lc3, pcm_block)
        self._n_blocks_sent += 1

    # --- bring-up status line (so the first HW runs are diagnosable, not guesswork) ---

    async def _status_loop(self):
        prev_up = 0
        while not self._stop.is_set():
            await asyncio.sleep(1.0)
            # ERLE (echo-cancellation quality) is the meaningful AEC-health metric: how far `clean`
            # sits below `mic` during playback. The estimator's residual_ms readout was MISLEADING --
            # it drifts with the playout center after barge-flushes (played_samples counts flushed
            # audio) even while the AEC cancels fine, so a scary `res -31000 ms` had clean ~0.0002.
            # We show ERLE here instead and keep residual_ms internal-only. (2026-07-01.)
            lock = "locked" if self.delay_est.locked else "acquiring"
            d_str = f"{lock}(conf{self.delay_est.confidence:.2f})"
            erle_db = (20.0 * np.log10(self._mic_rms / max(self._clean_rms, 1e-6))
                       if self._playing else 0.0)
            up_rate = self._n_uplink - prev_up
            prev_up = self._n_uplink
            # mic_rms = raw mic level; clean_rms = level after AEC; erle = cancellation in dB (higher =
            # better; >~20 dB during play means the AEC is cancelling well). barge_ins = backend-VAD
            # interrupts handled. conv = AEC has converged enough this reply to forward the mic.
            # floor = the adaptive residual floor; the barge gate fires when clean_rms > RATIO x floor,
            # so watch clean_rms/floor vs BARGE_FLOOR_RATIO to sanity-check/tune the ratio live.
            print(f"[stat] uplink={self._n_uplink}(+{up_rate}/s) sent={self._n_blocks_sent} "
                  f"play={int(self._playing)} delay={d_str} erle={erle_db:+.0f}dB "
                  f"mic_rms={self._mic_rms:.4f} clean_rms={self._clean_rms:.4f} "
                  f"floor={self._clean_floor:.4f} conv={int(self._aec_converged)} "
                  f"barge_ins={self._n_onsets}")

    async def _calibrate(self):
        """Acquire the echo alignment at session start so barge-in is armed from the AI's
        first reply -- no user-visible silence. Called when the mic is already active (a
        session just opened). If the backend is NOT already playing (user-speaks-first), play
        a brief chirp and lock on its echo; if something IS already playing (loopback), that
        audio locks it, so the chirp is skipped."""
        if self._stop.is_set() or self._playing or self.delay_est.locked:
            return
        print("[cal] mic active, backend silent -> playing calibration chirp to lock alignment")
        self._calibrating = True
        self._cal_audio = make_cal_chirp()
        for _ in range(int((CAL_SECONDS + 2.5) / 0.05)):   # wait for lock (chirp + margin)
            if self.delay_est.locked or self._stop.is_set() or not self._session_active:
                break
            await asyncio.sleep(0.05)
        if self.delay_est.locked:
            print(f"[cal] locked (res {self.delay_est.residual_ms:+.0f} ms, "
                  f"conf {self.delay_est.confidence:.2f}) -- barge-in armed for the first reply")
        else:
            print("[cal] not locked from chirp; will acquire online on the first reply")
        self._calibrating = False

    # --- conversation session lifecycle (driven by uplink presence) ---

    async def _session_loop(self):
        """Open a session when uplink audio starts (arm raised / mic forced), close it when
        uplink goes silent for SESSION_END_S (arm lowered). The firmware already gates the
        uplink on POSE_EAR/MODE_DICTATION, so uplink presence == 'user is in a conversation'."""
        loop = asyncio.get_running_loop()
        prev_uplink = self._n_uplink
        last_active = loop.time()
        while not self._stop.is_set():
            await asyncio.sleep(0.3)
            now = loop.time()
            active_now = self._n_uplink > prev_uplink     # frames arrived since last check
            prev_uplink = self._n_uplink
            if active_now:
                last_active = now
            if not self._session_active:
                if active_now:
                    await self._start_session()
            elif now - last_active > SESSION_END_S:
                await self._end_session()

    async def _start_session(self):
        print("[session] start -- uplink active (conversation begun)")
        self._reset_session_state()
        try:
            self.backend.start_session()              # may block ~1 s opening the AI socket
        except Exception as e:  # noqa: BLE001
            print(f"[session] backend start failed: {e!r}")
            return
        self._session_active = True
        if self.calibrate_on_start:
            asyncio.create_task(self._calibrate())

    async def _end_session(self):
        print("[session] end -- uplink silent (arm lowered); closing AI session")
        self._session_active = False
        await self.ble.flush()
        try:
            self.backend.end_session()
        except Exception as e:  # noqa: BLE001
            print(f"[session] backend end error: {e!r}")
        self._reset_session_state()

    def _reset_session_state(self):
        """Clear all per-conversation DSP/playout state (between conversations)."""
        self._playing = False
        self._aec_converged = False
        self._conv_blocks = 0
        self._fwd_hangover = 0
        self._clean_floor = 0.0        # re-seed the adaptive barge floor for the new conversation
        self._dl_origin_mic = None
        self._dl_residual = np.zeros(0, dtype=np.float32)
        self._cal_audio = np.zeros(0, dtype=np.float32)
        self._calibrating = False
        self.aec.reset()
        self.delay_est.reset()
        self._reset_mic_hist()

    # --- lifecycle ---

    async def run(self):
        """Start streaming + the DSP/downlink/session tasks; runs until stop()."""
        self.aec.reset()
        await self.ble.start(self._on_uplink)
        tasks = [
            asyncio.create_task(self._dsp_loop()),
            asyncio.create_task(self._downlink_loop()),
            asyncio.create_task(self._status_loop()),
            asyncio.create_task(self._session_loop()),
            asyncio.create_task(self._server_barge_loop()),
        ]
        try:
            await self._stop.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if self._dump_path and self._dump_fed:
                import soundfile as sf
                fed = np.concatenate(self._dump_fed)
                mic = np.concatenate(self._dump_mic) if self._dump_mic else np.zeros(0, np.float32)
                base = self._dump_path.rsplit(".", 1)[0]
                mic_path, aec_path = base + ".mic.wav", base + ".aec.wav"
                sf.write(self._dump_path, fed, SR)
                sf.write(mic_path, mic, SR)
                msg = (f"[dump] forwarded ({len(fed)/SR:.1f}s) -> {self._dump_path}; "
                       f"raw mic -> {mic_path}")
                if self._dump_aec:
                    sf.write(aec_path, np.concatenate(self._dump_aec), SR)
                    msg += f"; UN-GATED AEC clean ({len(np.concatenate(self._dump_aec))/SR:.1f}s) -> {aec_path}"
                print(msg)

    def stop(self):
        self._stop.set()
