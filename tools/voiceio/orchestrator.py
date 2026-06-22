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

import numpy as np

from voiceio.aec import DtlnAec
from voiceio.ble_link import BLOCK_SAMPLES, FRAME_BYTES, FRAME_SAMPLES, FRAMES_PER_BLOCK
from voiceio.clocks import AEC_DRIFT_RATIO
from voiceio.codec import Lc3Codec
from voiceio.delay_estimator import DelayEstimator
from voiceio.vad import BargeInVad

SR = 16000
HIST_SAMPLES = SR // 2        # 0.5 s mic window fed to the delay estimator
EST_EVERY_BLOCKS = 12         # refine the delay ~every 240 ms while the AI plays


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
                 init_delay_ms=None):
        """
        Args:
            ble_link: a connected BleLink (call .connect() first).
            backend: a VoiceBackend (EchoLoopbackBackend / GeminiLiveBackend).
            model_dir: DTLN model path prefix (e.g. ".../dtln_aec_512").
            lib_path: liblc3 shared lib.
            init_delay_ms: OPTIONAL warm-start for the online delay estimator (e.g. a
                measure_delay value) so the first response isn't un-cancelled while the
                estimator acquires. None = pure online acquisition. NOTE: this is no longer
                a hardcoded constant -- the loop delay is tracked live by DelayEstimator
                (AEC3-style), so it self-calibrates per unit/session and carries to
                production hardware unchanged.
        """
        self.ble = ble_link
        self.backend = backend
        self.codec = Lc3Codec(lib_path)
        self.aec = DtlnAec(model_dir)
        self.vad = BargeInVad(sr=SR)
        self.delay_est = DelayEstimator()
        if init_delay_ms is not None:
            self.delay_est.warm_start(init_delay_ms)

        self._mic_q = asyncio.Queue()
        self._last_mic_ts = 0
        self._dl_origin_mic = None     # coarse seed: mic ts32 when this response started playing
        self._playing = False          # is the AI currently being streamed out?
        self._dl_residual = np.zeros(0, dtype=np.float32)  # sub-block carry for encoding
        self._mic_hist = np.zeros(0, dtype=np.float32)     # recent mic for the delay estimator
        self._mic_hist_start = 0       # abs mic index of _mic_hist[0]
        self._since_est = 0
        self._stop = asyncio.Event()
        # --- bring-up instrumentation (printed by _status_loop) ---
        self._n_uplink = 0             # mic frames received from the band
        self._n_blocks_sent = 0        # downlink blocks transmitted
        self._n_onsets = 0             # barge-ins fired
        self._mic_rms = 0.0            # last raw mic level
        self._clean_rms = 0.0          # last echo-cancelled level (what the VAD sees)

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

    # --- DSP task: sequential AEC + VAD + barge-in + feed backend ---

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
                ref = np.zeros(n, dtype=np.float32)
                if self._playing and self._dl_origin_mic is not None:
                    played = self.ble.played_reference()
                    self._push_mic_hist(mic_ts, mic)
                    # Refine the loop delay online (~every 240 ms) -- no hardcoded constant.
                    self._since_est += 1
                    if self._since_est >= EST_EVERY_BLOCKS and len(self._mic_hist) >= HIST_SAMPLES:
                        self._since_est = 0
                        self.delay_est.estimate(self._mic_hist, self._mic_hist_start,
                                                played, self._dl_origin_mic)
                    ref_base_mic = self.delay_est.current(self._dl_origin_mic)
                    if ref_base_mic is not None:
                        ref = aligned_reference(played, ref_base_mic, mic_ts, n)
                    # else: estimator not locked yet -> pass mic through (brief, first turn)
                # AEC is blocking -> run in a thread; sequential (await) preserves order.
                clean = await loop.run_in_executor(None, self.aec.process, mic, ref)
                if len(clean) == 0:
                    continue
                self._clean_rms = float(np.sqrt(np.mean(clean ** 2) + 1e-12))
                # Barge-in: only meaningful while the AI is talking.
                if self._playing:
                    onsets = self.vad.process(clean)
                    if onsets:
                        self._n_onsets += 1
                        await self._barge_in()
                self.backend.feed_mic(clean)
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

    async def _barge_in(self):
        print("[orch] barge-in -> flush downlink + stop backend")
        await self.ble.flush()
        self.backend.barge_in()
        self._dl_residual = np.zeros(0, dtype=np.float32)
        self._playing = False
        self._reset_mic_hist()
        self.vad.reset()

    # --- downlink producer: assemble 20 ms blocks from backend audio ---

    async def _downlink_loop(self):
        import traceback
        while not self._stop.is_set():
            try:
                got = self.backend.next_audio(BLOCK_SAMPLES)
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
            d_ms = self.delay_est.loop_delay_ms
            d_str = f"{d_ms:.0f}ms/conf{self.delay_est.confidence:.2f}" if d_ms is not None else "acquiring"
            up_rate = self._n_uplink - prev_up
            prev_up = self._n_uplink
            # mic_rms = raw mic level (is the band hearing anything?); clean_rms = what the
            # VAD sees after AEC; floor = VAD's tracked echo floor. Barge-in needs
            # clean_rms > floor*~4 (12 dB) AND clean_rms > abs floor.
            print(f"[stat] uplink={self._n_uplink}(+{up_rate}/s) sent={self._n_blocks_sent} "
                  f"play={int(self._playing)} delay={d_str} "
                  f"mic_rms={self._mic_rms:.4f} clean_rms={self._clean_rms:.4f} "
                  f"vad_floor={self.vad.floor if self.vad.floor is not None else 0:.4f} "
                  f"barge_ins={self._n_onsets}")

    # --- lifecycle ---

    async def run(self):
        """Start streaming + the DSP/downlink tasks; runs until stop()."""
        self.aec.reset()
        self.vad.reset()
        await self.ble.start(self._on_uplink)
        dsp = asyncio.create_task(self._dsp_loop())
        dl = asyncio.create_task(self._downlink_loop())
        stat = asyncio.create_task(self._status_loop())
        try:
            await self._stop.wait()
        finally:
            dsp.cancel()
            dl.cancel()
            stat.cancel()
            await asyncio.gather(dsp, dl, stat, return_exceptions=True)

    def stop(self):
        self._stop.set()
