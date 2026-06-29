# Robust barge-in rebuild — implementation plan (2026-06-30)

**Goal:** model-agnostic, ambient-robust, hardware-improvement-friendly barge-in that fixes both
failure modes (false self-barge + AI-won't-stop), removing accreted band-aids as each new layer lands.

**Architecture:** our orchestrator is the single turn/barge AUTHORITY (model-agnostic); backends are
thin adapters behind a generic interface. Detector is layered: AEC → DTD (echo reject, uses known
reference) → neural VAD (ambient/speech reject). Backend server-side VAD is DISABLED; we drive turns.

**Driving constraints (user, 2026-06-30):** (1) robust from silence→urban-traffic (ambient-independent
→ neural VAD, not energy); (2) build reliable on TODAY's bad nonlinear HW so it's near-perfect when the
5V-rail/clean-speaker product HW lands (layered, degrades gracefully); (3) AI-model-agnostic (no
Gemini-specifics in core). Research basis: `docs/research/barge-in-aec-2026-research.md`.

## Cleanup contract (DON'T STACK — delete redundancy as replacements land)
| New layer | Deletes |
|---|---|
| Step 1 interface rails | (additive) |
| Step 1b | AEC-convergence gate experiment (`_aec_converged`/`_conv_blocks`/`AEC_CONV_*`/`conv=` status) — uncommitted, proven near-no-op |
| Step 2 neural VAD + manual activity | mute-zeros hack (`feed_mic(zeros)` during play); `end_turn()` auto-VAD reliance |
| Step 3 DTD | energy `BargeInVad` + `BARGE_ABS_FLOOR_DB` + `BARGE_ONSET_GRACE_S` |
| Step 4 (if baseline demands) | possibly DTLN → NKF-AEC / AEC3 cascade |

## Step 1 — model-agnostic control rails (no HW needed, unit-testable) — THIS CHANGE
- `VoiceBackend.barge_in(played_ms=None)`: contract = STOP now: cancel server generation, truncate
  context to `played_ms` of audio actually heard, and the CALLER flushes local playout. Each backend
  maps to its API (Gemini: activity/cancel + server auto-truncate; OpenAI: `response.cancel` +
  `conversation.item.truncate(audio_end_ms)` + `output_audio_buffer.clear`; custom: cancel LLM/TTS).
- Add `user_turn_start()` / `user_turn_end()` (no-op defaults) = the generic turn boundaries our
  detector will drive. `end_turn()` kept but marked deprecated (removed in Step 2).
- Update `EchoLoopbackBackend` + `GeminiLiveBackend` signatures; orchestrator passes `played_ms`.

## Step 1b — remove the failed convergence-gate experiment (cleanup) — THIS CHANGE
Revert `_aec_converged`/`_conv_blocks`/`AEC_CONV_RATIO/MIN_MIC/BLOCKS`, the detection block, the arm
condition's `and self._aec_converged`, the `conv=` status column, and the per-reply/​session resets.
Interim detector reverts to energy VAD + abs-floor + onset-grace (kept until Step 3's DTD).

## Step 2 — neural VAD authority + Gemini manual activity (needs HW verify)
Add Silero VAD (onnxruntime, open, CPU). Orchestrator detects user-turn start/end (ambient-robust) and
barge; drives `user_turn_start/end` + `barge_in`. Gemini adapter: disable `automaticActivityDetection`,
implement `activityStart/activityEnd`. DELETE mute-zeros + `end_turn` reliance. Verify: AI stops once,
no repeats; turn-taking still natural; no self-interrupt.

## Step 3 — DTD echo gate (needs HW verify)
Coherence/cross-correlation DTD on the known reference gates the barge decision (fire only if neural-VAD
speech AND not-echo). DELETE energy `BargeInVad` + abs-floor + onset-grace. Verify false-barge rate.

## Step 4 — front-end upgrade ONLY IF baseline demands
Measure residual at onset/strong-coupling. If too high, evaluate NKF-AEC (open) or WebRTC AEC3→DTLN.

## TEST PROCEDURE — barge-in A/B harness (run on HW; data decides path + Silero + Step 4)
Prereqs: band flashed/connected (speaker+mic working); `.env` has GEMINI_API_KEY; a traffic/cafe
noise source (phone or laptop) for Leg 2; a quiet room for Legs 1 & 3. `tee` every run to a log.
Activate a session the normal way: raise-to-ear + speak so the band streams mic (the voice_loop
"force mic with serial 'j'" hint is STALE — j was removed 2026-06-24). Use the SAME interactions
across modes for a fair compare.

LEG 1 — Interruption A/B (the main decision: does the model STOP on a real barge?)
  For mode in {auto, manual}:
    run: tools/dtln-venv/bin/python tools/voice_loop.py --backend gemini --barge-mode <mode> 2>&1 | tee /tmp/barge_<mode>.log
    - trigger a long reply ("tell me a long story about the ocean")
    - talk OVER it for ~1-2s ("stop — wait"). Do it on the FIRST reply and 2-3 later replies.
    - read: [barge] TRIGGER -> [instr] interrupted -> [barge] OUTCOME = STOPPED (good) / STILL
      SENDING (bad) / FREEZE (#1228).
LEG 2 — Ambient robustness (decides Silero-even-in-auto):
  For mode in {auto, manual}:
    run + tee /tmp/ambient_<mode>.log; trigger a reply; then STAY SILENT and play traffic/cafe
    noise near the mic at realistic level for ~30-60s across a couple replies. Escalate level.
    - read [stat]: barge_ins (ours) and srv_int (Gemini VAD) must stay 0. Any rise = false barge
      from that source. (srv_int rising in auto = Gemini's VAD NOT ambient-robust -> Silero needed.)
LEG 3 — Baseline residual (decides Step-4 front-end upgrade) [passive, quiet room]:
    run; trigger replies; stay silent; read [stat] during play=1: mic_rms vs clean_rms. Big gap =
    good cancellation; clean_rms near mic_rms (esp. first second / onset) = high residual -> Step 4.
Then send all logs; analyze [barge] OUTCOME / [instr] / [stat] to pick: interruption path, Silero
yes/no, Step 4 yes/no.

## Open follow-ups
- Targeted research gap: exact Krisp/VoiceFilter status (Step-3 alternative) — low priority.
- Problem-2 exact events confirmed: Gemini server auto-truncates ("only info sent to client retained");
  client must flush local playout. OpenAI: response.cancel + conversation.item.truncate + output_audio_buffer.clear.
