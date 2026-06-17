#ifndef MIC_VAD_H
#define MIC_VAD_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PDM mic capture. mic_vad_init() at boot; start/stop toggled from the serial
 * console (A.0) or the POSE_EAR gate (A.1). While started, the capture thread
 * logs per-block RMS + voiced-band (300-3000 Hz) energy as [MIC]. (A.1 adds the
 * latched-floor voice-onset detector on top of this.) */
void  mic_vad_init(void);
/* Begin/end a listen session. start() is idempotent (no-op if already running),
 * so concurrent callers (POSE_EAR gate + bench 'm' probe) can't race the session
 * state. stop() ends the session; the next start() re-latches a fresh floor. */
void  mic_vad_start(void);
void  mic_vad_stop(void);

/* True once a voiced-onset has been detected since the last mic_vad_start().
 * Reading it CLEARS the latch (one-shot per session). */
bool mic_vad_voice_onset(void);

/* True while near-field voice is ongoing -- a hot block occurred within
 * VAD_VOICE_HOLD_MS. Non-clearing (reflects current state). Used to hold a
 * session through a lean (pose out of cone) while the user keeps speaking. */
bool mic_vad_voice_active(void);

/* Pure: RMS of a 16-bit PCM block. Exposed for host unit test. */
float mic_vad_block_rms(const int16_t *samples, size_t n);

/* Pure: sum of energy bins e[lo..hi] inclusive, indices clamped to [0,n).
 * Returns 0 if lo>hi or n==0. Exposed for host unit test. */
float mic_vad_band_sum(const float *e, size_t n, int lo, int hi);

#ifdef __cplusplus
}
#endif

#endif /* MIC_VAD_H */
