#ifndef MIC_VAD_H
#define MIC_VAD_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A.0: PDM mic feasibility probe. mic_vad_init() at boot; start/stop toggled
 * from the serial console. While started, the capture thread logs per-block
 * RMS energy as [MIC]. (A.1 will add voice-onset + the pose-gated power enable.) */
void  mic_vad_init(void);
void  mic_vad_start(void);
void  mic_vad_stop(void);

/* Pure: RMS of a 16-bit PCM block. Exposed for host unit test. */
float mic_vad_block_rms(const int16_t *samples, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* MIC_VAD_H */
