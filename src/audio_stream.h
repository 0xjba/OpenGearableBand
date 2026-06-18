/*
 * audio_stream -- bridges the mic capture to the BLE audio sink for dictation.
 *
 * Sits between mic_vad (PCM capture) and ble_audio (BLE notify). The mic capture
 * thread calls audio_stream_feed() once per 20 ms block; when streaming is active
 * this LC3-encodes the block and sends it. When not active it is a cheap no-op.
 *
 * Gate (audio_stream_active): MODE_DICTATION AND a host subscribed. The mic is
 * already turned on during MODE_DICTATION by the A.1 ear-gate, so PCM is already
 * flowing; this module only decides whether each block is also encoded + sent.
 *
 * Encoding lives HERE (not in mic_vad and not buried in ble_audio) so a future
 * SD sink can consume the same encoded block by adding one call in feed() --
 * no change to capture or codec. (SD is out of scope for B; this is just the
 * seam, not an abstraction layer.)
 */

#ifndef AUDIO_STREAM_H
#define AUDIO_STREAM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the audio stream path (inits the LC3 encoder). Call once at boot.
 * Returns 0 on success, negative errno on failure (codec init failed). */
int audio_stream_init(void);

/* True when blocks fed now would be encoded + streamed:
 * gesture_mode == MODE_DICTATION AND a host is subscribed. */
bool audio_stream_active(void);

/* Feed one captured PCM block from the mic thread. No-op unless active. Expects
 * a 320-sample (20 ms @ 16 kHz) block; other sizes are ignored (defensive). */
void audio_stream_feed(const int16_t *pcm, size_t nsamp);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_STREAM_H */
