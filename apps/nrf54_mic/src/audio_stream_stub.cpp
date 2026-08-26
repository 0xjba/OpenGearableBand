/*
 * M3c.1 stub for audio_stream: mic_vad calls audio_stream_feed() per block, but
 * the LC3->BLE uplink isn't wired until M3c.2. These no-ops let mic_vad link and
 * run standalone so the PDM capture + VAD can be validated in isolation.
 */
#include "audio_stream.h"

int audio_stream_init(void) { return 0; }
bool audio_stream_active(void) { return false; }
void audio_stream_feed(const int16_t *pcm, size_t nsamp) { (void)pcm; (void)nsamp; }
