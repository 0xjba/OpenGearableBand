/*
 * M3c.2 stub for audio_downlink: ble_audio's downlink characteristic forwards to
 * audio_downlink_feed()/flush(). The OLED variant has NO speaker/downlink (M4
 * removes it), so these are no-ops -- the uplink path is what M3c.2 exercises.
 */
#include "audio_downlink.h"

void audio_downlink_feed(const uint8_t *lc3, size_t len) { (void)lc3; (void)len; }
void audio_downlink_flush(void) {}
