/*
 * OLED variant has no speaker/downlink. gesture_mode calls audio_out_active()
 * (the "listening lean" that holds the ear session while the AI plays through
 * the speaker) -- always false here. ble_audio's downlink char forwards to
 * audio_downlink_feed/flush -- no-ops here. These stubs replace the entire
 * audio-downlink + playback stack for this variant.
 */
#include "audio_out.h"
#include "audio_downlink.h"

bool audio_out_active(void) { return false; }

void audio_downlink_feed(const uint8_t *lc3, size_t len) { (void)lc3; (void)len; }
void audio_downlink_flush(void) {}
