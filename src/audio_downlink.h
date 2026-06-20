/*
 * audio_downlink -- BLE downlink bridge: raw LC3 from the phone -> decode -> audio_out.
 *
 * Mirror of audio_stream (uplink). The ble_audio downlink write callback hands LC3
 * payloads to audio_downlink_feed() (BLE RX context, must stay light); a dedicated
 * decode thread drains them, LC3-decodes, and calls audio_out_write(). The phone
 * does all heavy lifting; the device only does the <1 ms/frame decode + play.
 */
#ifndef AUDIO_DOWNLINK_H
#define AUDIO_DOWNLINK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot init (the decode thread auto-starts; LC3 decoder is inited by
 * lc3_codec_init via the uplink). Returns 0. */
int audio_downlink_init(void);

/* Feed one raw LC3 payload (1+ contiguous 40-byte frames) from the BLE write
 * callback. Non-blocking: copies into a queue, drops + counts on overflow. */
void audio_downlink_feed(const uint8_t *lc3, size_t len);

/* Barge-in: purge queued frames + flush audio_out (instant silence). */
void audio_downlink_flush(void);

/* Count of payloads dropped (queue full / oversized) since boot -- bring-up only. */
uint32_t audio_downlink_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_DOWNLINK_H */
