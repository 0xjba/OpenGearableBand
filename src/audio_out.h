/*
 * audio_out -- speaker output engine (downlink path). I2S TX -> MAX98357A.
 *
 * Producer/consumer, mirroring the uplink split (mic_vad/audio_stream/ble_audio).
 * Callers push mono 16-bit PCM via audio_out_write(); a dedicated feeder thread
 * drains it to I2S, duplicating mono to L/R for the stereo frame. The amp's SD
 * pin is held in shutdown except between start() and stop(). The future BLE
 * downlink (LC3 decode) feeds the same audio_out_write() seam -- no API change.
 */
#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time boot init: bind I2S + amp GPIO, set amp muted, ready the feeder.
 * Returns 0 on success, -ENODEV if a device isn't ready. */
int audio_out_init(void);

/* Begin a playback session at sample_rate (Hz): enables the amp, configures and
 * starts I2S. Returns 0, or -EBUSY if a session is already active. */
int audio_out_start(uint32_t sample_rate);

/* Enqueue mono 16-bit PCM. Non-blocking; drops (logged) on ring overflow; no-op
 * if no session is active. This is the seam the BLE downlink feeds. */
void audio_out_write(const int16_t *mono_pcm, size_t nsamp);

/* End the session: the feeder stops I2S and mutes the amp. */
void audio_out_stop(void);

/* Barge-in flush: clear the ring AND stop the session immediately (drops both
 * the buffered ring audio and the queued I2S blocks -> instant silence). Use
 * this for interruption; plain stop() leaves the ring to be reset on next start. */
void audio_out_flush(void);

/* Free space in the playback ring, in bytes. A producer uses this for
 * backpressure -- keep the ring FULL rather than feeding just-in-time, so the
 * feeder never underruns on scheduling jitter (the cause of playback cracking). */
size_t audio_out_ring_free(void);

/* Bytes currently buffered in the playback ring. A producer polls this to drain
 * the ring (let the buffered tail play out) before calling stop(). */
size_t audio_out_ring_used(void);

/* True while a playback session is active (between start and stop/auto-stop).
 * A producer polls this to stop feeding when the session ends. */
bool audio_out_active(void);

/* Playback volume, 0-100 % (clamped), applied to all output. Default 100. A real
 * product lever, and a stopgap for the dev-board 3V3 rail-sag at high volume
 * (loud peaks overdraw the shared rail until a bulk cap / boost is added). */
void audio_out_set_volume(uint8_t percent);
uint8_t audio_out_get_volume(void);

/* Total capacity of the playback ring, in bytes (the high-water for ring_used).
 * The downlink status reporter sends this so the host needs no hard-coded size. */
size_t audio_out_ring_capacity(void);

/* Buffer-event latch bits, set by audio_out and cleared on read. Lets the downlink
 * status reporter hand the host a hard over/underflow signal, not just inference. */
#define AUDIO_OUT_EV_OVERFLOW 0x01   /* a write was dropped (ring full) */
#define AUDIO_OUT_EV_UNDERRUN 0x02   /* the feeder pulled < a full block (ring low) */

/* Return the events latched since the last call, then clear them (read-and-clear). */
uint8_t audio_out_take_event_flags(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_OUT_H */
