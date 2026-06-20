/*
 * lc3_codec -- LC3 audio encoder wrapper (ported from oneDiary, verbatim logic).
 *
 * Wraps Google's liblc3 (ships in NCS as a Zephyr module, CONFIG_LIBLC3=y) for
 * on-device encoding of the dictation mic audio.
 *
 * Encoding parameters (fixed; chosen to match our 16 kHz PDM capture):
 *   - 16 kHz sample rate
 *   - 10 ms frame  -> 160 samples/frame
 *   - 32 kbps      -> 40 bytes/encoded frame
 *   - Our mic block is 320 samples (20 ms) = exactly two LC3 frames -> 80 bytes.
 *
 * Memory: static lc3_encoder_mem_16k_t (~2 KB), no heap.
 * FPU: required (CONFIG_FPU + CONFIG_FPU_SHARING -- the encode runs on the mic
 * capture thread alongside the CMSIS FFT VAD, so FPU context must be saved
 * across context switches).
 *
 * extern "C": added for this project (gestureband is C++); the original oneDiary
 * file was pure C and had no guards.
 */

#ifndef LC3_CODEC_H
#define LC3_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LC3 encoding parameters */
#define LC3_FRAME_DURATION_US    10000    /* 10 ms */
#define LC3_SAMPLE_RATE_HZ       16000
#define LC3_BITRATE_BPS          32000
#define LC3_FRAME_SAMPLES        160      /* 10 ms @ 16 kHz */
#define LC3_FRAME_BYTES          40       /* 32 kbps * 10 ms / 8 = 40 bytes */

/* Two LC3 frames per 20 ms mic block (320 samples) */
#define LC3_FRAMES_PER_PDM_BUF   2
#define LC3_ENCODED_PDM_BUF_SIZE (LC3_FRAME_BYTES * LC3_FRAMES_PER_PDM_BUF) /* 80 */

/* Initialise the LC3 encoder (static memory). Call once at boot before any
 * encode. Returns 0 on success, negative errno on failure. */
int lc3_codec_init(void);

/* Encode one 10 ms frame: 160 int16 PCM in -> 40-byte LC3 out.
 * Returns 0 on success, negative errno on failure. */
int lc3_codec_encode_frame(const int16_t *pcm_in, uint8_t *out);

/* Encode a full 20 ms block: 320 int16 PCM in -> 80 bytes out (2 frames).
 * out must be >= LC3_ENCODED_PDM_BUF_SIZE. *out_len is set to 80 on success.
 * Returns 0 on success, negative errno on failure. */
int lc3_codec_encode_pcm_buffer(const int16_t *pcm_in,
				uint8_t *out, uint16_t *out_len);

/* Decode one 10 ms frame: 40-byte LC3 in -> 160 int16 PCM out (16 kHz).
 * Returns 0 on success (incl. liblc3 PLC), negative errno on failure. */
int lc3_codec_decode_frame(const uint8_t *in, int16_t *pcm_out);

/* True once lc3_codec_init() has succeeded. */
bool lc3_codec_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* LC3_CODEC_H */
