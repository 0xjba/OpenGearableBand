/*
 * lc3_codec -- LC3 audio encoder wrapper.
 *
 * Ported from oneDiary (src/lc3_codec.c), verbatim logic. Uses Google's liblc3
 * (included in nRF Connect SDK as a Zephyr module).
 *
 * The mic gives us 320 samples (20 ms). We split into two 160-sample halves and
 * encode each as one LC3 frame. Output: 80 bytes total (vs 640 raw).
 */

#include "lc3_codec.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <lc3.h>

LOG_MODULE_REGISTER(lc3_codec, CONFIG_LOG_DEFAULT_LEVEL);

/* Static encoder memory -- no heap allocation needed */
static lc3_encoder_mem_16k_t encoder_mem;
static lc3_encoder_t encoder;
static bool initialized;

int lc3_codec_init(void)
{
	/*
	 * Setup LC3 encoder:
	 *   dt_us     = 10000 (10 ms frame)
	 *   sr_hz     = 16000 (16 kHz audio)
	 *   sr_pcm_hz = 0     (same as sr_hz, no resampling)
	 *   mem       = static encoder memory
	 */
	encoder = lc3_setup_encoder(LC3_FRAME_DURATION_US,
				    LC3_SAMPLE_RATE_HZ,
				    0, /* sr_pcm_hz: 0 = same as sr_hz */
				    &encoder_mem);

	if (encoder == NULL) {
		LOG_ERR("LC3 encoder setup failed");
		return -EINVAL;
	}

	/* Verify frame parameters match our expectations */
	int frame_samples = lc3_frame_samples(LC3_FRAME_DURATION_US,
					      LC3_SAMPLE_RATE_HZ);
	int frame_bytes = lc3_frame_bytes(LC3_FRAME_DURATION_US,
					  LC3_BITRATE_BPS);

	if (frame_samples != LC3_FRAME_SAMPLES) {
		LOG_ERR("LC3 frame_samples mismatch: got %d, expected %d",
			frame_samples, LC3_FRAME_SAMPLES);
		return -EINVAL;
	}

	LOG_INF("LC3 encoder: %d Hz, %d kbps, %d ms frames (%d samples -> %d bytes)",
		LC3_SAMPLE_RATE_HZ, LC3_BITRATE_BPS / 1000,
		LC3_FRAME_DURATION_US / 1000, LC3_FRAME_SAMPLES, frame_bytes);

	initialized = true;
	return 0;
}

int lc3_codec_encode_frame(const int16_t *pcm_in, uint8_t *out)
{
	if (!initialized) {
		return -EINVAL;
	}

	/*
	 * lc3_encode(encoder, fmt, pcm, stride=1 (mono), nbytes=40 (32 kbps), out)
	 * Returns 0 on success, -1 on error.
	 */
	int ret = lc3_encode(encoder, LC3_PCM_FORMAT_S16,
			     pcm_in, 1, LC3_FRAME_BYTES, out);

	if (ret != 0) {
		LOG_ERR("LC3 encode failed: %d", ret);
		return -EIO;
	}

	return 0;
}

int lc3_codec_encode_pcm_buffer(const int16_t *pcm_in,
				uint8_t *out, uint16_t *out_len)
{
	if (!initialized) {
		return -EINVAL;
	}

	int ret;

	/* Frame 1: samples [0..159] */
	ret = lc3_codec_encode_frame(pcm_in, out);
	if (ret) {
		return ret;
	}

	/* Frame 2: samples [160..319] */
	ret = lc3_codec_encode_frame(&pcm_in[LC3_FRAME_SAMPLES],
				     &out[LC3_FRAME_BYTES]);
	if (ret) {
		return ret;
	}

	*out_len = LC3_ENCODED_PDM_BUF_SIZE; /* 80 bytes */
	return 0;
}

bool lc3_codec_is_ready(void)
{
	return initialized;
}
