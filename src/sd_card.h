/*
 * sd_card -- microSD over SPI (FAT). PRIMARY role (future sub-project):
 * WRITING/recording audio to SD. The production listen path is BLE from the
 * phone, NOT SD -- so the only read path here is a TEST-ONLY WAV playback used
 * to exercise audio_out with real audio.
 */
#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Mount the FAT volume ("/SD:"). Lazy-safe (idempotent). 0 / negative errno. */
int sd_card_init(void);

/* TEST-ONLY: stream a 16-bit PCM WAV from SD through audio_out. Mono is passed
 * through; stereo is downmixed to mono. Blocks until the clip finishes.
 * Returns 0 / negative errno. */
int sd_card_play_wav_test(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
