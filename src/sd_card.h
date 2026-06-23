/*
 * sd_card -- microSD over SPI (FAT). PRIMARY role (future sub-project):
 * WRITING/recording audio to SD. The production listen path is BLE from the
 * phone, NOT SD. For now this module only mounts the FAT volume.
 */
#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Mount the FAT volume ("/SD:"). Lazy-safe (idempotent). 0 / negative errno. */
int sd_card_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
