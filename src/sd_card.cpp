/*
 * sd_card -- microSD over SPI (FAT).
 *
 * PRIMARY role (future sub-project): WRITING/recording audio to SD. The
 * production listen path is BLE from the phone, NOT SD playback. For now this
 * module only mounts the FAT volume so the recording path can be built on top.
 *
 * Mount point "/SD:" (FAT). SD/SPI/FAT recipe verified on this exact hardware.
 */
#include "sd_card.h"

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <ff.h>

LOG_MODULE_REGISTER(sd_card, LOG_LEVEL_INF);

/* ---- FAT mount ---- */
static FATFS fat_fs;
/* Field order matches the fs_mount_t declaration (C++ requires it). */
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data = &fat_fs,
};
static bool mounted;

/* Mount once; idempotent so callers don't have to track state. */
static int ensure_mounted(void)
{
    if (mounted) {
        return 0;
    }
    int rc = fs_mount(&mp);
    if (rc) {
        LOG_ERR("SD mount failed: %d (card inserted? FAT32? wiring?)", rc);
        return rc;
    }
    mounted = true;
    LOG_INF("SD mounted at %s", mp.mnt_point);
    return 0;
}

int sd_card_init(void)
{
    return ensure_mounted();
}
