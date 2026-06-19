/*
 * sd_card -- microSD over SPI (FAT).
 *
 * PRIMARY future role: WRITING/recording audio to SD (a later sub-project).
 * The production listen path is BLE from the phone, NOT SD playback -- so the
 * only read path here is sd_card_play_wav_test(), a TEST-ONLY helper that
 * streams a 16-bit PCM WAV from the card through audio_out to exercise the
 * speaker with real audio.
 *
 * Mount point "/SD:" (FAT). Proven SD/SPI/FAT/WAV recipe borrowed from the
 * standalone spkr_test/ app, which runs on this exact hardware.
 */
#include "sd_card.h"
#include "audio_out.h"

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/util.h>   /* MIN */
#include <zephyr/logging/log.h>
#include <ff.h>
#include <string.h>

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

/* ---- WAV header parse (RIFF chunk walk; from spkr_test) ---- */
struct wav_info {
    uint32_t rate;
    uint16_t channels;
    uint16_t bits;
    uint32_t data_off;   /* byte offset of PCM start */
    uint32_t data_len;   /* PCM byte count */
};

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

/* Walk the RIFF chunks to find `fmt ` and `data`. 0 on success, negative on
 * malformed input. Leaves the file positioned at the start of the data chunk. */
static int parse_wav(struct fs_file_t *f, struct wav_info *w)
{
    uint8_t hdr[12];

    if (fs_read(f, hdr, 12) != 12) {
        return -1;
    }
    if (memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
        return -2;
    }

    bool have_fmt = false;

    while (1) {
        uint8_t ch[8];

        if (fs_read(f, ch, 8) != 8) {
            return -3;
        }
        uint32_t sz = rd32(ch + 4);

        if (!memcmp(ch, "fmt ", 4)) {
            uint8_t fmt[16];

            if (fs_read(f, fmt, 16) != 16) {
                return -4;
            }
            w->channels = rd16(fmt + 2);
            w->rate     = rd32(fmt + 4);
            w->bits     = rd16(fmt + 14);
            have_fmt = true;
            if (sz > 16) {
                fs_seek(f, sz - 16, FS_SEEK_CUR);
            }
        } else if (!memcmp(ch, "data", 4)) {
            w->data_len = sz;
            w->data_off = (uint32_t)fs_tell(f);
            return have_fmt ? 0 : -5;
        } else {
            fs_seek(f, sz + (sz & 1), FS_SEEK_CUR);   /* skip, word-align */
        }
    }
}

/* ---- TEST-ONLY playback ---- */
/* Frames per read chunk. Kept static (not on the stack) -- the console thread
 * that calls this also runs the FAT stack, so we keep its frame budget tight. */
#define PLAY_FRAMES 512

static int16_t filebuf[PLAY_FRAMES * 2];   /* stereo worst case */
static int16_t monobuf[PLAY_FRAMES];       /* downmix target */

int sd_card_play_wav_test(const char *path)
{
    int rc = ensure_mounted();
    if (rc) {
        return rc;
    }

    struct fs_file_t f;
    fs_file_t_init(&f);
    rc = fs_open(&f, path, FS_O_READ);
    if (rc) {
        LOG_ERR("open %s failed: %d", path, rc);
        return rc;
    }

    struct wav_info w = {};
    rc = parse_wav(&f, &w);
    if (rc) {
        LOG_ERR("WAV parse failed: %d", rc);
        fs_close(&f);
        return rc;
    }
    LOG_INF("WAV: %u Hz, %u ch, %u-bit, %u KB PCM",
            w.rate, w.channels, w.bits, w.data_len / 1024);

    if (w.bits != 16 || w.channels < 1 || w.channels > 2) {
        LOG_ERR("need a 16-bit mono/stereo WAV (got %u-bit, %u ch)",
                w.bits, w.channels);
        fs_close(&f);
        return -EINVAL;
    }

    rc = audio_out_start(w.rate);
    if (rc) {
        LOG_ERR("audio_out_start(%u) failed: %d", w.rate, rc);
        fs_close(&f);
        return rc;
    }

    const uint32_t bytes_per_frame = w.channels * 2U;   /* 16-bit */
    uint32_t consumed = 0;                              /* bytes of data chunk */

    while (consumed < w.data_len) {
        uint32_t remain = w.data_len - consumed;
        uint32_t want = MIN((uint32_t)(PLAY_FRAMES * bytes_per_frame), remain);

        int r = fs_read(&f, filebuf, want);
        if (r <= 0) {
            LOG_WRN("fs_read returned %d -- stopping", r);
            break;
        }
        consumed += r;

        uint32_t frames = (uint32_t)r / bytes_per_frame;
        if (frames == 0) {
            break;
        }

        const int16_t *mono;
        if (w.channels == 1) {
            mono = filebuf;
        } else {
            /* Downmix L/R -> mono (average) to feed the mono audio_out seam. */
            for (uint32_t i = 0; i < frames; i++) {
                int32_t l = filebuf[2 * i];
                int32_t r2 = filebuf[2 * i + 1];
                monobuf[i] = (int16_t)((l + r2) / 2);
            }
            mono = monobuf;
        }

        /* Backpressure: keep the audio_out ring FULL (not just-in-time) so the
         * feeder never underruns on producer-scheduling jitter -- that underrun
         * was the playback cracking. Wait for room, then write; bail if the
         * session ended (e.g. an I2S error auto-stopped it). */
        while (audio_out_ring_free() < frames * 2U && audio_out_active()) {
            k_msleep(2);
        }
        if (!audio_out_active()) {
            break;
        }
        audio_out_write(mono, frames);
    }

    /* Backpressure leaves the ring full at end-of-file; drain it (let the feeder
     * play the buffered tail) before stopping, so stop()'s DROP doesn't truncate
     * the end. */
    while (audio_out_active() && audio_out_ring_used() > PLAY_FRAMES * 2U) {
        k_msleep(5);
    }
    k_msleep(60);   /* flush the last queued I2S blocks */
    audio_out_stop();
    fs_close(&f);
    LOG_INF("WAV playback done");
    return 0;
}
