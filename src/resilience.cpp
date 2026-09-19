/*
 * resilience -- see resilience.h.
 */
#include "resilience.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_reset.h>

LOG_MODULE_REGISTER(resilience, LOG_LEVEL_INF);

/* POLICY, not a verified hardware fact. There is no authority for "how many
 * failed boots means give up" -- this is a design choice, stated so it can be
 * argued with rather than mistaken for a spec.
 * Consecutive unhealthy boots before we stop trusting the full firmware.
 * 5 tolerates a user power-cycling a few times in frustration without demoting
 * a working device, but still trips within ~6s of a 1.09s reset loop. */
#define SAFE_MODE_THRESHOLD  5

/* POLICY, not a verified hardware fact.
 * Uptime that counts as "the firmware works". Must comfortably exceed
 * the time to bring up every optional peripheral, or a late-boot crash would
 * clear the counter and defeat the loop detector. */
#define RESILIENCE_HEALTHY_MS  30000

/* POLICY, not a verified hardware fact. The correct value is bounded below by
 * the longest legitimate blocking operation in the system -- measure that before
 * shortening it. 8s is long enough for a busy BLE + audio path and short enough
 * that a user has not given up on a hung device. */
#define WDT_TIMEOUT_MS  8000

#define NVS_KEY_BOOT_COUNT  1
#define NVS_KEY_SHUTDOWN    2   /* 1 = last power-off was a low-battery ship */

static struct nvs_fs fs;
static bool nvs_ok;
static struct resilience_boot_info info;
static int wdt_task_id = -1;

/* ---- reset cause ---------------------------------------------------------
 * RESETREAS is sticky: the bits accumulate until explicitly cleared, so we read
 * then clear, otherwise every later boot inherits this one's reason.
 * A value of ZERO is the important case -- no bit set means power-on reset or
 * BROWNOUT, i.e. the supply went away. That is precisely the reading we did not
 * have on 2026-09-05 and had to infer.
 */
static const char *decode_reset(uint32_t r)
{
	if (r == 0)                                      return "power-on/BROWNOUT";
	if (r & NRF_RESET_RESETREAS_RESETPIN_MASK)       return "pin reset";
	if (r & NRF_RESET_RESETREAS_DOG0_MASK)           return "WATCHDOG";
	if (r & NRF_RESET_RESETREAS_DOG1_MASK)           return "WATCHDOG";
	if (r & NRF_RESET_RESETREAS_SREQ_MASK)           return "software";
	if (r & NRF_RESET_RESETREAS_LOCKUP_MASK)         return "CPU LOCKUP";
	if (r & NRF_RESET_RESETREAS_OFF_MASK)            return "wake from OFF";
	return "other";
}

/* ---- persistent boot counter --------------------------------------------- */
static int nvs_setup(void)
{
	struct flash_pages_info pi;
	fs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(fs.flash_device)) {
		return -ENODEV;
	}
	fs.offset = FIXED_PARTITION_OFFSET(storage_partition);
	int rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &pi);
	if (rc) {
		return rc;
	}
	fs.sector_size  = pi.size;
	fs.sector_count = 3U;   /* 3 sectors keeps NVS wear-levelling happy and
				 * stays well inside the 32K storage partition. */
	return nvs_mount(&fs);
}

static uint16_t boot_count_bump(void)
{
	uint16_t n = 0;
	if (!nvs_ok) {
		return 0;   /* no persistence -> never demote to safe mode */
	}
	if (nvs_read(&fs, NVS_KEY_BOOT_COUNT, &n, sizeof(n)) != sizeof(n)) {
		n = 0;
	}
	n++;
	nvs_write(&fs, NVS_KEY_BOOT_COUNT, &n, sizeof(n));
	return n;
}

void resilience_mark_healthy(void)
{
	uint16_t zero = 0;
	if (nvs_ok) {
		nvs_write(&fs, NVS_KEY_BOOT_COUNT, &zero, sizeof(zero));
	}
	LOG_INF("boot marked healthy (counter cleared)");
}
static void healthy_work_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	resilience_mark_healthy();
}
static K_WORK_DELAYABLE_DEFINE(healthy_work, healthy_work_handler);

/* ---- public -------------------------------------------------------------- */
const struct resilience_boot_info *resilience_boot(bool button_held)
{
	uint32_t reas = nrf_reset_resetreas_get(NRF_RESET);
	nrf_reset_resetreas_clear(NRF_RESET, reas);

	nvs_ok = (nvs_setup() == 0);
	if (!nvs_ok) {
		LOG_WRN("NVS unavailable -- boot-loop detection disabled");
	}

	info.reset_cause = reas;
	info.cause_str   = decode_reset(reas);

	/* A ship-mode wake is a power-on reset; say so if we shipped on purpose. */
	uint8_t shut = 0;
	if (nvs_ok && nvs_read(&fs, NVS_KEY_SHUTDOWN, &shut, sizeof(shut)) == sizeof(shut)
	    && shut == 1) {
		if (reas == 0) {
			info.cause_str = "woke from LOW-BATTERY shutdown";
		}
		shut = 0;
		nvs_write(&fs, NVS_KEY_SHUTDOWN, &shut, sizeof(shut));
	}
	info.boot_count  = boot_count_bump();
	info.user_forced = button_held;
	info.safe_mode   = button_held || (info.boot_count >= SAFE_MODE_THRESHOLD);

	LOG_INF("boot: reset=%s (0x%08x)  consecutive=%u", info.cause_str,
		reas, info.boot_count);
	if (info.safe_mode) {
		LOG_WRN("SAFE MODE (%s) -- comms only, optional hardware skipped",
			button_held ? "button held at boot" : "boot loop detected");
		/* A user-forced entry is a deliberate recovery action, so clear the
		 * counter now: they should get a normal boot next time. */
		if (button_held) {
			resilience_mark_healthy();
		} else {
			/* Once this minimal image has stayed up for the healthy window,
			 * clear the counter so the NEXT boot retries the full firmware.
			 * Without this, automatic safe mode was PERMANENT: nothing else
			 * clears the counter, and a reflash does not erase NVS -- so a
			 * device that tripped it stayed there even with fixed firmware.
			 * Found on HW 2026-09-19: each probe-rs flash causes two watchdog
			 * resets while the core is halted, so two quick flashes reached
			 * the threshold. If the full firmware is still broken it simply
			 * returns here after SAFE_MODE_THRESHOLD more boots -- a bounded
			 * retry, not a trap. */
			k_work_schedule(&healthy_work, K_MSEC(RESILIENCE_HEALTHY_MS));
		}
	} else {
		k_work_schedule(&healthy_work, K_MSEC(RESILIENCE_HEALTHY_MS));
	}
	return &info;
}

bool resilience_in_safe_mode(void) { return info.safe_mode; }

void resilience_note_low_battery_shutdown(void)
{
	uint8_t one = 1;
	if (nvs_ok) {
		nvs_write(&fs, NVS_KEY_SHUTDOWN, &one, sizeof(one));
	}
}

static void wdt_expired(int channel, void *user)
{
	ARG_UNUSED(channel); ARG_UNUSED(user);
	/* Nothing clever here: the point is that we reboot rather than hang, and
	 * that RESETREAS records DOG so the next boot can say why. */
}

int resilience_watchdog_start(void)
{
	const struct device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	if (!device_is_ready(wdt)) {
		LOG_ERR("watchdog device not ready -- hangs will NOT self-recover");
		return -ENODEV;
	}
	int rc = task_wdt_init(wdt);
	if (rc && rc != -EALREADY) {
		LOG_ERR("task_wdt_init failed: %d", rc);
		return rc;
	}
	wdt_task_id = task_wdt_add(WDT_TIMEOUT_MS, wdt_expired, NULL);
	if (wdt_task_id < 0) {
		LOG_ERR("task_wdt_add failed: %d", wdt_task_id);
		return wdt_task_id;
	}
	LOG_INF("watchdog armed (%d ms)", WDT_TIMEOUT_MS);
	return 0;
}

void resilience_watchdog_feed(void)
{
	if (wdt_task_id >= 0) {
		task_wdt_feed(wdt_task_id);
	}
}
