/*
 * nrf54_button -- BLE button test for the XIAO nRF54LM20A Sense, built on the
 * resilience module so it also demonstrates the production boot-survival rules.
 *
 * BOOT ORDER IS THE POINT (see resilience.h for why):
 *   1. sample the button          -- the user's recovery gesture
 *   2. resilience_boot()          -- reset cause + boot-loop check
 *   3. radio                      -- comms come up BEFORE anything optional, so
 *                                    a broken peripheral can never cost us the
 *                                    one channel that can report the problem
 *   4. watchdog                   -- armed once we are reachable
 *   5. optional peripherals       -- guarded, skipped entirely in safe mode
 *
 * USER-FACING RECOVERY (no technical knowledge, nothing to return):
 *   * Plug into a USB-C charger or computer. External power bypasses a failing
 *     battery, which is the single most common cause of a device that "won't
 *     turn on".
 *   * Hold the button while plugging in  -> forces SAFE MODE. The device
 *     advertises as "gband-SAFE" so the phone app can spot it and reflash.
 *   * Hold the button for 10s at any time -> clean restart.
 *   * If the firmware itself hangs, the watchdog reboots it after 8s and the
 *     next boot logs "reset=WATCHDOG" instead of leaving a silent brick.
 *
 * WIRING: LilyPad button between P1.00 and GND. Internal pull-up, active low.
 *
 * nRF CONNECT: connect, enable notifications on the Button characteristic
 * 00001524-1212-EFDE-1523-785FEABCD123. Press -> 0x01, release -> 0x00.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/lbs.h>

#include "resilience.h"
#include "battery.h"
#include "led_status.h"

LOG_MODULE_REGISTER(btn_test, LOG_LEVEL_INF);

/* [UNIT] 20ms sample, 2 agreeing samples -> ~40ms debounce. */
#define POLL_MS        20
#define DEBOUNCE_N     2
/* [USER] Long-press to restart. 10s is long enough that it cannot be hit by
 * accident in a pocket, short enough that a user will hold it that long when
 * told to. Matches the convention people already know from phones. */
#define LONG_PRESS_MS  10000

/* Node label, not the sw0 alias: sw0 is the board's ONBOARD button (P0.09)
 * and shadowing it would break anything expecting that one. */
static const struct gpio_dt_spec button =
	GPIO_DT_SPEC_GET(DT_NODELABEL(userbtn), gpios);
static const struct device *const console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static atomic_t btn_state;

/* ---- advertising ---- */
static uint8_t adv_name[16];
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, 0),
};
/* Service UUID goes in the SCAN RESPONSE: a 128-bit UUID plus the name will not
 * both fit in the 31-byte advertising PDU. */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};
/* ---- gestureband status characteristic ----------------------------------
 * One read+notify characteristic carrying a plain ASCII line. nRF Connect
 * renders it as text, so a non-technical user can read battery %, charge state
 * and the last reset cause without a serial cable or any tooling.
 */
#define GB_STATUS_SVC_VAL \
	BT_UUID_128_ENCODE(0x8e7a0001, 0x1f2b, 0x4c3d, 0x9a5e, 0x6b7c8d9e0f10)
#define GB_STATUS_CHR_VAL \
	BT_UUID_128_ENCODE(0x8e7a0002, 0x1f2b, 0x4c3d, 0x9a5e, 0x6b7c8d9e0f10)
static const struct bt_uuid_128 gb_svc = BT_UUID_INIT_128(GB_STATUS_SVC_VAL);
static const struct bt_uuid_128 gb_chr = BT_UUID_INIT_128(GB_STATUS_CHR_VAL);

static char status_line[64] = "starting";
static bool status_subscribed;

static ssize_t status_read(struct bt_conn *c, const struct bt_gatt_attr *a,
			   void *buf, uint16_t len, uint16_t off)
{
	return bt_gatt_attr_read(c, a, buf, len, off, status_line,
				 strlen(status_line));
}
static void status_ccc(const struct bt_gatt_attr *a, uint16_t value)
{
	ARG_UNUSED(a);
	status_subscribed = (value == BT_GATT_CCC_NOTIFY);
}
BT_GATT_SERVICE_DEFINE(gb_status_svc,
	BT_GATT_PRIMARY_SERVICE(&gb_svc),
	BT_GATT_CHARACTERISTIC(&gb_chr.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, status_read, NULL, NULL),
	BT_GATT_CCC(status_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void status_publish(void)
{
	if (status_subscribed) {
		bt_gatt_notify(NULL, &gb_status_svc.attrs[2], status_line,
			       strlen(status_line));
	}
}

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, \
	BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

static void start_adv(void)
{
	int err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("advertising start failed: %d", err);
	} else {
		LOG_INF("advertising as \"%s\"", adv_name);
	}
}
static void adv_work_handler(struct k_work *w) { ARG_UNUSED(w); start_adv(); }
static K_WORK_DEFINE(adv_work, adv_work_handler);

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);
	if (err) { LOG_ERR("connection failed: 0x%02x", err); }
	else     { LOG_INF("connected"); }
}
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	/* Deferred: starting advertising from inside the disconnect callback races
	 * the stack's own teardown. */
	LOG_INF("disconnected (0x%02x) -> re-advertise", reason);
	k_work_submit(&adv_work);
}
BT_CONN_CB_DEFINE(app_conn_cb) = {
	.connected = on_connected, .disconnected = on_disconnected,
};

/* ---- console (output only on this board: the CMSIS-DAP bridge does not carry
 * host->device RX, so these are for a future native-USB or UART build) ---- */
static void uart_rx_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t ch;
	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) { return; }
	while (uart_fifo_read(dev, &ch, 1) == 1) {
		switch (ch) {
		case 'r': LOG_INF("reboot (cold)"); sys_reboot(SYS_REBOOT_COLD); break;
		case 'b': LOG_INF("no UF2 on nRF54L -- use ./flash_nrf54.sh"); break;
		case 's': LOG_INF("button: %s",
				  atomic_get(&btn_state) ? "PRESSED" : "released"); break;
		default: break;
		}
	}
}

int main(void)
{
	if (device_is_ready(console_dev)) {
		uart_irq_callback_user_data_set(console_dev, uart_rx_cb, NULL);
		uart_irq_rx_enable(console_dev);
	}
	LOG_INF("=== gestureband nRF54LM20A Sense -- button/BLE ===");

	/* 1. Sample the button before anything else: held at boot = recovery. */
	bool held = false;
	bool btn_ok = gpio_is_ready_dt(&button) &&
		      gpio_pin_configure_dt(&button, GPIO_INPUT) == 0;
	if (btn_ok) {
		k_sleep(K_MSEC(30));            /* let the pull-up settle */
		held = gpio_pin_get_dt(&button) > 0;
	} else {
		LOG_ERR("button GPIO not ready -- check the P1.00 wiring");
	}

	/* 2. Reset cause + boot-loop check. */
	const struct resilience_boot_info *bi = resilience_boot(held);

	/* 3. Radio FIRST -- before any optional hardware. */
	const char *name = bi->safe_mode ? "gband-SAFE" : CONFIG_BT_DEVICE_NAME;
	strncpy((char *)adv_name, name, sizeof(adv_name) - 1);
	ad[1].data_len = strlen((char *)adv_name);

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d -- rebooting to retry", err);
		k_sleep(K_SECONDS(2));
		sys_reboot(SYS_REBOOT_COLD);
	}
	bt_set_name(name);
	if (bt_lbs_init(NULL)) {
		LOG_ERR("LBS init failed");
	}
	start_adv();
	LOG_INF("reachable: reset=%s boots=%u%s", bi->cause_str, bi->boot_count,
		bi->safe_mode ? " [SAFE MODE]" : "");

	/* 4. Watchdog, armed now that we are reachable. */
	resilience_watchdog_start();

	/* 5. Optional peripherals would go here, each guarded, and skipped in
	 *    safe mode. This app has none -- the button is a bare SoC GPIO. */
	if (bi->safe_mode) {
		LOG_WRN("safe mode: optional hardware skipped; reflash or power-cycle");
	}

	/* 5. Optional peripherals, guarded. The LED comes up even in safe mode --
	 *    it is a bare GPIO with nothing to fail, and it is the ONLY way the
	 *    user learns the device is in recovery. The PMIC is I2C and therefore
	 *    skippable, which matters: an I2C peripheral is what took the device
	 *    down in the first place. */
	led_status_init();
	bool batt_ok = false;
	if (!bi->safe_mode) {
		batt_ok = (battery_init() == 0);
	}

	int last_raw = btn_ok ? gpio_pin_get_dt(&button) : 0;
	int stable = last_raw, agree = 0;
	int64_t press_started = 0;
	atomic_set(&btn_state, stable);

	struct battery_status bs = {};
	int ticks = 0;

	while (true) {
		resilience_watchdog_feed();

		/* [UNIT] Battery every 5s: the PMIC is slow-changing and each read is
		 * an I2C transaction we would rather not do at the button poll rate. */
		if (ticks % (5000 / POLL_MS) == 0) {
			if (batt_ok) {
				battery_read(&bs);
			}
			if (bs.valid) {
				bt_bas_set_battery_level(bs.percent);
				snprintk(status_line, sizeof(status_line),
					 "%u%% %u.%02uV %s | reset=%s boots=%u%s",
					 bs.percent, bs.millivolts / 1000,
					 (bs.millivolts % 1000) / 10,
					 battery_state_str(bs.state), bi->cause_str,
					 bi->boot_count, bi->safe_mode ? " SAFE" : "");
			} else {
				snprintk(status_line, sizeof(status_line),
					 "battery n/a | reset=%s boots=%u%s",
					 bi->cause_str, bi->boot_count,
					 bi->safe_mode ? " SAFE" : "");
			}
			/* Log only on change: the same line every 5s would bury everything
			 * else, but a silent battery path is impossible to verify. */
			static char last_line[64];
			if (strcmp(last_line, status_line) != 0) {
				strncpy(last_line, status_line, sizeof(last_line) - 1);
				LOG_INF("status: %s", status_line);
			}
			status_publish();

			/* LED: safe mode outranks everything -- if the device needs
			 * recovery that is the one thing the user must notice. */
			if (bi->safe_mode)                        led_status_set(LED_SAFE_MODE);
			else if (bs.state == BATTERY_CHARGING)    led_status_set(LED_CHARGING);
			else if (bs.state == BATTERY_CHARGED)     led_status_set(LED_CHARGED);
			else if (bs.state == BATTERY_NONE)        led_status_set(LED_LOW_BATTERY);
			else if (bs.valid && bs.percent < 15)     led_status_set(LED_LOW_BATTERY);
			else                                      led_status_set(LED_OFF);
		}
		ticks++;

		if (btn_ok) {
			int raw = gpio_pin_get_dt(&button);
			if (raw >= 0) {
				agree = (raw == last_raw) ? agree + 1 : 0;
				last_raw = raw;

				if (agree >= DEBOUNCE_N && raw != stable) {
					stable = raw;
					atomic_set(&btn_state, stable);
					press_started = stable ? k_uptime_get() : 0;
					LOG_INF("[BTN] %s", stable ? "PRESSED" : "released");
					int rc = bt_lbs_send_button_state(stable != 0);
					if (rc && rc != -ENOTCONN) {
						LOG_WRN("notify failed: %d", rc);
					}
				}
				/* Long-press restart: the user-facing "turn it off and on
				 * again" that needs no cable and no instructions. */
				if (stable && press_started &&
				    (k_uptime_get() - press_started) >= LONG_PRESS_MS) {
					LOG_WRN("long press -> restarting");
					k_sleep(K_MSEC(50));
					sys_reboot(SYS_REBOOT_COLD);
				}
			}
		}
		k_sleep(K_MSEC(POLL_MS));
	}
	return 0;
}
