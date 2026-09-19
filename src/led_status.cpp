/*
 * led_status -- see led_status.h.
 */
#include "led_status.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_status, LOG_LEVEL_INF);

static const struct gpio_dt_spec red   = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led),   gpios);
static const struct gpio_dt_spec green = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led), gpios);
static bool leds_ok;
static enum led_mode cur_mode;
static int phase;

/* POLICY, not verified hardware facts -- chosen for legibility, not measured.
 * Blink cadences, ms. Distinct on purpose: a user must be able to tell
 * "charging" from "something is wrong" across a room without a manual. */
#define TICK_MS        250
#define SAFE_ON_TICKS    1   /* 250ms on, 250ms off -- urgent  */
#define CHG_PERIOD       8   /* 2s cycle                       */
#define LOW_PERIOD      16   /* 4s cycle, one short blink      */

static void apply(bool r, bool g)
{
	if (!leds_ok) return;
	gpio_pin_set_dt(&red,   r ? 1 : 0);
	gpio_pin_set_dt(&green, g ? 1 : 0);
}

static void tick(struct k_work *w);
static K_WORK_DELAYABLE_DEFINE(led_work, tick);

static void tick(struct k_work *w)
{
	ARG_UNUSED(w);
	phase++;
	switch (cur_mode) {
	case LED_OFF:         apply(false, false); break;
	case LED_CHARGED:     apply(false, true);  break;
	case LED_SAFE_MODE:   apply((phase % 2) < SAFE_ON_TICKS, false); break;
	case LED_CHARGING:    apply((phase % CHG_PERIOD) < CHG_PERIOD / 2, false); break;
	case LED_LOW_BATTERY: apply((phase % LOW_PERIOD) == 0, false); break;
	}
	k_work_schedule(&led_work, K_MSEC(TICK_MS));
}

int led_status_init(void)
{
	if (!gpio_is_ready_dt(&red) || !gpio_is_ready_dt(&green)) {
		LOG_WRN("onboard LEDs not available");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&red,   GPIO_OUTPUT_INACTIVE) ||
	    gpio_pin_configure_dt(&green, GPIO_OUTPUT_INACTIVE)) {
		LOG_WRN("LED configure failed");
		return -EIO;
	}
	leds_ok = true;
	k_work_schedule(&led_work, K_MSEC(TICK_MS));
	LOG_INF("status LED ready (R P1.22 / G P1.24, active low)");
	return 0;
}

void led_status_set(enum led_mode mode)
{
	if (mode != cur_mode) {
		cur_mode = mode;
		phase = 0;
	}
}
