/*
 * led_status -- onboard RGB LED as the device's only zero-knowledge UI.
 *
 * The user cannot read a serial log. The LED is what tells them whether the
 * device is charging, charged, flat, or in recovery -- which is the difference
 * between "it's charging, leave it" and "send it back".
 *
 * Pins come from the board dtsi: R P1.22, G P1.24, B P1.23, all ACTIVE_LOW.
 * Driven as plain GPIO (not PWM) -- this is indication, not animation.
 */
#ifndef LED_STATUS_H_
#define LED_STATUS_H_

#ifdef __cplusplus
extern "C" {
#endif

enum led_mode {
	LED_OFF = 0,
	LED_CHARGING,     /* red, slow breath-style blink  */
	LED_CHARGED,      /* green, solid                  */
	LED_LOW_BATTERY,  /* red, one blink every 4s       */
	LED_SAFE_MODE,    /* red, fast urgent blink        */
};

/* Non-fatal: if the LEDs are missing the device still runs, silently. */
int  led_status_init(void);
void led_status_set(enum led_mode mode);

#ifdef __cplusplus
}
#endif
#endif /* LED_STATUS_H_ */
