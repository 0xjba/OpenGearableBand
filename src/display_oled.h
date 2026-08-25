/*
 * display_oled -- text output on the 0.91" SSD1306 OLED (the OLED variant's
 * replacement for the speaker). Thin wrapper over Zephyr's Character Frame
 * Buffer (CFB) on the display chosen by `zephyr,display`.
 *
 * Board: XIAO nRF54LM20A Sense, OLED on the header I2C bus (see
 * docs/nrf54lm20a-board.md). Isolated module -- no dependency into the FSM.
 */
#ifndef DISPLAY_OLED_H
#define DISPLAY_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the OLED + CFB. Returns 0 on success, negative errno on failure
 * (device not ready, CFB init failed). Safe to call once at boot. */
int display_oled_init(void);

/* Clear the panel and print up to two text lines. Either may be NULL/empty.
 * Lines beyond the panel height are clipped by CFB. Returns 0 on success. */
int display_oled_show(const char *line1, const char *line2);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_OLED_H */
