/*
 * display_oled -- see display_oled.h.
 *
 * Uses the CFB (Character Frame Buffer) subsystem so text rendering is
 * font-based and panel-agnostic. The concrete panel comes from the
 * `zephyr,display` chosen node (SSD1306 in the board/app overlay).
 */
#include "display_oled.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(display_oled, LOG_LEVEL_INF);

static const struct device *disp_dev;

int display_oled_init(void)
{
	disp_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(disp_dev)) {
		LOG_ERR("display device not ready (wiring? I2C addr? overlay?)");
		disp_dev = NULL;
		return -ENODEV;
	}

	/* Panel starts blanked after init on some drivers; ensure it's on. */
	display_blanking_off(disp_dev);

	int rc = cfb_framebuffer_init(disp_dev);
	if (rc) {
		LOG_ERR("cfb_framebuffer_init failed: %d", rc);
		disp_dev = NULL;
		return rc;
	}

	/* Font index 0 is the smallest built-in font -- fine for a 128x32 panel.
	 * A larger font can be selected once we know how much text fits on HW. */
	cfb_framebuffer_set_font(disp_dev, 0);
	cfb_framebuffer_clear(disp_dev, true);
	LOG_INF("OLED ready (%u x %u px)",
		cfb_get_display_parameter(disp_dev, CFB_DISPLAY_WIDTH),
		cfb_get_display_parameter(disp_dev, CFB_DISPLAY_HEIGHT));
	return 0;
}

int display_oled_show(const char *line1, const char *line2)
{
	if (disp_dev == NULL) {
		return -ENODEV;
	}

	cfb_framebuffer_clear(disp_dev, false);

	/* Row height in pixels = font height. Line 2 sits one font-row below. */
	uint8_t fw = 0, fh = 0;
	cfb_get_font_size(disp_dev, 0, &fw, &fh);

	if (line1 != NULL && line1[0] != '\0') {
		cfb_print(disp_dev, line1, 0, 0);
	}
	if (line2 != NULL && line2[0] != '\0') {
		cfb_print(disp_dev, line2, 0, fh);
	}

	return cfb_framebuffer_finalize(disp_dev);
}
