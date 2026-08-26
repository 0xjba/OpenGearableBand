/*
 * display_oled -- see display_oled.h.
 *
 * Uses the CFB (Character Frame Buffer) subsystem. Text is auto-sized to fill
 * the panel: a single line picks the largest built-in font that still fits the
 * string width and is centered vertically; two lines use a font that fits both
 * halves. This avoids leaving the bottom of the panel dark on short messages.
 */
#include "display_oled.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(display_oled, LOG_LEVEL_INF);

static const struct device *disp_dev;

/* Built-in fonts, sorted by height ascending (populated at init). */
struct font_info { uint8_t idx, w, h; };
static struct font_info fonts[8];
static int num_fonts;
static uint16_t panel_w, panel_h;

int display_oled_init(void)
{
	disp_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(disp_dev)) {
		LOG_ERR("display device not ready (wiring? I2C addr? overlay?)");
		disp_dev = NULL;
		return -ENODEV;
	}

	display_blanking_off(disp_dev);

	int rc = cfb_framebuffer_init(disp_dev);
	if (rc) {
		LOG_ERR("cfb_framebuffer_init failed: %d", rc);
		disp_dev = NULL;
		return rc;
	}

	/* Draw dark background + lit text (only text pixels draw power). CFB
	 * compensates for the panel's inverse-display command, so the visual flip
	 * must be done here via the framebuffer 'inverted' flag (persists until
	 * the next cfb_framebuffer_init). */
	cfb_framebuffer_invert(disp_dev);

	panel_w = cfb_get_display_parameter(disp_dev, CFB_DISPLAY_WIDTH);
	panel_h = cfb_get_display_parameter(disp_dev, CFB_DISPLAY_HEIGHT);

	/* Catalogue the built-in fonts, sorted by height (simple insertion). */
	int n = cfb_get_numof_fonts(disp_dev);
	num_fonts = 0;
	for (int i = 0; i < n && num_fonts < (int)ARRAY_SIZE(fonts); i++) {
		uint8_t w = 0, h = 0;
		if (cfb_get_font_size(disp_dev, i, &w, &h) != 0 || w == 0 || h == 0) {
			continue;
		}
		int p = num_fonts++;
		while (p > 0 && fonts[p - 1].h > h) {
			fonts[p] = fonts[p - 1];
			p--;
		}
		fonts[p] = (struct font_info){ (uint8_t)i, w, h };
	}

	cfb_framebuffer_clear(disp_dev, true);
	LOG_INF("OLED ready (%u x %u px, %d fonts)", panel_w, panel_h, num_fonts);
	return 0;
}

/* Largest font (by height, <= maxh) whose rendered width fits `len` chars.
 * Falls back to the smallest font if none fit. Returns the font_info. */
static struct font_info pick_font(size_t len, uint16_t maxh)
{
	for (int i = num_fonts - 1; i >= 0; i--) {
		if (fonts[i].h <= maxh && (len * fonts[i].w) <= panel_w) {
			return fonts[i];
		}
	}
	return fonts[0];   /* smallest available */
}

int display_oled_show(const char *line1, const char *line2)
{
	if (disp_dev == NULL) {
		return -ENODEV;
	}
	if (num_fonts == 0) {
		return -ENOTSUP;
	}

	cfb_framebuffer_clear(disp_dev, false);

	bool have2 = (line2 != NULL && line2[0] != '\0');
	bool have1 = (line1 != NULL && line1[0] != '\0');

	if (have2) {
		/* Two lines: largest font fitting both halves, stacked to fill. */
		size_t longest = strlen(line1) > strlen(line2) ? strlen(line1)
							       : strlen(line2);
		struct font_info f = pick_font(longest, panel_h / 2);
		cfb_framebuffer_set_font(disp_dev, f.idx);
		cfb_print(disp_dev, line1, 0, 0);
		cfb_print(disp_dev, line2, 0, f.h);
	} else if (have1) {
		size_t len = strlen(line1);
		int max_chars = panel_w / fonts[0].w;   /* chars/line at smallest font */

		if ((int)len <= max_chars) {
			/* Fits on one line: biggest font that fits, centered. */
			struct font_info f = pick_font(len, panel_h);
			cfb_framebuffer_set_font(disp_dev, f.idx);
			int16_t y = (panel_h > f.h) ? (int16_t)((panel_h - f.h) / 2) : 0;
			cfb_print(disp_dev, line1, 0, y);
		} else {
			/* Too long for one line: word-wrap onto two lines (small font).
			 * Break at the last space <= max_chars, else hard-split. Anything
			 * past 2 lines is clipped (128x32 holds ~24 chars; scrolling is a
			 * future item). */
			int split = max_chars;
			for (int i = max_chars; i > 0; i--) {
				if (line1[i] == ' ') { split = i; break; }
			}
			char a[24], b[24];
			int alen = split < (int)sizeof(a) - 1 ? split : (int)sizeof(a) - 1;
			memcpy(a, line1, alen);
			a[alen] = '\0';
			const char *rest = line1 + split + (line1[split] == ' ' ? 1 : 0);
			size_t blen = strlen(rest);
			if (blen > sizeof(b) - 1) {
				blen = sizeof(b) - 1;
			}
			memcpy(b, rest, blen);
			b[blen] = '\0';

			cfb_framebuffer_set_font(disp_dev, fonts[0].idx);
			cfb_print(disp_dev, a, 0, 0);
			cfb_print(disp_dev, b, 0, fonts[0].h);
		}
	}

	return cfb_framebuffer_finalize(disp_dev);
}

void display_oled_estate_test(void)
{
	if (disp_dev == NULL) {
		return;
	}

	cfb_framebuffer_clear(disp_dev, false);

	/* Full-panel border: outlines the exact usable pixel area. */
	struct cfb_position tl = { 0, 0 };
	struct cfb_position br = { (uint16_t)(panel_w - 1), (uint16_t)(panel_h - 1) };
	cfb_draw_rect(disp_dev, &tl, &br);

	/* Corner-to-corner X: both diagonals reach all four corners/edges. */
	struct cfb_position a0 = { 0, 0 };
	struct cfb_position a1 = { (uint16_t)(panel_w - 1), (uint16_t)(panel_h - 1) };
	cfb_draw_line(disp_dev, &a0, &a1);
	struct cfb_position b0 = { (uint16_t)(panel_w - 1), 0 };
	struct cfb_position b1 = { 0, (uint16_t)(panel_h - 1) };
	cfb_draw_line(disp_dev, &b0, &b1);

	cfb_framebuffer_finalize(disp_dev);

	/* Report the capacity numbers to the serial log. */
	LOG_INF("=== screen estate: %u x %u px ===", panel_w, panel_h);
	for (int i = 0; i < num_fonts; i++) {
		LOG_INF("  font[%u] %ux%u  ->  %u chars x %u lines",
			fonts[i].idx, fonts[i].w, fonts[i].h,
			panel_w / fonts[i].w, panel_h / fonts[i].h);
	}
}
