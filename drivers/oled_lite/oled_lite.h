/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Thread context. Rendering changes the framebuffer until the next flush. */
int oled_lite_init(void);
void oled_lite_clear(void);
int oled_lite_pixel(unsigned int x, unsigned int y, bool on);
/* Draw opaque 6x8 cells (5x7 bitmap font) at pixel coordinates. Supports
 * letters (folded to uppercase), digits, space, hyphen, period and colon.
 * Up to 21 characters per line at x=0. No line wrapping.
 */
int oled_lite_text(unsigned int x, unsigned int y, const char *text);
int oled_lite_flush(void);
