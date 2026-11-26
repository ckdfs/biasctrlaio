/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_PILOT_MUX_H
#define BIAS_PILOT_MUX_H

#include <bias/analog_board.h>

#define PILOT_MUX_SWITCH_COUNT 8

/* The caller serializes operations and owns the immutable caps and callback
 * context for the lifetime of the matrix. The setter returns zero or -errno.
 */
typedef int (*pilot_mux_set_gpio_t)(void *context, uint8_t index, bool high);

struct pilot_mux {
	const struct analog_board_caps *caps;
	pilot_mux_set_gpio_t set_gpio;
	void *context;
	uint8_t active;
	bool safe_to_route;
};

int pilot_mux_init(struct pilot_mux *mux, const struct analog_board_caps *caps,
		   pilot_mux_set_gpio_t set_gpio, void *context);
int pilot_mux_set_route(struct pilot_mux *mux, uint8_t pilot, uint8_t dc, bool enable);
/* Writes every switch even after errors. A failed sweep inhibits routing
 * until a complete sweep succeeds. Returns the first failed LOW write.
 */
int pilot_mux_disconnect_all(struct pilot_mux *mux);

/* Zephyr binding, thread context. Boot disconnects all switches before the
 * daughterboard supplies its route table via pilot_mux_gpio_init().
 */
int pilot_mux_gpio_init(const struct analog_board_caps *caps);
int pilot_mux_gpio_set_route(uint8_t pilot, uint8_t dc, bool enable);
int pilot_mux_gpio_disconnect_all(void);

#endif
