/* SPDX-License-Identifier: Apache-2.0 */
#include "pilot_mux.h"
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#define SWITCH_SPEC(node, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),
static const struct gpio_dt_spec switches[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pilot_sw_gpios, SWITCH_SPEC)
};
BUILD_ASSERT(ARRAY_SIZE(switches) == PILOT_MUX_SWITCH_COUNT);

static bool configured[PILOT_MUX_SWITCH_COUNT];
static K_MUTEX_DEFINE(mux_lock);

static int set_switch(void *context, uint8_t index, bool high)
{
	ARG_UNUSED(context);
	const struct gpio_dt_spec *pin = &switches[index];

	if (!configured[index]) {
		if (!gpio_is_ready_dt(pin)) {
			return -ENODEV;
		}
		int rc = gpio_pin_configure_dt(pin, GPIO_OUTPUT_LOW);

		if (rc) {
			return rc;
		}
		configured[index] = true;
		if (!high) {
			return 0;
		}
	}
	return gpio_pin_set_raw(pin->port, pin->pin, high);
}

static struct pilot_mux matrix = { .set_gpio = set_switch };

int pilot_mux_gpio_init(const struct analog_board_caps *caps)
{
	k_mutex_lock(&mux_lock, K_FOREVER);
	int rc = pilot_mux_init(&matrix, caps, set_switch, NULL);

	k_mutex_unlock(&mux_lock);
	return rc;
}

int pilot_mux_gpio_set_route(uint8_t pilot, uint8_t dc, bool enable)
{
	k_mutex_lock(&mux_lock, K_FOREVER);
	int rc = pilot_mux_set_route(&matrix, pilot, dc, enable);

	k_mutex_unlock(&mux_lock);
	return rc;
}

int pilot_mux_gpio_disconnect_all(void)
{
	k_mutex_lock(&mux_lock, K_FOREVER);
	int rc = pilot_mux_disconnect_all(&matrix);

	k_mutex_unlock(&mux_lock);
	return rc;
}

SYS_INIT(pilot_mux_gpio_disconnect_all, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
