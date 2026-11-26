/* SPDX-License-Identifier: Apache-2.0 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "pilot_mux.h"

/* Deliberately shuffled so a table offset cannot stand in for switch_index. */
static const struct analog_pilot_route routes[] = {
	{1, 4, 6}, {0, 1, 1}, {1, 1, 2}, {0, 4, 5},
	{0, 3, 4}, {1, 2, 3}, {0, 0, 0}, {1, 5, 7}
};
static const struct analog_board_caps caps = {
	.dc_count = 6, .pilot_count = 2, .route_count = 8, .routes = routes
};

struct gpio_mock {
	unsigned int levels;
	unsigned int calls;
	unsigned int low_calls[8];
	unsigned int low_fail;
	bool high_fail;
};

static int write_gpio(void *ctx, uint8_t index, bool high)
{
	struct gpio_mock *gpio = ctx;
	assert(index < 8);
	unsigned int bit = 1u << index;
	gpio->calls++;
	if (high) {
		gpio->levels |= bit;
		return gpio->high_fail ? -ETIMEDOUT : 0;
	}
	gpio->low_calls[index]++;
	if (gpio->low_fail & bit) {
		return -EIO;
	}
	gpio->levels &= ~bit;
	return 0;
}

static void check_sweep(struct gpio_mock *gpio)
{
	for (int i = 0; i < 8; ++i) {
		assert(gpio->low_calls[i] == 1);
	}
	memset(gpio->low_calls, 0, sizeof(gpio->low_calls));
}

static void test_routes(void)
{
	struct gpio_mock gpio = { .levels = 255 };
	struct pilot_mux mux;
	assert(pilot_mux_init(&mux, &caps, write_gpio, &gpio) == 0);
	check_sweep(&gpio);
	assert(gpio.levels == 0);

	for (int i = 0; i < 8; ++i) {
		const struct analog_pilot_route *r = &routes[i];
		assert(pilot_mux_set_route(&mux, r->pilot, r->dc, true) == 0);
		assert(gpio.levels == (1u << r->switch_index));
		assert(pilot_mux_set_route(&mux, r->pilot, r->dc, false) == 0);
		assert(gpio.levels == 0);
	}
	assert(pilot_mux_set_route(&mux, 0, 1, true) == 0);
	assert(pilot_mux_set_route(&mux, 0, 0, true) == 0);
	assert(pilot_mux_set_route(&mux, 1, 4, true) == 0);
	unsigned int calls = gpio.calls;
	assert(pilot_mux_set_route(&mux, 1, 1, true) == -EBUSY);
	assert(pilot_mux_set_route(&mux, 0, 4, true) == -EBUSY);
	assert(pilot_mux_set_route(&mux, 0, 2, true) == -ENOENT);
	assert(pilot_mux_set_route(&mux, 2, 0, false) == -EINVAL);
	assert(pilot_mux_set_route(&mux, 0, 6, true) == -EINVAL);
	assert(gpio.calls == calls && gpio.levels == 0x43);
	assert(pilot_mux_set_route(&mux, 0, 1, false) == 0);
	assert(pilot_mux_set_route(&mux, 1, 1, true) == 0);
	assert(gpio.levels == 0x45);
}

static void test_shutdown(void)
{
	struct gpio_mock gpio = { .low_fail = 2 };
	struct pilot_mux mux;
	assert(pilot_mux_init(&mux, &caps, write_gpio, &gpio) == -EIO);
	check_sweep(&gpio);
	assert(pilot_mux_set_route(&mux, 0, 0, true) == -EACCES);
	gpio.low_fail = 0;
	assert(pilot_mux_init(&mux, &caps, write_gpio, &gpio) == 0);
	check_sweep(&gpio);
	gpio.high_fail = true;
	assert(pilot_mux_set_route(&mux, 0, 0, true) == -ETIMEDOUT);
	check_sweep(&gpio);
	assert(gpio.levels == 0);
	gpio.low_fail = 1;
	assert(pilot_mux_set_route(&mux, 0, 0, true) == -EIO);
	check_sweep(&gpio);
	assert(gpio.levels == 1);
	assert(pilot_mux_set_route(&mux, 1, 1, true) == -EACCES);
	gpio.low_fail = 0;
	gpio.high_fail = false;
	assert(pilot_mux_disconnect_all(&mux) == 0);
	check_sweep(&gpio);
	assert(pilot_mux_disconnect_all(&mux) == 0);
	check_sweep(&gpio);
	assert(gpio.levels == 0);
	assert(pilot_mux_set_route(&mux, 1, 1, true) == 0);
}

static void test_invalid_table(void)
{
	struct gpio_mock gpio = {0};
	struct pilot_mux mux;
	const struct analog_pilot_route invalid[] = {
		{2, 0, 1}, {0, 6, 1}, {0, 1, 8}, {0, 1, 0}, {0, 0, 1}
	};
	struct analog_pilot_route pair[] = {{0, 0, 0}, {0, 1, 1}};
	struct analog_board_caps bad = caps;
	bad.routes = pair;
	bad.route_count = 2;
	for (unsigned int i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) {
		pair[1] = invalid[i];
		assert(pilot_mux_init(&mux, &bad, write_gpio, &gpio) == -EINVAL);
		check_sweep(&gpio);
		assert(pilot_mux_set_route(&mux, 0, 0, true) == -EACCES);
	}
	bad.route_count = 9;
	assert(pilot_mux_init(&mux, &bad, write_gpio, &gpio) == -EINVAL);
	bad.routes = NULL;
	bad.route_count = 1;
	assert(pilot_mux_init(&mux, &bad, write_gpio, &gpio) == -EINVAL);
}

int main(void)
{
	test_routes();
	test_shutdown();
	test_invalid_table();
	return 0;
}
