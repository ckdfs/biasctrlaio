/* SPDX-License-Identifier: Apache-2.0 */
#include "pilot_mux.h"
#include <errno.h>
#include <stddef.h>

int pilot_mux_disconnect_all(struct pilot_mux *mux)
{
	if (!mux || !mux->set_gpio) {
		return -EINVAL;
	}

	int error = 0;

	for (uint8_t i = 0; i < PILOT_MUX_SWITCH_COUNT; ++i) {
		int rc = mux->set_gpio(mux->context, i, false);

		if (rc && !error) {
			error = rc;
		}
	}
	mux->safe_to_route = (error == 0);
	if (!error) {
		mux->active = 0;
	}
	return error;
}

static bool valid_routes(const struct analog_board_caps *caps)
{
	if (!caps || !caps->routes || !caps->route_count ||
	    caps->route_count > PILOT_MUX_SWITCH_COUNT) {
		return false;
	}

	for (uint8_t i = 0; i < caps->route_count; ++i) {
		const struct analog_pilot_route *route = &caps->routes[i];

		if (route->pilot >= caps->pilot_count || route->dc >= caps->dc_count ||
		    route->switch_index >= PILOT_MUX_SWITCH_COUNT) {
			return false;
		}
		for (uint8_t j = 0; j < i; ++j) {
			const struct analog_pilot_route *prior = &caps->routes[j];

			if (prior->switch_index == route->switch_index ||
			    (prior->pilot == route->pilot && prior->dc == route->dc)) {
				return false;
			}
		}
	}
	return true;
}

int pilot_mux_init(struct pilot_mux *mux, const struct analog_board_caps *caps,
		   pilot_mux_set_gpio_t set_gpio, void *context)
{
	if (!mux || !set_gpio) {
		return -EINVAL;
	}
	*mux = (struct pilot_mux){ .set_gpio = set_gpio, .context = context };
	int rc = pilot_mux_disconnect_all(mux);

	if (rc) {
		return rc;
	}
	if (!valid_routes(caps)) {
		return -EINVAL;
	}
	mux->caps = caps;
	return 0;
}

int pilot_mux_set_route(struct pilot_mux *mux, uint8_t pilot, uint8_t dc, bool enable)
{
	if (!mux || !mux->caps || !mux->safe_to_route) {
		return -EACCES;
	}
	const struct analog_board_caps *caps = mux->caps;
	const struct analog_pilot_route *target = NULL;
	bool conflict = false;

	if (pilot >= caps->pilot_count || dc >= caps->dc_count) {
		return -EINVAL;
	}
	for (uint8_t i = 0; i < caps->route_count; ++i) {
		const struct analog_pilot_route *route = &caps->routes[i];

		if (route->dc != dc) {
			continue;
		}
		if (route->pilot == pilot) {
			target = route;
		} else if (mux->active & (1u << route->switch_index)) {
			conflict = true;
		}
	}
	if (!target) {
		return -ENOENT;
	}
	if (enable && conflict) {
		return -EBUSY;
	}

	uint8_t bit = 1u << target->switch_index;
	int rc = mux->set_gpio(mux->context, target->switch_index, enable);

	if (rc) {
		/* The failed write may have reached the hardware. */
		int shutdown_rc = pilot_mux_disconnect_all(mux);

		return shutdown_rc ? shutdown_rc : rc;
	}
	if (enable) {
		mux->active |= bit;
	} else {
		mux->active &= ~bit;
	}
	return 0;
}
