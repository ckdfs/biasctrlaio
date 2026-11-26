/* SPDX-License-Identifier: Apache-2.0 */
#include "analog_board_mzm.h"

#include <bias/analog_board.h>
#include <dac8568.h>
#include <eeprom_board.h>
#include <pilot_mux.h>

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>

static const uint8_t mzm_dc_dac_channels[] = { 0, 1, 2, 3, 4, 5 };
static const uint8_t mzm_pilot_dac_channels[] = { 6, 7 };

/* Switch indices match biasctrl_h503.dts zephyr,user pilot-sw-gpios:
 * 0:PA0 1:PA1 2:PA2 3:PA3 4:PA4 5:PA6 6:PA15 7:PB6
 */
static const struct analog_pilot_route mzm_routes[] = {
	{ .pilot = 0, .dc = 0, .switch_index = 0 }, /* DITHER1 -> VA (PA0) */
	{ .pilot = 0, .dc = 1, .switch_index = 1 }, /* DITHER1 -> VB (PA1) */
	{ .pilot = 1, .dc = 1, .switch_index = 2 }, /* DITHER2 -> VB (PA2) */
	{ .pilot = 1, .dc = 2, .switch_index = 3 }, /* DITHER2 -> VC (PA3) */
	{ .pilot = 0, .dc = 3, .switch_index = 4 }, /* DITHER1 -> VD (PA4) */
	{ .pilot = 0, .dc = 4, .switch_index = 5 }, /* DITHER1 -> VE (PA6) */
	{ .pilot = 1, .dc = 4, .switch_index = 6 }, /* DITHER2 -> VE (PA15) */
	{ .pilot = 1, .dc = 5, .switch_index = 7 }, /* DITHER2 -> VF (PB6) */
};

static const struct analog_board_caps mzm_caps = {
	.board_id = "20261110-MZM",
	.dc_count = ARRAY_SIZE(mzm_dc_dac_channels),
	.pilot_count = ARRAY_SIZE(mzm_pilot_dac_channels),
	.route_count = ARRAY_SIZE(mzm_routes),
	.dc_dac_channels = mzm_dc_dac_channels,
	.pilot_dac_channels = mzm_pilot_dac_channels,
	.routes = mzm_routes,
};

static int mzm_init(void)
{
	int rc = pilot_mux_gpio_init(&mzm_caps);

	if (rc < 0) {
		return rc;
	}
	return dac8568_init();
}

static int mzm_set_dc_code(uint8_t dc, uint16_t code)
{
	if (dc >= mzm_caps.dc_count) {
		return -EINVAL;
	}
	int rc = dac8568_set_dc_code(mzm_dc_dac_channels[dc], code);

	if (rc < 0) {
		(void)pilot_mux_gpio_disconnect_all();
	}
	return rc;
}

static int mzm_set_pilot_code(uint8_t pilot, uint16_t code)
{
	if (pilot >= mzm_caps.pilot_count) {
		return -EINVAL;
	}
	int rc = dac8568_set_pilot_code(mzm_pilot_dac_channels[pilot] - 6U, code);

	if (rc < 0 && !k_is_in_isr()) {
		(void)pilot_mux_gpio_disconnect_all();
	}
	return rc;
}

static int mzm_set_route(uint8_t pilot, uint8_t dc, bool enabled)
{
	return pilot_mux_gpio_set_route(pilot, dc, enabled);
}

static int mzm_disconnect_all(void)
{
	return pilot_mux_gpio_disconnect_all();
}

static int mzm_load_calibration(struct bias_eeprom_descriptor_v1 *descriptor)
{
	return eeprom_board_load(descriptor);
}

static int mzm_store_calibration(const struct bias_eeprom_descriptor_v1 *descriptor)
{
	return eeprom_board_store(descriptor);
}

static const analog_board_ops_t mzm_ops = {
	.caps = &mzm_caps,
	.init = mzm_init,
	.set_dc_code = mzm_set_dc_code,
	.set_pilot_code = mzm_set_pilot_code,
	.set_route = mzm_set_route,
	.disconnect_all = mzm_disconnect_all,
	.load_calibration = mzm_load_calibration,
	.store_calibration = mzm_store_calibration,
};

const analog_board_ops_t *analog_board_get_ops(void)
{
	return &mzm_ops;
}

void mzm_20261110_fill_default_descriptor(struct bias_eeprom_descriptor_v1 *desc)
{
	if (desc == NULL) {
		return;
	}
	memset(desc, 0, sizeof(*desc));
	desc->magic = BIAS_EEPROM_MAGIC;
	desc->schema_version = BIAS_EEPROM_SCHEMA_V1;
	desc->length_bytes = (uint16_t)sizeof(*desc);
	memcpy(desc->board_id, "20261110-MZM", sizeof(desc->board_id));
	desc->tia_gain_ohms = 47000U;
	for (size_t i = 0; i < BIAS_EEPROM_V1_CHANNELS; ++i) {
		desc->channels[i].vpi_uv = 5000000U;
		desc->channels[i].subtractor_gain_ppm = 8000000;
		desc->channels[i].subtractor_offset_uv = -10000000;
	}
}
