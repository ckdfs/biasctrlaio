/* SPDX-License-Identifier: Apache-2.0 */
#include "mocks.h"

#include <dac8568.h>
#include <errno.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

struct mock_board_state mock_state;

static const uint8_t mock_dc_chs[] = { 0, 1, 2, 3, 4, 5 };
static const uint8_t mock_pilot_chs[] = { 6, 7 };
static const struct analog_pilot_route mock_routes[] = {
	{ .pilot = 0, .dc = 0, .switch_index = 0 },
	{ .pilot = 0, .dc = 1, .switch_index = 1 },
	{ .pilot = 1, .dc = 1, .switch_index = 2 },
	{ .pilot = 1, .dc = 2, .switch_index = 3 },
	{ .pilot = 0, .dc = 3, .switch_index = 4 },
	{ .pilot = 0, .dc = 4, .switch_index = 5 },
	{ .pilot = 1, .dc = 4, .switch_index = 6 },
	{ .pilot = 1, .dc = 5, .switch_index = 7 },
};

static const struct analog_board_caps mock_caps = {
	.board_id = "20261110-MZM",
	.dc_count = ARRAY_SIZE(mock_dc_chs),
	.pilot_count = ARRAY_SIZE(mock_pilot_chs),
	.route_count = ARRAY_SIZE(mock_routes),
	.dc_dac_channels = mock_dc_chs,
	.pilot_dac_channels = mock_pilot_chs,
	.routes = mock_routes,
};

static int mock_init(void)
{
	return 0;
}

static int mock_set_dc_code(uint8_t dc, uint16_t code)
{
	if (dc >= mock_caps.dc_count) {
		return -EINVAL;
	}
	if (mock_state.set_dc_fail_rc != 0) {
		return mock_state.set_dc_fail_rc;
	}
	mock_state.dc_code[dc] = code;
	return 0;
}

static int mock_set_pilot_code(uint8_t pilot, uint16_t code)
{
	if (pilot >= mock_caps.pilot_count) {
		return -EINVAL;
	}
	if (mock_state.set_pilot_fail_rc != 0) {
		return mock_state.set_pilot_fail_rc;
	}
	mock_state.pilot_code[pilot] = code;
	return 0;
}

static int mock_set_route(uint8_t pilot, uint8_t dc, bool enabled)
{
	if (pilot >= mock_caps.pilot_count || dc >= mock_caps.dc_count) {
		return -EINVAL;
	}
	if (mock_state.set_route_fail_rc != 0) {
		return mock_state.set_route_fail_rc;
	}
	mock_state.route[pilot][dc] = enabled;
	return 0;
}

static int mock_disconnect_all(void)
{
	mock_state.disconnect_count++;
	memset(mock_state.route, 0, sizeof(mock_state.route));
	if (mock_state.disconnect_fail_rc != 0) {
		return mock_state.disconnect_fail_rc;
	}
	return 0;
}

static int validate_desc_header(const struct bias_eeprom_descriptor_v1 *d)
{
	if (d->magic != BIAS_EEPROM_MAGIC || d->length_bytes != sizeof(*d)) {
		return -EBADMSG;
	}
	if (d->schema_version != BIAS_EEPROM_SCHEMA_V1) {
		return -ENOTSUP;
	}
	return memcmp(d->board_id, "20261110-MZM", sizeof(d->board_id)) == 0 ? 0 : -ENODEV;
}

static uint32_t calc_desc_crc(const struct bias_eeprom_descriptor_v1 *d)
{
	return crc32_ieee((const uint8_t *)d,
			  offsetof(struct bias_eeprom_descriptor_v1, crc32));
}

static int mock_load_calibration(struct bias_eeprom_descriptor_v1 *descriptor)
{
	mock_state.load_cal_count++;
	if (!descriptor) {
		return -EINVAL;
	}
	if (mock_state.load_cal_fail_rc != 0) {
		return mock_state.load_cal_fail_rc;
	}
	struct bias_eeprom_descriptor_v1 d;

	memcpy(&d, mock_state.eeprom, sizeof(d));
	int rc = validate_desc_header(&d);

	if (rc != 0) {
		return rc;
	}
	if (d.crc32 != calc_desc_crc(&d)) {
		return -EBADMSG;
	}
	*descriptor = d;
	return 0;
}

static int mock_store_calibration(const struct bias_eeprom_descriptor_v1 *descriptor)
{
	mock_state.store_cal_count++;
	if (!descriptor) {
		return -EINVAL;
	}
	if (mock_state.store_cal_fail_rc != 0) {
		return mock_state.store_cal_fail_rc;
	}
	struct bias_eeprom_descriptor_v1 d = *descriptor;
	int rc = validate_desc_header(&d);

	if (rc != 0) {
		return rc;
	}
	d.crc32 = calc_desc_crc(&d);
	memcpy(mock_state.eeprom, &d, sizeof(d));
	return 0;
}

const analog_board_ops_t mock_board_ops = {
	.caps = &mock_caps,
	.init = mock_init,
	.set_dc_code = mock_set_dc_code,
	.set_pilot_code = mock_set_pilot_code,
	.set_route = mock_set_route,
	.disconnect_all = mock_disconnect_all,
	.load_calibration = mock_load_calibration,
	.store_calibration = mock_store_calibration,
};

static int mock_storage_read(uint8_t offset, void *data, size_t len)
{
	if (!data || (size_t)offset + len > sizeof(mock_state.eeprom)) {
		return -EINVAL;
	}
	if (mock_state.storage_read_fail_rc != 0) {
		return mock_state.storage_read_fail_rc;
	}
	mock_state.read_count++;
	memcpy(data, &mock_state.eeprom[offset], len);
	return 0;
}

static int mock_storage_write_page(uint8_t offset, const void *data, size_t len)
{
	if (!data || len == 0U || len > BIAS_EEPROM_PAGE_BYTES ||
	    (offset % BIAS_EEPROM_PAGE_BYTES) + len > BIAS_EEPROM_PAGE_BYTES ||
	    (size_t)offset + len > sizeof(mock_state.eeprom)) {
		return -EINVAL;
	}
	if (mock_state.storage_write_fail_rc != 0) {
		return mock_state.storage_write_fail_rc;
	}
	mock_state.write_page_count++;
	memcpy(&mock_state.eeprom[offset], data, len);
	return 0;
}

const cal_storage_ops_t mock_storage_ops = {
	.read = mock_storage_read,
	.write_page = mock_storage_write_page,
};

void mock_reset_all(void)
{
	memset(&mock_state, 0, sizeof(mock_state));
	memset(mock_state.eeprom, 0xff, sizeof(mock_state.eeprom));
}

void mock_install_mzm_descriptor(int32_t offset_uv, int32_t gain_ppm, uint32_t vpi_uv)
{
	struct bias_eeprom_descriptor_v1 desc;

	mzm_20261110_fill_default_descriptor(&desc);
	desc.channels[0].vpi_uv = vpi_uv;
	desc.channels[0].subtractor_offset_uv = offset_uv;
	desc.channels[0].subtractor_gain_ppm = gain_ppm;
	(void)mock_store_calibration(&desc);
}

/* Stubbed production symbols for qemu_cortex_m3 */
const analog_board_ops_t *analog_board_get_ops(void)
{
	return &mock_board_ops;
}

void mzm_20261110_fill_default_descriptor(struct bias_eeprom_descriptor_v1 *desc)
{
	if (!desc) {
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

int bias_ads131m02_init(enum bias_ads131m02_rate rate)
{
	ARG_UNUSED(rate);
	return 0;
}

int bias_ads131m02_read_reg(uint8_t address, uint16_t *value)
{
	ARG_UNUSED(address);
	if (!value) {
		return -EINVAL;
	}
	*value = 0;
	return 0;
}

int bias_ads131m02_write_reg(uint8_t address, uint16_t value)
{
	ARG_UNUSED(address);
	ARG_UNUSED(value);
	return 0;
}

int bias_ads131m02_start(bias_ads131m02_callback callback, void *context)
{
	ARG_UNUSED(callback);
	ARG_UNUSED(context);
	return 0;
}

int bias_ads131m02_stop(void)
{
	return 0;
}

int dac8568_status(void)
{
	return 0;
}

int dac8568_set_dc_code_async(uint8_t dc, uint16_t code)
{
	return mock_board_ops.set_dc_code(dc, code);
}
