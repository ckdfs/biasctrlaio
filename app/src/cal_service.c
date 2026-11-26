/* SPDX-License-Identifier: Apache-2.0 */
#include "cal_service.h"

#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/crc.h>
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(bias_eeprom))
#include <shared_i2c.h>
static const struct i2c_dt_spec u21 = I2C_DT_SPEC_GET(DT_NODELABEL(bias_eeprom));
#endif

#define RECORD_OFFSET 160u
#define RECORD_MAGIC 0x314d5a4du /* "MZM1" */
#define WIRE_CRC(obj) crc32_ieee((const uint8_t *)&(obj), offsetof(__typeof__(obj), crc32))

/* Little-endian, binary32. Use a fixed-width flag instead of persisting
 * implementation-defined bool layout. Bind the record to its descriptor. */
struct cal_mzm_record_v1 {
	uint32_t magic, descriptor_crc;
	dsp_affine_fit_t fit;
	float null_v, vpi_v, cal_pilot_v, obs_dc_seed;
	uint32_t has_obs_dc_seed, crc32;
};
BUILD_ASSERT(sizeof(struct cal_mzm_record_v1) == 56);
BUILD_ASSERT(RECORD_OFFSET >= BIAS_EEPROM_V1_SIZE && RECORD_OFFSET % BIAS_EEPROM_PAGE_BYTES == 0);
BUILD_ASSERT(RECORD_OFFSET + sizeof(struct cal_mzm_record_v1) <= BIAS_EEPROM_CAPACITY_BYTES);

int cal_service_voltage_bounds(const struct bias_eeprom_descriptor_v1 *d, float *lo, float *hi)
{
	if (!d || !lo || !hi || !d->channels[0].subtractor_gain_ppm) {
		return -EINVAL;
	}
	double offset = d->channels[0].subtractor_offset_uv * 1e-6;
	double end = offset + 2.5e-6 * d->channels[0].subtractor_gain_ppm;
	*lo = MIN(offset, end);
	*hi = MAX(offset, end);
	return 0;
}

int cal_service_voltage_to_dac_code(const struct bias_eeprom_descriptor_v1 *d,
				    float voltage, uint16_t *code)
{
	float lo, hi;
	if (!code || !isfinite(voltage) || cal_service_voltage_bounds(d, &lo, &hi)) {
		return -EINVAL;
	}
	if (voltage < lo || voltage > hi) {
		return -ERANGE;
	}
	double fraction = ((double)voltage * 1e6 - d->channels[0].subtractor_offset_uv) /
			  (2.5 * d->channels[0].subtractor_gain_ppm);
	*code = (uint16_t)(CLAMP(fraction, 0., 1.) * 65535. + 0.5);
	return 0;
}

static int transfer(const cal_storage_ops_t *storage, struct cal_mzm_record_v1 *r, bool write)
{
	int rc = 0;
	if (storage && !(write ? storage->write_page != NULL : storage->read != NULL)) {
		return -EINVAL;
	}
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(bias_eeprom))
	if (!storage) {
		if (!i2c_is_ready_dt(&u21)) {
			return -ENODEV;
		}
		k_mutex_lock(&bias_i2c1_mutex, K_FOREVER);
	}
#else
	if (!storage) {
		return -ENODEV;
	}
#endif
	for (size_t pos = 0; pos < sizeof(*r) && !rc;) {
		uint8_t addr = RECORD_OFFSET + pos;
		uint8_t *p = (uint8_t *)r + pos;
		size_t n = write ? MIN(BIAS_EEPROM_PAGE_BYTES, sizeof(*r) - pos) : sizeof(*r);
		if (storage) {
			rc = write ? storage->write_page(addr, p, n) : storage->read(addr, p, n);
		}
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(bias_eeprom))
		else if (write) {
			uint8_t page[1 + BIAS_EEPROM_PAGE_BYTES] = {addr};
			memcpy(page + 1, p, n);
			rc = i2c_write_dt(&u21, page, n + 1);
			if (!rc) {
				k_msleep(BIAS_EEPROM_WRITE_CYCLE_MS);
			}
		} else {
			rc = i2c_write_read_dt(&u21, &addr, 1, p, n);
		}
#endif
		pos += n;
	}
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(bias_eeprom))
	if (!storage) {
		k_mutex_unlock(&bias_i2c1_mutex);
	}
#endif
	return rc;
}

static bool valid_record(const struct cal_mzm_record_v1 *r)
{
	for (size_t i = 0; i < 2; i++) {
		if (!isfinite(r->fit.o[i]) || !isfinite(r->fit.m[i][0]) ||
		    !isfinite(r->fit.m[i][1])) {
			return false;
		}
	}
	double uv = (double)r->vpi_v * 1e6;
	return isfinite(r->null_v) && uv >= 0.5 && uv < UINT32_MAX + 0.5 &&
	       isfinite(r->cal_pilot_v) && r->cal_pilot_v > 0 &&
	       isfinite(r->obs_dc_seed) && r->has_obs_dc_seed <= 1;
}

int cal_service_save(const analog_board_ops_t *board, const cal_storage_ops_t *storage,
		     struct bias_eeprom_descriptor_v1 *desc, const ctrl_calibration_t *cal)
{
	if (!board || !board->store_calibration || !desc || !cal ||
	    !desc->channels[0].subtractor_gain_ppm || (storage && !storage->write_page)) {
		return -EINVAL;
	}
	struct cal_mzm_record_v1 r = {
		.magic = RECORD_MAGIC, .fit = cal->fit, .null_v = cal->null_v,
		.vpi_v = cal->vpi_v, .cal_pilot_v = cal->cal_pilot_v,
		.has_obs_dc_seed = cal->has_obs_dc_seed, .obs_dc_seed = cal->obs_dc_seed,
	};
	if (!valid_record(&r)) {
		return -EINVAL;
	}
	struct bias_eeprom_descriptor_v1 d = *desc;
	d.channels[0].vpi_uv = (uint32_t)((double)cal->vpi_v * 1e6 + 0.5);
	d.crc32 = WIRE_CRC(d);
	r.descriptor_crc = d.crc32;
	r.crc32 = WIRE_CRC(r);
	int rc = board->store_calibration(&d);
	if (!rc) {
		rc = transfer(storage, &r, true);
	}
	if (!rc) {
		*desc = d;
	}
	return rc;
}

int cal_service_load_descriptor(const analog_board_ops_t *board, struct bias_eeprom_descriptor_v1 *desc)
{
	if (!board || !board->load_calibration || !desc) {
		return -EINVAL;
	}
	struct bias_eeprom_descriptor_v1 d;
	int rc = board->load_calibration(&d);
	if (rc) {
		return rc;
	}
	if (d.magic != BIAS_EEPROM_MAGIC || d.schema_version != BIAS_EEPROM_SCHEMA_V1 ||
	    d.length_bytes != sizeof(d) || memcmp(d.board_id, "20261110-MZM", sizeof(d.board_id)) ||
	    d.crc32 != WIRE_CRC(d) || !d.channels[0].subtractor_gain_ppm) {
		return -EBADMSG;
	}
	*desc = d;
	return 0;
}

int cal_service_load(const analog_board_ops_t *board, const cal_storage_ops_t *storage,
		     struct bias_eeprom_descriptor_v1 *desc, ctrl_calibration_t *cal)
{
	if (!desc || !cal) {
		return -EINVAL;
	}
	struct bias_eeprom_descriptor_v1 d;
	int rc = cal_service_load_descriptor(board, &d);
	if (rc) {
		return rc;
	}
	struct cal_mzm_record_v1 r;
	rc = transfer(storage, &r, false);
	if (rc) {
		return rc;
	}
	if (r.magic != RECORD_MAGIC || r.crc32 != WIRE_CRC(r) || r.descriptor_crc != d.crc32 ||
	    !valid_record(&r) || d.channels[0].vpi_uv != (uint32_t)((double)r.vpi_v * 1e6 + 0.5)) {
		return -EBADMSG;
	}
	*desc = d;
	*cal = (ctrl_calibration_t){
		.fit = r.fit, .null_v = r.null_v, .vpi_v = r.vpi_v, .cal_pilot_v = r.cal_pilot_v,
		.has_obs_dc_seed = r.has_obs_dc_seed, .obs_dc_seed = r.obs_dc_seed,
	};
	return 0;
}
