/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_EEPROM_SCHEMA_H
#define BIAS_EEPROM_SCHEMA_H

#include <stddef.h>
#include <stdint.h>

#define BIAS_EEPROM_CAPACITY_BYTES 256u
#define BIAS_EEPROM_PAGE_BYTES 8u
#define BIAS_EEPROM_WRITE_CYCLE_MS 5u

#define BIAS_EEPROM_MAGIC UINT32_C(0x53414942) /* "BIAS" on the wire */
#define BIAS_EEPROM_SCHEMA_V1 1u
#define BIAS_EEPROM_V1_CHANNELS 6u
#define BIAS_EEPROM_V1_SIZE 148u

/* V1 starts at EEPROM byte 0. Integers are little-endian, signed fields
 * are two's complement. Convert on hosts with a different byte order.
 * The board ID is ASCII, NUL-padded (e.g. "20261110-MZM").
 */
struct bias_channel_cal_v1 {
	uint32_t vpi_uv;
	int32_t adc_zero_code;
	/* Subtractor output in uV = DAC uV * gain_ppm / 1000000 + offset_uv. */
	int32_t subtractor_gain_ppm;
	int32_t subtractor_offset_uv;
	/* Signed correction to the measured bias voltage, in uV. */
	int32_t bias_zero_uv;
};

struct bias_eeprom_descriptor_v1 {
	uint32_t magic;
	uint16_t schema_version;
	uint16_t length_bytes; /* Includes crc32. */
	char board_id[12];
	uint32_t tia_gain_ohms;
	struct bias_channel_cal_v1 channels[BIAS_EEPROM_V1_CHANNELS]; /* VA..VF */
	/* CRC-32/ISO-HDLC over all preceding bytes as stored, excluding crc32. */
	uint32_t crc32;
};

#ifdef __cplusplus
#define BIAS_SCHEMA_ASSERT static_assert
#else
#define BIAS_SCHEMA_ASSERT _Static_assert
#endif

BIAS_SCHEMA_ASSERT(sizeof(struct bias_channel_cal_v1) == 20, "v1 channel layout");
BIAS_SCHEMA_ASSERT(offsetof(struct bias_eeprom_descriptor_v1, channels) == 24, "v1 header");
BIAS_SCHEMA_ASSERT(offsetof(struct bias_eeprom_descriptor_v1, crc32) + 4 ==
		   sizeof(struct bias_eeprom_descriptor_v1), "v1 CRC must be last");
BIAS_SCHEMA_ASSERT(sizeof(struct bias_eeprom_descriptor_v1) == BIAS_EEPROM_V1_SIZE,
		   "v1 descriptor layout");
BIAS_SCHEMA_ASSERT(BIAS_EEPROM_V1_SIZE <= BIAS_EEPROM_CAPACITY_BYTES, "v1 exceeds U21");
#undef BIAS_SCHEMA_ASSERT

#endif /* BIAS_EEPROM_SCHEMA_H */
