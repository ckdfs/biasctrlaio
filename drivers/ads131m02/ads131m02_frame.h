/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_ADS131M02_FRAME_H
#define BIAS_ADS131M02_FRAME_H

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include "ads131m02.h"

/* MSB-first CRC-16-CCITT, polynomial 0x1021, initial value 0xffff. */
static inline uint16_t ads_crc(const uint8_t *bytes, size_t count)
{
	uint16_t crc = 0xffff;

	for (size_t i = 0; i < count; i++) {
		uint8_t x = (crc >> 8) ^ bytes[i];

		x ^= x >> 4;
		crc = (crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ x;
	}
	return crc;
}

static inline int32_t ads_signed24(const uint8_t *p)
{
	int32_t raw = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];

	return (raw ^ 0x800000) - 0x800000;
}

static inline int ads_frame_crc(const uint8_t *frame)
{
	uint16_t expected = ((uint16_t)frame[9] << 8) | frame[10];

	return ads_crc(frame, 9) == expected ? 0 : -EBADMSG;
}

static inline int ads_decode(const uint8_t *frame, struct bias_ads131m02_sample *sample)
{
	int rc = ads_frame_crc(frame);

	if (rc) {
		return rc;
	}
	uint16_t status = ((uint16_t)frame[0] << 8) | frame[1];

	if (status & 0x1000) { /* ADC reports input CRC failure. */
		return -EBADMSG;
	}
	if (status & 0x0400) { /* Unexpected ADC reset. */
		return -EIO;
	}
	sample->status = status;
	sample->channel[0] = ads_signed24(frame + 3);
	sample->channel[1] = ads_signed24(frame + 6);
	return 0;
}
#endif
