/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_ADS131M02_H
#define BIAS_ADS131M02_H

#include <stdint.h>

/* HR mode with the daughterboard's 8.192 MHz oscillator. */
enum bias_ads131m02_rate {
	BIAS_ADS131M02_32_KSPS, /* OSR = 128 */
	BIAS_ADS131M02_64_KSPS, /* TBM selects OSR = 64 */
};

struct bias_ads131m02_sample {
	int32_t channel[2];
	uint16_t status;
};

/* Called on DRDY in interrupt context. Copy the sample before returning.
 * A negative error and NULL sample indicate a halted stream. Stop in a thread.
 */
typedef void (*bias_ads131m02_callback)(const struct bias_ads131m02_sample *sample,
				       int error, void *context);

/* Thread APIs. Initialize before accessing registers or starting acquisition. */
int bias_ads131m02_init(enum bias_ads131m02_rate rate);
int bias_ads131m02_read_reg(uint8_t address, uint16_t *value);
/* MODE and CLOCK are managed by init. */
int bias_ads131m02_write_reg(uint8_t address, uint16_t value);
int bias_ads131m02_start(bias_ads131m02_callback callback, void *context);
/* Ends acquisition and discards the pending frame. Returns any stream fault. */
int bias_ads131m02_stop(void);

#endif
