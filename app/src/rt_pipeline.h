/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_RT_PIPELINE_H
#define BIAS_RT_PIPELINE_H

#include <bias/analog_board.h>
#include <dsp_types.h>
#include <zephyr/kernel.h>

/* Signed 24-bit ADC at gain 1. Pilot amplitude is peak volts at the MZM VA
 * electrode: DAC8568 outputs 0..5.0 V with zero dither at 2.5 V (code 32768),
 * attenuated by -R23/R22 = -12k/120k = -0.1 V/V -> +/-0.25 V peak max.
 */
#define RT_PIPELINE_ADC_V_PER_COUNT (1.2f / 8388608.0f)
#define RT_PIPELINE_PILOT_MAX_V 0.25f

typedef struct {
	harmonic_data_t harmonics;
	uint32_t seq; /* Starts at zero on each start. */
	uint32_t timestamp_cycles; /* Arrival of the final sample in the block. */
	float pilot_amplitude_v;
} rt_pipeline_block_t;

/* Runs in system workqueue context after cutoff. cleanup_error is the first
 * additional failure encountered while isolating the outputs or stopping ADC.
 */
typedef void (*rt_pipeline_fault_cb_t)(int error, int cleanup_error);

/* Thread context. Caller initializes the board and owns route selection.
 * NULL ops selects the production board. Reinitialize after fault cleanup.
 */
int rt_pipeline_init(const analog_board_ops_t *ops, rt_pipeline_fault_cb_t callback);
int rt_pipeline_start(void);
int rt_pipeline_stop(void);
int rt_pipeline_set_dc_code(uint8_t dc, uint16_t code);
int rt_pipeline_set_pilot_amplitude(float amp_v);
int rt_pipeline_get_block(rt_pipeline_block_t *block, k_timeout_t timeout);

int rt_pipeline_fault(void);
uint32_t rt_pipeline_drop_count(void);
uint32_t rt_pipeline_crc_error_count(void);

/* Start without an ADC for tests. Supply mock board ops to init and feed
 * samples from a single producer. A negative error injects an ADC fault.
 * Quiesce the producer before stop/reinitialization.
 */
int rt_pipeline_start_manual(void);
int rt_pipeline_ingest(int32_t ch0, int32_t ch1, int error, uint32_t timestamp_cycles);

#endif
