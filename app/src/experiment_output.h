/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "ctrl_thread.h"

/* Thread-context wire formatting. */
int cli_measure_row(const rt_pipeline_block_t *b, bool step, float bias, uint16_t code,
		    cli_write_fn_t out, void *ctx);
int cli_tel_row(const ctrl_thread_status_t *st, const rt_pipeline_block_t *b,
		float phase, float error, float radius, const float iq[4]);
void cli_adc_result(bool dump, unsigned segment, unsigned count, const rt_adc_stats_t *stats,
		    cli_write_fn_t out, void *ctx);
void cli_hw_result(const uint16_t regs[4], uint32_t desc_crc, uint32_t record_crc,
		   int cal_rc, cli_write_fn_t out, void *ctx);
