/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "cal_service.h"
#include "rt_pipeline.h"
#include "cli_service.h"
#include <ctrl_modulator.h>

typedef enum {
	CTRL_STATE_IDLE, CTRL_STATE_SCAN, CTRL_STATE_CAL, CTRL_STATE_RUNNING, CTRL_STATE_FAULT,
	CTRL_STATE_EXP_HOLD
} ctrl_state_t;

typedef struct {
	ctrl_state_t state;
	bias_point_t target;
	float custom_deg, bias_v, scan_vpi_v, vpi_v, null_v;
	bool calibrated, locked, hold_assist_active;
	bool frozen;
	uint32_t updates, scan_step, recal_count;
	unsigned scan_pass;
	harmonic_data_t harmonics;
	int fault, cleanup_error, load_error;
} ctrl_thread_status_t;

typedef struct {
	float pilot_v;
	float gain;
	float kp;
	float ki;
	float rho_thresh;
	uint16_t hold_ticks;
	uint8_t wait_blocks;
	uint8_t avg_blocks;
	bool use_gain;
	bool recal_auto;
} ctrl_thread_cfg_t;

/* Thread context. Initialize the board first. NULL ops select production.
 * A missing/invalid affine record returns its load error but permits a new
 * scan if the board descriptor is valid. Reinitialize to clear a fault.
 * In manual mode feed rt_pipeline_ingest continuously from a separate
 * producer (including during synchronized DC writes). */
int ctrl_thread_init(const analog_board_ops_t *board, const cal_storage_ops_t *storage, bool manual);
int ctrl_thread_start(void);
int ctrl_thread_stop(void);
int ctrl_thread_set_target(bias_point_t point, float custom_deg);
int ctrl_thread_start_cal(bool two_pass);
int ctrl_thread_save_cal(void);
int ctrl_thread_load_cal(void);
int ctrl_thread_get_status(ctrl_thread_status_t *out);
int ctrl_thread_get_cfg(ctrl_thread_cfg_t *out);
int ctrl_thread_set_cfg(const ctrl_thread_cfg_t *cfg);

int ctrl_thread_measure(bool step, float initial, float final, float pilot, unsigned wait,
			unsigned count, cli_write_fn_t out, void *ctx);
int ctrl_thread_adc(bool dump, unsigned count, cli_write_fn_t out, void *ctx);
int ctrl_thread_hw_info(cli_write_fn_t out, void *ctx);
void ctrl_thread_telemetry(bool enable);
