/* SPDX-License-Identifier: Apache-2.0 */
#include "ctrl_thread.h"
#include "experiment_output.h"

#include <ctrl_bias_engine.h>
#include <ctrl_mod_mzm.h>
#include <errno.h>
#include <math.h>

#define SCAN_INTERVALS 256u
#define DEFAULT_PILOT_V 0.1592f
#define BLOCK_DT ((float)DSP_GOERTZEL_BLOCK_SIZE / (float)DSP_SAMPLE_RATE_HZ)

static K_THREAD_STACK_DEFINE(ctrl_stack, 2560);
static struct k_thread worker;
static K_MUTEX_DEFINE(lock);
static K_SEM_DEFINE(wake, 0, 1);
static bool created, initialized, manual_mode, settle, auto_recal;
static const analog_board_ops_t *board;
static const cal_storage_ops_t *storage;
static struct bias_eeprom_descriptor_v1 desc;
static ctrl_thread_status_t status;
static ctrl_thread_cfg_t cfg_state;
static ctrl_mod_mzm_t mzm;
static ctrl_bias_scan_t scan;
static float lo, hi;
static uint16_t safe_code;
static uint32_t sequence, discard_rem, recal_streak;
static struct window {
	double i[2], q[2], dc;
	unsigned count;
} window;

static bool telemetry;
static K_SEM_DEFINE(measure_done, 0, 1);
static struct {
	bool active, step;
	float initial, final;
	uint16_t final_code;
	uint32_t initial_boundary, final_boundary, first, remaining;
	int result;
	cli_write_fn_t out;
	void *ctx;
} measurement;

static void reset_cfg(void)
{
	cfg_state = (ctrl_thread_cfg_t){
		.pilot_v = DEFAULT_PILOT_V,
		.gain = 0.20f,
		.kp = 0.0f,
		.ki = 0.85f,
		.rho_thresh = 0.25f,
		.hold_ticks = 15U,
		.wait_blocks = 3U,
		.avg_blocks = 1U,
		.use_gain = true,
		.recal_auto = false,
	};
}

static float compute_running_dt(void)
{
	return (float)(cfg_state.wait_blocks + cfg_state.avg_blocks) * BLOCK_DT;
}

static float compute_gain_ki(float gain, float vpi_v, float dt)
{
	return gain * vpi_v / ((float)(DSP_TAU / 2.0) * dt);
}

static void finish_measure(int rc)
{
	if (measurement.active) {
		measurement.result = rc;
		measurement.active = false;
		k_sem_give(&measure_done);
	}
}

static int idle(void)
{
	if (!initialized) {
		return -ENODEV;
	}
	if (status.state == CTRL_STATE_FAULT) {
		return status.fault;
	}
	return status.state == CTRL_STATE_IDLE ? 0 : -EBUSY;
}

static int set_bias(float v)
{
	uint16_t code;
	int rc = cal_service_voltage_to_dac_code(&desc, v, &code);
	if (!rc) {
		rc = rt_pipeline_set_dc_code(0, code);
	}
	if (!rc) {
		status.bias_v = v;
	}
	return rc;
}

static void record_cleanup(int rc)
{
	if (rc && !status.cleanup_error) {
		status.cleanup_error = rc;
	}
}

/* Even if one cutoff operation fails, attempt the other two. A latched RT
 * fault prevents further streaming and rejects rt_pipeline_set_dc_code, so
 * reset VA through the board after stopping sample production. */
static int shutdown(int error)
{
	status.locked = status.hold_assist_active = status.frozen = false;
	auto_recal = false;
	recal_streak = 0;
	discard_rem = 0;
	int rc = board->disconnect_all();
	record_cleanup(rc);
	rc = rt_pipeline_stop();
	if (rc != rt_pipeline_fault()) {
		record_cleanup(rc);
	}
	if (!error) {
		error = rc;
	}
	rc = board->set_dc_code(0, safe_code);
	record_cleanup(rc);
	if (!rc) {
		status.bias_v = CLAMP(0.f, lo, hi);
	}
	status.fault = error ? error : status.cleanup_error;
	status.state = status.fault ? CTRL_STATE_FAULT : CTRL_STATE_IDLE;
	finish_measure(status.fault ? status.fault : -ECANCELED);
	return status.fault;
}

static void pipeline_fault(int error, int cleanup_error)
{
	k_mutex_lock(&lock, K_FOREVER);
	record_cleanup(cleanup_error);
	if (status.state != CTRL_STATE_FAULT) {
		shutdown(error);
	}
	k_mutex_unlock(&lock);
}

static bool scanning(void)
{
	return status.state == CTRL_STATE_SCAN || status.state == CTRL_STATE_CAL;
}

static int start_stream(ctrl_state_t state, float voltage, float amplitude)
{
	int rc = set_bias(voltage);
	if (!rc) {
		rc = rt_pipeline_set_pilot_amplitude(amplitude);
	}
	if (!rc) {
		rc = board->set_route(0, 0, true);
	}
	if (!rc) {
		rc = manual_mode ? rt_pipeline_start_manual() : rt_pipeline_start();
	}
	if (rc) {
		return shutdown(rc);
	}
	status.state = state;
	status.updates = 0;
	sequence = 0;
	window = (struct window){0};
	k_sem_give(&wake);
	return 0;
}

static int start_running_locked(bool is_recal)
{
	ctrl_mzm_config_t *cfg = &mzm.config;
	const ctrl_calibration_t *cal = &cfg->calibration;
	float dt = compute_running_dt();

	cfg->min_v = lo;
	cfg->max_v = hi;
	cfg->dt = dt;
	cfg->pilot_v = cfg_state.pilot_v;
	if (cfg_state.use_gain) {
		cfg->kp = 0.0f;
		cfg->ki = compute_gain_ki(cfg_state.gain, cal->vpi_v, dt);
	} else {
		cfg->kp = cfg_state.kp;
		cfg->ki = cfg_state.ki;
	}

	double degrees = status.custom_deg;
	switch (status.target) {
	case BIAS_POINT_QUAD: degrees = 90; break;
	case BIAS_POINT_MIN: degrees = 0; break;
	case BIAS_POINT_MAX: degrees = 180; break;
	default: break;
	}
	double phase = remainder(degrees, 360.) * (DSP_TAU / 360.);
	double seed = (double)cal->null_v + (double)cal->vpi_v * phase / (DSP_TAU / 2);
	double period = 2. * (double)cal->vpi_v;
	double mid = ((double)lo + (double)hi) / 2;
	seed = mid + remainder(seed - mid, period);
	float sweep[CTRL_DC_CHANNELS] = {(float)seed};
	ctrl_command_t cmd;
	if (sweep[0] < lo || sweep[0] > hi) {
		return -ERANGE;
	}
	if (ctrl_mzm_strategy.start(&mzm, status.target, (float)phase, sweep, &cmd) != DSP_OK) {
		return shutdown(-EDOM);
	}

	discard_rem = cfg_state.wait_blocks;
	recal_streak = 0;
	window = (struct window){0};
	status.locked = false;
	status.hold_assist_active = false;
	status.frozen = false;

	if (!is_recal) {
		status.recal_count = 0;
		auto_recal = false;
		return start_stream(CTRL_STATE_RUNNING, cmd.bias_v[0], cfg->pilot_v);
	}
	status.state = CTRL_STATE_RUNNING;
	return set_bias(cmd.bias_v[0]);
}

static int scan_window(const harmonic_data_t *h)
{
	if (settle) {
		settle = false;
		return 0;
	}
	if (ctrl_bias_scan_add(&scan, status.bias_v, h) != DSP_OK) {
		return -EDOM;
	}
	if (status.scan_step < SCAN_INTERVALS) {
		status.scan_step++;
	} else if (status.state == CTRL_STATE_SCAN) {
		if (dsp_vpi_result(&scan.tracker, &status.scan_vpi_v) != DSP_OK) {
			return -ENODATA;
		}
		return shutdown(0);
	} else if (!scan.fitting) {
		/* Negative signed H2 quadrature identifies the dark null on 20261110-MZM
		 * (U12A inverting TIA plus 2 kHz analog/ADC phase lag) when CH1 DC is absent.
		 */
		if (ctrl_bias_scan_begin_fit(&scan, -1) != DSP_OK) {
			return -EDOM;
		}
		status.scan_vpi_v = scan.vpi;
		status.scan_step = 0;
		status.scan_pass = 2;
	} else {
		if (ctrl_bias_scan_finish(&scan, cfg_state.pilot_v, &mzm.config.calibration) != DSP_OK) {
			return -EDOM;
		}
		status.calibrated = true;
		status.vpi_v = mzm.config.calibration.vpi_v;
		status.null_v = mzm.config.calibration.null_v;
		if (auto_recal) {
			auto_recal = false;
			status.recal_count++;
			return start_running_locked(true);
		}
		return shutdown(0);
	}
	settle = true;
	double fraction = (double)status.scan_step / SCAN_INTERVALS;
	return set_bias((double)lo + fraction * ((double)hi - (double)lo));
}

static int consume(const rt_pipeline_block_t *b)
{
	const harmonic_data_t *h = &b->harmonics;
	if (b->seq != sequence++) {
		return -EILSEQ;
	}
	if (status.state == CTRL_STATE_EXP_HOLD) {
		if (!measurement.active) {
			return 0;
		}
		if (b->seq == measurement.initial_boundary) {
			status.bias_v = measurement.initial;
		}
		if (b->seq == measurement.final_boundary) {
			status.bias_v = measurement.final;
		}
		if ((int32_t)(b->seq - measurement.first) >= 0) {
			int rc = cli_measure_row(b, measurement.step, status.bias_v,
						 measurement.final_code, measurement.out, measurement.ctx);
			if (rc) {
				return rc;
			}
			if (!--measurement.remaining) {
				finish_measure(0);
			}
		}
		return 0;
	}
	float amp = cfg_state.pilot_v;
	if (b->pilot_amplitude_v != amp) {
		return -EILSEQ;
	}

	if (scanning()) {
		window.i[0] += (double)b->iq[0];
		window.q[0] += (double)b->iq[1];
		window.i[1] += (double)b->iq[2];
		window.q[1] += (double)b->iq[3];
		window.dc += (double)h->dc_power;
		if (++window.count < DSP_CONTROL_DECIMATION) {
			return 0;
		}
		status.harmonics = (harmonic_data_t){
			.h1_magnitude = hypot(window.i[0], window.q[0]) / window.count,
			.h2_magnitude = hypot(window.i[1], window.q[1]) / window.count,
			.h1_phase = atan2(window.q[0], window.i[0]),
			.h2_phase = atan2(window.q[1], window.i[1]),
			.dc_power = window.dc / window.count,
		};
		window = (struct window){0};
		status.updates++;
		return scan_window(&status.harmonics);
	}

	if (discard_rem > 0U) {
		discard_rem--;
		return 0;
	}

	window.i[0] += (double)b->iq[0];
	window.q[0] += (double)b->iq[1];
	window.i[1] += (double)b->iq[2];
	window.q[1] += (double)b->iq[3];
	window.dc += (double)h->dc_power;
	if (++window.count < cfg_state.avg_blocks) {
		return 0;
	}
	status.harmonics = (harmonic_data_t){
		.h1_magnitude = hypot(window.i[0], window.q[0]) / window.count,
		.h2_magnitude = hypot(window.i[1], window.q[1]) / window.count,
		.h1_phase = atan2(window.q[0], window.i[0]),
		.h2_phase = atan2(window.q[1], window.i[1]),
		.dc_power = window.dc / window.count,
	};
	float iq[4] = {window.i[0] / window.count, window.q[0] / window.count,
		       window.i[1] / window.count, window.q[1] / window.count};
	window = (struct window){0};
	status.updates++;

	harmonic_data_t pilots[CTRL_PILOTS] = {status.harmonics};
	pilots[0].dc_power = 0;
	ctrl_command_t cmd;
	if (ctrl_mzm_strategy.update(&mzm, pilots, &cmd) != DSP_OK) {
		return -EDOM;
	}
	status.locked = cmd.locked;
	status.hold_assist_active = cmd.hold_assist_active;
	status.frozen = cmd.frozen;
	float measured_bias = status.bias_v;
	int rc = cmd.frozen ? 0 : set_bias(cmd.bias_v[0]);
	if (!rc && telemetry) {
		ctrl_thread_status_t sample = status;
		sample.bias_v = measured_bias;
		rc = cli_tel_row(&sample, b, mzm.observer.phase_rad, mzm.observer.error,
				 mzm.observer.radius, iq);
	}
	if (!rc) {
		if (!cmd.frozen && cfg_state.recal_auto && cmd.rho_ema > cfg_state.rho_thresh) {
			recal_streak++;
		} else {
			recal_streak = 0;
		}
		if (recal_streak >= cfg_state.hold_ticks) {
			recal_streak = 0;
			auto_recal = true;
			scan = (ctrl_bias_scan_t){0};
			status.scan_pass = 1;
			status.scan_step = 0;
			status.scan_vpi_v = 0;
			status.locked = false;
			status.hold_assist_active = false;
			status.frozen = false;
			settle = true;
			status.state = CTRL_STATE_CAL;
			return set_bias(lo);
		}
		discard_rem = cfg_state.wait_blocks;
	}
	return rc;
}

static void run(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&wake, K_FOREVER);
		for (;;) {
			k_mutex_lock(&lock, K_FOREVER);
			if (!scanning() && status.state != CTRL_STATE_RUNNING &&
			    status.state != CTRL_STATE_EXP_HOLD) {
				k_mutex_unlock(&lock);
				break;
			}
			/* Serialize dequeue with start/stop so a previous stream's
			 * final block cannot escape into the next scan. */
			rt_pipeline_block_t block;
			bool holding = status.state == CTRL_STATE_EXP_HOLD;
			k_timeout_t timeout = holding && manual_mode ? K_SECONDS(1) : K_MSEC(25);
			int rc = rt_pipeline_get_block(&block, timeout);
			int fault = rt_pipeline_fault();
			if (fault) {
				rc = fault;
			} else if (rc == -EAGAIN) {
				rc = holding ? -ETIMEDOUT : 0;
			} else if (!rc) {
				rc = consume(&block);
			}
			if (rc && status.state != CTRL_STATE_FAULT) {
				shutdown(rc);
			}
			k_mutex_unlock(&lock);
		}
	}
}

int ctrl_thread_init(const analog_board_ops_t *ops, const cal_storage_ops_t *store, bool manual)
{
	if (!ops) {
		ops = analog_board_get_ops();
	}
	if (!ops || !ops->load_calibration || !ops->store_calibration || !ops->disconnect_all ||
	    !ops->set_dc_code || !ops->set_route || (store && (!store->read || !store->write_page))) {
		return -EINVAL;
	}
	k_mutex_lock(&lock, K_FOREVER);
	int rc = -EBUSY;
	if (initialized && status.state != CTRL_STATE_IDLE && status.state != CTRL_STATE_FAULT) {
		goto out;
	}
	rc = rt_pipeline_init(ops, pipeline_fault);
	if (rc) {
		goto out;
	}
	initialized = false;
	board = ops;
	storage = store;
	manual_mode = manual;
	reset_cfg();
	status = (ctrl_thread_status_t){.target = BIAS_POINT_QUAD};
	int load_error = cal_service_load(board, storage, &desc, &mzm.config.calibration);
	/* On first calibration U21 can contain a valid board descriptor and
	 * an erased affine record. Verify the descriptor before enabling VA. */
	rc = load_error == -EBADMSG ? cal_service_load_descriptor(board, &desc) : load_error;
	if (!rc) {
		rc = cal_service_voltage_bounds(&desc, &lo, &hi);
	}
	if (!rc) {
		rc = cal_service_voltage_to_dac_code(&desc, CLAMP(0.f, lo, hi), &safe_code);
	}
	if (rc) {
		status.load_error = rc;
		goto out;
	}
	rc = shutdown(0);
	if (rc) {
		goto out;
	}
	rc = load_error;
	status.load_error = rc;
	status.calibrated = !rc;
	if (!rc) {
		status.vpi_v = mzm.config.calibration.vpi_v;
		status.null_v = mzm.config.calibration.null_v;
	}
	initialized = true;
	if (!created) {
		k_thread_create(&worker, ctrl_stack, K_THREAD_STACK_SIZEOF(ctrl_stack), run,
				NULL, NULL, NULL, 5, 0, K_NO_WAIT);
		created = true;
	}
out:
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_start_cal(bool two_pass)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		auto_recal = false;
		scan = (ctrl_bias_scan_t){0};
		status.scan_pass = 1;
		status.scan_step = 0;
		status.scan_vpi_v = 0;
		settle = true;
		rc = start_stream(two_pass ? CTRL_STATE_CAL : CTRL_STATE_SCAN, lo, cfg_state.pilot_v);
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_start(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (rc) {
		goto out;
	}
	if (!status.calibrated) {
		rc = -ENODATA;
		goto out;
	}
	rc = start_running_locked(false);
out:
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_stop(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (rc == -EBUSY && status.state != CTRL_STATE_FAULT) {
		rc = shutdown(0);
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_set_target(bias_point_t point, float custom_deg)
{
	if ((unsigned)point > BIAS_POINT_CUSTOM || !isfinite(custom_deg)) {
		return -EINVAL;
	}
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		status.target = point;
		status.custom_deg = custom_deg;
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_save_cal(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		rc = status.calibrated ? cal_service_save(board, storage, &desc,
							  &mzm.config.calibration) : -ENODATA;
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_load_cal(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		struct bias_eeprom_descriptor_v1 next;
		ctrl_calibration_t cal;
		rc = cal_service_load(board, storage, &next, &cal);
		if (!rc) {
			desc = next;
			mzm.config.calibration = cal;
			cal_service_voltage_bounds(&desc, &lo, &hi);
			cal_service_voltage_to_dac_code(&desc, CLAMP(0.f, lo, hi), &safe_code);
			status.calibrated = true;
			status.vpi_v = cal.vpi_v;
			status.null_v = cal.null_v;
		}
		status.load_error = rc;
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_get_status(ctrl_thread_status_t *out)
{
	if (!out) {
		return -EINVAL;
	}
	k_mutex_lock(&lock, K_FOREVER);
	*out = status;
	int rc = initialized ? 0 : -ENODEV;
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_get_cfg(ctrl_thread_cfg_t *out)
{
	if (!out) {
		return -EINVAL;
	}
	k_mutex_lock(&lock, K_FOREVER);
	*out = cfg_state;
	if (cfg_state.use_gain) {
		out->kp = 0.0f;
		out->ki = (status.calibrated && status.vpi_v > 0.0f)
			  ? compute_gain_ki(cfg_state.gain, status.vpi_v, compute_running_dt())
			  : 0.0f;
	}
	int rc = initialized ? 0 : -ENODEV;
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_set_cfg(const ctrl_thread_cfg_t *cfg)
{
	if (!cfg) {
		return -EINVAL;
	}
	if (!isfinite(cfg->pilot_v) || cfg->pilot_v <= 0.0f || cfg->pilot_v > RT_PIPELINE_PILOT_MAX_V ||
	    cfg->wait_blocks > 25U || cfg->avg_blocks < 1U || cfg->avg_blocks > 16U ||
	    !isfinite(cfg->rho_thresh) || cfg->rho_thresh <= 0.0f || cfg->rho_thresh > 10.0f ||
	    cfg->hold_ticks < 1U || cfg->hold_ticks > 1000U) {
		return -ERANGE;
	}
	if (cfg->use_gain) {
		if (!isfinite(cfg->gain) || cfg->gain <= 0.0f || cfg->gain >= 2.0f) {
			return -ERANGE;
		}
	} else {
		if (!isfinite(cfg->kp) || cfg->kp < 0.0f || !isfinite(cfg->ki) || cfg->ki <= 0.0f) {
			return -ERANGE;
		}
	}

	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		cfg_state = *cfg;
	}
	k_mutex_unlock(&lock);
	return rc;
}

static K_MUTEX_DEFINE(measure_client);

int ctrl_thread_measure(bool step, float initial, float final, float pilot, unsigned wait,
			unsigned count, cli_write_fn_t out, void *ctx)
{
	if (k_mutex_lock(&measure_client, K_NO_WAIT)) {
		return -EBUSY;
	}
	k_mutex_lock(&lock, K_FOREVER);
	int rc = status.state == CTRL_STATE_EXP_HOLD ? 0 : idle();
	uint16_t first_code, final_code;
	if (!rc && (!isfinite(pilot) || pilot < 0 || pilot > RT_PIPELINE_PILOT_MAX_V ||
		    wait > 250 || !count || count > (step ? 200U : 2000U))) {
		rc = -ERANGE;
	}
	if (!rc) {
		rc = cal_service_voltage_to_dac_code(&desc, initial, &first_code);
	}
	if (!rc) {
		rc = cal_service_voltage_to_dac_code(&desc, final, &final_code);
	}
	if (!rc && status.state == CTRL_STATE_IDLE) {
		rc = start_stream(CTRL_STATE_EXP_HOLD, status.bias_v, 0.f);
	}
	if (!rc) {
		rc = rt_pipeline_schedule(first_code, pilot, step, final_code, &measurement.first);
		if (rc) {
			shutdown(rc);
		} else {
			measurement.final_boundary = measurement.first - 1;
			measurement.initial_boundary = measurement.final_boundary - (step ? 15U : 0U);
			measurement.initial = initial;
			measurement.first += wait;
			measurement.step = step;
			measurement.remaining = count;
			measurement.final = final;
			measurement.final_code = final_code;
			measurement.out = out;
			measurement.ctx = ctx;
			measurement.active = true;
			k_sem_reset(&measure_done);
		}
	}
	k_mutex_unlock(&lock);
	if (!rc) {
		k_sem_take(&measure_done, K_FOREVER);
		rc = measurement.result;
	}
	k_mutex_unlock(&measure_client);
	return rc;
}

void ctrl_thread_telemetry(bool enable)
{
	k_mutex_lock(&lock, K_FOREVER);
	telemetry = enable;
	k_mutex_unlock(&lock);
}

int ctrl_thread_adc(bool dump, unsigned count, cli_write_fn_t out, void *ctx)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc && (count < (dump ? 1U : 1000U) || count > (dump ? 16384U : 1000000U))) {
		rc = -ERANGE;
	}
	for (unsigned segment = 0; !rc && count; ++segment) {
		unsigned n = dump ? MIN(count, DSP_GOERTZEL_BLOCK_SIZE) : count;
		rt_adc_stats_t stats;
		rc = rt_pipeline_capture(dump, n, manual_mode, &stats);
		if (!rc && !dump && !stats.total) {
			rc = -EIO;
		}
		if (rc) {
			shutdown(rc);
			break;
		}
		cli_adc_result(dump, segment, n, &stats, out, ctx);
		count -= n;
	}
	k_mutex_unlock(&lock);
	return rc;
}

int ctrl_thread_hw_info(cli_write_fn_t out, void *ctx)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = board && status.state == CTRL_STATE_IDLE ? 0 : idle();
	uint16_t regs[4];
	if (!rc) {
		rc = rt_pipeline_read_regs(regs);
	}
	if (!rc) {
		struct bias_eeprom_descriptor_v1 d = {0};
		ctrl_calibration_t cal;
		uint32_t crc = 0;
		int cal_rc = cal_service_inspect(board, storage, &d, &cal, &crc);
		cli_hw_result(regs, d.crc32, crc, cal_rc, out, ctx);
		rc = cal_rc;
		if ((regs[0] >> 8) != 0x22 || regs[2] != 0x1111 || regs[3] != 0x0322) {
			rc = -EIO;
		}
	}
	k_mutex_unlock(&lock);
	return rc;
}
