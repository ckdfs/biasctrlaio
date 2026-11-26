/* SPDX-License-Identifier: Apache-2.0 */
#include "ctrl_thread.h"
#include "rt_pipeline.h"

#include <ctrl_bias_engine.h>
#include <ctrl_mod_mzm.h>
#include <errno.h>
#include <math.h>

#define SCAN_INTERVALS 256u
#define SCAN_PILOT_V 0.13f
#define CONTROL_DT ((float)(DSP_CONTROL_DECIMATION * DSP_GOERTZEL_BLOCK_SIZE) / DSP_SAMPLE_RATE_HZ)

static K_THREAD_STACK_DEFINE(ctrl_stack, 2560);
static struct k_thread worker;
static K_MUTEX_DEFINE(lock);
static K_SEM_DEFINE(wake, 0, 1);
static bool created, initialized, manual_mode, settle;
static const analog_board_ops_t *board;
static const cal_storage_ops_t *storage;
static struct bias_eeprom_descriptor_v1 desc;
static ctrl_thread_status_t status;
static ctrl_mod_mzm_t mzm;
static ctrl_bias_scan_t scan;
static float lo, hi;
static uint16_t safe_code;
static uint32_t sequence;
static struct window {
	double i[2], q[2], dc;
	unsigned count;
} window;

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
	status.locked = status.hold_assist_active = false;
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
		 * (U12A inverting TIA plus 2 kHz analog/ADC phase lag).
		 */
		if (ctrl_bias_scan_begin_fit(&scan, -1) != DSP_OK) {
			return -EDOM;
		}
		status.scan_vpi_v = scan.vpi;
		status.scan_step = 0;
		status.scan_pass = 2;
	} else {
		if (ctrl_bias_scan_finish(&scan, SCAN_PILOT_V, &mzm.config.calibration) != DSP_OK) {
			return -EDOM;
		}
		status.calibrated = true;
		status.vpi_v = mzm.config.calibration.vpi_v;
		status.null_v = mzm.config.calibration.null_v;
		return shutdown(0);
	}
	settle = true;
	double fraction = (double)status.scan_step / SCAN_INTERVALS;
	return set_bias((double)lo + fraction * ((double)hi - (double)lo));
}

static int consume(const rt_pipeline_block_t *b)
{
	const harmonic_data_t *h = &b->harmonics;
	float amp = scanning() ? SCAN_PILOT_V : mzm.config.pilot_v;
	if (b->seq != sequence++ || b->pilot_amplitude_v != amp) {
		return -EILSEQ;
	}
	window.i[0] += (double)h->h1_magnitude * cos(h->h1_phase);
	window.q[0] += (double)h->h1_magnitude * sin(h->h1_phase);
	window.i[1] += (double)h->h2_magnitude * cos(h->h2_phase);
	window.q[1] += (double)h->h2_magnitude * sin(h->h2_phase);
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
	if (scanning()) {
		return scan_window(&status.harmonics);
	}
	harmonic_data_t pilots[CTRL_PILOTS] = {status.harmonics};
	pilots[0].dc_power = 0;
	ctrl_command_t cmd;
	if (ctrl_mzm_strategy.update(&mzm, pilots, &cmd) != DSP_OK) {
		return -EDOM;
	}
	status.locked = cmd.locked;
	status.hold_assist_active = cmd.hold_assist_active;
	return set_bias(cmd.bias_v[0]);
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
			if (!scanning() && status.state != CTRL_STATE_RUNNING) {
				k_mutex_unlock(&lock);
				break;
			}
			/* Serialize dequeue with start/stop so a previous stream's
			 * final block cannot escape into the next scan. */
			rt_pipeline_block_t block;
			int rc = rt_pipeline_get_block(&block, K_MSEC(25));
			int fault = rt_pipeline_fault();
			if (fault) {
				rc = fault;
			} else if (rc == -EAGAIN) {
				rc = 0;
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

int ctrl_thread_start_cal(bool two_pass)
{
	k_mutex_lock(&lock, K_FOREVER);
	int rc = idle();
	if (!rc) {
		scan = (ctrl_bias_scan_t){0};
		status.scan_pass = 1;
		status.scan_step = 0;
		status.scan_vpi_v = 0;
		settle = true;
		rc = start_stream(two_pass ? CTRL_STATE_CAL : CTRL_STATE_SCAN, lo, SCAN_PILOT_V);
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
	ctrl_mzm_config_t *cfg = &mzm.config;
	const ctrl_calibration_t *cal = &cfg->calibration;
	cfg->min_v = lo;
	cfg->max_v = hi;
	cfg->kp = 0.25f;
	cfg->ki = 0.85f;
	cfg->dt = CONTROL_DT;
	cfg->pilot_v = cal->cal_pilot_v;
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
	float sweep[CTRL_DC_CHANNELS] = {seed};
	ctrl_command_t cmd;
	if (sweep[0] < lo || sweep[0] > hi) {
		rc = -ERANGE;
	} else if (ctrl_mzm_strategy.start(&mzm, status.target, phase, sweep, &cmd) != DSP_OK) {
		rc = shutdown(-EDOM);
	} else {
		rc = start_stream(CTRL_STATE_RUNNING, cmd.bias_v[0], cfg->pilot_v);
	}
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
