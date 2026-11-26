/* SPDX-License-Identifier: Apache-2.0 */
#include "cal_service.h"
#include "cli_service.h"
#include "ctrl_thread.h"
#include "mocks.h"
#include "rt_pipeline.h"

#include <ctrl_mod_mzm.h>
#include <dsp_goertzel.h>
#include <dsp_pilot_gen.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

extern struct k_msgq rt_pipeline_blocks;

#define TEST_PI (DSP_TAU / 2.0)
#define TEST_OFFSET_UV (-5000000)
#define TEST_GAIN_PPM 4400000
#define PLANT_VPI 3.7
#define PLANT_DARK_V (-0.31)
#define SCAN_PILOT_V 0.13f

static atomic_t rt_fault_cb_count;
static int rt_fault_cb_err;
static int rt_fault_cb_cleanup;

static void test_rt_fault_cb(int error, int cleanup_error)
{
	rt_fault_cb_err = error;
	rt_fault_cb_cleanup = cleanup_error;
	atomic_inc(&rt_fault_cb_count);
}

/* =========================================================================
 * Analytical MZM plant + bilinear photoreceiver matching tests/host/sim/test_mzm.c
 * ========================================================================= */

static double plant_power = 1.0;
static double plant_drift_v;

static double bessel_jn(int n, double x)
{
	double term = 1.0;

	for (int k = 1; k <= n; ++k) {
		term *= (x * 0.5) / (double)k;
	}
	double sum = term;
	double x2 = -0.25 * x * x;

	for (int k = 1; k <= 12; ++k) {
		term *= x2 / ((double)k * (double)(k + n));
		sum += term;
	}
	return sum;
}

static double current_bias_volts(void)
{
	return ((double)TEST_OFFSET_UV +
		((double)mock_state.dc_code[0] / 65535.0) * 2.5 * (double)TEST_GAIN_PPM) *
	       1e-6;
}

static double plant_phase_rad(double bias_v)
{
	return TEST_PI * (bias_v - PLANT_DARK_V - plant_drift_v) / PLANT_VPI;
}

static void apply_bilinear_rx(double omega, double k_pole,
			      double in_re, double in_im,
			      double *out_re, double *out_im)
{
	/* H(e^{j*omega}) = 1 / (1 + j * tan(omega/2) / k_pole) */
	double t = tan(omega * 0.5) / k_pole;
	double denom = 1.0 + t * t;
	double h_re = 1.0 / denom;
	double h_im = -t / denom;

	*out_re = in_re * h_re - in_im * h_im;
	*out_im = in_re * h_im + in_im * h_re;
}

static rt_pipeline_block_t make_plant_block(uint32_t seq, float pilot_v)
{
	double bias_v = current_bias_volts();
	double phi = plant_phase_rad(bias_v);
	double m = TEST_PI * (double)pilot_v / PLANT_VPI;
	double j0 = bessel_jn(0, m);
	double j1 = bessel_jn(1, m);
	double j2 = bessel_jn(2, m);

	double h1_light_re = 0.00004;
	double h1_light_im = plant_power * j1 * sin(phi) +
			     0.003 * plant_power * (double)pilot_v *
			     (1.0 - cos(phi) * (j0 - j2));
	double h2_light_re = plant_power * j2 * cos(phi);
	double h2_light_im = -0.00002;

	double k_pole = tan(TEST_PI / 64.0) / tan(82.0 * TEST_PI / 180.0);
	double x1_re, x1_im, x2_re, x2_im;

	apply_bilinear_rx(TEST_PI / 32.0, k_pole, h1_light_re, h1_light_im, &x1_re, &x1_im);
	apply_bilinear_rx(TEST_PI / 16.0, k_pole, h2_light_re, h2_light_im, &x2_re, &x2_im);

	return (rt_pipeline_block_t){
		.harmonics = {
			.h1_magnitude = (float)hypot(x1_re, x1_im),
			.h2_magnitude = (float)hypot(x2_re, x2_im),
			.h1_phase = (float)atan2(x1_im, x1_re),
			.h2_phase = (float)atan2(x2_im, x2_re),
			.dc_power = 0.0f,
		},
		.seq = seq,
		.timestamp_cycles = seq,
		.pilot_amplitude_v = pilot_v,
	};
}

/* =========================================================================
 * Feeder thread for ctrl_thread / cli_service closed-loop & cal tests
 * ========================================================================= */

static K_THREAD_STACK_DEFINE(feeder_stack, 1536);
static struct k_thread feeder_thread;
static K_SEM_DEFINE(feeder_wake, 0, 1);
static K_SEM_DEFINE(feeder_parked, 0, 1);
static bool feeder_created;
static atomic_t feeder_active;
static uint32_t feeder_seq;
static float feeder_run_pilot_v;

static void feeder_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		k_sem_take(&feeder_wake, K_FOREVER);

		uint16_t cached_dc = UINT16_MAX;
		float cached_pilot = -1.0f;
		double cached_power = -1.0;
		double cached_drift = 1e9;
		rt_pipeline_block_t cached_blk = { 0 };
		bool have_cache = false;

		while (atomic_get(&feeder_active) != 0) {
			if (rt_pipeline_fault() != 0) {
				break;
			}
			if (k_msgq_num_used_get(&rt_pipeline_blocks) == 0U) {
				uint16_t dc = mock_state.dc_code[0];

				if (!have_cache || dc != cached_dc ||
				    feeder_run_pilot_v != cached_pilot ||
				    plant_power != cached_power ||
				    plant_drift_v != cached_drift) {
					cached_blk = make_plant_block(0U, feeder_run_pilot_v);
					cached_dc = dc;
					cached_pilot = feeder_run_pilot_v;
					cached_power = plant_power;
					cached_drift = plant_drift_v;
					have_cache = true;
				}
				rt_pipeline_block_t blk = cached_blk;

				blk.seq = feeder_seq;
				blk.timestamp_cycles = feeder_seq;

				if (k_msgq_put(&rt_pipeline_blocks, &blk, K_NO_WAIT) == 0) {
					feeder_seq++;
					if ((blk.seq % DSP_CONTROL_DECIMATION) ==
					    (DSP_CONTROL_DECIMATION - 1U)) {
						int rc = rt_pipeline_ingest(0, 0, 0, 0);

						if (rc == -ESHUTDOWN || rc < 0) {
							break;
						}
					}
				}
			} else {
				k_msleep(1);
			}
		}
		atomic_clear(&feeder_active);
		k_sem_give(&feeder_parked);
	}
}

static void start_feeder(float run_pilot_v)
{
	if (!feeder_created) {
		k_thread_create(&feeder_thread, feeder_stack,
				K_THREAD_STACK_SIZEOF(feeder_stack),
				feeder_entry, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
		feeder_created = true;
	}
	k_sem_reset(&feeder_parked);
	feeder_seq = 0;
	feeder_run_pilot_v = run_pilot_v;
	atomic_set(&feeder_active, 1);
	k_sem_give(&feeder_wake);
}

static void stop_feeder(void)
{
	if (atomic_cas(&feeder_active, 1, 0)) {
		(void)k_sem_take(&feeder_parked, K_MSEC(500));
	}
}

static void wait_ctrl_idle_or_fault(ctrl_thread_status_t *out, int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	for (;;) {
		zassert_ok(ctrl_thread_get_status(out));
		if (out->state == CTRL_STATE_IDLE || out->state == CTRL_STATE_FAULT) {
			return;
		}
		zassert_true(k_uptime_get() < deadline, "timed out waiting for ctrl_thread");
		k_msleep(20);
	}
}

/* =========================================================================
 * Suite 1: rt_pipeline
 * ========================================================================= */

static int32_t volts_to_adc_counts(float v)
{
	return (int32_t)(v / RT_PIPELINE_ADC_V_PER_COUNT);
}

ZTEST(p4_suite, test_rt_pipeline_handoff_and_pilot_boundary)
{
	mock_reset_all();
	atomic_clear(&rt_fault_cb_count);

	zassert_ok(rt_pipeline_init(&mock_board_ops, test_rt_fault_cb));
	zassert_equal(rt_pipeline_set_pilot_amplitude(0.30f), -EINVAL);
	zassert_ok(rt_pipeline_set_pilot_amplitude(0.10f));
	zassert_ok(rt_pipeline_start_manual());

	/* Block 0: 1 kHz (0.30 V, cos) + 2 kHz (0.15 V, sin) on CH0, +0.42 V on CH1.
	 * Update pilot amplitude mid-block and verify it applies only at block 1.
	 */
	for (uint32_t n = 0; n < DSP_GOERTZEL_BLOCK_SIZE; ++n) {
		double t = DSP_TAU * (double)n / (double)DSP_PILOT_PERIOD_SAMPLES;
		float v0 = 0.30f * (float)cos(t) + 0.15f * (float)sin(2.0 * t);
		float v1 = 0.42f;

		if (n == 500U) {
			zassert_ok(rt_pipeline_set_pilot_amplitude(0.20f));
		}
		zassert_ok(rt_pipeline_ingest(volts_to_adc_counts(v0),
					      volts_to_adc_counts(v1), 0, n));
	}

	rt_pipeline_block_t blk0;

	zassert_ok(rt_pipeline_get_block(&blk0, K_NO_WAIT));
	zassert_equal(blk0.seq, 0U);
	zassert_equal(blk0.timestamp_cycles, DSP_GOERTZEL_BLOCK_SIZE - 1U);
	zassert_within(blk0.pilot_amplitude_v, 0.10f, 1e-6f);
	zassert_within(blk0.harmonics.h1_magnitude, 0.30f, 1e-3f);
	zassert_within(blk0.harmonics.h1_phase, 0.0f, 1e-2f);
	zassert_within(blk0.harmonics.h2_magnitude, 0.15f, 1e-3f);
	zassert_within(blk0.harmonics.h2_phase, (float)(-TEST_PI / 2.0), 1e-2f);
	zassert_within(blk0.harmonics.dc_power, 0.42f, 1e-3f);

	/* Block 1 should pick up 0.20 V pilot cleanly at sample 0. */
	for (uint32_t n = 0; n < DSP_GOERTZEL_BLOCK_SIZE; ++n) {
		zassert_ok(rt_pipeline_ingest(volts_to_adc_counts(0.10f),
					      volts_to_adc_counts(-0.20f), 0, 10000U + n));
	}
	rt_pipeline_block_t blk1;

	zassert_ok(rt_pipeline_get_block(&blk1, K_NO_WAIT));
	zassert_equal(blk1.seq, 1U);
	zassert_within(blk1.pilot_amplitude_v, 0.20f, 1e-6f);
	zassert_within(blk1.harmonics.dc_power, -0.20f, 1e-3f);

	/* Peak DAC code for 0.20 V pilot at sample 16 (sin = +1) */
	zassert_true(mock_state.pilot_code[0] != 32768U);

	zassert_ok(rt_pipeline_stop());
	zassert_ok(rt_pipeline_set_dc_code(0, 12345U));
	zassert_equal(mock_state.dc_code[0], 12345U);
	zassert_equal(atomic_get(&rt_fault_cb_count), 0);
}

ZTEST(p4_suite, test_rt_pipeline_overflow_and_crc_faults)
{
	mock_reset_all();
	atomic_clear(&rt_fault_cb_count);

	zassert_ok(rt_pipeline_init(&mock_board_ops, test_rt_fault_cb));
	zassert_ok(rt_pipeline_start_manual());

	/* Fill k_msgq (capacity DSP_CONTROL_DECIMATION = 10 blocks) */
	for (uint32_t b = 0; b < DSP_CONTROL_DECIMATION; ++b) {
		for (uint32_t n = 0; n < DSP_GOERTZEL_BLOCK_SIZE; ++n) {
			zassert_ok(rt_pipeline_ingest(0, 0, 0, b * DSP_GOERTZEL_BLOCK_SIZE + n));
		}
	}
	/* 11th block must overflow k_msgq, increment drop_count, and trip FAULT */
	int rc = 0;

	for (uint32_t n = 0; n < DSP_GOERTZEL_BLOCK_SIZE; ++n) {
		rc = rt_pipeline_ingest(0, 0, 0, n);
	}
	zassert_equal(rc, -EOVERFLOW);
	zassert_equal(rt_pipeline_fault(), -EOVERFLOW);
	zassert_equal(rt_pipeline_drop_count(), 1U);
	zassert_equal(rt_pipeline_crc_error_count(), 0U);

	k_msleep(10);
	zassert_equal(atomic_get(&rt_fault_cb_count), 1);
	zassert_equal(rt_fault_cb_err, -EOVERFLOW);
	zassert_true(mock_state.disconnect_count >= 1U);

	/* Reinitialize and verify CRC error (-EBADMSG) trip */
	atomic_clear(&rt_fault_cb_count);
	mock_state.disconnect_count = 0;
	zassert_ok(rt_pipeline_init(&mock_board_ops, test_rt_fault_cb));
	zassert_ok(rt_pipeline_start_manual());

	rc = rt_pipeline_ingest(0, 0, -EBADMSG, 42U);
	zassert_equal(rc, -EBADMSG);
	zassert_equal(rt_pipeline_fault(), -EBADMSG);
	zassert_equal(rt_pipeline_crc_error_count(), 1U);
	zassert_equal(rt_pipeline_drop_count(), 1U);

	k_msleep(10);
	zassert_equal(atomic_get(&rt_fault_cb_count), 1);
	zassert_equal(rt_fault_cb_err, -EBADMSG);
	zassert_true(mock_state.disconnect_count >= 1U);
	zassert_equal(rt_pipeline_stop(), -EBADMSG);
}

/* =========================================================================
 * Suite 2: cal_service
 * ========================================================================= */

ZTEST(p4_suite, test_cal_service_roundtrip_crc_and_bounds)
{
	mock_reset_all();
	mock_install_mzm_descriptor(TEST_OFFSET_UV, TEST_GAIN_PPM, 5000000U);

	struct bias_eeprom_descriptor_v1 desc;

	zassert_ok(cal_service_load_descriptor(&mock_board_ops, &desc));

	float lo = 0.0f, hi = 0.0f;

	zassert_ok(cal_service_voltage_bounds(&desc, &lo, &hi));
	zassert_within(lo, -5.0f, 1e-4f);
	zassert_within(hi, 6.0f, 1e-4f);

	uint16_t code = 0;

	zassert_ok(cal_service_voltage_to_dac_code(&desc, -5.0f, &code));
	zassert_equal(code, 0U);
	zassert_ok(cal_service_voltage_to_dac_code(&desc, 6.0f, &code));
	zassert_equal(code, 65535U);
	zassert_equal(cal_service_voltage_to_dac_code(&desc, -5.1f, &code), -ERANGE);
	zassert_equal(cal_service_voltage_to_dac_code(&desc, 6.1f, &code), -ERANGE);
	zassert_equal(cal_service_voltage_to_dac_code(&desc, NAN, &code), -EINVAL);
	zassert_equal(cal_service_voltage_to_dac_code(&desc, INFINITY, &code), -EINVAL);

	struct bias_eeprom_descriptor_v1 bad_desc = desc;

	bad_desc.channels[0].subtractor_gain_ppm = 0;
	zassert_equal(cal_service_voltage_bounds(&bad_desc, &lo, &hi), -EINVAL);
	zassert_equal(cal_service_voltage_to_dac_code(&bad_desc, 0.0f, &code), -EINVAL);

	/* 7-page save/load round-trip (56 B record -> 7 * 8 B pages) */
	const ctrl_calibration_t cal_in = {
		.fit = {
			.o = { 0.001f, -0.002f },
			.m = { { 0.12f, -0.03f }, { 0.04f, 0.11f } },
		},
		.null_v = -0.31f,
		.vpi_v = 3.70f,
		.cal_pilot_v = 0.13f,
		.has_obs_dc_seed = true,
		.obs_dc_seed = 0.25f,
	};

	mock_state.write_page_count = 0;
	zassert_ok(cal_service_save(&mock_board_ops, &mock_storage_ops, &desc, &cal_in));
	zassert_equal(mock_state.write_page_count, 7U);
	zassert_equal(desc.channels[0].vpi_uv, 3700000U);

	struct bias_eeprom_descriptor_v1 desc_out;
	ctrl_calibration_t cal_out;

	zassert_ok(cal_service_load(&mock_board_ops, &mock_storage_ops, &desc_out, &cal_out));
	zassert_within(cal_out.null_v, cal_in.null_v, 1e-6f);
	zassert_within(cal_out.vpi_v, cal_in.vpi_v, 1e-6f);
	zassert_within(cal_out.cal_pilot_v, cal_in.cal_pilot_v, 1e-6f);
	zassert_equal(cal_out.has_obs_dc_seed, cal_in.has_obs_dc_seed);
	zassert_within(cal_out.obs_dc_seed, cal_in.obs_dc_seed, 1e-6f);
	zassert_within(cal_out.fit.m[0][0], cal_in.fit.m[0][0], 1e-6f);

	/* Corrupt descriptor byte -> -EBADMSG */
	uint8_t saved = mock_state.eeprom[30];

	mock_state.eeprom[30] ^= 0x5AU;
	zassert_equal(cal_service_load(&mock_board_ops, &mock_storage_ops,
				       &desc_out, &cal_out), -EBADMSG);
	mock_state.eeprom[30] = saved;

	/* Corrupt cal_mzm_record_v1 byte -> -EBADMSG */
	saved = mock_state.eeprom[170];
	mock_state.eeprom[170] ^= 0xA5U;
	zassert_equal(cal_service_load(&mock_board_ops, &mock_storage_ops,
				       &desc_out, &cal_out), -EBADMSG);
	mock_state.eeprom[170] = saved;

	/* Re-verify clean load */
	zassert_ok(cal_service_load(&mock_board_ops, &mock_storage_ops, &desc_out, &cal_out));
}

/* =========================================================================
 * Suite 3: ctrl_thread
 * ========================================================================= */

ZTEST(p4_suite, test_ctrl_thread_cal_persist_lock_and_fault)
{
	mock_reset_all();
	plant_power = 1.0;
	plant_drift_v = 0.0;
	mock_install_mzm_descriptor(TEST_OFFSET_UV, TEST_GAIN_PPM, 5000000U);

	/* Valid descriptor + erased affine record -> returns -EBADMSG, remains initialized */
	zassert_equal(ctrl_thread_init(&mock_board_ops, &mock_storage_ops, true), -EBADMSG);

	ctrl_thread_status_t st;

	zassert_ok(ctrl_thread_get_status(&st));
	zassert_equal(st.state, CTRL_STATE_IDLE,
		      "scan vpi state=%d fault=%d cln=%d step=%u upd=%u vpi_mv=%d",
		      (int)st.state, st.fault, st.cleanup_error,
		      (unsigned int)st.scan_step, (unsigned int)st.updates,
		      (int)(st.scan_vpi_v * 1000.0f));
	zassert_false(st.calibrated);
	zassert_equal(st.load_error, -EBADMSG);

	/* Pass 1 only: scan vpi */
	zassert_ok(ctrl_thread_start_cal(false));
	start_feeder(SCAN_PILOT_V);
	wait_ctrl_idle_or_fault(&st, 20000);
	stop_feeder();
	zassert_equal(st.state, CTRL_STATE_IDLE,
		      "scan vpi state=%d fault=%d cln=%d step=%u upd=%u vpi_mv=%d",
		      (int)st.state, st.fault, st.cleanup_error,
		      (unsigned int)st.scan_step, (unsigned int)st.updates,
		      (int)(st.scan_vpi_v * 1000.0f));
	zassert_false(st.calibrated);
	zassert_within(st.scan_vpi_v, (float)PLANT_VPI, 0.08f);

	/* Two-pass cal bias */
	zassert_ok(ctrl_thread_start_cal(true));
	start_feeder(SCAN_PILOT_V);
	wait_ctrl_idle_or_fault(&st, 20000);
	stop_feeder();
	zassert_equal(st.state, CTRL_STATE_IDLE,
		      "scan vpi state=%d fault=%d cln=%d step=%u upd=%u vpi_mv=%d",
		      (int)st.state, st.fault, st.cleanup_error,
		      (unsigned int)st.scan_step, (unsigned int)st.updates,
		      (int)(st.scan_vpi_v * 1000.0f));
	zassert_true(st.calibrated);
	zassert_within(st.vpi_v, (float)PLANT_VPI, 0.08f);
	zassert_within(st.null_v, (float)PLANT_DARK_V, 0.08f);

	/* Save -> reinit -> load */
	zassert_ok(ctrl_thread_save_cal());
	zassert_ok(ctrl_thread_init(&mock_board_ops, &mock_storage_ops, true));
	zassert_ok(ctrl_thread_get_status(&st));
	zassert_true(st.calibrated);
	zassert_equal(st.load_error, 0);
	zassert_within(st.vpi_v, (float)PLANT_VPI, 0.08f);
	zassert_within(st.null_v, (float)PLANT_DARK_V, 0.08f);
	zassert_ok(ctrl_thread_load_cal());

	/* Closed-loop locking across QUAD, MAX, MIN, CUSTOM */
	const struct {
		bias_point_t point;
		float custom_deg;
		double expected_deg;
	} targets[] = {
		{ BIAS_POINT_QUAD, 90.0f, 90.0 },
		{ BIAS_POINT_MAX, 180.0f, 180.0 },
		{ BIAS_POINT_MIN, 0.0f, 0.0 },
		{ BIAS_POINT_CUSTOM, 135.0f, 135.0 },
	};

	for (size_t i = 0; i < ARRAY_SIZE(targets); ++i) {
		zassert_ok(ctrl_thread_set_target(targets[i].point, targets[i].custom_deg));
		zassert_ok(ctrl_thread_start());
		start_feeder(SCAN_PILOT_V);

		int64_t deadline = k_uptime_get() + 5000;

		do {
			zassert_ok(ctrl_thread_get_status(&st));
			zassert_not_equal(st.state, CTRL_STATE_FAULT);
			if (st.locked && st.hold_assist_active) {
				break;
			}
			zassert_true(k_uptime_get() < deadline, "lock timeout on target %u",
				     (unsigned int)i);
			k_msleep(2);
		} while (true);

		stop_feeder();
		double locked_bias_v = st.bias_v;
		double phase_deg = remainder(plant_phase_rad(locked_bias_v) * (180.0 / TEST_PI) -
					     targets[i].expected_deg, 360.0);

		zassert_true(fabs(phase_deg) < 2.0,
			     "target %u phase error %d mdeg", (unsigned int)i,
			     (int)(phase_deg * 1000.0));

		zassert_ok(ctrl_thread_stop());
		zassert_ok(ctrl_thread_get_status(&st));
		zassert_equal(st.state, CTRL_STATE_IDLE,
		      "stop state=%d fault=%d cln=%d",
		      (int)st.state, st.fault, st.cleanup_error);
		zassert_within(st.bias_v, 0.0f, 1e-4f);
	}

	/* Fault cutoff + VA reset to safe 0 V */
	zassert_ok(ctrl_thread_set_target(BIAS_POINT_QUAD, 90.0f));
	zassert_ok(ctrl_thread_start());
	start_feeder(SCAN_PILOT_V);
	k_msleep(10);
	stop_feeder();

	mock_state.disconnect_count = 0;
	zassert_equal(rt_pipeline_ingest(0, 0, -EIO, 0), -EIO);
	wait_ctrl_idle_or_fault(&st, 2000);
	zassert_equal(st.state, CTRL_STATE_FAULT);
	zassert_equal(st.fault, -EIO);
	zassert_false(st.locked);
	zassert_false(st.hold_assist_active);
	zassert_true(mock_state.disconnect_count >= 1U);

	uint16_t zero_code = 0;
	struct bias_eeprom_descriptor_v1 desc;

	zassert_ok(cal_service_load_descriptor(&mock_board_ops, &desc));
	zassert_ok(cal_service_voltage_to_dac_code(&desc, 0.0f, &zero_code));
	zassert_equal(mock_state.dc_code[0], zero_code);
	zassert_equal(ctrl_thread_start(), -EIO);

	/* Explicit recovery */
	zassert_ok(ctrl_thread_init(&mock_board_ops, &mock_storage_ops, true));
	zassert_ok(ctrl_thread_get_status(&st));
	zassert_equal(st.state, CTRL_STATE_IDLE,
		      "scan vpi state=%d fault=%d cln=%d step=%u upd=%u vpi_mv=%d",
		      (int)st.state, st.fault, st.cleanup_error,
		      (unsigned int)st.scan_step, (unsigned int)st.updates,
		      (int)(st.scan_vpi_v * 1000.0f));
}

/* =========================================================================
 * Suite 4: cli_service
 * ========================================================================= */

struct cli_cap {
	char buf[1024];
	size_t len;
};

static void cli_capture_cb(const char *text, void *ctx)
{
	struct cli_cap *cap = ctx;
	size_t n = strlen(text);
	size_t avail = sizeof(cap->buf) - 1U - cap->len;

	if (n > avail) {
		n = avail;
	}
	memcpy(&cap->buf[cap->len], text, n);
	cap->len += n;
	cap->buf[cap->len] = '\0';
}

static int run_cli(const char *cmd, struct cli_cap *cap)
{
	cap->len = 0;
	cap->buf[0] = '\0';
	return cli_service_exec_line(cmd, cli_capture_cb, cap);
}

ZTEST(p4_suite, test_cli_service_commands)
{
	struct cli_cap cap;
	ctrl_thread_status_t st;

	mock_reset_all();
	plant_power = 1.0;
	plant_drift_v = 0.0;
	mock_install_mzm_descriptor(TEST_OFFSET_UV, TEST_GAIN_PPM, 5000000U);
	zassert_equal(ctrl_thread_init(&mock_board_ops, &mock_storage_ops, true), -EBADMSG);

	zassert_ok(run_cli("help", &cap));
	zassert_not_null(strstr(cap.buf, "cal bias"));
	zassert_not_null(strstr(cap.buf, "eeprom init"));

	zassert_ok(run_cli("status", &cap));
	zassert_not_null(strstr(cap.buf, "state=IDLE"));
	zassert_not_null(strstr(cap.buf, "drops="));
	zassert_not_null(strstr(cap.buf, "crc_errs="));

	zassert_ok(run_cli("target quad", &cap));
	zassert_ok(run_cli("target min", &cap));
	zassert_ok(run_cli("target max", &cap));
	zassert_ok(run_cli("target custom 135", &cap));
	zassert_ok(run_cli("status", &cap));
	zassert_not_null(strstr(cap.buf, "target=CUSTOM(135.000 deg)"));

	zassert_equal(run_cli("target custom abc", &cap), -EINVAL);
	zassert_equal(run_cli("target custom 12.3.4", &cap), -EINVAL);
	zassert_equal(run_cli("target custom nan", &cap), -EINVAL);
	zassert_equal(run_cli("target quad extra", &cap), -EINVAL);
	zassert_equal(run_cli("foobar", &cap), -EINVAL);
	zassert_not_null(strstr(cap.buf, "Unknown command"));

	/* scan vpi */
	zassert_ok(run_cli("scan vpi", &cap));
	start_feeder(SCAN_PILOT_V);
	wait_ctrl_idle_or_fault(&st, 20000);
	stop_feeder();
	zassert_equal(st.state, CTRL_STATE_IDLE,
		      "scan vpi state=%d fault=%d cln=%d step=%u upd=%u vpi_mv=%d",
		      (int)st.state, st.fault, st.cleanup_error,
		      (unsigned int)st.scan_step, (unsigned int)st.updates,
		      (int)(st.scan_vpi_v * 1000.0f));

	/* cal bias + save + load */
	zassert_ok(run_cli("cal bias", &cap));
	start_feeder(SCAN_PILOT_V);
	wait_ctrl_idle_or_fault(&st, 20000);
	stop_feeder();
	zassert_true(st.calibrated);

	zassert_ok(run_cli("save", &cap));
	zassert_ok(run_cli("load", &cap));

	/* start + running status + stop */
	zassert_ok(run_cli("start", &cap));
	start_feeder(SCAN_PILOT_V);
	k_msleep(10);
	zassert_ok(run_cli("status", &cap));
	zassert_not_null(strstr(cap.buf, "state=RUNNING"));
	stop_feeder();
	zassert_ok(run_cli("stop", &cap));
	zassert_ok(run_cli("status", &cap));
	zassert_not_null(strstr(cap.buf, "state=IDLE"));

	/* eeprom init writes default 20261110-MZM descriptor */
	memset(mock_state.eeprom, 0xff, sizeof(mock_state.eeprom));
	zassert_ok(run_cli("eeprom init", &cap));
	zassert_not_null(strstr(cap.buf, "eeprom init: ok"));

	struct bias_eeprom_descriptor_v1 desc;
	float lo = 0.0f, hi = 0.0f;
	uint16_t zero_code = 0U;

	zassert_ok(cal_service_load_descriptor(&mock_board_ops, &desc));
	zassert_mem_equal(desc.board_id, "20261110-MZM", sizeof(desc.board_id));
	zassert_equal(desc.tia_gain_ohms, 47000U);
	zassert_equal(desc.channels[0].subtractor_gain_ppm, 8000000);
	zassert_equal(desc.channels[0].subtractor_offset_uv, -10000000);
	zassert_ok(cal_service_voltage_bounds(&desc, &lo, &hi));
	zassert_within(lo, -10.0f, 1e-4f);
	zassert_within(hi, 10.0f, 1e-4f);
	zassert_ok(cal_service_voltage_to_dac_code(&desc, 0.0f, &zero_code));
	zassert_equal(zero_code, 32768U);

	/* reinit invokes ctrl_thread_init(NULL, NULL, false); without DT bias_eeprom
	 * on qemu_cortex_m3, storage=NULL returns -ENODEV explicitly.
	 */
	mock_state.load_cal_count = 0;
	zassert_equal(run_cli("reinit", &cap), -ENODEV);
	zassert_true(mock_state.load_cal_count >= 1U);
	zassert_not_null(strstr(cap.buf, "reinit: err=-19"));
}

static void p4_after(void *fixture)
{
	ARG_UNUSED(fixture);
	stop_feeder();
	(void)ctrl_thread_stop();
	(void)rt_pipeline_stop();
}

ZTEST_SUITE(p4_suite, NULL, NULL, NULL, p4_after, NULL);
