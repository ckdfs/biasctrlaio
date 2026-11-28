/* SPDX-License-Identifier: Apache-2.0 */
#include "cli_service.h"
#include "ctrl_thread.h"
#include "experiment_output.h"

#include <analog_board_mzm.h>
#include <bias/analog_board.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/version.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define CLI_THREAD_PRIO 8
#define CLI_STACK_SIZE 2048
#define CLI_RING_SIZE 128U
#define CLI_LINE_SIZE 96U
#define CLI_MAX_ARGS 6

static K_MUTEX_DEFINE(console_tx);
static atomic_t quiet_console;

static void default_write(const char *text, void *ctx)
{
	ARG_UNUSED(ctx);
	k_mutex_lock(&console_tx, K_FOREVER);
	printk("%s", text);
	k_mutex_unlock(&console_tx);
}

static void cli_printf(cli_write_fn_t out, void *ctx, const char *fmt, ...)
{
	char buf[160];
	va_list ap;

	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (n < 0) {
		return;
	}
	cli_write_fn_t writer = (out != NULL) ? out : default_write;
	writer(buf, ctx);
}

static int32_t float_to_mv(float v)
{
	if (!isfinite(v)) {
		return 0;
	}
	double scaled = (double)v * 1000.0;

	if (scaled >= (double)INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= (double)INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)(scaled >= 0.0 ? (scaled + 0.5) : (scaled - 0.5));
}

static void fmt_fixed3(char *buf, size_t sz, float val)
{
	if (!isfinite(val)) {
		(void)snprintf(buf, sz, "nan");
		return;
	}
	double scaled = (double)val * 1000.0;

	if (scaled > (double)INT32_MAX) {
		(void)snprintf(buf, sz, "ovf");
		return;
	}
	if (scaled < (double)(INT32_MIN + 1)) {
		(void)snprintf(buf, sz, "-ovf");
		return;
	}
	int32_t iv = (int32_t)(scaled >= 0.0 ? (scaled + 0.5) : (scaled - 0.5));
	bool neg = iv < 0;
	uint32_t mag = neg ? (uint32_t)(-iv) : (uint32_t)iv;

	(void)snprintf(buf, sz, "%s%u.%03u", neg ? "-" : "",
		       (unsigned int)(mag / 1000U), (unsigned int)(mag % 1000U));
}

static int parse_decimal(const char *s, float *out)
{
	if (s == NULL || *s == '\0' || out == NULL) {
		return -EINVAL;
	}
	bool neg = false;

	if (*s == '-') {
		neg = true;
		s++;
	} else if (*s == '+') {
		s++;
	}

	bool has_digit = false;
	bool seen_dot = false;
	double int_part = 0.0;
	double frac_part = 0.0;
	double frac_scale = 0.1;

	for (; *s != '\0'; s++) {
		unsigned char c = (unsigned char)*s;

		if (c >= '0' && c <= '9') {
			has_digit = true;
			double d = (double)(c - '0');

			if (!seen_dot) {
				int_part = int_part * 10.0 + d;
				if (int_part > 1e6) {
					return -ERANGE;
				}
			} else {
				frac_part += d * frac_scale;
				frac_scale *= 0.1;
			}
		} else if (c == '.' && !seen_dot) {
			seen_dot = true;
		} else {
			return -EINVAL;
		}
	}
	if (!has_digit) {
		return -EINVAL;
	}

	double val = int_part + frac_part;

	if (neg) {
		val = -val;
	}
	float f = (float)val;

	if (!isfinite(f)) {
		return -EINVAL;
	}
	*out = f;
	return 0;
}

static int parse_count(const char *str, unsigned *count)
{
	if (str == NULL || *str == '\0' || count == NULL) {
		return -EINVAL;
	}
	unsigned n = 0;

	for (; *str; ++str) {
		if (*str < '0' || *str > '9') {
			return -EINVAL;
		}
		n = n * 10 + (*str - '0');
		if (n > 1000000) {
			return -ERANGE;
		}
	}
	*count = n;
	return 0;
}

static const char *state_name(ctrl_state_t st)
{
	switch (st) {
	case CTRL_STATE_IDLE:
		return "IDLE";
	case CTRL_STATE_SCAN:
		return "SCAN";
	case CTRL_STATE_CAL:
		return "CAL";
	case CTRL_STATE_RUNNING:
		return "RUNNING";
	case CTRL_STATE_EXP_HOLD:
		return "EXP_HOLD";
	case CTRL_STATE_FAULT:
		return "FAULT";
	default:
		return "UNKNOWN";
	}
}

static const char *target_name(bias_point_t tgt)
{
	switch (tgt) {
	case BIAS_POINT_QUAD:
		return "QUAD";
	case BIAS_POINT_MAX:
		return "MAX";
	case BIAS_POINT_MIN:
		return "MIN";
	case BIAS_POINT_CUSTOM:
		return "CUSTOM";
	default:
		return "UNKNOWN";
	}
}

static void cmd_help(cli_write_fn_t out, void *ctx)
{
	cli_printf(out, ctx,
		   "Commands:\n"
		   "  help\n"
		   "  status\n"
		   "  cal bias\n"
		   "  scan vpi\n"
		   "  start\n"
		   "  stop\n"
		   "  target quad|min|max|custom <deg>\n");
	cli_printf(out, ctx,
		   "  ctrl cfg [<param> <val>]\n"
		   "  save\n"
		   "  load\n"
		   "  reinit\n"
		   "  eeprom init\n");
	cli_printf(out, ctx, "  hw info\n  adc stat|dump <n>\n"
		   "  meas <bias_mv> <pilot_mv> <wait_blocks> <n_blocks>\n"
		   "  step <v0_mv> <v1_mv> <pilot_mv> <n_blocks>\n  telemetry on|off\n");
}

static int cmd_status(cli_write_fn_t out, void *ctx)
{
	ctrl_thread_status_t st;
	int st_rc = ctrl_thread_get_status(&st);
	uint32_t drops = rt_pipeline_drop_count();
	uint32_t crc_errs = rt_pipeline_crc_error_count();
	int rt_flt = rt_pipeline_fault();
	char deg_buf[16];
	char h1m[16], h1p[16], h2m[16], h2p[16], dcm[16];

	fmt_fixed3(deg_buf, sizeof(deg_buf), st.custom_deg);
	fmt_fixed3(h1m, sizeof(h1m), st.harmonics.h1_magnitude * 1000.0f);
	fmt_fixed3(h1p, sizeof(h1p), st.harmonics.h1_phase);
	fmt_fixed3(h2m, sizeof(h2m), st.harmonics.h2_magnitude * 1000.0f);
	fmt_fixed3(h2p, sizeof(h2p), st.harmonics.h2_phase);
	fmt_fixed3(dcm, sizeof(dcm), st.harmonics.dc_power * 1000.0f);

	cli_printf(out, ctx,
		   "state=%s init=%d target=%s(%s deg) cal=%d locked=%d hold_assist=%d frozen=%d recal=%u\n",
		   state_name(st.state), st_rc == 0 ? 1 : 0, target_name(st.target), deg_buf,
		   st.calibrated ? 1 : 0, st.locked ? 1 : 0, st.hold_assist_active ? 1 : 0,
		   st.frozen ? 1 : 0, (unsigned int)st.recal_count);
	cli_printf(out, ctx,
		   "bias=%d mV scan_vpi=%d mV vpi=%d mV null=%d mV updates=%u pass=%u step=%u\n",
		   (int)float_to_mv(st.bias_v), (int)float_to_mv(st.scan_vpi_v),
		   (int)float_to_mv(st.vpi_v), (int)float_to_mv(st.null_v),
		   (unsigned int)st.updates, st.scan_pass, (unsigned int)st.scan_step);
	cli_printf(out, ctx,
		   "harmonics h1=%s mV@%s rad h2=%s mV@%s rad dc=%s mV\n",
		   h1m, h1p, h2m, h2p, dcm);
	cli_printf(out, ctx,
		   "drops=%u crc_errs=%u fault=%d cleanup_err=%d load_err=%d rt_fault=%d\n",
		   (unsigned int)drops, (unsigned int)crc_errs,
		   st.fault, st.cleanup_error, st.load_error, rt_flt);
	return 0;
}

static int cmd_ctrl_cfg(int argc, char **argv, cli_write_fn_t out, void *ctx)
{
	ctrl_thread_cfg_t cfg;
	int rc = ctrl_thread_get_cfg(&cfg);

	if (rc && rc != -ENODEV) {
		cli_printf(out, ctx, "ctrl cfg: err=%d\n", rc);
		return rc;
	}
	if (argc == 2) {
		char pilot[16], gain[16], kp[16], ki[16], rho[16];

		fmt_fixed3(pilot, sizeof(pilot), cfg.pilot_v * 1000.0f);
		fmt_fixed3(gain, sizeof(gain), cfg.gain);
		fmt_fixed3(kp, sizeof(kp), cfg.kp);
		fmt_fixed3(ki, sizeof(ki), cfg.ki);
		fmt_fixed3(rho, sizeof(rho), cfg.rho_thresh);
		cli_printf(out, ctx,
			   "pilot=%s mV wait=%u avg=%u mode=%s G=%s kp=%s ki=%s recal=%u rho_thresh=%s hold_ticks=%u\n",
			   pilot, (unsigned int)cfg.wait_blocks, (unsigned int)cfg.avg_blocks,
			   cfg.use_gain ? "gain" : "pi", gain, kp, ki,
			   cfg.recal_auto ? 1U : 0U, rho, (unsigned int)cfg.hold_ticks);
		return 0;
	}
	if (argc != 4) {
		cli_printf(out, ctx,
			   "Usage: ctrl cfg [<pilot|wait|avg|gain|kp|ki|recal|rho_thresh|hold_ticks> <val>]\n");
		return -EINVAL;
	}

	const char *key = argv[2];
	const char *val_str = argv[3];
	float fval = 0.0f;
	unsigned uval = 0U;

	if (strcmp(key, "pilot") == 0) {
		rc = parse_decimal(val_str, &fval);
		if (!rc) {
			cfg.pilot_v = fval / 1000.0f;
		}
	} else if (strcmp(key, "wait") == 0) {
		rc = parse_count(val_str, &uval);
		if (!rc) {
			if (uval > 25U) {
				rc = -ERANGE;
			} else {
				cfg.wait_blocks = (uint8_t)uval;
			}
		}
	} else if (strcmp(key, "avg") == 0) {
		rc = parse_count(val_str, &uval);
		if (!rc) {
			if (uval < 1U || uval > 16U) {
				rc = -ERANGE;
			} else {
				cfg.avg_blocks = (uint8_t)uval;
			}
		}
	} else if (strcmp(key, "gain") == 0) {
		rc = parse_decimal(val_str, &fval);
		if (!rc) {
			cfg.use_gain = true;
			cfg.gain = fval;
			cfg.kp = 0.0f;
		}
	} else if (strcmp(key, "kp") == 0) {
		rc = parse_decimal(val_str, &fval);
		if (!rc) {
			if (cfg.use_gain && cfg.ki <= 0.0f) {
				cfg.ki = 0.85f;
			}
			cfg.use_gain = false;
			cfg.kp = fval;
		}
	} else if (strcmp(key, "ki") == 0) {
		rc = parse_decimal(val_str, &fval);
		if (!rc) {
			cfg.use_gain = false;
			cfg.ki = fval;
		}
	} else if (strcmp(key, "recal") == 0) {
		rc = parse_count(val_str, &uval);
		if (!rc) {
			if (uval > 1U) {
				rc = -ERANGE;
			} else {
				cfg.recal_auto = (uval != 0U);
			}
		}
	} else if (strcmp(key, "rho_thresh") == 0) {
		rc = parse_decimal(val_str, &fval);
		if (!rc) {
			cfg.rho_thresh = fval;
		}
	} else if (strcmp(key, "hold_ticks") == 0) {
		rc = parse_count(val_str, &uval);
		if (!rc) {
			if (uval < 1U || uval > 1000U) {
				rc = -ERANGE;
			} else {
				cfg.hold_ticks = (uint16_t)uval;
			}
		}
	} else {
		cli_printf(out, ctx, "ctrl cfg: unknown param '%s'\n", key);
		return -EINVAL;
	}

	if (!rc) {
		rc = ctrl_thread_set_cfg(&cfg);
	}
	cli_printf(out, ctx, rc == 0 ? "ctrl cfg: ok\n" : "ctrl cfg: err=%d\n", rc);
	return rc;
}

static int cmd_target(int argc, char **argv, cli_write_fn_t out, void *ctx)
{
	int rc;

	if (argc == 2 && strcmp(argv[1], "quad") == 0) {
		rc = ctrl_thread_set_target(BIAS_POINT_QUAD, 90.0f);
	} else if (argc == 2 && strcmp(argv[1], "min") == 0) {
		rc = ctrl_thread_set_target(BIAS_POINT_MIN, 0.0f);
	} else if (argc == 2 && strcmp(argv[1], "max") == 0) {
		rc = ctrl_thread_set_target(BIAS_POINT_MAX, 180.0f);
	} else if (argc == 3 && strcmp(argv[1], "custom") == 0) {
		float deg = 0.0f;

		rc = parse_decimal(argv[2], &deg);
		if (rc < 0) {
			cli_printf(out, ctx, "target custom: invalid degree '%s' (err=%d)\n",
				   argv[2], rc);
			return rc;
		}
		rc = ctrl_thread_set_target(BIAS_POINT_CUSTOM, deg);
	} else {
		cli_printf(out, ctx, "Usage: target quad|min|max|custom <deg>\n");
		return -EINVAL;
	}

	if (rc == 0) {
		cli_printf(out, ctx, "target: ok\n");
	} else {
		cli_printf(out, ctx, "target: err=%d\n", rc);
	}
	return rc;
}

static int cmd_eeprom_init(cli_write_fn_t out, void *ctx)
{
	ctrl_thread_status_t st;

	if (ctrl_thread_get_status(&st) == 0 &&
	    (st.state == CTRL_STATE_SCAN || st.state == CTRL_STATE_CAL ||
	     st.state == CTRL_STATE_RUNNING || st.state == CTRL_STATE_EXP_HOLD)) {
		cli_printf(out, ctx, "eeprom init: busy (err=%d)\n", -EBUSY);
		return -EBUSY;
	}

	const analog_board_ops_t *ops = analog_board_get_ops();

	if (ops == NULL || ops->store_calibration == NULL) {
		cli_printf(out, ctx, "eeprom init: err=%d\n", -ENODEV);
		return -ENODEV;
	}

	struct bias_eeprom_descriptor_v1 desc;

	mzm_20261110_fill_default_descriptor(&desc);
	int rc = ops->store_calibration(&desc);

	if (rc == 0 && ops->load_calibration != NULL) {
		rc = ops->load_calibration(&desc);
	}
	if (rc == 0) {
		cli_printf(out, ctx, "eeprom init: ok crc=0x%08X\n",
			   (unsigned int)desc.crc32);
	} else {
		cli_printf(out, ctx, "eeprom init: err=%d\n", rc);
	}
	return rc;
}

/* The wire limit includes the CR inserted by the UART console before LF. */
static int send_row(char *row, int length, cli_write_fn_t out, void *ctx)
{
	if (length < 0 || length + 1 > 135) {
		return -EOVERFLOW;
	}
	(out ? out : default_write)(row, ctx);
	return 0;
}

int cli_measure_row(const rt_pipeline_block_t *b, bool step, float bias, uint16_t code,
		    cli_write_fn_t out, void *ctx)
{
	char row[135], values[7][16];
	const float volts[] = {bias, b->pilot_amplitude_v, b->harmonics.dc_power,
			       b->min_v[0], b->max_v[0], b->min_v[1], b->max_v[1]};
	for (unsigned i = 0; i < 3; ++i) {
		fmt_fixed3(values[i], sizeof(values[i]), volts[i] * 1000.f);
	}
	/* Extrema use 0.1 mV. This leaves room for both 32-bit clocks and
	 * full-scale signed I/Q even on the longest sample row. */
	for (unsigned i = 3; i < ARRAY_SIZE(volts); ++i) {
		int tenths = float_to_mv(volts[i] * 10.f);
		unsigned magnitude = tenths < 0 ? -tenths : tenths;
		snprintf(values[i], sizeof(values[i]), "%s%u.%u", tenths < 0 ? "-" : "",
			 magnitude / 10, magnitude % 10);
	}
	int n = snprintf(row, sizeof(row),
			 "%s,%u,%u,%s,%u,%s,%d,%d,%d,%d,%s,%s,%s,%s,%s,%u,%u\n",
			 step ? "STEP" : "MEAS", b->seq, b->t_ms, values[0], code, values[1],
			 float_to_mv(b->iq[0] * 1000.f), float_to_mv(b->iq[1] * 1000.f),
			 float_to_mv(b->iq[2] * 1000.f), float_to_mv(b->iq[3] * 1000.f),
			 values[2], values[3], values[4], values[5], values[6],
			 rt_pipeline_drop_count(), rt_pipeline_crc_error_count());
	return send_row(row, n, out, ctx);
}

int cli_tel_row(const ctrl_thread_status_t *st, const rt_pipeline_block_t *b,
		float phase, float error, float radius, const float iq[4])
{
	char row[135], bias[16], phi[16], err[16], r[16], dc[16];
	fmt_fixed3(bias, sizeof(bias), st->bias_v * 1000.f);
	fmt_fixed3(phi, sizeof(phi), phase);
	fmt_fixed3(err, sizeof(err), error);
	fmt_fixed3(r, sizeof(r), radius);
	fmt_fixed3(dc, sizeof(dc), st->harmonics.dc_power * 1000.f);
	int n = snprintf(row, sizeof(row), "TEL,%u,%u,%s,%s,%s,%s,%d,%d,%d,%d,%s,%d,%d,%u,%u,%u\n",
			 st->updates, b->t_ms, bias, phi, err, r,
			 float_to_mv(iq[0] * 1000.f), float_to_mv(iq[1] * 1000.f),
			 float_to_mv(iq[2] * 1000.f), float_to_mv(iq[3] * 1000.f),
			 dc, st->locked ? 1 : 0, st->frozen ? 1 : 0,
			 (unsigned int)st->recal_count,
			 rt_pipeline_drop_count(), rt_pipeline_crc_error_count());
	return send_row(row, n, NULL, NULL);
}

void cli_hw_result(const uint16_t regs[4], uint32_t desc_crc, uint32_t record_crc,
		   int cal_rc, cli_write_fn_t out, void *ctx)
{
	cli_printf(out, ctx, "SYSCLK=%u ADS131M02 ID=0x%04X STAT=0x%04X MODE=0x%04X CLOCK=0x%04X\n",
		   sys_clock_hw_cycles_per_sec(), regs[0], regs[1], regs[2], regs[3]);
	cli_printf(out, ctx, "U21 desc_crc=0x%08X cal_status=%d cal_crc=0x%08X\n",
		   desc_crc, cal_rc, record_crc);
	cli_printf(out, ctx, "fw=P7 rev=%s board=%s zephyr=%s build=%s\n",
		   BIAS_FW_REV, CONFIG_BOARD, KERNEL_VERSION_STRING,
		   STRINGIFY(BUILD_VERSION));
}

void cli_adc_result(bool dump, unsigned segment, unsigned count, const rt_adc_stats_t *stats,
		    cli_write_fn_t out, void *ctx)
{
	if (!dump) {
		uint64_t mhz = ((uint64_t)sys_clock_hw_cycles_per_sec() * count * 1000 +
				stats->total / 2) / stats->total;
		cli_printf(out, ctx, "ADC_STAT,n=%u,total_cyc=%llu,fs_hz=%u.%03u,dt_min_cyc=%u,dt_max_cyc=%u,drops=%u,crc=%u\n",
			   count, (unsigned long long)stats->total, (unsigned)(mhz / 1000),
			   (unsigned)(mhz % 1000), stats->min, stats->max,
			   rt_pipeline_drop_count(), rt_pipeline_crc_error_count());
		return;
	}
	cli_printf(out, ctx, "DUMP_SEG,%u,%u,%u\n", segment, count, stats->first);
	for (unsigned i = 0; i < count;) {
		char row[160] = "DUMP_DATA";
		size_t used = strlen(row);
		for (unsigned j = 0; j < 16 && i < count; ++j, ++i) {
			used += snprintf(row + used, sizeof(row) - used, ",%d",
					 rt_pipeline_dump_sample(i));
		}
		strcpy(row + used, "\n");
		(out ? out : default_write)(row, ctx);
	}
}

static int experiment(int argc, char **argv, cli_write_fn_t out, void *ctx)
{
	unsigned count, wait = 0;
	if (!strcmp(argv[0], "hw") && argc == 2 && !strcmp(argv[1], "info")) {
		return ctrl_thread_hw_info(out, ctx);
	}
	if (!strcmp(argv[0], "adc") && argc == 3 &&
	    (!strcmp(argv[1], "dump") || !strcmp(argv[1], "stat"))) {
		int rc = parse_count(argv[2], &count);
		return rc ? rc : ctrl_thread_adc(!strcmp(argv[1], "dump"), count, out, ctx);
	}
	if (!strcmp(argv[0], "telemetry") && argc == 2 &&
	    (!strcmp(argv[1], "on") || !strcmp(argv[1], "off"))) {
		bool enable = !strcmp(argv[1], "on");
		ctrl_thread_telemetry(enable);
		atomic_set(&quiet_console, enable);
		return 0;
	}
	bool step = !strcmp(argv[0], "step");
	if (argc != 5 || (!step && strcmp(argv[0], "meas"))) {
		return -EINVAL;
	}
	float initial, final, pilot;
	int rc = parse_decimal(argv[1], &initial);
	if (rc) {
		return rc;
	}
	final = initial;
	rc = step ? parse_decimal(argv[2], &final) : parse_count(argv[3], &wait);
	if (!rc) {
		rc = parse_decimal(argv[step ? 3 : 2], &pilot);
	}
	if (!rc) {
		rc = parse_count(argv[4], &count);
	}
	return rc ? rc : ctrl_thread_measure(step, initial / 1000.f, final / 1000.f,
					     pilot / 1000.f, wait, count, out, ctx);
}

int cli_service_exec_line(const char *line, cli_write_fn_t out, void *ctx)
{
	if (line == NULL) {
		return -EINVAL;
	}

	size_t len = strlen(line);

	if (len >= CLI_LINE_SIZE) {
		cli_printf(out, ctx, "line too long (err=%d)\n", -E2BIG);
		return -E2BIG;
	}

	char buf[CLI_LINE_SIZE];

	memcpy(buf, line, len + 1U);

	char *argv[CLI_MAX_ARGS];
	int argc = 0;
	char *save = NULL;
	char *tok = strtok_r(buf, " \t\r\n", &save);

	while (tok != NULL) {
		if (argc >= CLI_MAX_ARGS) {
			cli_printf(out, ctx, "too many arguments (err=%d)\n", -E2BIG);
			return -E2BIG;
		}
		argv[argc++] = tok;
		tok = strtok_r(NULL, " \t\r\n", &save);
	}
	if (argc == 0) {
		return 0;
	}

	if (!strcmp(argv[0], "hw") || !strcmp(argv[0], "adc") || !strcmp(argv[0], "meas") ||
	    !strcmp(argv[0], "step") || !strcmp(argv[0], "telemetry")) {
		int rc = experiment(argc, argv, out, ctx);
		cli_printf(out, ctx, rc ? "%s: err=%d\n" : "%s: ok\n", argv[0], rc);
		return rc;
	}
	if (argc == 1 && strcmp(argv[0], "help") == 0) {
		cmd_help(out, ctx);
		return 0;
	}
	if (argc == 1 && strcmp(argv[0], "status") == 0) {
		return cmd_status(out, ctx);
	}
	if (argc == 2 && strcmp(argv[0], "cal") == 0 && strcmp(argv[1], "bias") == 0) {
		int rc = ctrl_thread_start_cal(true);

		cli_printf(out, ctx, rc == 0 ? "cal bias: ok\n" : "cal bias: err=%d\n", rc);
		return rc;
	}
	if (argc == 2 && strcmp(argv[0], "scan") == 0 && strcmp(argv[1], "vpi") == 0) {
		int rc = ctrl_thread_start_cal(false);

		cli_printf(out, ctx, rc == 0 ? "scan vpi: ok\n" : "scan vpi: err=%d\n", rc);
		return rc;
	}
	if (argc == 1 && strcmp(argv[0], "start") == 0) {
		int rc = ctrl_thread_start();

		cli_printf(out, ctx, rc == 0 ? "start: ok\n" : "start: err=%d\n", rc);
		return rc;
	}
	if (argc == 1 && strcmp(argv[0], "stop") == 0) {
		int rc = ctrl_thread_stop();

		cli_printf(out, ctx, rc == 0 ? "stop: ok\n" : "stop: err=%d\n", rc);
		return rc;
	}
	if (strcmp(argv[0], "target") == 0) {
		return cmd_target(argc, argv, out, ctx);
	}
	if (argc >= 2 && strcmp(argv[0], "ctrl") == 0 && strcmp(argv[1], "cfg") == 0) {
		return cmd_ctrl_cfg(argc, argv, out, ctx);
	}
	if (argc == 1 && strcmp(argv[0], "save") == 0) {
		int rc = ctrl_thread_save_cal();

		cli_printf(out, ctx, rc == 0 ? "save: ok\n" : "save: err=%d\n", rc);
		return rc;
	}
	if (argc == 1 && strcmp(argv[0], "load") == 0) {
		int rc = ctrl_thread_load_cal();

		cli_printf(out, ctx, rc == 0 ? "load: ok\n" : "load: err=%d\n", rc);
		return rc;
	}
	if (argc == 1 && strcmp(argv[0], "reinit") == 0) {
		int rc = ctrl_thread_init(NULL, NULL, false);

		if (rc == 0) {
			cli_printf(out, ctx, "reinit: ok\n");
		} else if (rc == -EBADMSG) {
			cli_printf(out, ctx, "reinit: uncalibrated (err=%d), run cal bias\n", rc);
		} else {
			cli_printf(out, ctx, "reinit: err=%d\n", rc);
		}
		return rc;
	}
	if (argc == 2 && strcmp(argv[0], "eeprom") == 0 && strcmp(argv[1], "init") == 0) {
		return cmd_eeprom_init(out, ctx);
	}

	cli_printf(out, ctx, "Unknown command: %s (type 'help')\n", argv[0]);
	return -EINVAL;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) && DT_HAS_CHOSEN(zephyr_console)

static const struct device *const console_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static K_THREAD_STACK_DEFINE(cli_stack, CLI_STACK_SIZE);
static struct k_thread cli_thread;
static K_SEM_DEFINE(rx_sem, 0, 1);

static uint8_t rx_ring[CLI_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static atomic_t rx_overflow;
static atomic_t cli_started;

static void uart_rx_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	bool notify = false;

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) > 0) {
		uint8_t byte;
		int n = uart_fifo_read(dev, &byte, 1);

		if (n <= 0) {
			break;
		}
		uint16_t next = (uint16_t)((rx_head + 1U) % CLI_RING_SIZE);

		if (next == rx_tail) {
			atomic_set(&rx_overflow, 1);
		} else {
			rx_ring[rx_head] = byte;
			rx_head = next;
		}
		notify = true;
	}
	if (notify) {
		k_sem_give(&rx_sem);
	}
}

static bool pop_rx_byte(uint8_t *out)
{
	unsigned int key = irq_lock();
	bool has_data = (rx_head != rx_tail);

	if (has_data) {
		*out = rx_ring[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1U) % CLI_RING_SIZE);
	}
	irq_unlock(key);
	return has_data;
}

static void cli_thread_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	char line_buf[CLI_LINE_SIZE];
	size_t line_len = 0;
	bool line_overflow = false;
	bool prev_cr = false;

	printk("Phase 7 CLI ready. Type 'help'.\n> ");

	for (;;) {
		k_sem_take(&rx_sem, K_FOREVER);

		if (atomic_cas(&rx_overflow, 1, 0)) {
			line_len = 0;
			line_overflow = false;
			printk("\nUART RX overflow\n> ");
		}

		uint8_t byte;

		while (pop_rx_byte(&byte)) {
			if (byte == '\n' && prev_cr) {
				prev_cr = false;
				continue;
			}
			prev_cr = (byte == '\r');

			if (byte == '\r' || byte == '\n') {
				if (!atomic_get(&quiet_console)) {
					printk("\n");
				}
				if (line_overflow) {
					printk("Line too long (err=%d)\n", -E2BIG);
				} else {
					line_buf[line_len] = '\0';
					if (line_len > 0U) {
						(void)cli_service_exec_line(line_buf, NULL, NULL);
					}
				}
				line_len = 0;
				line_overflow = false;
				if (!atomic_get(&quiet_console)) {
					printk("> ");
				}
			} else if (byte == '\b' || byte == 0x7fU) {
				if (line_overflow) {
					line_overflow = false;
					line_len = 0;
					printk("\n> ");
				} else if (line_len > 0U) {
					line_len--;
					if (!atomic_get(&quiet_console)) {
						printk("\b \b");
					}
				}
			} else if (byte >= 0x20U && byte < 0x7fU) {
				if (line_len + 1U < CLI_LINE_SIZE) {
					line_buf[line_len++] = (char)byte;
					if (!atomic_get(&quiet_console)) {
						printk("%c", byte);
					}
				} else {
					line_overflow = true;
				}
			}
		}
	}
}

int cli_service_start(void)
{
	if (!atomic_cas(&cli_started, 0, 1)) {
		return -EALREADY;
	}
	if (!device_is_ready(console_uart)) {
		atomic_clear(&cli_started);
		return -ENODEV;
	}

	uart_irq_rx_disable(console_uart);
	rx_head = 0;
	rx_tail = 0;
	atomic_clear(&rx_overflow);
	k_sem_reset(&rx_sem);

	int rc = uart_irq_callback_user_data_set(console_uart, uart_rx_isr, NULL);

	if (rc < 0) {
		atomic_clear(&cli_started);
		return rc;
	}

	k_thread_create(&cli_thread, cli_stack, K_THREAD_STACK_SIZEOF(cli_stack),
			cli_thread_entry, NULL, NULL, NULL,
			CLI_THREAD_PRIO, 0, K_NO_WAIT);
	uart_irq_rx_enable(console_uart);
	return 0;
}

#else

int cli_service_start(void)
{
	return -ENOTSUP;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN && DT_HAS_CHOSEN(zephyr_console) */
