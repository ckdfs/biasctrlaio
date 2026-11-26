/* SPDX-License-Identifier: Apache-2.0 */
#include "cli_service.h"
#include "ctrl_thread.h"
#include "rt_pipeline.h"

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
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define CLI_THREAD_PRIO 8
#define CLI_STACK_SIZE 2048
#define CLI_RING_SIZE 128U
#define CLI_LINE_SIZE 96U
#define CLI_MAX_ARGS 6

static void default_write(const char *text, void *ctx)
{
	ARG_UNUSED(ctx);
	printk("%s", text);
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

static int parse_decimal_deg(const char *s, float *out)
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
		   "  target quad|min|max|custom <deg>\n"
		   "  save\n"
		   "  load\n"
		   "  reinit\n"
		   "  eeprom init\n");
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
		   "state=%s init=%d target=%s(%s deg) cal=%d locked=%d hold_assist=%d\n",
		   state_name(st.state), st_rc == 0 ? 1 : 0, target_name(st.target), deg_buf,
		   st.calibrated ? 1 : 0, st.locked ? 1 : 0, st.hold_assist_active ? 1 : 0);
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

		rc = parse_decimal_deg(argv[2], &deg);
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
	     st.state == CTRL_STATE_RUNNING)) {
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

	printk("Phase 4 CLI ready. Type 'help'.\n> ");

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
				printk("\n");
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
				printk("> ");
			} else if (byte == '\b' || byte == 0x7fU) {
				if (line_overflow) {
					line_overflow = false;
					line_len = 0;
					printk("\n> ");
				} else if (line_len > 0U) {
					line_len--;
					printk("\b \b");
				}
			} else if (byte >= 0x20U && byte < 0x7fU) {
				if (line_len + 1U < CLI_LINE_SIZE) {
					line_buf[line_len++] = (char)byte;
					printk("%c", byte);
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
