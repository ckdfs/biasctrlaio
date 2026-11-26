/* SPDX-License-Identifier: Apache-2.0 */
#include "cli_service.h"
#include "ctrl_thread.h"

#include <bias/analog_board.h>
#include <keys.h>
#include <oled_lite.h>

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define OLED_REFRESH_MS 500U
#define LED_PERIOD_NORMAL_MS 500U
#define LED_PERIOD_FAULT_MS 150U

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec spare_pa8 =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), spare_pa8_gpios);

static const char *oled_state_str(ctrl_state_t state)
{
	switch (state) {
	case CTRL_STATE_IDLE:
		return "IDLE";
	case CTRL_STATE_SCAN:
		return "SCAN";
	case CTRL_STATE_CAL:
		return "CAL";
	case CTRL_STATE_RUNNING:
		return "RUN";
	case CTRL_STATE_FAULT:
		return "FLT";
	default:
		return "UNK";
	}
}

static const char *oled_target_str(bias_point_t target)
{
	switch (target) {
	case BIAS_POINT_QUAD:
		return "QUAD";
	case BIAS_POINT_MAX:
		return "MAX";
	case BIAS_POINT_MIN:
		return "MIN";
	case BIAS_POINT_CUSTOM:
		return "CUST";
	default:
		return "UNK";
	}
}

static int oled_mv_clamped(float v)
{
	if (!isfinite(v)) {
		return 0;
	}
	double mv = (double)v * 1000.0;

	if (mv > 99999.0) {
		return 99999;
	}
	if (mv < -99999.0) {
		return -99999;
	}
	return (int)(mv >= 0.0 ? (mv + 0.5) : (mv - 0.5));
}

static void format_oled_lines(int board_rc, int keys_rc, int cli_rc,
			      int st_rc, const ctrl_thread_status_t *st,
			      char lines[3][22])
{
	if (st_rc < 0) {
		(void)snprintf(lines[0], 22, "BRD:%d CTRL:%d", board_rc, st->load_error);
		(void)snprintf(lines[1], 22, "KEYS:%d CLI:%d", keys_rc, cli_rc);
		(void)snprintf(lines[2], 22, "RUN EEPROM INIT");
		return;
	}

	(void)snprintf(lines[0], 22, "ST:%s CAL:%d LCK:%d",
		       oled_state_str(st->state), st->calibrated ? 1 : 0,
		       st->locked ? 1 : 0);
	(void)snprintf(lines[1], 22, "TGT:%s VA:%dMV",
		       oled_target_str(st->target), oled_mv_clamped(st->bias_v));
	if (st->state == CTRL_STATE_FAULT) {
		(void)snprintf(lines[2], 22, "FLT:%d CLN:%d", st->fault, st->cleanup_error);
	} else if (!st->calibrated) {
		(void)snprintf(lines[2], 22, "UNCAL LOAD:%d", st->load_error);
	} else {
		(void)snprintf(lines[2], 22, "VPI:%dMV HA:%d",
			       oled_mv_clamped(st->vpi_v),
			       st->hold_assist_active ? 1 : 0);
	}
}

static void render_oled_if_changed(int board_rc, int keys_rc, int cli_rc,
				   int st_rc, const ctrl_thread_status_t *st,
				   bool *initialized_view, char prev[3][22])
{
	char next[3][22];

	format_oled_lines(board_rc, keys_rc, cli_rc, st_rc, st, next);
	if (*initialized_view &&
	    memcmp(prev, next, sizeof(next)) == 0) {
		return;
	}

	oled_lite_clear();
	(void)oled_lite_text(0, 0, "BIASCTRL H503 P4");
	(void)oled_lite_text(0, 16, next[0]);
	(void)oled_lite_text(0, 32, next[1]);
	(void)oled_lite_text(0, 48, next[2]);
	(void)oled_lite_flush();

	memcpy(prev, next, sizeof(next));
	*initialized_view = true;
}

int main(void)
{
	const analog_board_ops_t *ops = analog_board_get_ops();
	bool led_ready = false;

	if (gpio_is_ready_dt(&spare_pa8)) {
		(void)gpio_pin_configure_dt(&spare_pa8, GPIO_OUTPUT_INACTIVE);
	}
	if (gpio_is_ready_dt(&led) &&
	    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) == 0) {
		led_ready = true;
	}

	printk("\nbiasctrl_h503 Phase 4, SYSCLK %u Hz\n",
	       (unsigned int)DT_PROP(DT_NODELABEL(rcc), clock_frequency));

	int board_rc = (ops != NULL && ops->init != NULL) ? ops->init() : -ENODEV;

	printk("analog_board init (%s): %d\n",
	       (ops != NULL && ops->caps != NULL) ? ops->caps->board_id : "none",
	       board_rc);

	int oled_rc = oled_lite_init();

	printk("oled_lite init: %d\n", oled_rc);

	int keys_rc = bias_keys_init(NULL, NULL);

	printk("keys init: %d\n", keys_rc);

	int ctrl_rc = ctrl_thread_init(ops, NULL, false);

	printk("ctrl_thread init: %d\n", ctrl_rc);

	int cli_rc = cli_service_start();

	printk("cli_service start: %d\n", cli_rc);

	bool hw_boot_fault = (board_rc < 0 || oled_rc < 0 || keys_rc < 0 ||
			      cli_rc < 0 || (ctrl_rc < 0 && ctrl_rc != -EBADMSG));
	bool oled_has_frame = false;
	char prev_lines[3][22] = { { 0 } };
	ctrl_thread_status_t st = { 0 };
	int st_rc = ctrl_thread_get_status(&st);

	if (oled_rc == 0) {
		render_oled_if_changed(board_rc, keys_rc, cli_rc, st_rc, &st,
				       &oled_has_frame, prev_lines);
	}

	bool in_fault = hw_boot_fault || (st_rc < 0) || (st.state == CTRL_STATE_FAULT);
	uint32_t blink_ms = in_fault ? LED_PERIOD_FAULT_MS : LED_PERIOD_NORMAL_MS;
	int64_t next_blink = k_uptime_get() + blink_ms;
	int64_t next_oled = k_uptime_get() + OLED_REFRESH_MS;

	while (true) {
		int64_t now = k_uptime_get();

		if (now >= next_oled) {
			st_rc = ctrl_thread_get_status(&st);
			in_fault = hw_boot_fault || (st_rc < 0) || (st.state == CTRL_STATE_FAULT);
			blink_ms = in_fault ? LED_PERIOD_FAULT_MS : LED_PERIOD_NORMAL_MS;
			if (oled_rc == 0) {
				render_oled_if_changed(board_rc, keys_rc, cli_rc, st_rc, &st,
						       &oled_has_frame, prev_lines);
			}
			next_oled = now + OLED_REFRESH_MS;
		}
		if (led_ready && now >= next_blink) {
			(void)gpio_pin_toggle_dt(&led);
			next_blink = now + blink_ms;
		}
		k_msleep(50);
	}
}
