/* SPDX-License-Identifier: Apache-2.0 */
#include "diag_cli.h"

#include <ads131m02.h>
#include <analog_board_mzm.h>
#include <bias/analog_board.h>
#include <dac8568.h>
#include <keys.h>
#include <oled_lite.h>
#include <shared_i2c.h>

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define CLI_RING_SIZE 128U
#define CLI_LINE_SIZE 96U

static const struct device *const console_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static struct diag_boot_status boot_info;

static uint8_t rx_ring[CLI_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_overflow;

static char line_buf[CLI_LINE_SIZE];
static size_t line_len;

static struct {
	uint32_t target;
	uint32_t count;
	uint32_t t_first;
	uint32_t t_last;
	int64_t sum0;
	int64_t sum1;
	int32_t min0;
	int32_t max0;
	int32_t min1;
	int32_t max1;
	int stream_err;
	atomic_t done;
} adc_cap;

static void uart_rx_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) > 0) {
		uint8_t byte;

		while (uart_fifo_read(dev, &byte, 1) == 1) {
			uint16_t next = (uint16_t)((rx_head + 1U) % CLI_RING_SIZE);

			if (next == rx_tail) {
				rx_overflow = true;
			} else {
				rx_ring[rx_head] = byte;
				rx_head = next;
			}
		}
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

static void adc_sample_cb(const struct bias_ads131m02_sample *sample, int error, void *context)
{
	ARG_UNUSED(context);

	if (atomic_get(&adc_cap.done) != 0) {
		return;
	}
	if (error != 0 || sample == NULL) {
		adc_cap.stream_err = error ? error : -EIO;
		__DMB();
		atomic_set(&adc_cap.done, 1);
		return;
	}
	if (adc_cap.count < adc_cap.target) {
		uint32_t now = k_cycle_get_32();
		int32_t c0 = sample->channel[0];
		int32_t c1 = sample->channel[1];

		if (adc_cap.count == 0U) {
			adc_cap.t_first = now;
			adc_cap.min0 = c0;
			adc_cap.max0 = c0;
			adc_cap.min1 = c1;
			adc_cap.max1 = c1;
		} else {
			if (c0 < adc_cap.min0) {
				adc_cap.min0 = c0;
			}
			if (c0 > adc_cap.max0) {
				adc_cap.max0 = c0;
			}
			if (c1 < adc_cap.min1) {
				adc_cap.min1 = c1;
			}
			if (c1 > adc_cap.max1) {
				adc_cap.max1 = c1;
			}
		}
		adc_cap.t_last = now;
		adc_cap.sum0 += c0;
		adc_cap.sum1 += c1;
		adc_cap.count++;
		if (adc_cap.count >= adc_cap.target) {
			__DMB();
			atomic_set(&adc_cap.done, 1);
		}
	}
}

static bool parse_u32(const char *s, int base, uint32_t min_val, uint32_t max_val, uint32_t *out)
{
	char *end = NULL;
	unsigned long val;

	if (s == NULL || *s == '\0' || *s == '-') {
		return false;
	}
	errno = 0;
	val = strtoul(s, &end, base);
	if (errno != 0 || end == s || *end != '\0' || val < min_val || val > max_val) {
		return false;
	}
	*out = (uint32_t)val;
	return true;
}

static void print_help(void)
{
	printk("Commands:\n");
	printk("  help\n");
	printk("  status\n");
	printk("  i2c scan\n");
	printk("  dac dc <0..5> <0..65535>\n");
	printk("  dac pilot <0..1> <0..65535>\n");
	printk("  dac status\n");
	printk("  mux <pilot 0..1> <dc 0..5> <0|1>\n");
	printk("  mux off\n");
	printk("  adc init [32|64]\n");
	printk("  adc reg <addr_hex>\n");
	printk("  adc sample [1..10000]\n");
	printk("  eeprom read\n");
	printk("  eeprom write-default\n");
	printk("  oled <text>\n");
	printk("  keys\n");
}

static void cmd_status(void)
{
	const analog_board_ops_t *ops = analog_board_get_ops();
	uint16_t adc_id = 0;
	int id_rc = bias_ads131m02_read_reg(0x00, &adc_id);
	bool k1 = false, k2 = false, k3 = false;
	int krc1 = bias_keys_read(BIAS_KEY_1, &k1);
	int krc2 = bias_keys_read(BIAS_KEY_2, &k2);
	int krc3 = bias_keys_read(BIAS_KEY_3, &k3);

	printk("Board: %s, SYSCLK=%u Hz\n", ops->caps->board_id,
	       (unsigned int)DT_PROP(DT_NODELABEL(rcc), clock_frequency));
	printk("Boot rc: board=%d oled=%d keys=%d eeprom=%d adc=%d\n",
	       boot_info.board_rc, boot_info.oled_rc, boot_info.keys_rc,
	       boot_info.eeprom_rc, boot_info.adc_rc);
	printk("DAC: init=%d status=%d\n", dac8568_init(), dac8568_status());
	if (id_rc == 0) {
		printk("ADC: ID=0x%04X\n", adc_id);
	} else {
		printk("ADC: read_reg(ID) err=%d\n", id_rc);
	}
	printk("Keys: K1=%d(%d) K2=%d(%d) K3=%d(%d)\n",
	       k1 ? 1 : 0, krc1, k2 ? 1 : 0, krc2, k3 ? 1 : 0, krc3);
}

static void cmd_i2c_scan(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t dummy = 0;
	struct i2c_msg probe = { .buf = &dummy, .len = 0, .flags = I2C_MSG_WRITE | I2C_MSG_STOP };
	unsigned int found = 0;

	if (!device_is_ready(bus)) {
		printk("I2C1 not ready\n");
		return;
	}
	k_mutex_lock(&bias_i2c1_mutex, K_FOREVER);
	for (unsigned int addr = 0x08U; addr <= 0x77U; ++addr) {
		int rc = i2c_transfer(bus, &probe, 1, addr);

		if (rc == 0) {
			printk("I2C1 ACK 0x%02X\n", addr);
			found++;
		} else if (rc != -EIO) {
			printk("I2C1 err at 0x%02X (%d)\n", addr, rc);
			break;
		}
	}
	k_mutex_unlock(&bias_i2c1_mutex);
	printk("I2C1 scan done (%u found)\n", found);
}

static void cmd_dac(int argc, char **argv)
{
	const analog_board_ops_t *ops = analog_board_get_ops();
	uint32_t ch, code;

	if (argc == 2 && strcmp(argv[1], "status") == 0) {
		printk("DAC: init=%d status=%d\n", dac8568_init(), dac8568_status());
		return;
	}
	if (argc == 4 && strcmp(argv[1], "dc") == 0 &&
	    parse_u32(argv[2], 10, 0, 5, &ch) &&
	    parse_u32(argv[3], 0, 0, 65535, &code)) {
		int rc = ops->set_dc_code((uint8_t)ch, (uint16_t)code);

		printk("dac dc %u=%u -> %d\n", ch, code, rc);
		return;
	}
	if (argc == 4 && strcmp(argv[1], "pilot") == 0 &&
	    parse_u32(argv[2], 10, 0, 1, &ch) &&
	    parse_u32(argv[3], 0, 0, 65535, &code)) {
		int rc = ops->set_pilot_code((uint8_t)ch, (uint16_t)code);

		if (rc == 0) {
			for (unsigned int i = 0; i < 100U && dac8568_status() == -EBUSY; ++i) {
				k_busy_wait(10);
			}
			rc = dac8568_status();
		}
		printk("dac pilot %u=%u -> %d\n", ch, code, rc);
		return;
	}
	printk("Usage: dac dc <0..5> <0..65535> | dac pilot <0..1> <0..65535> | dac status\n");
}

static void cmd_mux(int argc, char **argv)
{
	const analog_board_ops_t *ops = analog_board_get_ops();
	uint32_t pilot, dc, en;

	if (argc == 2 && strcmp(argv[1], "off") == 0) {
		int rc = ops->disconnect_all();

		printk("mux off -> %d\n", rc);
		return;
	}
	if (argc == 4 &&
	    parse_u32(argv[1], 10, 0, 1, &pilot) &&
	    parse_u32(argv[2], 10, 0, 5, &dc) &&
	    parse_u32(argv[3], 10, 0, 1, &en)) {
		int rc = ops->set_route((uint8_t)pilot, (uint8_t)dc, en != 0U);

		printk("mux p%u->dc%u en=%u -> %d\n", pilot, dc, en, rc);
		return;
	}
	printk("Usage: mux <0..1> <0..5> <0|1> | mux off\n");
}

static void cmd_adc(int argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "init") == 0) {
		enum bias_ads131m02_rate rate = BIAS_ADS131M02_64_KSPS;
		uint32_t ksps = 64U;

		if (argc == 3) {
			if (!parse_u32(argv[2], 10, 32, 64, &ksps) ||
			    (ksps != 32U && ksps != 64U)) {
				printk("Usage: adc init [32|64]\n");
				return;
			}
			rate = (ksps == 32U) ? BIAS_ADS131M02_32_KSPS : BIAS_ADS131M02_64_KSPS;
		}
		int rc = bias_ads131m02_init(rate);
		uint16_t id = 0, st = 0, mode = 0, clk = 0;

		if (rc == 0) {
			(void)bias_ads131m02_read_reg(0x00, &id);
			(void)bias_ads131m02_read_reg(0x01, &st);
			(void)bias_ads131m02_read_reg(0x02, &mode);
			(void)bias_ads131m02_read_reg(0x03, &clk);
			printk("adc init %u kSPS ok: ID=0x%04X STAT=0x%04X MODE=0x%04X CLK=0x%04X\n",
			       ksps, id, st, mode, clk);
		} else {
			printk("adc init %u kSPS err=%d\n", ksps, rc);
		}
		return;
	}
	if (argc == 3 && strcmp(argv[1], "reg") == 0) {
		uint32_t addr = 0;
		uint16_t val = 0;

		if (!parse_u32(argv[2], 16, 0, 0x3f, &addr)) {
			printk("Usage: adc reg <00..3f>\n");
			return;
		}
		int rc = bias_ads131m02_read_reg((uint8_t)addr, &val);

		printk("adc reg 0x%02X -> %d val=0x%04X\n", addr, rc, val);
		return;
	}
	if ((argc == 2 || argc == 3) && strcmp(argv[1], "sample") == 0) {
		uint32_t target = 1000U;

		if (argc == 3 && !parse_u32(argv[2], 10, 1, 10000, &target)) {
			printk("Usage: adc sample [1..10000]\n");
			return;
		}
		memset(&adc_cap, 0, sizeof(adc_cap));
		adc_cap.target = target;
		atomic_set(&adc_cap.done, 0);

		int start_rc = bias_ads131m02_start(adc_sample_cb, NULL);

		if (start_rc < 0) {
			printk("adc start err=%d\n", start_rc);
			return;
		}
		int64_t deadline = k_uptime_get() + 2000;

		while (atomic_get(&adc_cap.done) == 0 && k_uptime_get() < deadline) {
			k_msleep(1);
		}
		int stop_rc = bias_ads131m02_stop();
		uint32_t elapsed_us = (adc_cap.count > 1U) ?
			k_cyc_to_us_near32(adc_cap.t_last - adc_cap.t_first) : 0U;

		if (adc_cap.count > 1U && elapsed_us > 0U) {
			uint64_t sps = ((uint64_t)(adc_cap.count - 1U) * 1000000ULL) / elapsed_us;
			int32_t mean0 = (int32_t)(adc_cap.sum0 / (int64_t)adc_cap.count);
			int32_t mean1 = (int32_t)(adc_cap.sum1 / (int64_t)adc_cap.count);

			printk("adc sample n=%u/%u dt=%u us (%u SPS) err=%d stop=%d\n",
			       adc_cap.count, target, elapsed_us, (unsigned int)sps,
			       adc_cap.stream_err, stop_rc);
			printk("  CH0 min=%d mean=%d max=%d\n",
			       adc_cap.min0, mean0, adc_cap.max0);
			printk("  CH1 min=%d mean=%d max=%d\n",
			       adc_cap.min1, mean1, adc_cap.max1);
		} else {
			printk("adc sample timeout/fail n=%u/%u stream_err=%d stop=%d\n",
			       adc_cap.count, target, adc_cap.stream_err, stop_rc);
		}
		return;
	}
	printk("Usage: adc init [32|64] | adc reg <hex> | adc sample [1..10000]\n");
}

static void cmd_eeprom(int argc, char **argv)
{
	static struct bias_eeprom_descriptor_v1 desc;
	const analog_board_ops_t *ops = analog_board_get_ops();

	if (argc == 2 && strcmp(argv[1], "read") == 0) {
		int rc = ops->load_calibration(&desc);

		if (rc == 0) {
			printk("EEPROM ok: id=%.12s ver=%u len=%u tia=%u ohm crc=0x%08X\n",
			       desc.board_id, desc.schema_version, desc.length_bytes,
			       desc.tia_gain_ohms, desc.crc32);
			for (size_t i = 0; i < BIAS_EEPROM_V1_CHANNELS; ++i) {
				printk("  CH%u vpi=%u uV gain_ppm=%d off=%d uV\n",
				       (unsigned int)i, desc.channels[i].vpi_uv,
				       desc.channels[i].subtractor_gain_ppm,
				       desc.channels[i].subtractor_offset_uv);
			}
		} else {
			printk("eeprom read err=%d\n", rc);
		}
		return;
	}
	if (argc == 2 && strcmp(argv[1], "write-default") == 0) {
		mzm_20261110_fill_default_descriptor(&desc);
		int rc = ops->store_calibration(&desc);

		if (rc == 0) {
			rc = ops->load_calibration(&desc);
		}
		printk("eeprom write-default -> %d crc=0x%08X\n", rc, desc.crc32);
		return;
	}
	printk("Usage: eeprom read | eeprom write-default\n");
}

static bool is_oled_char(char c)
{
	return isalnum((unsigned char)c) || c == ' ' || c == '-' || c == '.' || c == ':';
}

static void cmd_oled(const char *raw_arg)
{
	char text[22];
	size_t len = 0;

	while (raw_arg != NULL && *raw_arg == ' ') {
		raw_arg++;
	}
	while (raw_arg != NULL && raw_arg[len] != '\0' && len < 21U) {
		char c = raw_arg[len];

		text[len] = is_oled_char(c) ? c : ' ';
		len++;
	}
	text[len] = '\0';

	oled_lite_clear();
	int rc = oled_lite_text(0, 0, "BIASCTRL H503");

	if (rc == 0 && len > 0U) {
		rc = oled_lite_text(0, 16, text);
	}
	if (rc == 0) {
		rc = oled_lite_flush();
	}
	printk("oled -> %d\n", rc);
}

static void cmd_keys(void)
{
	bool k1 = false, k2 = false, k3 = false;
	int r1 = bias_keys_read(BIAS_KEY_1, &k1);
	int r2 = bias_keys_read(BIAS_KEY_2, &k2);
	int r3 = bias_keys_read(BIAS_KEY_3, &k3);

	printk("KEY1=%d(%d) KEY2=%d(%d) KEY3=%d(%d)\n",
	       k1 ? 1 : 0, r1, k2 ? 1 : 0, r2, k3 ? 1 : 0, r3);
}

static void dispatch_line(char *line)
{
	char *argv[6];
	int argc = 0;
	char raw_oled[CLI_LINE_SIZE];

	if (strncmp(line, "oled", 4) == 0 && (line[4] == '\0' || line[4] == ' ')) {
		strncpy(raw_oled, line + 4, sizeof(raw_oled) - 1U);
		raw_oled[sizeof(raw_oled) - 1U] = '\0';
		cmd_oled(raw_oled);
		return;
	}

	char *save = NULL;
	char *tok = strtok_r(line, " \t", &save);

	while (tok != NULL && argc < 6) {
		argv[argc++] = tok;
		tok = strtok_r(NULL, " \t", &save);
	}
	if (argc == 0) {
		return;
	}
	if (strcmp(argv[0], "help") == 0) {
		print_help();
	} else if (strcmp(argv[0], "status") == 0) {
		cmd_status();
	} else if (argc == 2 && strcmp(argv[0], "i2c") == 0 && strcmp(argv[1], "scan") == 0) {
		cmd_i2c_scan();
	} else if (strcmp(argv[0], "dac") == 0) {
		cmd_dac(argc, argv);
	} else if (strcmp(argv[0], "mux") == 0) {
		cmd_mux(argc, argv);
	} else if (strcmp(argv[0], "adc") == 0) {
		cmd_adc(argc, argv);
	} else if (strcmp(argv[0], "eeprom") == 0) {
		cmd_eeprom(argc, argv);
	} else if (strcmp(argv[0], "keys") == 0) {
		cmd_keys();
	} else {
		printk("Unknown command: %s (type 'help')\n", argv[0]);
	}
}

int diag_cli_init(const struct diag_boot_status *boot)
{
	if (boot != NULL) {
		boot_info = *boot;
	}
	if (!device_is_ready(console_uart)) {
		return -ENODEV;
	}
	uart_irq_rx_disable(console_uart);
	int rc = uart_irq_callback_user_data_set(console_uart, uart_rx_isr, NULL);

	if (rc < 0) {
		return rc;
	}
	uart_irq_rx_enable(console_uart);
	printk("Phase 2 diag CLI ready. Type 'help'.\n> ");
	return 0;
}

void diag_cli_poll(void)
{
	uint8_t byte;

	if (rx_overflow) {
		rx_overflow = false;
		printk("\nUART RX overflow\n> ");
	}
	while (pop_rx_byte(&byte)) {
		if (byte == '\r' || byte == '\n') {
			printk("\n");
			line_buf[line_len] = '\0';
			dispatch_line(line_buf);
			line_len = 0;
			printk("> ");
		} else if (byte == '\b' || byte == 0x7fU) {
			if (line_len > 0U) {
				line_len--;
				printk("\b \b");
			}
		} else if (byte >= 0x20U && byte < 0x7fU) {
			if (line_len + 1U < CLI_LINE_SIZE) {
				line_buf[line_len++] = (char)byte;
				printk("%c", byte);
			}
		}
	}
}
