/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include "ads131m02.h"
#include "ads131m02_dma.h"
#include "ads131m02_frame.h"

#define ADC_NODE DT_NODELABEL(bias_adc)
#define REG_ID 0
#define REG_MODE 2
#define REG_CLOCK 3
#define RREG 0xa000
#define WREG 0x6000
#define WREG_ACK 0x4000
#define MODE_24BIT_CCITT (BIT(12) | BIT(8) | BIT(4) | BIT(0))
#define CLOCK_BOTH_HR (BIT(9) | BIT(8) | BIT(1))
#define CLOCK_TBM BIT(5)

static const struct spi_dt_spec bus = SPI_DT_SPEC_GET(ADC_NODE, SPI_WORD_SET(8));
static const struct gpio_dt_spec sync_pin =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), adc_sync_gpios);
static K_MUTEX_DEFINE(control);
static bool ready;
static bool streaming;

static int exchange(const struct spi_dt_spec *spec, uint16_t command, uint16_t data,
		    uint16_t *response)
{
	uint8_t tx[ADS_FRAME_BYTES] = {0};
	uint8_t rx[ADS_FRAME_BYTES];
	size_t crc_at = 3;

	sys_put_be16(command, tx);
	if ((command & 0xe000) == WREG) {
		sys_put_be16(data, tx + 3);
		crc_at = 6;
	}
	sys_put_be16(ads_crc(tx, crc_at), tx + crc_at);
	struct spi_buf out = { .buf = tx, .len = sizeof(tx) };
	struct spi_buf in = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set write = { .buffers = &out, .count = 1 };
	const struct spi_buf_set read = { .buffers = &in, .count = 1 };
	int rc = spi_transceive_dt(spec, &write, &read);

	if (rc == 0) {
		rc = ads_frame_crc(rx);
	}
	if (rc == 0) {
		*response = sys_get_be16(rx);
	}
	return rc;
}

static int read_reg(uint8_t address, uint16_t *value)
{
	uint16_t previous;
	int rc = exchange(&bus, RREG | (address << 7), 0, &previous);

	return rc ? rc : exchange(&bus, 0, 0, value);
}

static int write_reg(uint8_t address, uint16_t value)
{
	uint16_t response;
	int rc = exchange(&bus, WREG | (address << 7), value, &response);

	if (rc == 0) {
		rc = exchange(&bus, 0, 0, &response);
	}
	if (rc == 0 && response != (WREG_ACK | (address << 7))) {
		rc = -EPROTO;
	}
	if (rc == 0) {
		rc = read_reg(address, &response);
	}
	return rc ? rc : (response == value ? 0 : -EIO);
}

/* Acquire the thread-side control plane. ISRs never take this mutex. */
static int claim(void)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	return k_mutex_lock(&control, K_FOREVER);
}

static int idle_status(void)
{
	return streaming ? -EBUSY : (ready ? 0 : -EACCES);
}

int bias_ads131m02_read_reg(uint8_t address, uint16_t *value)
{
	if (value == NULL || address > 0x3f) {
		return -EINVAL;
	}
	int rc = claim();

	if (!rc) {
		rc = idle_status();
		if (!rc) {
			rc = read_reg(address, value);
		}
		k_mutex_unlock(&control);
	}
	return rc;
}

int bias_ads131m02_write_reg(uint8_t address, uint16_t value)
{
	if (address < 4 || address == 5 || address > 0x12) {
		return -EINVAL;
	}
	int rc = claim();

	if (!rc) {
		rc = idle_status();
		if (!rc) {
			rc = write_reg(address, value);
		}
		k_mutex_unlock(&control);
	}
	return rc;
}

static int configure(enum bias_ads131m02_rate rate)
{
	uint16_t id;
	int rc;

	if (!spi_is_ready_dt(&bus) || !gpio_is_ready_dt(&sync_pin)) {
		printk("ADS131M02 not ready: spi=%d sync=%d\n",
		       spi_is_ready_dt(&bus), gpio_is_ready_dt(&sync_pin));
		return -ENODEV;
	}
	k_msleep(6);
	rc = gpio_pin_configure_dt(&sync_pin, GPIO_OUTPUT_ACTIVE);
	if (rc) {
		return rc;
	}
	k_busy_wait(300);
	rc = gpio_pin_set_dt(&sync_pin, 0);
	if (rc) {
		return rc;
	}
	k_msleep(2); /* register acquisition after hardware reset */
	uint16_t prev = 0;

	rc = exchange(&bus, RREG | (REG_ID << 7), 0, &prev);
	if (rc == 0) {
		rc = exchange(&bus, 0, 0, &id);
	}
	if (rc) {
		printk("ADS131M02 ID read err=%d (prev=0x%04X)\n", rc, prev);
		return rc;
	}
	if ((id >> 8) != 0x22) {
		printk("ADS131M02 unexpected ID=0x%04X (prev=0x%04X)\n", id, prev);
		return -ENODEV;
	}
	rc = write_reg(REG_MODE, MODE_24BIT_CCITT);
	if (rc) {
		return rc;
	}
	uint16_t clock = CLOCK_BOTH_HR;

	if (rate == BIAS_ADS131M02_64_KSPS) {
		clock |= CLOCK_TBM;
	}
	return write_reg(REG_CLOCK, clock);
}

int bias_ads131m02_init(enum bias_ads131m02_rate rate)
{
	if (rate != BIAS_ADS131M02_32_KSPS && rate != BIAS_ADS131M02_64_KSPS) {
		return -EINVAL;
	}
	int rc = claim();

	if (!rc) {
		if (streaming) {
			rc = -EBUSY;
		} else {
			rc = configure(rate);
			ready = rc == 0;
		}
		k_mutex_unlock(&control);
	}
	return rc;
}

#ifdef CONFIG_SOC_SERIES_STM32H5X
static const struct spi_dt_spec stream_bus =
	SPI_DT_SPEC_GET(ADC_NODE, SPI_WORD_SET(8) | SPI_LOCK_ON);
static const struct gpio_dt_spec cs = SPI_CS_GPIOS_DT_SPEC_GET(ADC_NODE);
static const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(ADC_NODE, drdy_gpios);
static struct gpio_callback interrupt;
static bias_ads131m02_callback deliver;
static void *cookie;
static uint8_t null_frame[ADS_FRAME_BYTES];
static uint8_t frames[2][ADS_FRAME_BYTES];
static unsigned int active;
static bool pending;
static int fault;

static void drdy_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct bias_ads131m02_sample sample;
	unsigned int previous = active;
	bool completed = pending;
	int rc = 0;

	if (fault) {
		return;
	}
	if (completed) {
		rc = ads_dma_complete();
		if (rc == -EAGAIN) {
			rc = -EOVERFLOW;
		}
		if (!rc) {
			rc = gpio_pin_set_dt(&cs, 0);
		}
	}
	if (!rc) {
		active ^= 1;
		rc = gpio_pin_set_dt(&cs, 1);
	}
	if (!rc) {
		ads_dma_launch(null_frame, frames[active]);
		pending = true;
		if (!completed) {
			return;
		}
		rc = ads_decode(frames[previous], &sample);
	}
	fault = rc;
	deliver(rc ? NULL : &sample, rc, cookie);
}

static int begin_stream(bias_ads131m02_callback callback, void *context)
{
	uint16_t status;
	int rc;

	if (!gpio_is_ready_dt(&drdy)) {
		return -ENODEV;
	}
	rc = gpio_pin_configure_dt(&drdy, GPIO_INPUT);
	if (rc) {
		return rc;
	}
	gpio_init_callback(&interrupt, drdy_edge, BIT(drdy.pin));
	rc = gpio_add_callback(drdy.port, &interrupt);
	if (rc) {
		return rc;
	}
	/* A valid NULL transfer configures SPI and reserves it until stop(). */
	rc = exchange(&stream_bus, 0, 0, &status);
	if (rc) {
		/* The SPI driver releases the bus on a transport error, but a
		 * successful transfer with invalid output CRC retains ownership.
		 */
		if (rc == -EBADMSG) {
			spi_release_dt(&stream_bus);
		}
		goto remove_callback;
	}
	rc = ads_dma_prepare();
	if (rc) {
		goto release_bus;
	}
	deliver = callback;
	cookie = context;
	fault = 0;
	pending = false;
	sys_put_be16(ads_crc(null_frame, 3), null_frame + 3);

	/* Resynchronize the conversion FIFO after the pause in acquisition.
	 * Keep this short pulse below the hardware-reset threshold.
	 */
	unsigned int key = irq_lock();
	rc = gpio_pin_set_dt(&sync_pin, 1);
	if (!rc) {
		k_busy_wait(2);
		rc = gpio_pin_set_dt(&sync_pin, 0);
	}
	if (!rc) {
		rc = gpio_pin_interrupt_configure_dt(&drdy, GPIO_INT_EDGE_TO_ACTIVE);
	}
	irq_unlock(key);
	if (!rc) {
		streaming = true;
		return 0;
	}
	ads_dma_shutdown();
release_bus:
	spi_release_dt(&stream_bus);
remove_callback:
	gpio_remove_callback(drdy.port, &interrupt);
	return rc;
}

int bias_ads131m02_start(bias_ads131m02_callback callback, void *context)
{
	if (!callback) {
		return -EINVAL;
	}
	int rc = claim();

	if (!rc) {
		rc = idle_status();
		if (!rc) {
			rc = begin_stream(callback, context);
		}
		k_mutex_unlock(&control);
	}
	return rc;
}

int bias_ads131m02_stop(void)
{
	int rc = claim();

	if (rc) {
		return rc;
	}
	if (!streaming) {
		rc = -EALREADY;
		goto out;
	}
	rc = gpio_pin_interrupt_configure_dt(&drdy, GPIO_INT_DISABLE);
	if (!rc) {
		rc = ads_dma_shutdown();
	}
	if (!rc) {
		rc = gpio_pin_set_dt(&cs, 0);
	}
	if (!rc) {
		rc = gpio_remove_callback(drdy.port, &interrupt);
	}
	if (!rc) {
		rc = spi_release_dt(&stream_bus);
	}
	if (!rc) {
		streaming = false;
		rc = fault;
	}
out:
	k_mutex_unlock(&control);
	return rc;
}
#else
int bias_ads131m02_start(bias_ads131m02_callback callback, void *context)
{
	return -ENOTSUP;
}

int bias_ads131m02_stop(void)
{
	return -ENOTSUP;
}
#endif
