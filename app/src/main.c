/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool led_ready;

BUILD_ASSERT(DT_PROP_LEN(USER_NODE, pilot_sw_gpios) == 8);
static const struct gpio_dt_spec pilots[] = {
	DT_FOREACH_PROP_ELEM_SEP(USER_NODE, pilot_sw_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,))
};

#define IDLE_OUTPUT(prop) { #prop, GPIO_DT_SPEC_GET(USER_NODE, prop) }
static const struct {
	const char *name;
	struct gpio_dt_spec gpio;
} idle_outputs[] = {
	IDLE_OUTPUT(spare_pa8_gpios),
	IDLE_OUTPUT(dac_clr_gpios),
	IDLE_OUTPUT(dac_sync_gpios),
	IDLE_OUTPUT(dac_ldac_gpios),
	IDLE_OUTPUT(adc_cs_gpios),
	IDLE_OUTPUT(adc_sync_gpios),
};
static const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(USER_NODE, adc_drdy_gpios);

struct key {
	const char *name;
	struct gpio_dt_spec gpio;
	struct gpio_callback callback;
	struct k_work work;
};

#define KEY(alias) { .name = DT_PROP(DT_ALIAS(alias), label), \
		     .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios) }
static struct key keys[] = { KEY(sw0), KEY(sw1), KEY(sw2) };

static void fault(const char *name, const char *operation, int error)
{
	printk("FAULT: %s %s failed (%d)\n", name, operation, error);
	while (led_ready) {
		int rc = gpio_pin_toggle_dt(&led);

		if (rc < 0) {
			printk("FAULT: LED1 fault blink failed (%d)\n", rc);
			break;
		}
		k_msleep(100);
	}
	k_thread_suspend(k_current_get());
}

static void configure_gpio(const char *name, const struct gpio_dt_spec *gpio, gpio_flags_t flags)
{
	int rc = gpio_is_ready_dt(gpio) ? gpio_pin_configure_dt(gpio, flags) : -ENODEV;

	if (rc < 0) {
		printk("%s pin %u: ", gpio->port->name, gpio->pin);
		fault(name, "GPIO setup", rc);
	}
}

static void report_key(struct k_work *work)
{
	struct key *key = CONTAINER_OF(work, struct key, work);

	printk("%s pressed\n", key->name);
}

static void key_interrupt(const struct device *port, struct gpio_callback *callback,
			  gpio_port_pins_t pins)
{
	struct key *key = CONTAINER_OF(callback, struct key, callback);

	k_work_submit(&key->work);
}

static void init_key(struct key *key)
{
	configure_gpio(key->name, &key->gpio, GPIO_INPUT);
	k_work_init(&key->work, report_key);
	gpio_init_callback(&key->callback, key_interrupt, BIT(key->gpio.pin));

	int rc = gpio_add_callback(key->gpio.port, &key->callback);

	if (rc < 0) {
		fault(key->name, "callback registration", rc);
	}
	rc = gpio_pin_interrupt_configure_dt(&key->gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc < 0) {
		fault(key->name, "interrupt setup", rc);
	}
}

static void scan_i2c(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t dummy;
	/* Address-only write with STOP, as in Zephyr's I2C shell scan. */
	struct i2c_msg probe = { .buf = &dummy, .len = 0, .flags = I2C_MSG_WRITE | I2C_MSG_STOP };
	unsigned int found = 0;

	if (!device_is_ready(bus)) {
		fault("I2C1", "device readiness", -ENODEV);
	}
	for (unsigned int addr = 0x08; addr <= 0x77; addr++) {
		int rc = i2c_transfer(bus, &probe, 1, addr);

		if (rc == 0) {
			printk("I2C1 ACK at 0x%02X\n", addr);
			found++;
		} else if (rc != -EIO) {
			/* STM32 v2 reports address NACK as -EIO. */
			printk("I2C1 address 0x%02X: ", addr);
			fault("I2C1", "probe", rc);
		}
	}
	printk("I2C1 scan complete, %u responding addresses\n", found);
}

int main(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(pilots); i++) {
		configure_gpio("pilot switch", &pilots[i], GPIO_OUTPUT_INACTIVE);
	}
	for (size_t i = 0; i < ARRAY_SIZE(idle_outputs); i++) {
		configure_gpio(idle_outputs[i].name, &idle_outputs[i].gpio, GPIO_OUTPUT_INACTIVE);
	}
	configure_gpio("ADC DRDY", &drdy, GPIO_INPUT);
	configure_gpio("LED1", &led, GPIO_OUTPUT_INACTIVE);
	led_ready = true;

	printk("biasctrl_h503 bring-up, configured SYSCLK %u Hz\n",
	       (unsigned int)DT_PROP(DT_NODELABEL(rcc), clock_frequency));
	for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
		init_key(&keys[i]);
	}
	scan_i2c();

	while (true) {
		int rc = gpio_pin_toggle_dt(&led);

		if (rc < 0) {
			fault("LED1", "heartbeat", rc);
		}
		k_msleep(500);
	}
}
