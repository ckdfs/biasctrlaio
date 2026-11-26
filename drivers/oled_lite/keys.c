/* SPDX-License-Identifier: Apache-2.0 */
#include "keys.h"
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct bias_key_entry {
	struct gpio_dt_spec spec;
	struct gpio_callback cb;
	struct k_work_delayable work;
	enum bias_key_id id;
	bool stable_pressed;
	bool configured;
};

static struct bias_key_entry key_entries[BIAS_KEY_COUNT] = {
	[BIAS_KEY_1] = {
		.spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
		.id = BIAS_KEY_1,
	},
	[BIAS_KEY_2] = {
		.spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
		.id = BIAS_KEY_2,
	},
	[BIAS_KEY_3] = {
		.spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
		.id = BIAS_KEY_3,
	},
};

BUILD_ASSERT(ARRAY_SIZE(key_entries) == BIAS_KEY_COUNT);

static bias_keys_callback_t user_callback;
static void *user_context;
static bool initialized;

static void key_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct bias_key_entry *entry = CONTAINER_OF(dwork, struct bias_key_entry, work);
	int raw = gpio_pin_get_dt(&entry->spec);

	if (raw < 0) {
		return;
	}
	bool pressed = (raw != 0);

	if (pressed != entry->stable_pressed) {
		entry->stable_pressed = pressed;
		if (user_callback != NULL) {
			user_callback(entry->id, pressed, user_context);
		}
	}
}

static void key_isr_handler(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	struct bias_key_entry *entry = CONTAINER_OF(cb, struct bias_key_entry, cb);

	k_work_reschedule(&entry->work, K_MSEC(BIAS_KEY_DEBOUNCE_MS));
}

static void cleanup_keys(void)
{
	for (size_t i = 0; i < BIAS_KEY_COUNT; ++i) {
		struct bias_key_entry *entry = &key_entries[i];

		if (entry->configured) {
			(void)gpio_pin_interrupt_configure_dt(&entry->spec, GPIO_INT_DISABLE);
			(void)gpio_remove_callback(entry->spec.port, &entry->cb);
			(void)k_work_cancel_delayable(&entry->work);
			entry->configured = false;
		}
	}
	user_callback = NULL;
	user_context = NULL;
	initialized = false;
}

int bias_keys_init(bias_keys_callback_t callback, void *context)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	if (initialized) {
		cleanup_keys();
	}
	for (size_t i = 0; i < BIAS_KEY_COUNT; ++i) {
		if (!gpio_is_ready_dt(&key_entries[i].spec)) {
			return -ENODEV;
		}
	}

	user_callback = callback;
	user_context = context;

	for (size_t i = 0; i < BIAS_KEY_COUNT; ++i) {
		struct bias_key_entry *entry = &key_entries[i];
		int rc = gpio_pin_configure_dt(&entry->spec, GPIO_INPUT);

		if (rc < 0) {
			cleanup_keys();
			return rc;
		}
		rc = gpio_pin_get_dt(&entry->spec);
		if (rc < 0) {
			cleanup_keys();
			return rc;
		}
		entry->stable_pressed = (rc != 0);
		k_work_init_delayable(&entry->work, key_work_handler);
		gpio_init_callback(&entry->cb, key_isr_handler, BIT(entry->spec.pin));
		rc = gpio_add_callback(entry->spec.port, &entry->cb);
		if (rc < 0) {
			cleanup_keys();
			return rc;
		}
		entry->configured = true;
		rc = gpio_pin_interrupt_configure_dt(&entry->spec, GPIO_INT_EDGE_BOTH);
		if (rc < 0) {
			cleanup_keys();
			return rc;
		}
	}

	initialized = true;
	return 0;
}

int bias_keys_read(enum bias_key_id key, bool *pressed)
{
	if ((unsigned int)key >= BIAS_KEY_COUNT || pressed == NULL) {
		return -EINVAL;
	}
	if (!initialized) {
		return -EACCES;
	}
	*pressed = key_entries[key].stable_pressed;
	return 0;
}

int bias_keys_deinit(void)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	cleanup_keys();
	return 0;
}
