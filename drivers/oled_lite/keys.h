/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

#define BIAS_KEY_COUNT 3U
#define BIAS_KEY_DEBOUNCE_MS 20U

enum bias_key_id {
	BIAS_KEY_1 = 0,
	BIAS_KEY_2 = 1,
	BIAS_KEY_3 = 2,
};

typedef void (*bias_keys_callback_t)(enum bias_key_id key, bool pressed, void *context);

/* Thread context. Configures KEY1-KEY3 with 20 ms software debounce. */
int bias_keys_init(bias_keys_callback_t callback, void *context);
int bias_keys_read(enum bias_key_id key, bool *pressed);
int bias_keys_deinit(void);
