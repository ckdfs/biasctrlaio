/* SPDX-License-Identifier: Apache-2.0 */
#include "rt_pipeline.h"

#include <ads131m02.h>
#include <dac8568.h>
#include <dsp_goertzel.h>
#include <dsp_pilot_gen.h>
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#define USER_NODE DT_PATH(zephyr_user)
#if DT_NODE_HAS_PROP(USER_NODE, pilot_sw_gpios)
#define SWITCH(node, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),
static const struct gpio_dt_spec switches[] = {
	DT_FOREACH_PROP_ELEM(USER_NODE, pilot_sw_gpios, SWITCH)
};
BUILD_ASSERT(ARRAY_SIZE(switches) == 8);
#elif defined(CONFIG_BOARD_BIASCTRL_H503)
#error "Missing pilot switch GPIOs"
#endif

K_MSGQ_DEFINE(rt_pipeline_blocks, sizeof(rt_pipeline_block_t), DSP_CONTROL_DECIMATION, 4);
static K_MUTEX_DEFINE(control);
static K_SEM_DEFINE(dc_done, 0, 1);
static atomic_t dc_pending;
static uint8_t pending_dc;
static uint16_t pending_code;
static int dc_result;
static atomic_t running;
static atomic_t fault;
static atomic_t drops;
static atomic_t crc_errors;
static atomic_t cleanup_pending;
static atomic_t cutoff_error;

static const analog_board_ops_t *board;
static rt_pipeline_fault_cb_t fault_cb;
static bool adc_started;
static bool physical_dac;
static goertzel_state_t h1, h2;
static dc_accum_t dc_monitor;
static pilot_gen_t pilot;
static float requested_amp;
static uint32_t sequence;

static void cleanup(struct k_work *work)
{
	ARG_UNUSED(work);
	k_mutex_lock(&control, K_FOREVER);
	int reason = atomic_get(&fault);
	int err = atomic_get(&cutoff_error);
	int rc = board->disconnect_all();

	if (!err) {
		err = rc;
	}
	if (adc_started) {
		rc = bias_ads131m02_stop();
		/* stop also returns the original ADC stream fault after release. */
		if (rc && rc != reason && !err) {
			err = rc;
		}
		adc_started = rc && rc != reason && rc != -EALREADY;
	}
	rt_pipeline_fault_cb_t notify = fault_cb;
	k_mutex_unlock(&control);
	if (notify) {
		notify(reason, err);
	}
	atomic_clear(&cleanup_pending);
}

static K_WORK_DEFINE(fault_work, cleanup);

static int trip(int error)
{
	unsigned int key = irq_lock();
	bool first = atomic_cas(&fault, 0, error);

	if (first) {
		atomic_set(&cleanup_pending, 1);
		atomic_clear(&running);
#if DT_NODE_HAS_PROP(USER_NODE, pilot_sw_gpios)
		for (size_t i = 0; i < ARRAY_SIZE(switches); ++i) {
			int rc = gpio_pin_set_raw(switches[i].port, switches[i].pin, 0);

			if (rc < 0) {
				atomic_cas(&cutoff_error, 0, rc);
			}
		}
#endif
	}
	irq_unlock(key);
	if (first) {
		if (error == -EBADMSG) {
			atomic_inc(&crc_errors);
		}
		if (error == -EOVERFLOW || error == -EBADMSG || error == -ETIMEDOUT) {
			atomic_inc(&drops);
		}
		atomic_clear(&dc_pending);
		k_sem_give(&dc_done);
		k_work_submit(&fault_work);
	}
	return atomic_get(&fault);
}

int rt_pipeline_fault(void)
{
	return atomic_get(&fault);
}

uint32_t rt_pipeline_drop_count(void)
{
	return atomic_get(&drops);
}

uint32_t rt_pipeline_crc_error_count(void)
{
	return atomic_get(&crc_errors);
}

int rt_pipeline_init(const analog_board_ops_t *ops, rt_pipeline_fault_cb_t callback)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	const analog_board_ops_t *defaults = analog_board_get_ops();

	if (!ops) {
		ops = defaults;
	}
	if (!ops || !ops->set_pilot_code || !ops->set_dc_code || !ops->disconnect_all) {
		return -EINVAL;
	}
	k_mutex_lock(&control, K_FOREVER);
	int rc = -EBUSY;

	if (!atomic_get(&running) && !atomic_get(&cleanup_pending) && !adc_started) {
		board = ops;
		physical_dac = ops == defaults;
		fault_cb = callback;
		requested_amp = 0;
		atomic_clear(&fault);
		atomic_clear(&cutoff_error);
		atomic_clear(&drops);
		atomic_clear(&crc_errors);
		rc = 0;
	}
	k_mutex_unlock(&control);
	return rc;
}

static int goertzel_result_fast(const goertzel_state_t *g, float *mag, float *phase)
{
	if (!goertzel_block_ready(g)) {
		return -EIO;
	}
	float re = (float)(g->cos_term * g->s1 - g->s2);
	float im = (float)(g->sin_term * g->s1);

	if (!isfinite(re) || !isfinite(im)) {
		return -EIO;
	}
	*mag = (2.0f * sqrtf(re * re + im * im)) / (float)g->block_size;
	*phase = atan2f(im, re);
	return 0;
}

static int publish_block(uint32_t timestamp)
{
	rt_pipeline_block_t block = {
		.seq = sequence++,
		.timestamp_cycles = timestamp,
		.pilot_amplitude_v = pilot.amplitude,
	};
	harmonic_data_t *h = &block.harmonics;

	if (goertzel_result_fast(&h1, &h->h1_magnitude, &h->h1_phase) != 0 ||
	    goertzel_result_fast(&h2, &h->h2_magnitude, &h->h2_phase) != 0) {
		return trip(-EIO);
	}
	h->dc_power = dc_accum_get_mean(&dc_monitor);
	if (!isfinite(h->dc_power)) {
		return trip(-EIO);
	}
	if (k_msgq_put(&rt_pipeline_blocks, &block, K_NO_WAIT)) {
		return trip(-EOVERFLOW);
	}
	goertzel_reset(&h1);
	goertzel_reset(&h2);
	dc_accum_reset(&dc_monitor);
	return 0;
}

int rt_pipeline_ingest(int32_t ch0, int32_t ch1, int error, uint32_t timestamp_cycles)
{
	if (!atomic_get(&running)) {
		int rc = rt_pipeline_fault();

		return rc ? rc : -ESHUTDOWN;
	}
	if (error) {
		return trip(error < 0 ? error : -EIO);
	}
	if (!h1.count && pilot_gen_set_amplitude(&pilot, requested_amp) != DSP_OK) {
		return trip(-EIO);
	}

	/* Submit DITHER1 first so SPI1 DMA overlaps with Goertzel execution. */
	float v = pilot_gen_next(&pilot);
	float code = 32768.0f + v * (32768.0f / RT_PIPELINE_PILOT_MAX_V);
	uint32_t pilot_start = k_cycle_get_32();
	int rc = board->set_pilot_code(0, (uint16_t)CLAMP(code + 0.5f, 0.0f, 65535.0f));

	if (rc < 0) {
		return trip(rc);
	}

	float sample = ch0 * RT_PIPELINE_ADC_V_PER_COUNT;

	if (goertzel_process_sample(&h1, sample) != DSP_OK ||
	    goertzel_process_sample(&h2, sample) != DSP_OK ||
	    dc_accum_process(&dc_monitor, ch1 * RT_PIPELINE_ADC_V_PER_COUNT) != DSP_OK) {
		return trip(-EIO);
	}

	if (atomic_get(&dc_pending)) {
		int dc_rc = 0;

		if (physical_dac) {
			const uint32_t wait_limit = k_us_to_cyc_ceil32(8);

			do {
				dc_rc = dac8568_status();
			} while (dc_rc == -EBUSY &&
				 (k_cycle_get_32() - pilot_start) < wait_limit);
		}
		if (dc_rc == 0) {
			dc_rc = physical_dac ?
				dac8568_set_dc_code_async(pending_dc, pending_code) :
				board->set_dc_code(pending_dc, pending_code);
		}
		if (dc_rc == -EBUSY) {
			dc_rc = -ETIMEDOUT;
		}
		dc_result = dc_rc;
		__DMB();
		atomic_clear(&dc_pending);
		k_sem_give(&dc_done);
		if (dc_rc < 0) {
			return trip(dc_rc);
		}
	}
	return dc_accum_ready(&dc_monitor) ? publish_block(timestamp_cycles) : 0;
}

static void adc_callback(const struct bias_ads131m02_sample *sample, int error, void *unused)
{
	ARG_UNUSED(unused);
	if (!sample) {
		rt_pipeline_ingest(0, 0, error ? error : -EIO, k_cycle_get_32());
	} else {
		rt_pipeline_ingest(sample->channel[0], sample->channel[1], error, k_cycle_get_32());
	}
}

static int start(bool hardware)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	k_mutex_lock(&control, K_FOREVER);
	int rc = rt_pipeline_fault();

	if (rc || !board || atomic_get(&running)) {
		rc = rc ? rc : (board ? -EBUSY : -ENODEV);
		goto out;
	}
	if (goertzel_init(&h1, DSP_PILOT_FREQ_HZ, DSP_SAMPLE_RATE_HZ,
			  DSP_GOERTZEL_BLOCK_SIZE) != DSP_OK ||
	    goertzel_init(&h2, 2 * DSP_PILOT_FREQ_HZ, DSP_SAMPLE_RATE_HZ,
			  DSP_GOERTZEL_BLOCK_SIZE) != DSP_OK ||
	    dc_accum_init(&dc_monitor, DSP_GOERTZEL_BLOCK_SIZE) != DSP_OK ||
	    pilot_gen_init(&pilot, DSP_PILOT_FREQ_HZ, DSP_SAMPLE_RATE_HZ, requested_amp) != DSP_OK) {
		rc = trip(-EIO);
		goto out;
	}
	if (hardware) {
		rc = bias_ads131m02_init(BIAS_ADS131M02_64_KSPS);
		if (rc) {
			rc = trip(rc);
			goto out;
		}
	}
	sequence = 0;
	k_msgq_purge(&rt_pipeline_blocks);
	k_sem_reset(&dc_done);
	atomic_clear(&dc_pending);
	atomic_set(&running, 1);
	if (hardware) {
		rc = bias_ads131m02_start(adc_callback, NULL);
		if (rc) {
			rc = trip(rc);
		} else {
			adc_started = true;
			rc = rt_pipeline_fault();
		}
	}
out:
	k_mutex_unlock(&control);
	return rc;
}

int rt_pipeline_start(void)
{
	return start(true);
}

int rt_pipeline_start_manual(void)
{
	return start(false);
}

int rt_pipeline_stop(void)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	k_mutex_lock(&control, K_FOREVER);
	int rc = rt_pipeline_fault();

	if (!rc && atomic_clear(&running) && adc_started) {
		rc = bias_ads131m02_stop();
		if (rc) {
			rc = trip(rc);
		} else {
			adc_started = false;
		}
	}
	if (!rc) {
		rc = rt_pipeline_fault();
	}
	k_mutex_unlock(&control);
	return rc;
}

int rt_pipeline_set_pilot_amplitude(float amp_v)
{
	if (!isfinite(amp_v) || amp_v < 0 || amp_v > RT_PIPELINE_PILOT_MAX_V) {
		return -EINVAL;
	}
	unsigned int key = irq_lock();
	int rc = rt_pipeline_fault();

	if (!rc && board) {
		requested_amp = amp_v;
	} else if (!rc) {
		rc = -ENODEV;
	}
	irq_unlock(key);
	return rc;
}

int rt_pipeline_get_block(rt_pipeline_block_t *block, k_timeout_t timeout)
{
	return block ? k_msgq_get(&rt_pipeline_blocks, block, timeout) : -EINVAL;
}

int rt_pipeline_set_dc_code(uint8_t dc, uint16_t code)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	k_mutex_lock(&control, K_FOREVER);
	int rc = rt_pipeline_fault();

	if (!rc && !board) {
		rc = -ENODEV;
	} else if (!rc && board->caps && dc >= board->caps->dc_count) {
		rc = -EINVAL;
	} else if (!rc) {
		if (!atomic_get(&running)) {
			rc = board->set_dc_code(dc, code);
			if (!rc) {
				rc = rt_pipeline_fault();
			}
		} else {
			pending_dc = dc;
			pending_code = code;
			dc_result = -ETIMEDOUT;
			__DMB();
			k_sem_reset(&dc_done);
			atomic_set(&dc_pending, 1);

			if (k_sem_take(&dc_done, K_MSEC(5)) != 0) {
				atomic_clear(&dc_pending);
				rc = -ETIMEDOUT;
			} else {
				int flt = rt_pipeline_fault();

				rc = flt ? flt : dc_result;
			}
		}
		if (rc < 0) {
			rc = trip(rc);
		}
	}
	k_mutex_unlock(&control);
	return rc;
}
