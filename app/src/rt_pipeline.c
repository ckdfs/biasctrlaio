/* SPDX-License-Identifier: Apache-2.0 */
#include "rt_pipeline.h"

#include <ads131m02.h>
#include <zephyr/sys/byteorder.h>
#ifdef CONFIG_BOARD_BIASCTRL_H503
#include <cmsis_core.h>
#endif
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

static int32_t low[2], high[2];
static struct {
	bool pending, step;
	uint32_t boundary;
	uint16_t code, final_code;
	float amp;
} scheduled;

static uint8_t packed[DSP_GOERTZEL_BLOCK_SIZE][3];
BUILD_ASSERT(sizeof(packed) == 3840);
static K_SEM_DEFINE(capture_done, 0, 1);
static bool capturing, capture_dump;
static uint32_t capture_count, capture_pos, previous;
static rt_adc_stats_t timing;

static void capture_sample(int32_t sample, uint32_t now)
{
	if (capture_pos > capture_count) {
		return;
	}
	if (!capture_pos) {
		timing.first = now;
	}
	if (capture_dump) {
		sys_put_le24((uint32_t)sample, packed[capture_pos]);
	} else if (capture_pos) {
		uint32_t dt = now - previous;
		timing.total += dt;
		timing.min = MIN(timing.min, dt);
		timing.max = MAX(timing.max, dt);
	}
	previous = now;
	if (++capture_pos == capture_count + (capture_dump ? 0U : 1U)) {
		capture_pos = capture_count + 1;
		k_sem_give(&capture_done);
	}
}

int32_t rt_pipeline_dump_sample(unsigned index)
{
	uint32_t raw = sys_get_le24(packed[index]);
	return (int32_t)(raw ^ 0x800000U) - 0x800000;
}

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
		k_sem_give(&capture_done);
		k_work_submit(&fault_work);
	}
	return atomic_get(&fault);
}

int rt_pipeline_fault(void)
{
	return atomic_get(&fault);
}

bool rt_pipeline_dc_pending(void)
{
	return atomic_get(&dc_pending) != 0;
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

static int goertzel_result_fast(const goertzel_state_t *g, float *mag, float *phase, float *iq)
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
	iq[0] = re * (2.f / DSP_GOERTZEL_BLOCK_SIZE);
	iq[1] = im * (2.f / DSP_GOERTZEL_BLOCK_SIZE);
	return 0;
}

static int publish_block(uint32_t timestamp)
{
	rt_pipeline_block_t block = {
		.seq = sequence++,
		.t_ms = k_uptime_get_32(),
		.timestamp_cycles = timestamp,
		.pilot_amplitude_v = pilot.amplitude,
	};
	harmonic_data_t *h = &block.harmonics;

	if (goertzel_result_fast(&h1, &h->h1_magnitude, &h->h1_phase, block.iq) != 0 ||
	    goertzel_result_fast(&h2, &h->h2_magnitude, &h->h2_phase, block.iq + 2) != 0) {
		return trip(-EIO);
	}
	for (unsigned i = 0; i < 2; ++i) {
		block.min_v[i] = low[i] * RT_PIPELINE_ADC_V_PER_COUNT;
		block.max_v[i] = high[i] * RT_PIPELINE_ADC_V_PER_COUNT;
		low[i] = INT32_MAX;
		high[i] = INT32_MIN;
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

/* Complete SPI1 DMA and latch the output within the shared deadline. */
static int finish_dac(uint32_t start_cycle, uint32_t limit)
{
	int rc;
	do {
		rc = dac8568_status();
	} while (rc == -EBUSY && k_cycle_get_32() - start_cycle < limit);
	return rc == -EBUSY ? -ETIMEDOUT : rc;
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
	if (capturing) {
		capture_sample(ch0, timestamp_cycles);
		return 0;
	}
	low[0] = MIN(low[0], ch0);
	high[0] = MAX(high[0], ch0);
	low[1] = MIN(low[1], ch1);
	high[1] = MAX(high[1], ch1);
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

	bool boundary = scheduled.pending && dc_accum_ready(&dc_monitor) &&
			sequence == scheduled.boundary;
	if (boundary || atomic_get(&dc_pending)) {
		uint8_t channel = boundary ? 0 : pending_dc;
		uint16_t value = boundary ? scheduled.code : pending_code;
		int dc_rc = 0;

		if (physical_dac) {
			dc_rc = finish_dac(pilot_start, k_us_to_cyc_ceil32(8));
		}
		if (dc_rc == 0) {
			dc_rc = physical_dac ?
				dac8568_set_dc_code_async(channel, value) :
				board->set_dc_code(channel, value);
		}
		if (dc_rc == -EBUSY) {
			dc_rc = -ETIMEDOUT;
		}
		if (boundary) {
			requested_amp = scheduled.amp;
			scheduled.pending = scheduled.step;
			scheduled.step = false;
			scheduled.code = scheduled.final_code;
			scheduled.boundary += 15;
		} else {
			dc_result = dc_rc;
			atomic_clear(&dc_pending);
			k_sem_give(&dc_done);
		}
		if (dc_rc < 0) {
			return trip(dc_rc);
		}
	}
	if (!dc_accum_ready(&dc_monitor)) {
		return 0;
	}
	rc = publish_block(timestamp_cycles);
	if (!rc && boundary && physical_dac) {
		rc = finish_dac(pilot_start, k_us_to_cyc_ceil32(12));
		if (rc) {
			return trip(rc);
		}
	}
	return rc;
}

static void adc_callback(const struct bias_ads131m02_sample *sample, int error, void *unused)
{
	ARG_UNUSED(unused);
	uint32_t timestamp;
#ifdef CONFIG_BOARD_BIASCTRL_H503
	timestamp = DWT->CYCCNT;
#else
	timestamp = k_cycle_get_32();
#endif
	if (!sample) {
		rt_pipeline_ingest(0, 0, error ? error : -EIO, timestamp);
	} else {
		rt_pipeline_ingest(sample->channel[0], sample->channel[1], error, timestamp);
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
	low[0] = low[1] = INT32_MAX;
	high[0] = high[1] = INT32_MIN;
	scheduled.pending = false;
#ifdef CONFIG_BOARD_BIASCTRL_H503
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
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

int rt_pipeline_schedule(uint16_t code, float amplitude, bool step, uint16_t final_code,
			 uint32_t *first_seq)
{
	unsigned int key = irq_lock();
	int rc = rt_pipeline_fault();
	if (!rc && (!atomic_get(&running) || scheduled.pending)) {
		rc = -EBUSY;
	}
	if (!rc) {
		scheduled.boundary = sequence;
		scheduled.code = code;
		scheduled.final_code = final_code;
		scheduled.step = step;
		scheduled.amp = amplitude;
		scheduled.pending = true;
		*first_seq = sequence + 1 + (step ? 15U : 0U);
	}
	irq_unlock(key);
	return rc;
}

int rt_pipeline_read_regs(uint16_t regs[4])
{
	k_mutex_lock(&control, K_FOREVER);
	int rc = atomic_get(&running) ? -EBUSY : bias_ads131m02_read_reg(0, &regs[0]);
	if (rc == -EACCES) {
		/* Cold boot only. Preserve the live configuration on subsequent reads. */
		rc = bias_ads131m02_init(BIAS_ADS131M02_64_KSPS);
		if (!rc) {
			rc = bias_ads131m02_read_reg(0, &regs[0]);
		}
	}
	for (unsigned i = 1; !rc && i < 4; ++i) {
		rc = bias_ads131m02_read_reg(i, &regs[i]);
	}
	k_mutex_unlock(&control);
	return rc;
}

/* Called by the controller while it owns the idle board. Stop the ADC before
 * returning the segment to its UART consumer. */
int rt_pipeline_capture(bool dump, uint32_t count, bool manual, rt_adc_stats_t *stats)
{
	if (!count || count > (dump ? ARRAY_SIZE(packed) : 1000000U)) {
		return -ERANGE;
	}
	capture_pos = 0;
	capture_count = count;
	capture_dump = dump;
	timing = (rt_adc_stats_t){.min = UINT32_MAX};
	capturing = true;
	k_sem_reset(&capture_done);
	int rc = board->set_pilot_code(0, 32768);
	if (!rc && physical_dac) {
		rc = finish_dac(k_cycle_get_32(), k_us_to_cyc_ceil32(1000));
	}
	if (!rc) {
		rc = start(!manual);
	}
	if (!rc) {
		k_timeout_t timeout = K_MSEC(count / 64 + 250);
#ifdef CONFIG_BOARD_BIASCTRL_H503
		/* CYCCNT can halt in WFI. Keep the core awake during a rate
		 * measurement. Interrupts and higher priority threads stay enabled. */
		if (!dump && !manual) {
			int64_t deadline = k_uptime_get() + count / 64 + 250;
			while ((rc = k_sem_take(&capture_done, K_NO_WAIT)) &&
			       k_uptime_get() < deadline) {
				k_busy_wait(50);
			}
			if (rc) {
				rc = -ETIMEDOUT;
			}
		} else
#endif
		{
			rc = k_sem_take(&capture_done, timeout);
		}
		if (rc == -EAGAIN) {
			rc = -ETIMEDOUT;
		}
		int stop_rc = rt_pipeline_stop();
		if (stop_rc) {
			rc = stop_rc;
		}
	}
	capturing = false;
	*stats = timing;
	return rc;
}
