#include "dsp_goertzel.h"
#include "dsp_tone_ratio.h"

dsp_status_t goertzel_init(goertzel_state_t *g, float target_freq,
                           float sample_rate, uint32_t block_size)
{
    if (!g || !block_size) return DSP_INVALID_ARGUMENT;
    uint32_t cycles, period;
    dsp_status_t status = dsp_tone_ratio(target_freq, sample_rate, &cycles, &period);
    if (status != DSP_OK) return status;
    if (block_size % period != 0) return DSP_INCOHERENT;

    double omega = DSP_TAU * cycles / period;
    double c = cos(omega);
    *g = (goertzel_state_t){.block_size = block_size, .coeff = 2 * c,
                            .cos_term = c, .sin_term = sin(omega)};
    return DSP_OK;
}

dsp_status_t goertzel_process_sample(goertzel_state_t *g, float sample)
{
    if (!g || !isfinite(sample)) return DSP_INVALID_ARGUMENT;
    if (g->count >= g->block_size) return DSP_BLOCK_FULL;
    double s = sample + g->coeff * g->s1 - g->s2;
    g->s2 = g->s1;
    g->s1 = s;
    ++g->count;
    return DSP_OK;
}

bool goertzel_block_ready(const goertzel_state_t *g)
{
    return g && g->block_size && g->count == g->block_size;
}

dsp_status_t goertzel_get_result(const goertzel_state_t *g, float *magnitude, float *phase)
{
    if (!g || !magnitude || !phase) return DSP_INVALID_ARGUMENT;
    if (!goertzel_block_ready(g)) return DSP_INCOMPLETE;
    /* Final zero-input step aligns the DFT phase to sample zero. */
    double re = g->cos_term * g->s1 - g->s2;
    double im = g->sin_term * g->s1;
    *magnitude = (float)(2 * hypot(re, im) / g->block_size);
    *phase = (float)atan2(im, re);
    return DSP_OK;
}

void goertzel_reset(goertzel_state_t *g)
{
    g->s1 = g->s2 = 0;
    g->count = 0;
}

dsp_status_t dc_accum_init(dc_accum_t *d, uint32_t block_size)
{
    if (!d || !block_size) return DSP_INVALID_ARGUMENT;
    *d = (dc_accum_t){.block_size = block_size};
    return DSP_OK;
}

dsp_status_t dc_accum_process(dc_accum_t *d, float sample)
{
    if (!d || !isfinite(sample)) return DSP_INVALID_ARGUMENT;
    if (d->count >= d->block_size) return DSP_BLOCK_FULL;
    d->sum += sample;
    ++d->count;
    return DSP_OK;
}

bool dc_accum_ready(const dc_accum_t *d)
{
    return d && d->block_size && d->count == d->block_size;
}

float dc_accum_get_mean(const dc_accum_t *d)
{
    return dc_accum_ready(d) ? (float)(d->sum / d->block_size) : NAN;
}

void dc_accum_reset(dc_accum_t *d)
{
    d->sum = 0;
    d->count = 0;
}
