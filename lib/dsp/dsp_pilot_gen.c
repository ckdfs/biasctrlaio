#include "dsp_pilot_gen.h"
#include "dsp_tone_ratio.h"

dsp_status_t pilot_gen_init(pilot_gen_t *gen, float frequency, float sample_rate,
                            float amplitude)
{
    if (!gen || !isfinite(amplitude)) return DSP_INVALID_ARGUMENT;
    uint32_t cycles, period;
    dsp_status_t status = dsp_tone_ratio(frequency, sample_rate, &cycles, &period);
    if (status != DSP_OK) return status;
    *gen = (pilot_gen_t){.period = period, .step = cycles,
                         .radians_per_tick = (float)(DSP_TAU / period), .amplitude = amplitude};
    return DSP_OK;
}

float pilot_gen_current(const pilot_gen_t *gen)
{
    return gen->amplitude * sinf(gen->phase_index * gen->radians_per_tick);
}

float pilot_gen_next(pilot_gen_t *gen)
{
    float value = pilot_gen_current(gen);
    uint32_t until_wrap = gen->period - gen->step;
    if (gen->phase_index >= until_wrap)
        gen->phase_index -= until_wrap;
    else
        gen->phase_index += gen->step;
    return value;
}

void pilot_gen_reset(pilot_gen_t *gen)
{
    gen->phase_index = 0;
}

dsp_status_t pilot_gen_set_amplitude(pilot_gen_t *gen, float amplitude)
{
    if (!gen || !isfinite(amplitude)) return DSP_INVALID_ARGUMENT;
    gen->amplitude = amplitude;
    return DSP_OK;
}

uint32_t pilot_gen_get_phase_index(const pilot_gen_t *gen)
{
    return gen->phase_index;
}
