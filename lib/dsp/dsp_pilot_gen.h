#ifndef DSP_PILOT_GEN_H
#define DSP_PILOT_GEN_H

#include "dsp_types.h"

/* Each DAC channel owns its generator. Phase is phase_index / period turns. */
typedef struct {
    uint32_t phase_index, step, period;
    float radians_per_tick, amplitude;
} pilot_gen_t;

/* Accepts frequencies whose exact reduced period fits in uint32_t. */
dsp_status_t pilot_gen_init(pilot_gen_t *gen, float frequency, float sample_rate,
                            float amplitude);
float pilot_gen_current(const pilot_gen_t *gen);
float pilot_gen_next(pilot_gen_t *gen);
void pilot_gen_reset(pilot_gen_t *gen);
dsp_status_t pilot_gen_set_amplitude(pilot_gen_t *gen, float amplitude);
uint32_t pilot_gen_get_phase_index(const pilot_gen_t *gen);

#endif
