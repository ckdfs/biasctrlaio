#ifndef DSP_GOERTZEL_H
#define DSP_GOERTZEL_H

#include "dsp_types.h"
#include <stdbool.h>

/* Use one instance per H1 or H2 bin. Only positive, sub-Nyquist
 * frequencies with a whole number of cycles per block are valid. */
typedef struct {
    double coeff, cos_term, sin_term, s1, s2;
    uint32_t count, block_size;
} goertzel_state_t;

dsp_status_t goertzel_init(goertzel_state_t *g, float target_freq,
                           float sample_rate, uint32_t block_size);
dsp_status_t goertzel_process_sample(goertzel_state_t *g, float sample);
bool goertzel_block_ready(const goertzel_state_t *g);
/* For A*cos(w*n+p), returns A and p (radians). The signed H2
 * quadrature observation is magnitude*sin(phase). Reset between blocks. */
dsp_status_t goertzel_get_result(const goertzel_state_t *g, float *magnitude, float *phase);
void goertzel_reset(goertzel_state_t *g);

typedef struct {
    double sum;
    uint32_t count, block_size;
} dc_accum_t;

dsp_status_t dc_accum_init(dc_accum_t *d, uint32_t block_size);
dsp_status_t dc_accum_process(dc_accum_t *d, float sample);
bool dc_accum_ready(const dc_accum_t *d);
/* Returns NAN for an incomplete block. */
float dc_accum_get_mean(const dc_accum_t *d);
void dc_accum_reset(dc_accum_t *d);

#endif
