#ifndef DSP_ONLINE_LSQ_H
#define DSP_ONLINE_LSQ_H

#include "dsp_types.h"

/* Zero-initialize before each scan. The design row is [1, sin(phi), cos(phi)].
 * gram stores the lower triangle by row, rhs columns are signed H1 and H2. */
typedef struct {
    double gram[6], rhs[3][2];
} dsp_online_lsq_t;

typedef struct {
    float o[2], m[2][2]; /* h = o + M * [sin(phi), cos(phi)] */
} dsp_affine_fit_t;

dsp_status_t dsp_online_lsq_add(dsp_online_lsq_t *s, float sin_phi, float cos_phi,
                                float h1s, float h2s);
dsp_status_t dsp_online_lsq_solve(const dsp_online_lsq_t *s, dsp_affine_fit_t *fit);

/* Zero-initialize and stream increasing bias voltages with H1 magnitudes.
 * A valley must fall below 10% of the peak magnitude seen so far. */
typedef struct {
    double left_v, left_h, prev_v, prev_h, peak_h, valley_start;
    double first_min_v, last_min_v;
    uint32_t window_count, minima_count;
    _Bool descending;
} dsp_vpi_tracker_t;

dsp_status_t dsp_vpi_add(dsp_vpi_tracker_t *s, float bias_v, float h1_mag);
/* Mean spacing of successive minima. Requires at least two interior minima. */
dsp_status_t dsp_vpi_result(const dsp_vpi_tracker_t *s, float *vpi);

_Static_assert(sizeof(dsp_online_lsq_t) + sizeof(dsp_vpi_tracker_t) < 256,
               "Pass-1 and pass-2 scan state must fit the calibration budget");

#endif
