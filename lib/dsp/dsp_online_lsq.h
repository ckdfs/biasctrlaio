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

/* O(1)-RAM 2x2 centered complex covariance accumulator (6 doubles per harmonic). */
typedef struct {
    double mean_re, mean_im, c_xx, c_xy, c_yy, count;
} dsp_complex_cov_t;

dsp_status_t dsp_complex_cov_add(dsp_complex_cov_t *c, float mag, float phase);
dsp_status_t dsp_complex_cov_principal(const dsp_complex_cov_t *c, float *ref_phase,
                                       double *mean_proj, double *rms_scale);

/* O(1)-RAM Fitzgibbon 6x6 conic scatter + shoelace winding + CH1 DC 3x3 LSQ.
 * Inputs (x, y) correspond to signed projected harmonics (s2, s1). */
typedef struct {
    double scatter[21];
    double dc_rhs[3];
    double mu_x, mu_y;
    double scale_x, scale_y;
    double winding;
    double first_x, first_y;
    double prev_x, prev_y;
    uint32_t count;
} dsp_fitzgibbon_t;

void dsp_fitzgibbon_init(dsp_fitzgibbon_t *f, double mu_x, double mu_y,
                         double scale_x, double scale_y);
dsp_status_t dsp_fitzgibbon_add(dsp_fitzgibbon_t *f, double x, double y, double dc);
dsp_status_t dsp_fitzgibbon_solve(const dsp_fitzgibbon_t *f, dsp_affine_fit_t *fit);

_Static_assert(sizeof(dsp_online_lsq_t) + sizeof(dsp_vpi_tracker_t) < 256,
               "Pass-1 and pass-2 scan state must fit the calibration budget");

#endif
