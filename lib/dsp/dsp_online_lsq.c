#include "dsp_online_lsq.h"
#include <float.h>
#include <math.h>

dsp_status_t dsp_online_lsq_add(dsp_online_lsq_t *s, float sin_phi, float cos_phi,
                                float h1s, float h2s)
{
    if (!s || !isfinite(sin_phi) || !isfinite(cos_phi) || !isfinite(h1s) || !isfinite(h2s))
        return DSP_INVALID_ARGUMENT;
    double row[] = {1, sin_phi, cos_phi};
    unsigned k = 0;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j)
            s->gram[k++] += row[i] * row[j];
        s->rhs[i][0] += row[i] * h1s;
        s->rhs[i][1] += row[i] * h2s;
    }
    return DSP_OK;
}

dsp_status_t dsp_online_lsq_solve(const dsp_online_lsq_t *s, dsp_affine_fit_t *fit)
{
    if (!s || !fit) return DSP_INVALID_ARGUMENT;
    if (s->gram[0] < 3) return DSP_INCOMPLETE;

    /* Cholesky factorization of the symmetric Gram matrix. A small or
     * negative pivot means these phases do not identify the affine model. */
    double lower[3][3] = {{0}};
    double tolerance = 64 * DBL_EPSILON * fmax(s->gram[0], fmax(s->gram[2], s->gram[5]));
    unsigned k = 0;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j) {
            double value = s->gram[k++];
            for (unsigned p = 0; p < j; ++p)
                value -= lower[i][p] * lower[j][p];
            if (i == j) {
                if (value <= tolerance) return DSP_SINGULAR;
                lower[i][i] = sqrt(value);
            } else {
                lower[i][j] = value / lower[j][j];
            }
        }
    }

    dsp_affine_fit_t result;
    for (unsigned h = 0; h < 2; ++h) {
        double x[3];
        for (int i = 0; i < 3; ++i) {
            double value = s->rhs[i][h];
            for (int j = 0; j < i; ++j) value -= lower[i][j] * x[j];
            x[i] = value / lower[i][i];
        }
        for (int i = 2; i >= 0; --i) {
            for (int j = i + 1; j < 3; ++j) x[i] -= lower[j][i] * x[j];
            x[i] /= lower[i][i];
            if (fabs(x[i]) > FLT_MAX) return DSP_INVALID_ARGUMENT;
        }
        result.o[h] = (float)x[0];
        result.m[h][0] = (float)x[1];
        result.m[h][1] = (float)x[2];
    }
    *fit = result;
    return DSP_OK;
}

dsp_status_t dsp_vpi_add(dsp_vpi_tracker_t *s, float bias_v, float h1_mag)
{
    if (!s || !isfinite(bias_v) || !isfinite(h1_mag) || h1_mag < 0 ||
        (s->window_count && bias_v <= s->prev_v))
        return DSP_INVALID_ARGUMENT;
    s->peak_h = fmax(s->peak_h, h1_mag);
    if (s->window_count && h1_mag < s->prev_h) {
        s->descending = 1;
        s->valley_start = bias_v;
    }
    if (s->window_count == 2 && h1_mag > s->prev_h && s->descending) {
        if (s->prev_h < 0.1 * s->peak_h) {
            double v = (s->valley_start + s->prev_v) / 2;
            if (s->valley_start == s->prev_v) {
                /* Vertex of the quadratic through three (possibly uneven) samples. */
                double dl = s->prev_v - s->left_v, dr = bias_v - s->prev_v;
                double sl = (s->prev_h - s->left_h) / dl, sr = (h1_mag - s->prev_h) / dr;
                v -= 0.5 * (sl * dr + sr * dl) / (sr - sl);
            }
            if (!s->minima_count) s->first_min_v = v;
            s->last_min_v = v;
            ++s->minima_count;
        }
        s->descending = 0;
    }
    s->left_v = s->prev_v;
    s->left_h = s->prev_h;
    s->prev_v = bias_v;
    s->prev_h = h1_mag;
    if (s->window_count < 2) ++s->window_count;
    return DSP_OK;
}

dsp_status_t dsp_vpi_result(const dsp_vpi_tracker_t *s, float *vpi)
{
    if (!s || !vpi) return DSP_INVALID_ARGUMENT;
    if (s->minima_count < 2) return DSP_INCOMPLETE;
    *vpi = (float)((s->last_min_v - s->first_min_v) / (s->minima_count - 1));
    return DSP_OK;
}
