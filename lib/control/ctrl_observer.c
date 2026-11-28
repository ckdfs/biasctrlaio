#include "ctrl_observer.h"
#include <float.h>
#include <math.h>

#define PI (DSP_TAU / 2.0)
enum { HOLD_UPDATES = 25 };

static void clear_lock(ctrl_observer_t *s)
{
    s->locked = false;
    s->hold_assist_active = false;
    s->lock_streak = 0;
}

/* Ascending Bessel series for the calibrated pilot range. Reject series
 * that overflow or lose float precision to cancellation during summation. */
static double bessel(unsigned order, double m)
{
    double q = m * m / 4;
    double term = order == 1 ? m / 2 : q / 2;
    double sum = term, peak = term;
    for (unsigned k = 1; k <= 128; ++k) {
        term *= -q / (k * (k + order));
        sum += term;
        peak = fmax(peak, fabs(term));
        if (!isfinite(sum))
            break;
        if (fabs(term) <= DBL_EPSILON * fabs(sum))
            return peak * DBL_EPSILON <= FLT_EPSILON * fabs(sum) ? sum : NAN;
    }
    return NAN;
}

dsp_status_t ctrl_observer_set_pilot(ctrl_observer_t *s, float pilot_v)
{
    if (!s)
        return DSP_INVALID_ARGUMENT;
    s->scaled = false;
    clear_lock(s);
    s->frozen = false;
    s->rho_ema = 0.0f;
    if (!isfinite(pilot_v) || pilot_v < 0 || !(s->cal.vpi_v > 0))
        return DSP_INVALID_ARGUMENT;

    double m[2][2];
    for (unsigned row = 0; row < 2; ++row) {
        double cal = bessel(row + 1, PI * s->cal.cal_pilot_v / s->cal.vpi_v);
        double now = bessel(row + 1, PI * pilot_v / s->cal.vpi_v);
        if (!isfinite(cal) || !isfinite(now))
            return DSP_INVALID_ARGUMENT;
        if (fabs(cal) < 1e-8 || fabs(now) < 1e-8)
            return DSP_SINGULAR;
        for (unsigned col = 0; col < 2; ++col)
            m[row][col] = s->cal.fit.m[row][col] * (now / cal);
    }
    double a = m[0][0] * m[1][1], b = m[0][1] * m[1][0];
    double det = a - b;
    if (fabs(det) <= FLT_EPSILON * (fabs(a) + fabs(b)))
        return DSP_SINGULAR;
    s->inverse[0][0] = m[1][1] / det;
    s->inverse[0][1] = -m[0][1] / det;
    s->inverse[1][0] = -m[1][0] / det;
    s->inverse[1][1] = m[0][0] / det;
    s->scaled = true;
    return DSP_OK;
}

dsp_status_t ctrl_observer_init(ctrl_observer_t *s, const ctrl_calibration_t *cal,
                                float target_v, float sweep_v, float pilot_v)
{
    if (!s || !cal || !isfinite(cal->null_v) || !isfinite(cal->vpi_v) ||
        cal->vpi_v <= 0 || !isfinite(cal->cal_pilot_v) || cal->cal_pilot_v < 0 ||
        !isfinite(target_v) || !isfinite(sweep_v) ||
        (cal->has_ref_phases && (!isfinite(cal->h1_ref_phase) || !isfinite(cal->h2_ref_phase))))
        return DSP_INVALID_ARGUMENT;
    for (unsigned i = 0; i < 2; ++i)
        if (!isfinite(cal->fit.o[i]) || !isfinite(cal->fit.m[i][0]) ||
            !isfinite(cal->fit.m[i][1]))
            return DSP_INVALID_ARGUMENT;

    double target = PI * ((double)target_v - cal->null_v) / cal->vpi_v;
    double seed = PI * ((double)sweep_v - cal->null_v) / cal->vpi_v;
    if (fabs(seed) > FLT_MAX || fabs(target) > FLT_MAX)
        return DSP_INVALID_ARGUMENT;
    *s = (ctrl_observer_t){.cal = *cal, .target_v = target_v,
                           .target_rad = (float)remainder(target, DSP_TAU),
                           .target_sin = sin(target), .target_cos = cos(target),
                           .obs_x = sin(seed), .obs_y = cos(seed), .phase_rad = seed};
    return ctrl_observer_set_pilot(s, pilot_v);
}

dsp_status_t ctrl_observer_update(ctrl_observer_t *s, const harmonic_data_t *h,
                                  float bias_v)
{
    if (!s)
        return DSP_INVALID_ARGUMENT;
    s->observer_ok = s->radius_ok = s->error_ok = s->phase_ok = false;
    if (!s->scaled || !h || !isfinite(bias_v) || !isfinite(h->h1_phase) ||
        !isfinite(h->h2_phase) || !isfinite(h->h1_magnitude) || h->h1_magnitude < 0 ||
        !isfinite(h->h2_magnitude) || h->h2_magnitude < 0) {
        clear_lock(s);
        s->frozen = false;
        return DSP_INVALID_ARGUMENT;
    }

    double theta1 = s->cal.has_ref_phases ? (double)s->cal.h1_ref_phase : 0.0;
    double theta2 = s->cal.has_ref_phases ? (double)s->cal.h2_ref_phase : (DSP_TAU / 4.0);
    double h1 = (double)h->h1_magnitude * cos((double)h->h1_phase - theta1) - s->cal.fit.o[0];
    double h2 = (double)h->h2_magnitude * cos((double)h->h2_phase - theta2) - s->cal.fit.o[1];
    double x = s->inverse[0][0] * h1 + s->inverse[0][1] * h2;
    double y = s->inverse[1][0] * h1 + s->inverse[1][1] * h2;
    double radius = hypot(x, y);
    if (!isfinite(radius) || radius > FLT_MAX) {
        clear_lock(s);
        s->frozen = false;
        return DSP_INVALID_ARGUMENT;
    }
    s->radius = (float)radius;
    s->radius_ok = s->radius >= 0.15f;
    if (!s->radius_ok) {
        clear_lock(s);
        s->frozen = true;
        return DSP_INCOMPLETE;
    }

    s->frozen = false;
    double ux = x / radius;
    double uy = y / radius;
    double inst_rad = atan2(ux, uy);
    double step = remainder(inst_rad - (double)s->phase_rad, DSP_TAU);

    s->obs_x = (float)ux;
    s->obs_y = (float)uy;
    s->phase_rad += (float)step;
    s->error = (float)remainder((double)s->target_rad - inst_rad, DSP_TAU);
    s->rho_ema = 0.9f * s->rho_ema + 0.1f * fabsf(s->radius - 1.0f);
    s->has_sample = true;
    s->observer_ok = true;
    s->error_ok = fabsf(s->error) < (float)(0.20 / PI);
    /* Target hemisphere rejects the opposite fringe even at zero sine error. */
    s->phase_ok = s->obs_x * s->target_sin + s->obs_y * s->target_cos > 0.0f;
    s->locked = s->observer_ok && s->radius_ok && s->error_ok && s->phase_ok;
    if (!s->locked)
        clear_lock(s);
    else if (s->lock_streak < HOLD_UPDATES)
        ++s->lock_streak;
    s->hold_assist_active = s->lock_streak == HOLD_UPDATES;
    return DSP_OK;
}
