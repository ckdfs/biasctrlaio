#include "ctrl_observer.h"
#include <float.h>
#include <math.h>

#define PI (DSP_TAU / 2.0)
enum { DC_WARMUP = 5, HOLD_UPDATES = 25 };

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
        (cal->has_obs_dc_seed && !isfinite(cal->obs_dc_seed)))
        return DSP_INVALID_ARGUMENT;
    for (unsigned i = 0; i < 2; ++i)
        if (!isfinite(cal->fit.o[i]) || !isfinite(cal->fit.m[i][0]) ||
            !isfinite(cal->fit.m[i][1]))
            return DSP_INVALID_ARGUMENT;

    double target = PI * ((double)target_v - cal->null_v) / cal->vpi_v;
    double seed = PI * ((double)sweep_v - cal->null_v) / cal->vpi_v;
    if (fabs(seed) > FLT_MAX)
        return DSP_INVALID_ARGUMENT;
    *s = (ctrl_observer_t){.cal = *cal, .target_v = target_v,
                           .target_sin = sin(target), .target_cos = cos(target),
                           .obs_x = sin(seed), .obs_y = cos(seed), .phase_rad = seed};
    return ctrl_observer_set_pilot(s, pilot_v);
}

static void update_error(ctrl_observer_t *s, float weight)
{
    s->obs_raw = s->target_sin * s->obs_y - s->target_cos * s->obs_x;
    float corrected = s->obs_raw;
    if (weight > 0.01f) {
        if (!s->has_sample) {
            s->obs_dc = s->cal.has_obs_dc_seed ? s->cal.obs_dc_seed : s->obs_raw;
            s->dc_count = s->cal.has_obs_dc_seed ? DC_WARMUP : 0;
        } else if (s->dc_count < DC_WARMUP || s->locked) {
            s->obs_dc += 0.50f * (s->obs_raw - s->obs_dc);
            if (s->dc_count < DC_WARMUP)
                ++s->dc_count;
        }
        if (s->dc_count == DC_WARMUP)
            corrected -= s->obs_dc;
    }
    float alpha = 1 - 0.90f * weight;
    s->obs_filtered = s->has_sample ? s->obs_filtered + alpha * (corrected - s->obs_filtered)
                                    : corrected;
    s->error = s->obs_filtered + s->spring;
}

dsp_status_t ctrl_observer_update(ctrl_observer_t *s, const harmonic_data_t *h,
                                  float bias_v)
{
    if (!s)
        return DSP_INVALID_ARGUMENT;
    s->observer_ok = s->radius_ok = s->error_ok = s->phase_ok = s->bias_ok = false;
    if (!s->scaled || !h || !isfinite(bias_v) || !isfinite(h->h1_phase) ||
        !isfinite(h->h2_phase) || !isfinite(h->h1_magnitude) || h->h1_magnitude < 0 ||
        !isfinite(h->h2_magnitude) || h->h2_magnitude < 0) {
        clear_lock(s);
        return DSP_INVALID_ARGUMENT;
    }

    double bias_delta = ((double)bias_v - s->target_v) / s->cal.vpi_v;
    s->bias_ok = fabs(bias_delta) <= 0.30;
    double h1 = (double)h->h1_magnitude * cosf(h->h1_phase) - s->cal.fit.o[0];
    double h2 = (double)h->h2_magnitude * sinf(h->h2_phase) - s->cal.fit.o[1];
    double x = s->inverse[0][0] * h1 + s->inverse[0][1] * h2;
    double y = s->inverse[1][0] * h1 + s->inverse[1][1] * h2;
    double radius = hypot(x, y);
    if (radius > FLT_MAX) {
        clear_lock(s);
        return DSP_INVALID_ARGUMENT;
    }
    s->radius = (float)radius;
    s->radius_ok = radius >= 0.10f;
    if (!s->radius_ok) {
        clear_lock(s);
        return DSP_INCOMPLETE;
    }
    float weight = s->target_sin * s->target_sin;
    double spring = weight > 0.01f ? -0.60 * weight * bias_delta : 0;
    if (fabs(spring) > FLT_MAX) {
        clear_lock(s);
        return DSP_INVALID_ARGUMENT;
    }

    if (s->has_sample) {
        float ax = 0.08f + 0.22f * fabsf(s->target_cos);
        float ay = 0.02f - 0.015f * fabsf(s->target_sin);
        x = s->obs_x + ax * (x / radius - s->obs_x);
        y = s->obs_y + ay * (y / radius - s->obs_y);
        double norm = hypot(x, y);
        double step = remainder(atan2(x, y) - s->phase_rad, DSP_TAU);
        if (norm < 0.10 || fabs(step) > PI / 2) {
            clear_lock(s);
            return DSP_INCOMPLETE;
        }
        s->obs_x = (float)(x / norm);
        s->obs_y = (float)(y / norm);
        s->phase_rad += (float)step;
    }

    s->observer_ok = true;
    s->spring = (float)spring;
    update_error(s, weight);
    s->has_sample = true;
    s->error_ok = fabsf(s->error) < 0.20 / PI;
    /* Target hemisphere rejects the opposite fringe even at zero sine error. */
    s->phase_ok = s->obs_x * s->target_sin + s->obs_y * s->target_cos > 0;
    s->locked = s->bias_ok && s->observer_ok && s->radius_ok && s->error_ok && s->phase_ok;
    if (!s->locked)
        clear_lock(s);
    else if (s->lock_streak < HOLD_UPDATES)
        ++s->lock_streak;
    s->hold_assist_active = s->lock_streak == HOLD_UPDATES;
    return DSP_OK;
}
