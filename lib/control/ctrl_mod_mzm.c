#include "ctrl_mod_mzm.h"
#include <math.h>

static dsp_status_t start(void *ctx, bias_point_t point, float custom,
                          const float sweep[CTRL_DC_CHANNELS], ctrl_command_t *cmd)
{
    ctrl_mod_mzm_t *m = ctx;
    if (!m || !sweep || !cmd) return DSP_INVALID_ARGUMENT;
    const ctrl_mzm_config_t *c = &m->config;
    if (c->dc_channel >= CTRL_DC_CHANNELS || c->pilot_channel >= CTRL_PILOTS ||
        !isfinite(c->min_v) || !isfinite(c->max_v) || !(c->ki > 0))
        return DSP_INVALID_ARGUMENT;
    double phase;
    switch (point) {
    case BIAS_POINT_MIN: phase = 0; break;
    case BIAS_POINT_MAX: phase = DSP_TAU / 2; break;
    case BIAS_POINT_QUAD: phase = DSP_TAU / 4; break;
    case BIAS_POINT_CUSTOM: phase = custom; break;
    default: return DSP_INVALID_ARGUMENT;
    }

    /* Select the copy of the target nearest the coarse sweep and confine
     * the integrator to the surrounding half fringe on either side. */
    float seed = sweep[c->dc_channel];
    double vpi = c->calibration.vpi_v;
    if (!isfinite(seed) || !isfinite(phase) || !(vpi > 0) || !isfinite(vpi))
        return DSP_INVALID_ARGUMENT;
    double seed_phase = (seed - (double)c->calibration.null_v) / vpi * (DSP_TAU / 2);
    float anchor = seed + remainder(phase - seed_phase, DSP_TAU) * vpi / (DSP_TAU / 2);
    float lo = fmax(c->min_v, anchor - vpi / 2);
    float hi = fmin(c->max_v, anchor + vpi / 2);
    if (anchor < lo || anchor > hi || seed < lo || seed > hi) return DSP_INVALID_ARGUMENT;

    ctrl_mod_mzm_t next = {.config = *c};
    dsp_status_t rc = ctrl_observer_init(&next.observer, &c->calibration, anchor, seed, c->pilot_v);
    if (rc != DSP_OK) return rc;
    rc = ctrl_pid_init(&next.pid, c->kp, c->ki, c->dt, lo, hi, seed);
    if (rc != DSP_OK) return rc;
    uint8_t mask = (uint8_t)(1u << c->dc_channel);
    next.command.routes.dc_mask = mask;
    next.command.routes.pilot_mask[c->pilot_channel] = mask;
    next.command.bias_v[c->dc_channel] = seed;
    next.running = true;
    *m = next;
    *cmd = next.command;
    return DSP_OK;
}

dsp_status_t ctrl_mzm_set_pilot(ctrl_mod_mzm_t *m, float amplitude_v)
{
    if (!m || !m->running) return DSP_INVALID_ARGUMENT;
    dsp_status_t rc = ctrl_observer_set_pilot(&m->observer, amplitude_v);
    m->command.locked = m->command.hold_assist_active = false;
    if (rc == DSP_OK) m->config.pilot_v = amplitude_v;
    return rc;
}

static dsp_status_t update(void *ctx, const harmonic_data_t pilot[CTRL_PILOTS], ctrl_command_t *cmd)
{
    ctrl_mod_mzm_t *m = ctx;
    if (!m || !m->running || !pilot || !cmd) return DSP_INVALID_ARGUMENT;
    float *bias = &m->command.bias_v[m->config.dc_channel];
    dsp_status_t rc = ctrl_observer_update(&m->observer, &pilot[m->config.pilot_channel], *bias);
    if (rc == DSP_OK) rc = ctrl_pid_update(&m->pid, m->observer.error, bias);
    m->command.locked = rc == DSP_OK && m->observer.locked;
    m->command.hold_assist_active = rc == DSP_OK && m->observer.hold_assist_active;
    *cmd = m->command;
    return rc;
}

const modulator_strategy_t ctrl_mzm_strategy = {MOD_TYPE_MZM, start, update};
