#include "ctrl_bias_engine.h"
#include "ctrl_mod_mzm.h"
#include <math.h>

dsp_status_t ctrl_bias_engine_init(ctrl_bias_engine_t *e, float rate, float tone,
                                  float amplitude, unsigned block_size,
                                  unsigned decimation, unsigned pilot_channel)
{
    if (!e || !decimation || pilot_channel >= CTRL_PILOTS || !(amplitude > 0))
        return DSP_INVALID_ARGUMENT;
    ctrl_bias_engine_t next = {.decimation = decimation, .pilot_channel = pilot_channel};
    dsp_status_t rc = pilot_gen_init(&next.pilot, tone, rate, amplitude);
    if (rc != DSP_OK) return rc;
    for (unsigned k = 0; k < 2; ++k) {
        rc = goertzel_init(&next.bins[k], (k + 1) * tone, rate, block_size);
        if (rc != DSP_OK) return rc;
    }
    rc = dc_accum_init(&next.monitor, block_size);
    if (rc == DSP_OK) *e = next;
    return rc;
}

dsp_status_t ctrl_bias_engine_start(ctrl_bias_engine_t *e, const modulator_strategy_t *strategy,
                                   void *ctx, bias_point_t point, float custom_phase,
                                   const float sweep[CTRL_DC_CHANNELS])
{
    if (!e || !e->decimation || !strategy || !strategy->start || !strategy->update ||
        !sweep || e->blocks || e->monitor.count) return DSP_INVALID_ARGUMENT;
    if (strategy == &ctrl_mzm_strategy) {
        const ctrl_mod_mzm_t *m = ctx;
        if (!m || m->config.pilot_channel != e->pilot_channel ||
            m->config.pilot_v != e->pilot.amplitude) return DSP_INVALID_ARGUMENT;
    }
    ctrl_command_t cmd;
    dsp_status_t rc = strategy->start(ctx, point, custom_phase, sweep, &cmd);
    if (rc != DSP_OK) return rc;
    if (!ctrl_routes_valid(&cmd.routes) || !cmd.routes.pilot_mask[e->pilot_channel])
        return DSP_INVALID_ARGUMENT;
    e->strategy = strategy;
    e->ctx = ctx;
    e->command = cmd;
    return DSP_OK;
}

dsp_status_t ctrl_bias_engine_set_mzm_pilot(ctrl_bias_engine_t *e, float amplitude_v)
{
    if (!e || e->strategy != &ctrl_mzm_strategy || e->blocks || e->monitor.count)
        return DSP_INVALID_ARGUMENT;
    /* Validate both new states before publishing either. */
    ctrl_mod_mzm_t *m = e->ctx;
    ctrl_mod_mzm_t next = *m;
    dsp_status_t rc = ctrl_mzm_set_pilot(&next, amplitude_v);
    if (rc != DSP_OK) return rc;
    rc = pilot_gen_set_amplitude(&e->pilot, amplitude_v);
    if (rc != DSP_OK) return rc;
    *m = next;
    e->command = m->command;
    return DSP_OK;
}

dsp_status_t ctrl_bias_engine_feed(ctrl_bias_engine_t *e, float ch0, float ch1, bool *ready)
{
    if (!e || !ready || !e->decimation || !isfinite(ch0) || !isfinite(ch1))
        return DSP_INVALID_ARGUMENT;
    *ready = false;
    for (unsigned k = 0; k < 2; ++k) {
        dsp_status_t rc = goertzel_process_sample(&e->bins[k], ch0);
        if (rc != DSP_OK) return rc;
    }
    dsp_status_t rc = dc_accum_process(&e->monitor, ch1);
    if (rc != DSP_OK) return rc;
    pilot_gen_next(&e->pilot);
    if (!dc_accum_ready(&e->monitor)) return DSP_OK;
    for (unsigned k = 0; k < 2; ++k) {
        float mag, phase;
        rc = goertzel_get_result(&e->bins[k], &mag, &phase);
        if (rc != DSP_OK) return rc;
        e->i[k] += mag * cos(phase);
        e->q[k] += mag * sin(phase);
        goertzel_reset(&e->bins[k]);
    }
    e->dc += dc_accum_get_mean(&e->monitor);
    dc_accum_reset(&e->monitor);
    if (++e->blocks < e->decimation) return DSP_OK;
    e->last = (harmonic_data_t){
        .h1_magnitude = hypot(e->i[0], e->q[0]) / e->blocks,
        .h2_magnitude = hypot(e->i[1], e->q[1]) / e->blocks,
        .h1_phase = atan2(e->q[0], e->i[0]),
        .h2_phase = atan2(e->q[1], e->i[1]),
        .dc_power = e->dc / e->blocks
    };
    for (unsigned k = 0; k < 2; ++k) e->i[k] = e->q[k] = 0;
    e->dc = 0;
    e->blocks = 0;
    *ready = true;
    if (e->strategy) {
        harmonic_data_t pilots[CTRL_PILOTS] = {0};
        pilots[e->pilot_channel] = e->last;
        pilots[e->pilot_channel].dc_power = 0; /* CH1 is exposed only through last. */
        return e->strategy->update(e->ctx, pilots, &e->command);
    }
    return DSP_OK;
}

dsp_status_t ctrl_bias_scan_add(ctrl_bias_scan_t *s, float voltage, const harmonic_data_t *h)
{
    if (!s || !h || !isfinite(h->h2_phase) || !isfinite(h->h2_magnitude) ||
        h->h2_magnitude < 0 || !isfinite(h->h1_magnitude) || h->h1_magnitude < 0)
        return DSP_INVALID_ARGUMENT;
    float q = h->h2_magnitude * sinf(h->h2_phase);
    if (s->fitting) {
        double phase = (DSP_TAU / 2) * (voltage - (double)s->null_v) / s->vpi;
        return dsp_online_lsq_add(&s->lsq, sin(phase), cos(phase),
                                  h->h1_magnitude * cosf(h->h1_phase), q);
    }
    unsigned before = s->tracker.minima_count;
    dsp_status_t rc = dsp_vpi_add(&s->tracker, voltage, h->h1_magnitude);
    if (rc != DSP_OK) return rc;
    if (!before && s->tracker.minima_count) s->first_valley_q = s->previous_q;
    s->previous_q = q;
    return DSP_OK;
}

dsp_status_t ctrl_bias_scan_begin_fit(ctrl_bias_scan_t *s, int null_q_sign)
{
    if (!s || s->fitting || (null_q_sign != 1 && null_q_sign != -1))
        return DSP_INVALID_ARGUMENT;
    dsp_status_t rc = dsp_vpi_result(&s->tracker, &s->vpi);
    if (rc != DSP_OK) return rc;
    if (s->first_valley_q == 0) return DSP_SINGULAR;
    s->null_v = s->tracker.first_min_v;
    if (s->first_valley_q * null_q_sign < 0) s->null_v += s->vpi;
    s->fitting = true;
    return DSP_OK;
}

dsp_status_t ctrl_bias_scan_finish(const ctrl_bias_scan_t *s, float pilot_v, ctrl_calibration_t *cal)
{
    if (!s || !cal || !s->fitting || !(pilot_v > 0) || !isfinite(pilot_v))
        return DSP_INVALID_ARGUMENT;
    ctrl_calibration_t result = {.vpi_v = s->vpi, .null_v = s->null_v, .cal_pilot_v = pilot_v};
    dsp_status_t rc = dsp_online_lsq_solve(&s->lsq, &result.fit);
    if (rc == DSP_OK) *cal = result;
    return rc;
}
