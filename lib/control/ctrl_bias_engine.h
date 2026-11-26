#ifndef CTRL_BIAS_ENGINE_H
#define CTRL_BIAS_ENGINE_H

#include "ctrl_modulator.h"
#include "ctrl_observer.h"
#include "dsp_goertzel.h"
#include "dsp_pilot_gen.h"

typedef struct {
    pilot_gen_t pilot;
    goertzel_state_t bins[2];
    dc_accum_t monitor;
    double i[2], q[2], dc;
    unsigned blocks, decimation, pilot_channel;
    harmonic_data_t last;
    const modulator_strategy_t *strategy;
    void *ctx;
    ctrl_command_t command;
} ctrl_bias_engine_t;

dsp_status_t ctrl_bias_engine_init(ctrl_bias_engine_t *e, float rate, float tone,
                                  float amplitude, unsigned block_size,
                                  unsigned decimation, unsigned pilot_channel);
/* Optional. Without a strategy, feed produces demodulated scan windows. */
dsp_status_t ctrl_bias_engine_start(ctrl_bias_engine_t *e, const modulator_strategy_t *strategy,
                                   void *ctx, bias_point_t point, float custom_phase,
                                   const float sweep[CTRL_DC_CHANNELS]);
/* Change amplitude and observer scaling together between complete windows. */
dsp_status_t ctrl_bias_engine_set_mzm_pilot(ctrl_bias_engine_t *e, float amplitude_v);
/* Output pilot_gen_current(&e->pilot) before sampling. A successful feed
 * advances the pilot once. ready marks a complete decimated window, even
 * when strategy update rejects its observation. Inspect status first. */
dsp_status_t ctrl_bias_engine_feed(ctrl_bias_engine_t *e, float ch0, float ch1, bool *ready);

/* Zero-initialize for pass 1. Supply increasing voltages and one settled
 * demodulated window per voltage. The receiver polarity identifies which
 * alternating H1 valley is dark (signed H2 Q is positive or negative there). */
typedef struct {
    dsp_vpi_tracker_t tracker;
    dsp_online_lsq_t lsq;
    float previous_q, first_valley_q, null_v, vpi;
    bool fitting;
} ctrl_bias_scan_t;

_Static_assert(sizeof(ctrl_bias_scan_t) < 1024, "Calibration scan exceeds RAM budget");
dsp_status_t ctrl_bias_scan_add(ctrl_bias_scan_t *s, float voltage, const harmonic_data_t *h);
dsp_status_t ctrl_bias_scan_begin_fit(ctrl_bias_scan_t *s, int null_q_sign);
dsp_status_t ctrl_bias_scan_finish(const ctrl_bias_scan_t *s, float pilot_v, ctrl_calibration_t *cal);

#endif
