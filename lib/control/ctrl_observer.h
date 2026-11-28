#ifndef CTRL_OBSERVER_H
#define CTRL_OBSERVER_H

#include "dsp_online_lsq.h"
#include <stdbool.h>

typedef struct {
    dsp_affine_fit_t fit;
    float null_v, vpi_v, cal_pilot_v;
    float h1_ref_phase, h2_ref_phase;
    bool has_ref_phases;
} ctrl_calibration_t;

/* Caller-owned state. Read diagnostics after update; change calibration
 * or target by initializing again. phase_rad is continuous across +/-pi. */
typedef struct {
    ctrl_calibration_t cal;
    double inverse[2][2];
    float target_v, target_rad, target_sin, target_cos;
    float obs_x, obs_y, phase_rad, radius;
    float error, rho_ema;
    unsigned lock_streak;
    bool scaled, has_sample, observer_ok, radius_ok, error_ok, phase_ok;
    bool locked, hold_assist_active, frozen;
} ctrl_observer_t;

/* target_v is the calibrated anchor on the chosen physical branch.
 * The coarse sweep must place the bias near that target before takeover. */
dsp_status_t ctrl_observer_init(ctrl_observer_t *s, const ctrl_calibration_t *cal,
                                float target_v, float sweep_v, float pilot_v);
/* Recompute the effective inverse when pilot amplitude changes. On failure,
 * subsequent updates are disabled until valid scaling has been restored. */
dsp_status_t ctrl_observer_set_pilot(ctrl_observer_t *s, float pilot_v);
/* DSP_INCOMPLETE with frozen=true means radius < 0.15f (hold bias safely).
 * Any failed update breaks lock. Only consume error after DSP_OK. */
dsp_status_t ctrl_observer_update(ctrl_observer_t *s, const harmonic_data_t *h,
                                  float bias_v);

#endif
