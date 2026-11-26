#ifndef CTRL_MOD_MZM_H
#define CTRL_MOD_MZM_H

#include "ctrl_modulator.h"
#include "ctrl_observer.h"
#include "ctrl_pid.h"

/* Configure before start. dt is one decimated control window in seconds.
 * Use positive ki to seed the absolute-bias integrator from the sweep. */
typedef struct {
    ctrl_calibration_t calibration;
    unsigned dc_channel, pilot_channel;
    float pilot_v, kp, ki, dt, min_v, max_v;
} ctrl_mzm_config_t;

typedef struct {
    ctrl_mzm_config_t config;
    ctrl_pid_t pid;
    ctrl_observer_t observer;
    ctrl_command_t command;
    bool running;
} ctrl_mod_mzm_t;

extern const modulator_strategy_t ctrl_mzm_strategy;
/* Call at a window boundary together with pilot_gen_set_amplitude. */
dsp_status_t ctrl_mzm_set_pilot(ctrl_mod_mzm_t *m, float amplitude_v);

#endif
