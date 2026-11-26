#ifndef CTRL_PID_H
#define CTRL_PID_H

#include "dsp_types.h"

/* Integral contribution is stored in output volts, including ki. */
typedef struct {
    float kp, ki, dt, out_min, out_max;
    double integral_v;
} ctrl_pid_t;

/* Seeds the absolute bias command from the sweep, clamped to output limits.
 * ki == 0 selects proportional-only operation and drops the integral seed. */
dsp_status_t ctrl_pid_init(ctrl_pid_t *p, float kp, float ki, float dt,
                           float out_min, float out_max, float sweep_bias_v);
dsp_status_t ctrl_pid_update(ctrl_pid_t *p, float error, float *output_v);

#endif
