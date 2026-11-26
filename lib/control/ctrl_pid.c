#include "ctrl_pid.h"
#include <math.h>

static double clamp(double value, double lo, double hi)
{
    return fmin(hi, fmax(lo, value));
}

dsp_status_t ctrl_pid_init(ctrl_pid_t *p, float kp, float ki, float dt,
                           float out_min, float out_max, float sweep_bias_v)
{
    if (!p || !isfinite(kp) || !isfinite(ki) || !isfinite(dt) || dt <= 0 ||
        !isfinite(out_min) || !isfinite(out_max) || out_min >= out_max ||
        !isfinite(sweep_bias_v))
        return DSP_INVALID_ARGUMENT;

    *p = (ctrl_pid_t){kp, ki, dt, out_min, out_max,
                      ki == 0 ? 0 : clamp(sweep_bias_v, out_min, out_max)};
    return DSP_OK;
}

dsp_status_t ctrl_pid_update(ctrl_pid_t *p, float error, float *output_v)
{
    if (!p || !output_v || !isfinite(error) || !(p->dt > 0))
        return DSP_INVALID_ARGUMENT;

    if (p->ki != 0)
        p->integral_v = clamp(p->integral_v + (double)p->ki * p->dt * error,
                              p->out_min, p->out_max);
    *output_v = (float)clamp((double)p->kp * error + p->integral_v,
                             p->out_min, p->out_max);
    return DSP_OK;
}
