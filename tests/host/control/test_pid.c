#include "check.h"
#include "ctrl_pid.h"
#include <float.h>
#include <math.h>

int main(void)
{
    ctrl_pid_t p;
    float v;
    CHECK(ctrl_pid_init(&p, 0.5f, 2, 0.25f, -4, 3, -1.5f) == DSP_OK);
    CHECK(ctrl_pid_update(&p, 0, &v) == DSP_OK && v == -1.5f);
    CHECK(ctrl_pid_update(&p, 1, &v) == DSP_OK && v == -0.5f);
    CHECK(ctrl_pid_update(&p, FLT_MAX, &v) == DSP_OK && v == 3);
    CHECK(ctrl_pid_update(&p, -1, &v) == DSP_OK && v == 2);
    CHECK(ctrl_pid_update(&p, -FLT_MAX, &v) == DSP_OK && v == -4);
    CHECK(ctrl_pid_update(&p, 1, &v) == DSP_OK && v == -3);
    CHECK(ctrl_pid_update(&p, NAN, &v) == DSP_INVALID_ARGUMENT && v == -3);
    CHECK(ctrl_pid_update(&p, 0, &v) == DSP_OK && v == -3.5f);
    CHECK(ctrl_pid_init(&p, NAN, 1, 1, -1, 1, 0) == DSP_INVALID_ARGUMENT);
    CHECK(ctrl_pid_init(&p, 1, INFINITY, 1, -1, 1, 0) == DSP_INVALID_ARGUMENT);
    CHECK(ctrl_pid_init(&p, 1, 1, 0, -1, 1, 0) == DSP_INVALID_ARGUMENT);
    CHECK(ctrl_pid_init(&p, 1, 1, 1, 1, -1, 0) == DSP_INVALID_ARGUMENT);
    CHECK(ctrl_pid_init(&p, 1, 1, 1, -1, 1, NAN) == DSP_INVALID_ARGUMENT);
    CHECK(ctrl_pid_init(&p, 1, 1, 1, -1, 1, 8) == DSP_OK);
    CHECK(ctrl_pid_update(&p, 0, &v) == DSP_OK && v == 1);
    CHECK(ctrl_pid_init(&p, 0.5f, 0, 1, -1, 1, 1) == DSP_OK);
    CHECK(ctrl_pid_update(&p, 1, &v) == DSP_OK && v == 0.5f);
    return 0;
}
