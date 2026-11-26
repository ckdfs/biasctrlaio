#include "ctrl_observer.h"
#include "check.h"
#include <math.h>

#define NEAR(a,b) CHECK(fabs((a) - (b)) < 2e-6)
#define PI (DSP_TAU / 2)

static ctrl_calibration_t calibration(void)
{
    return (ctrl_calibration_t){.fit = {.o = {0.25f, -0.3f}, .m = {{1, 0.4f}, {-0.2f, 1}}},
                                .vpi_v = PI, .cal_pilot_v = 0.5f};
}

/* Independent tabulated Bessel values at m=0.5 (cal) and m=4 (runtime).
 * J1 changes sign and J2 does not. Cross terms expose column scaling. */
static harmonic_data_t sample(double phase, double power)
{
    double row1 = -0.0660433280235491 / 0.242268457674874;
    double row2 = 0.364128145852073 / 0.0306040234586826;
    double h1 = 0.25 + power * row1 * (sin(phase) + 0.4*cos(phase));
    double h2 = -0.3 + power * row2 * (-0.2*sin(phase) + cos(phase));
    double lag = copysign(82*PI/180, h2);
    return (harmonic_data_t){.h1_magnitude = fabs(h1), .h1_phase = h1 < 0 ? PI : 0,
                             .h2_magnitude = h2 / sin(lag), .h2_phase = lag};
}

int main(void)
{
    ctrl_calibration_t cal = calibration();
    ctrl_observer_t a, b;
    CHECK(ctrl_observer_init(&a, &cal, 2.2f, 2.0f, 0.5f) == DSP_OK);
    CHECK(ctrl_observer_set_pilot(&a, 4) == DSP_OK);
    b = a;
    for (unsigned i = 0; i < 500; ++i) {
        harmonic_data_t h = sample(2.2, 1);
        harmonic_data_t bright = sample(2.2, 3);
        bright.dc_power = i % 2 ? NAN : 1e30f;
        CHECK(ctrl_observer_update(&a, &h, 2.0f) == DSP_OK);
        CHECK(ctrl_observer_update(&b, &bright, 2.0f) == DSP_OK);
        if (!i) {
            NEAR(a.obs_x, sin(2.0));
            NEAR(a.obs_y, cos(2.0));
        }
        NEAR(a.radius, 1);
        NEAR(b.radius, 3);
        NEAR(hypot(a.obs_x, a.obs_y), 1);
        NEAR(a.phase_rad, b.phase_rad);
        NEAR(a.error, b.error);
        CHECK(a.locked == b.locked && a.hold_assist_active == b.hold_assist_active);
    }
    NEAR(a.phase_rad, 2.2);

    CHECK(ctrl_observer_set_pilot(&a, 0) == DSP_SINGULAR);
    harmonic_data_t h = sample(0, 1);
    CHECK(ctrl_observer_update(&a, &h, 0) != DSP_OK);
    CHECK(ctrl_observer_set_pilot(&a, 4) == DSP_OK);
    cal.cal_pilot_v = 0;
    CHECK(ctrl_observer_init(&a, &cal, 0, 0, 4) == DSP_SINGULAR);
    cal = calibration();
    cal.fit.m[1][0] = 2;
    cal.fit.m[1][1] = 2 * cal.fit.m[0][1];
    CHECK(ctrl_observer_init(&a, &cal, 0, 0, 4) == DSP_SINGULAR);

    cal = calibration();
    CHECK(ctrl_observer_init(&a, &cal, PI, 3.1f, 4) == DSP_OK);
    h = sample(-3.0, 1);
    for (unsigned i = 0; i < 100; ++i)
        CHECK(ctrl_observer_update(&a, &h, 3.1f) == DSP_OK);
    NEAR(a.phase_rad, DSP_TAU - 3.0);
    return 0;
}
