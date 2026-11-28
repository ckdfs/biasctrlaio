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

static void test_legacy_projection_and_unwrap(void)
{
    ctrl_calibration_t cal = calibration();
    ctrl_observer_t a, b;
    CHECK(ctrl_observer_init(&a, &cal, 2.2f, 2.0f, 0.5f) == DSP_OK);
    CHECK(ctrl_observer_set_pilot(&a, 4) == DSP_OK);
    b = a;
    for (unsigned i = 0; i < 50; ++i) {
        harmonic_data_t h = sample(2.2, 1);
        harmonic_data_t bright = sample(2.2, 3);
        bright.dc_power = i % 2 ? NAN : 1e30f;
        CHECK(ctrl_observer_update(&a, &h, 2.0f) == DSP_OK);
        CHECK(ctrl_observer_update(&b, &bright, 10.0f) == DSP_OK);
        NEAR(a.obs_x, sin(2.2));
        NEAR(a.obs_y, cos(2.2));
        NEAR(a.radius, 1);
        NEAR(b.radius, 3);
        NEAR(hypot(a.obs_x, a.obs_y), 1);
        NEAR(a.phase_rad, b.phase_rad);
        NEAR(a.error, 0.0);
        NEAR(b.error, 0.0);
        CHECK(!a.frozen && !b.frozen);
        CHECK(a.locked && b.locked);
        CHECK(a.hold_assist_active == (i >= 24));
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
    CHECK(ctrl_observer_update(&a, &h, 3.1f) == DSP_OK);
    NEAR(a.phase_rad, DSP_TAU - 3.0);
}

static void test_ref_phase_projection_and_circular_error(void)
{
    ctrl_calibration_t cal = {
        .fit = {.o = {0.03f, -0.02f}, .m = {{-0.45f, 0.02f}, {-0.01f, -0.01f}}},
        .null_v = -0.31f,
        .vpi_v = 3.7f,
        .cal_pilot_v = 0.1592f,
        .h1_ref_phase = -0.33f,
        .h2_ref_phase = -1.708f,
        .has_ref_phases = true,
    };
    ctrl_observer_t obs;
    float target_v = cal.null_v + 0.5f * cal.vpi_v; /* 90 deg */
    CHECK(ctrl_observer_init(&obs, &cal, target_v, target_v, cal.cal_pilot_v) == DSP_OK);

    double inst = 1.2; /* target (pi/2) - 1.2 = 0.370796 rad */
    double s1 = cal.fit.o[0] + cal.fit.m[0][0] * sin(inst) + cal.fit.m[0][1] * cos(inst);
    double s2 = cal.fit.o[1] + cal.fit.m[1][0] * sin(inst) + cal.fit.m[1][1] * cos(inst);
    harmonic_data_t h = {
        .h1_magnitude = (float)fabs(s1),
        .h1_phase = (float)(s1 >= 0 ? cal.h1_ref_phase : cal.h1_ref_phase + PI),
        .h2_magnitude = (float)fabs(s2),
        .h2_phase = (float)(s2 >= 0 ? cal.h2_ref_phase : cal.h2_ref_phase + PI),
    };
    /* Pass bias_v far past 0.30*Vpi and verify exact circular error (zero spring/obs_dc). */
    CHECK(ctrl_observer_update(&obs, &h, target_v + 1.5f * cal.vpi_v) == DSP_OK);
    NEAR(obs.radius, 1.0);
    NEAR(obs.error, remainder(PI / 2.0 - inst, DSP_TAU));
    CHECK(obs.phase_ok && !obs.error_ok && !obs.locked);
}

int main(void)
{
    test_legacy_projection_and_unwrap();
    test_ref_phase_projection_and_circular_error();
    return 0;
}
