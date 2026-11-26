#include "check.h"
#include "ctrl_observer.h"
#include <math.h>

#define NEAR(a,b) CHECK(fabs((a) - (b)) < 1e-6)
static const ctrl_calibration_t identity = {.fit.m = {{1,0},{0,1}}, .vpi_v = 2,
                                           .null_v = -1, .cal_pilot_v = 0.1f};
static const harmonic_data_t quad = {.h1_magnitude = 1};
static const harmonic_data_t null = {.h2_magnitude = 1, .h2_phase = DSP_TAU/4};

static void check_streak(void)
{
    ctrl_observer_t s;
    CHECK(ctrl_observer_init(&s, &identity, -1, -1, 0.1f) == DSP_OK);
    for (unsigned run = 0; run < 3; ++run) {
        for (unsigned i = 1; i <= 26; ++i) {
            CHECK(ctrl_observer_update(&s, &null, -1) == DSP_OK);
            CHECK(s.locked && s.bias_ok && s.observer_ok && s.radius_ok && s.error_ok && s.phase_ok);
            CHECK(s.hold_assist_active == (i >= 25));
        }
        if (!run) {
            CHECK(ctrl_observer_update(&s, &null, 3) == DSP_OK);
            CHECK(!s.bias_ok && s.phase_ok && s.error_ok);
        } else if (run == 1) {
            harmonic_data_t weak = {.h1_magnitude = 0.09f};
            CHECK(ctrl_observer_update(&s, &weak, -1) == DSP_INCOMPLETE);
            CHECK(!s.radius_ok);
        } else {
            harmonic_data_t invalid = {.h1_phase = NAN};
            CHECK(ctrl_observer_update(&s, &invalid, -1) == DSP_INVALID_ARGUMENT);
            CHECK(!s.observer_ok);
        }
        CHECK(!s.locked && !s.hold_assist_active && s.lock_streak == 0);
    }

    CHECK(ctrl_observer_init(&s, &identity, -1, 1, 0.1f) == DSP_OK);
    CHECK(ctrl_observer_update(&s, &null, -1) == DSP_OK);
    CHECK(!s.phase_ok && s.bias_ok && s.error_ok && !s.locked);
    CHECK(ctrl_observer_init(&s, &identity, -1, -0.8f, 0.1f) == DSP_OK);
    CHECK(ctrl_observer_update(&s, &null, -1) == DSP_OK);
    CHECK(s.phase_ok && s.bias_ok && !s.error_ok && !s.locked);
}

static void check_spring_and_ema(void)
{
    ctrl_observer_t s;
    for (unsigned i = 0; i <= 4; ++i) {
        float target = -1 + 0.5f * i;
        CHECK(ctrl_observer_init(&s, &identity, target, target, 0.1f) == DSP_OK);
        CHECK(ctrl_observer_update(&s, &quad, target + 0.2f) == DSP_OK);
        double sine = sin(i * DSP_TAU/8);
        NEAR(s.spring, -0.06 * sine * sine);
    }
    CHECK(ctrl_observer_init(&s, &identity, 0, -0.2f, 0.1f) == DSP_OK);
    CHECK(ctrl_observer_update(&s, &quad, -0.2f) == DSP_OK);
    NEAR(s.obs_dc, s.obs_raw);
    NEAR(s.error, s.obs_raw + s.spring);
    for (unsigned i = 1; i <= 5; ++i) {
        float previous = s.obs_dc, filtered = s.obs_filtered;
        CHECK(ctrl_observer_update(&s, &quad, 1) == DSP_OK);
        NEAR(s.obs_dc, 0.5 * (previous + s.obs_raw));
        NEAR(s.obs_filtered, filtered + 0.1 * (s.obs_raw - (i == 5 ? s.obs_dc : 0) - filtered));
    }
    float frozen = s.obs_dc;
    CHECK(ctrl_observer_update(&s, &quad, 1) == DSP_OK);
    NEAR(s.obs_dc, frozen);

    ctrl_calibration_t seeded = identity;
    seeded.has_obs_dc_seed = true;
    seeded.obs_dc_seed = 0.02f;
    CHECK(ctrl_observer_init(&s, &seeded, 0, 0, 0.1f) == DSP_OK);
    CHECK(ctrl_observer_update(&s, &quad, 0) == DSP_OK);
    NEAR(s.obs_dc, 0.02);
    NEAR(s.error, -0.02);
    CHECK(s.locked);
    CHECK(ctrl_observer_update(&s, &quad, 0) == DSP_OK);
    NEAR(s.obs_dc, 0.01);
}

int main(void)
{
    check_streak();
    check_spring_and_ema();
    return 0;
}
