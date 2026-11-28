#include "check.h"
#include "ctrl_observer.h"
#include <math.h>

#define NEAR(a,b) CHECK(fabs((a) - (b)) < 1e-6)
static const ctrl_calibration_t identity = {.fit.m = {{1,0},{0,1}}, .vpi_v = 2,
                                           .null_v = -1, .cal_pilot_v = 0.1f};
static const harmonic_data_t quad = {.h1_magnitude = 1};
static const harmonic_data_t null = {.h2_magnitude = 1, .h2_phase = DSP_TAU/4};

static void check_streak_and_thermal_drift(void)
{
    ctrl_observer_t s;
    CHECK(ctrl_observer_init(&s, &identity, -1, -1, 0.1f) == DSP_OK);
    for (unsigned run = 0; run < 2; ++run) {
        for (unsigned i = 1; i <= 26; ++i) {
            /* Thermal drift beyond 0.30*Vpi must NOT break lock. */
            float drifted_bias = (i > 12) ? 3.5f : -1.0f;
            CHECK(ctrl_observer_update(&s, &null, drifted_bias) == DSP_OK);
            CHECK(s.locked && s.observer_ok && s.radius_ok && s.error_ok && s.phase_ok);
            CHECK(!s.frozen);
            CHECK(s.hold_assist_active == (i >= 25));
        }
        if (!run) {
            harmonic_data_t weak = {.h1_magnitude = 0.14f};
            CHECK(ctrl_observer_update(&s, &weak, -1) == DSP_INCOMPLETE);
            CHECK(!s.radius_ok && s.frozen);
        } else {
            harmonic_data_t invalid = {.h1_phase = NAN};
            CHECK(ctrl_observer_update(&s, &invalid, -1) == DSP_INVALID_ARGUMENT);
            CHECK(!s.observer_ok && !s.frozen);
        }
        CHECK(!s.locked && !s.hold_assist_active && s.lock_streak == 0);
    }

    CHECK(ctrl_observer_init(&s, &identity, -1, -1, 0.1f) == DSP_OK);
    harmonic_data_t opposite = {.h2_magnitude = 1, .h2_phase = -DSP_TAU/4};
    CHECK(ctrl_observer_update(&s, &opposite, -1) == DSP_OK);
    CHECK(!s.phase_ok && !s.error_ok && !s.locked);

    CHECK(ctrl_observer_init(&s, &identity, -1, -1, 0.1f) == DSP_OK);
    harmonic_data_t offset_pt = {.h1_magnitude = sinf(0.3f), .h2_magnitude = cosf(0.3f),
                                 .h2_phase = DSP_TAU/4};
    CHECK(ctrl_observer_update(&s, &offset_pt, -1) == DSP_OK);
    NEAR(s.error, -0.3f);
    CHECK(s.phase_ok && !s.error_ok && !s.locked);
}

static void check_freeze_and_rho_ema(void)
{
    ctrl_observer_t s;
    CHECK(ctrl_observer_init(&s, &identity, 0, 0, 0.1f) == DSP_OK);
    CHECK(s.rho_ema == 0.0f && !s.frozen);

    /* Valid update with r = 1.5 -> rho_ema = 0.1 * 0.5 = 0.05 */
    harmonic_data_t bright = {.h1_magnitude = 1.5f};
    CHECK(ctrl_observer_update(&s, &bright, 0) == DSP_OK);
    NEAR(s.rho_ema, 0.05f);
    CHECK(!s.frozen && s.locked);

    /* Freeze on r = 0.14 < 0.15 must preserve rho_ema and return DSP_INCOMPLETE */
    harmonic_data_t faded = {.h1_magnitude = 0.14f};
    CHECK(ctrl_observer_update(&s, &faded, 0) == DSP_INCOMPLETE);
    CHECK(s.frozen && !s.radius_ok && !s.locked && !s.hold_assist_active);
    NEAR(s.rho_ema, 0.05f);

    /* Recovery on r = 1.0 -> frozen clears and rho_ema decays by 0.9 */
    CHECK(ctrl_observer_update(&s, &quad, 0) == DSP_OK);
    CHECK(!s.frozen && s.locked);
    NEAR(s.rho_ema, 0.045f);
}

int main(void)
{
    check_streak_and_thermal_drift();
    check_freeze_and_rho_ema();
    return 0;
}
