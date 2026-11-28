#include "ctrl_bias_engine.h"
#include "ctrl_mod_mzm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

#define PI (DSP_TAU / 2)
enum { BLOCK = 1280, DECIMATION = 10, DC = 4, PILOT = 1 };
static const double vpi = 3.7, dark_v = -0.31;

typedef struct {
    double power, drift_v, previous_light, receiver, filter_gain;
} plant_t;

static plant_t plant_init(void)
{
    /* Bilinear one-pole photoreceiver, arg H(1 kHz) = -82 degrees. */
    double k = tan(PI / 64) / tan(82 * PI / 180);
    return (plant_t){.power = 1, .filter_gain = k / (1 + k)};
}

static double physical_phase(double bias, double drift)
{
    return PI * (bias - dark_v - drift) / vpi;
}

static float optical_sample(plant_t *p, float bias, float dither, double tone_phase)
{
    /* Residual amplitude modulation mixes sin(phi) and cos(phi) into both
     * rows. Electrical pilot pickup produces additive harmonic offsets. */
    double phase = physical_phase(bias + dither, p->drift_v);
    double light = p->power * (1 + 0.006 * dither) * (1 - cos(phase)) / 2;
    p->receiver += p->filter_gain * (light + p->previous_light - 2 * p->receiver);
    p->previous_light = light;
    return p->receiver + 0.00004 * cos(tone_phase) + 0.00002 * sin(2 * tone_phase);
}

static void window(ctrl_bias_engine_t *e, plant_t *plant, float bias, float monitor)
{
    unsigned length = BLOCK * e->decimation;
    for (unsigned n = 0; n < length; ++n) {
        double angle = DSP_TAU * e->pilot.phase_index / e->pilot.period;
        float ac = optical_sample(plant, bias, pilot_gen_current(&e->pilot), angle);
        bool ready;
        CHECK(ctrl_bias_engine_feed(e, ac, monitor, &ready) == DSP_OK);
        CHECK(ready == (n + 1 == length));
    }
    CHECK(e->last.dc_power == monitor);
}

static ctrl_calibration_t calibrate(bool with_dc)
{
    ctrl_bias_engine_t e;
    plant_t plant = plant_init();
    ctrl_bias_scan_t scan = {0};
    CHECK(ctrl_bias_engine_init(&e, 64000, 1000, 0.65f, BLOCK, 1, PILOT) == DSP_OK);
    for (unsigned pass = 0; pass < 2; ++pass) {
        for (unsigned n = 0; n <= 440; ++n) {
            float bias = -5.0f + n * 0.025f;
            float dc = with_dc ? (float)(0.40 * (1.0 - cos(physical_phase(bias, 0.0)))) : 0.0f;
            window(&e, &plant, bias, dc); /* receiver settling after bias step */
            window(&e, &plant, bias, dc);
            CHECK(ctrl_bias_scan_add(&scan, bias, &e.last) == DSP_OK);
        }
        if (pass == 0) CHECK(ctrl_bias_scan_begin_fit(&scan, 1) == DSP_OK);
    }
    ctrl_calibration_t cal;
    CHECK(ctrl_bias_scan_finish(&scan, e.pilot.amplitude, &cal) == DSP_OK);
    CHECK(cal.has_ref_phases);
    CHECK(fabs(cal.vpi_v - vpi) < 0.01 && fabs(cal.null_v - dark_v) < 0.02);
    return cal;
}

static void check_coherent_average(void)
{
    ctrl_bias_engine_t e;
    CHECK(ctrl_bias_engine_init(&e, 64000, 1000, 1, BLOCK, 2, PILOT) == DSP_OK);
    for (unsigned n = 0; n < 2 * BLOCK; ++n) {
        double t = DSP_TAU * n / 64;
        float sample = (n < BLOCK ? 1 : -1) * (sin(t) + cos(2*t));
        bool ready;
        CHECK(ctrl_bias_engine_feed(&e, sample, n < BLOCK ? 0 : 2, &ready) == DSP_OK);
        CHECK(ready == (n + 1 == 2 * BLOCK));
    }
    CHECK(e.last.h1_magnitude < 1e-6 && e.last.h2_magnitude < 1e-6 && e.last.dc_power == 1);
}

static void run_closed_loop(ctrl_calibration_t cal, bias_point_t point, double target,
                            bool test_freeze_and_drift)
{
    plant_t plant = plant_init();
    ctrl_bias_engine_t e;
    CHECK(ctrl_bias_engine_init(&e, 64000, 1000, cal.cal_pilot_v, BLOCK, DECIMATION, PILOT) == DSP_OK);
    ctrl_mod_mzm_t m = {.config = {.calibration = cal, .dc_channel = DC, .pilot_channel = PILOT,
        .kp = 0.0f, .ki = (float)(0.25 * cal.vpi_v / (PI * 0.2)), .dt = 0.2f,
        .min_v = -10, .max_v = 10, .pilot_v = cal.cal_pilot_v}};
    float sweep[CTRL_DC_CHANNELS] = {0};
    sweep[DC] = cal.null_v + cal.vpi_v * target / PI + 0.16;
    CHECK(ctrl_bias_engine_start(&e, &ctrl_mzm_strategy, &m, point, target, sweep) == DSP_OK);
    CHECK(e.command.bias_v[DC] == sweep[DC] && m.pid.integral_v == sweep[DC]);
    CHECK(e.command.routes.dc_mask == (1u << DC) && e.command.routes.pilot_mask[0] == 0 &&
          e.command.routes.pilot_mask[PILOT] == (1u << DC));

    double worst = 0;
    unsigned streak = 0;
    for (unsigned step = 0; step < 550; ++step) {
        if (step == 120) plant.power = 0.63;
        if (step == 180) {
            CHECK(ctrl_bias_engine_set_mzm_pilot(&e, 0) == DSP_SINGULAR);
            CHECK(e.pilot.amplitude == cal.cal_pilot_v && m.observer.scaled);
            CHECK(ctrl_bias_engine_set_mzm_pilot(&e, 0.47f) == DSP_OK);
            CHECK(!e.command.locked && !e.command.hold_assist_active);
            streak = 0;
        }
        /* Drift across > 0.6 * Vpi when testing Chapter 5 freeze & cross-fringe robustness. */
        if (test_freeze_and_drift && step >= 220 && step < 340) {
            plant.drift_v += 0.020; /* total drift +2.4 V > 0.64 * Vpi */
        } else if (step >= 220 && step < 320) {
            plant.drift_v += 0.00017;
        }

        if (test_freeze_and_drift && step >= 360 && step < 370) {
            plant.power = 0.02; /* forces r < 0.15 -> freeze without fault */
        } else if (test_freeze_and_drift && step == 370) {
            plant.power = 0.63;
            streak = 0;
        }

        float bias = e.command.bias_v[DC];
        double phase_error = fabs(remainder(physical_phase(bias, plant.drift_v) - target, DSP_TAU));
        if (step == 260) {
            /* Same optical state and samples, independent CH1 input. */
            ctrl_bias_engine_t twin = e;
            ctrl_mod_mzm_t other = m;
            plant_t same = plant;
            twin.ctx = &other;
            window(&twin, &same, bias, 10000);
            window(&e, &plant, bias, -9000);
            CHECK(twin.command.bias_v[DC] == e.command.bias_v[DC] &&
                  twin.command.locked == e.command.locked && other.observer.error == m.observer.error);
        } else {
            window(&e, &plant, bias, step < 260 ? 1 : -9000);
        }
        const ctrl_observer_t *o = &m.observer;
        if (test_freeze_and_drift && step == 360) {
            streak = 0; /* step 360 contains the optical power step transient (no T_wait in sim) */
            continue;
        }
        if (test_freeze_and_drift && step >= 361 && step < 370) {
            CHECK(e.command.frozen && !e.command.locked && e.command.bias_v[DC] == bias);
            streak = 0;
            continue;
        }
        bool criteria = o->observer_ok && o->radius_ok && o->error_ok && o->phase_ok;
        streak = criteria ? streak + 1 : 0;
        CHECK(!e.command.frozen);
        CHECK(e.command.locked == criteria && e.command.hold_assist_active == (streak >= 25));
        if (step >= 450) {
            worst = fmax(worst, phase_error * 180 / PI);
            CHECK(criteria && e.command.hold_assist_active);
        }
    }
    CHECK(worst < 1.5);
}

static void check_mzm_strategy_freeze_and_fringe_clamp(void)
{
    const ctrl_calibration_t identity = {.fit.m = {{1, 0}, {0, 1}}, .vpi_v = 2.0f,
                                         .null_v = -1.0f, .cal_pilot_v = 0.1f};
    ctrl_mod_mzm_t m = {.config = {.calibration = identity, .dc_channel = 0, .pilot_channel = 0,
                                   .kp = 0.0f, .ki = 1.0f, .dt = 0.1f, .min_v = -10.0f, .max_v = 10.0f,
                                   .pilot_v = 0.1f}};
    float sweep[CTRL_DC_CHANNELS] = {0.0f};
    ctrl_command_t cmd;
    CHECK(ctrl_mzm_strategy.start(&m, BIAS_POINT_QUAD, 0.0f, sweep, &cmd) == DSP_OK);
    /* Integrator clamping spans full [min_v, max_v], not anchor +/- Vpi/2. */
    CHECK(m.pid.out_min == -10.0f && m.pid.out_max == 10.0f);

    /* Drive bias past anchor + Vpi/2 (1.0 V) to verify no artificial fringe clamp. */
    harmonic_data_t pilots[CTRL_PILOTS] = {{.h1_magnitude = 0.5f, .h2_magnitude = 0.8660254f,
                                            .h2_phase = DSP_TAU / 4.0}};
    for (int i = 0; i < 15; ++i) {
        CHECK(ctrl_mzm_strategy.update(&m, pilots, &cmd) == DSP_OK);
        CHECK(!cmd.frozen);
    }
    CHECK(cmd.bias_v[0] > 1.2f);

    /* Frozen update must preserve bias and pid.integral_v, set frozen=true, return DSP_OK. */
    float held_bias = cmd.bias_v[0];
    double held_integ = m.pid.integral_v;
    harmonic_data_t zero_j0[CTRL_PILOTS] = {{.h1_magnitude = 0.05f, .h2_magnitude = 0.05f,
                                             .h2_phase = DSP_TAU / 4.0}};
    CHECK(ctrl_mzm_strategy.update(&m, zero_j0, &cmd) == DSP_OK);
    CHECK(cmd.frozen && !cmd.locked && !cmd.hold_assist_active);
    CHECK(cmd.bias_v[0] == held_bias && m.pid.integral_v == held_integ);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    int degrees = atoi(argv[1]);
    double target = degrees * PI / 180;
    bias_point_t point = BIAS_POINT_CUSTOM;
    if (degrees == 0) point = BIAS_POINT_MIN;
    if (degrees == 90) point = BIAS_POINT_QUAD;
    if (degrees == 180) point = BIAS_POINT_MAX;
    check_coherent_average();
    check_mzm_strategy_freeze_and_fringe_clamp();

    /* Legacy zero-DC fallback path. */
    ctrl_calibration_t cal_no_dc = calibrate(false);
    run_closed_loop(cal_no_dc, point, target, false);

    /* Thesis Ch.5 Fitzgibbon + CH1 DC path with large thermal drift & r<0.15 freeze. */
    ctrl_calibration_t cal_dc = calibrate(true);
    run_closed_loop(cal_dc, point, target, true);

    printf("%d deg target: Chapter 5 Fitzgibbon+DC & zero-DC fallback passed\n", degrees);
    return 0;
}
