#include "check.h"
#include "dsp_goertzel.h"
#include "dsp_pilot_gen.h"
#include <math.h>

static void check_bins(unsigned n, float f, float interfering)
{
    goertzel_state_t h1, h2, other;
    dc_accum_t dc;
    CHECK(goertzel_init(&h1, f, 64000, n) == DSP_OK);
    CHECK(goertzel_init(&h2, 2 * f, 64000, n) == DSP_OK);
    CHECK(goertzel_init(&other, interfering, 64000, n) == DSP_OK);
    CHECK(dc_accum_init(&dc, n) == DSP_OK);
    for (unsigned block = 0; block < 2; ++block) {
        float mag, phase;
        CHECK(goertzel_get_result(&h1, &mag, &phase) == DSP_INCOMPLETE);
        CHECK(isnan(dc_accum_get_mean(&dc)));
        double lag = (block ? -82 : 82) * DSP_TAU / 360;
        for (unsigned i = 0; i < n; ++i) {
            double t = DSP_TAU * i / 64000;
            float sample = (float)(0.3 + 1.4 * cos(f*t + 0.32) + 0.16 * cos(2*f*t + lag));
            CHECK(goertzel_process_sample(&h1, sample) == DSP_OK);
            CHECK(goertzel_process_sample(&h2, sample) == DSP_OK);
            CHECK(goertzel_process_sample(&other, sample) == DSP_OK);
            CHECK(dc_accum_process(&dc, sample) == DSP_OK);
        }
        CHECK(goertzel_process_sample(&h1, 0) == DSP_BLOCK_FULL);
        CHECK(dc_accum_process(&dc, 0) == DSP_BLOCK_FULL);
        CHECK(goertzel_get_result(&h1, &mag, &phase) == DSP_OK);
        CHECK(fabs(mag - 1.4) < 1e-6 && fabs(phase - 0.32) < 1e-6);
        CHECK(goertzel_get_result(&h2, &mag, &phase) == DSP_OK);
        CHECK(fabs(mag - 0.16) < 1e-6 && fabs(mag * sin(phase) - 0.16 * sin(lag)) < 1e-6);
        CHECK(goertzel_get_result(&other, &mag, &phase) == DSP_OK && mag < 1e-7);
        CHECK(fabs(dc_accum_get_mean(&dc) - 0.3) < 1e-6);
        goertzel_reset(&h1);
        goertzel_reset(&h2);
        goertzel_reset(&other);
        dc_accum_reset(&dc);
    }
}

static void check_pilots(float f1, unsigned k1, float f2, unsigned k2, unsigned period)
{
    pilot_gen_t a, b;
    CHECK(pilot_gen_init(&a, f1, 64000, 1) == DSP_OK);
    CHECK(pilot_gen_init(&b, f2, 64000, 0.5f) == DSP_OK);
    CHECK(pilot_gen_next(&b) == 0);
    for (unsigned i = 0; i < 101 * period; ++i) {
        unsigned p1 = (i * k1) % period;
        unsigned p2 = ((i + 1) * k2) % period;
        CHECK(pilot_gen_get_phase_index(&a) == p1 && pilot_gen_get_phase_index(&b) == p2);
        CHECK(fabs(pilot_gen_next(&a) - sin(DSP_TAU * p1 / period)) < 1e-6);
        CHECK(fabs(pilot_gen_next(&b) - 0.5 * sin(DSP_TAU * p2 / period)) < 1e-6);
    }
    pilot_gen_reset(&a);
    CHECK(pilot_gen_next(&a) == 0 && pilot_gen_get_phase_index(&b) == k2);
    CHECK(pilot_gen_set_amplitude(&b, 2) == DSP_OK);
    CHECK(fabs(pilot_gen_current(&b) - 2 * sin(DSP_TAU * k2 / period)) < 1e-6);
}

int main(void)
{
    goertzel_state_t g;
    CHECK(goertzel_init(&g, 1000, 64000, 1279) == DSP_INCOHERENT);
    CHECK(goertzel_init(&g, nextafterf(962.5f, INFINITY), 64000, 5120) == DSP_INCOHERENT);
    CHECK(goertzel_init(&g, 32000, 64000, 1280) == DSP_INVALID_ARGUMENT);
    check_bins(1280, 1000, 1050);
    check_bins(5120, 962.5f, 1762.5f);
    check_bins(5120, 1762.5f, 962.5f);
    check_pilots(1000, 1, 3000, 3, 64);
    check_pilots(962.5f, 77, 1762.5f, 141, 5120);
    return 0;
}
