#include "check.h"
#include "dsp_online_lsq.h"
#include <math.h>

enum { ROWS = 467 };

/* Independent batch reference. Modified Gram-Schmidt operates on the
 * stored design columns directly, without forming the normal equations. */
static void batch_qr(double cols[3][ROWS], const double y[2][ROWS], double fit[2][3])
{
    double r[3][3] = {{0}};
    for (unsigned k = 0; k < 3; ++k) {
        for (unsigned i = 0; i < ROWS; ++i) r[k][k] += cols[k][i] * cols[k][i];
        r[k][k] = sqrt(r[k][k]);
        for (unsigned i = 0; i < ROWS; ++i) cols[k][i] /= r[k][k];
        for (unsigned j = k + 1; j < 3; ++j) {
            for (unsigned i = 0; i < ROWS; ++i) r[k][j] += cols[k][i] * cols[j][i];
            for (unsigned i = 0; i < ROWS; ++i) cols[j][i] -= r[k][j] * cols[k][i];
        }
    }
    for (unsigned h = 0; h < 2; ++h) {
        for (int k = 2; k >= 0; --k) {
            double z = 0;
            for (unsigned i = 0; i < ROWS; ++i) z += cols[k][i] * y[h][i];
            for (int j = k + 1; j < 3; ++j) z -= r[k][j] * fit[h][j];
            fit[h][k] = z / r[k][k];
        }
    }
}

static void check_fit(void)
{
    dsp_online_lsq_t state = {0};
    dsp_affine_fit_t fit;
    double cols[3][ROWS], y[2][ROWS], expected[2][3];
    double lag = 82 * DSP_TAU / 360;
    CHECK(dsp_online_lsq_solve(&state, &fit) == DSP_INCOMPLETE);
    for (unsigned i = 0; i < ROWS; ++i) {
        double phi = 0.31 + 1.32 * DSP_TAU * i / ROWS;
        float s = (float)sin(phi), c = (float)cos(phi);
        /* In-phase H1 and quadrature H2 with offsets, cross-coupling
         * and a small unmodelled harmonic in each observation. */
        float h1 = (float)(0.07 + (1.3*s + 0.18*c)*cos(lag) + 0.006*cos(3*phi));
        float h2 = (float)(-0.13 + (-0.21*s + 0.83*c)*sin(lag) + 0.004*sin(5*phi));
        cols[0][i] = 1;
        cols[1][i] = s;
        cols[2][i] = c;
        y[0][i] = h1;
        y[1][i] = h2;
        CHECK(dsp_online_lsq_add(&state, s, c, h1, h2) == DSP_OK);
    }
    batch_qr(cols, y, expected);
    CHECK(dsp_online_lsq_solve(&state, &fit) == DSP_OK);
    for (unsigned h = 0; h < 2; ++h) {
        double actual[] = {fit.o[h], fit.m[h][0], fit.m[h][1]};
        for (unsigned k = 0; k < 3; ++k)
            CHECK(fabs(actual[k] - expected[h][k]) < 1e-4 * fabs(expected[h][k]));
    }
    state = (dsp_online_lsq_t){0};
    for (unsigned i = 0; i < 8; ++i)
        CHECK(dsp_online_lsq_add(&state, 0, 1, 1, 2) == DSP_OK);
    CHECK(dsp_online_lsq_solve(&state, &fit) == DSP_SINGULAR);
}

static void check_minima(void)
{
    dsp_vpi_tracker_t tracker = {0};
    float vpi;
    CHECK(dsp_vpi_result(&tracker, &vpi) == DSP_INCOMPLETE);
    for (unsigned i = 0; i <= 400; ++i) {
        float v = -3.6f + i * 0.02f;
        float h = (float)fabs(sin((DSP_TAU / 2) * (v - 0.103) / 1.73));
        CHECK(dsp_vpi_add(&tracker, v, h) == DSP_OK);
    }
    CHECK(tracker.minima_count == 5);
    CHECK(dsp_vpi_result(&tracker, &vpi) == DSP_OK && fabs(vpi - 1.73) < 0.002);
    CHECK(dsp_vpi_add(&tracker, 0, 1) == DSP_INVALID_ARGUMENT);

    tracker = (dsp_vpi_tracker_t){0};
    const float h[] = {1, 0, 0, 1, 0.4f, 1, 0, 0, 1};
    for (unsigned i = 0; i < sizeof(h)/sizeof(h[0]); ++i)
        CHECK(dsp_vpi_add(&tracker, (float)i, h[i]) == DSP_OK);
    CHECK(tracker.minima_count == 2);
    CHECK(dsp_vpi_result(&tracker, &vpi) == DSP_OK && vpi == 5);
}

int main(void)
{
    check_fit();
    check_minima();
    return 0;
}
