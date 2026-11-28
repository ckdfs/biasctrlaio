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

static void check_complex_cov(void)
{
    dsp_complex_cov_t cov = {0};
    float ref = 0.0f;
    double mean_p = 0.0, rms = 0.0;
    CHECK(dsp_complex_cov_principal(&cov, &ref, &mean_p, &rms) == DSP_INCOMPLETE);

    /* Line along -1.708 rad with a large constant coherent offset (0.25, -0.18). */
    const double true_lag = -1.708;
    for (unsigned i = 0; i < 257; ++i) {
        double phi = DSP_TAU * i / 256.0;
        double s = -0.35 * cos(phi);
        double re = 0.25 + s * cos(true_lag);
        double im = -0.18 + s * sin(true_lag);
        CHECK(dsp_complex_cov_add(&cov, (float)hypot(re, im), (float)atan2(im, re)) == DSP_OK);
    }
    CHECK(dsp_complex_cov_principal(&cov, &ref, &mean_p, &rms) == DSP_OK);
    /* Principal axis equals true_lag modulo pi in canonical right half-plane. */
    double axis_err = remainder(ref - true_lag, DSP_TAU / 2.0);
    CHECK(fabs(axis_err) < 1e-5);
    CHECK(rms > 0.2 && isfinite(mean_p));

    /* Constant phasor (zero variance) must fail with DSP_SINGULAR. */
    cov = (dsp_complex_cov_t){0};
    for (unsigned i = 0; i < 10; ++i)
        CHECK(dsp_complex_cov_add(&cov, 0.5f, 0.2f) == DSP_OK);
    CHECK(dsp_complex_cov_principal(&cov, &ref, NULL, NULL) == DSP_SINGULAR);
}

static void run_fitzgibbon_case(double o1, double o2, const double m_true[2][2],
                                double dc_a, double dc_b, double phi_span)
{
    dsp_fitzgibbon_t f;
    dsp_fitzgibbon_init(&f, o2, o1, fabs(m_true[1][0]) + fabs(m_true[1][1]),
                        fabs(m_true[0][0]) + fabs(m_true[0][1]));
    dsp_affine_fit_t fit;
    CHECK(dsp_fitzgibbon_solve(&f, &fit) == DSP_INCOMPLETE);

    const unsigned n = 257;
    for (unsigned i = 0; i < n; ++i) {
        double phi = -0.4 + phi_span * i / (n - 1);
        double s1 = o1 + m_true[0][0] * sin(phi) + m_true[0][1] * cos(phi);
        double s2 = o2 + m_true[1][0] * sin(phi) + m_true[1][1] * cos(phi);
        double dc = dc_a - dc_b * cos(phi);
        CHECK(dsp_fitzgibbon_add(&f, s2, s1, dc) == DSP_OK);
    }
    CHECK(dsp_fitzgibbon_solve(&f, &fit) == DSP_OK);
    double det = (double)fit.m[0][0] * fit.m[1][1] - (double)fit.m[0][1] * fit.m[1][0];
    CHECK(fabs(det) > 1e-9);
    double max_err = 0.0;
    for (unsigned i = 0; i < 120; ++i) {
        double phi = -DSP_TAU / 2 + DSP_TAU * i / 120.0;
        double s1 = o1 + m_true[0][0] * sin(phi) + m_true[0][1] * cos(phi);
        double s2 = o2 + m_true[1][0] * sin(phi) + m_true[1][1] * cos(phi);
        double d1 = s1 - fit.o[0], d2 = s2 - fit.o[1];
        double u_sin = ( fit.m[1][1] * d1 - fit.m[0][1] * d2) / det;
        double u_cos = (-fit.m[1][0] * d1 + fit.m[0][0] * d2) / det;
        double r = hypot(u_sin, u_cos);
        double phi_hat = atan2(u_sin, u_cos);
        CHECK(fabs(r - 1.0) < 1e-4);
        max_err = fmax(max_err, fabs(remainder(phi_hat - phi, DSP_TAU)));
    }
    CHECK(max_err < 1e-4);
}

static void check_fitzgibbon_dc(void)
{
    /* Realistic high-kappa 20261110-MZM matrix (CCW) with cross-coupling and offset. */
    const double m_ccw[2][2] = {{-0.438, 0.015}, {-0.002, -0.0094}};
    run_fitzgibbon_case(0.025, -0.0012, m_ccw, 0.42, 0.39, 1.85 * DSP_TAU);

    /* Clockwise winding with large offset exceeding ellipse radius and partial 1.4-cycle scan. */
    const double m_cw[2][2] = {{0.38, -0.045}, {0.004, -0.0085}};
    run_fitzgibbon_case(2.1, -1.7, m_cw, 0.50, 0.45, 1.40 * DSP_TAU);
}

int main(void)
{
    check_fit();
    check_minima();
    check_complex_cov();
    check_fitzgibbon_dc();
    return 0;
}
