#include "dsp_online_lsq.h"
#include <float.h>
#include <math.h>

dsp_status_t dsp_online_lsq_add(dsp_online_lsq_t *s, float sin_phi, float cos_phi,
                                float h1s, float h2s)
{
    if (!s || !isfinite(sin_phi) || !isfinite(cos_phi) || !isfinite(h1s) || !isfinite(h2s))
        return DSP_INVALID_ARGUMENT;
    double row[] = {1, sin_phi, cos_phi};
    unsigned k = 0;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j)
            s->gram[k++] += row[i] * row[j];
        s->rhs[i][0] += row[i] * h1s;
        s->rhs[i][1] += row[i] * h2s;
    }
    return DSP_OK;
}

static dsp_status_t cholesky_3x3(const double sym[3][3], double lower[3][3])
{
    double max_diag = fmax(sym[0][0], fmax(sym[1][1], sym[2][2]));
    double tolerance = 64 * DBL_EPSILON * max_diag;
    if (!(max_diag > 0)) return DSP_SINGULAR;

    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j) {
            double value = sym[i][j];
            for (unsigned p = 0; p < j; ++p)
                value -= lower[i][p] * lower[j][p];
            if (i == j) {
                if (value <= tolerance) return DSP_SINGULAR;
                lower[i][i] = sqrt(value);
            } else {
                lower[i][j] = value / lower[j][j];
                lower[j][i] = 0.0;
            }
        }
    }
    return DSP_OK;
}

static void solve_cholesky_3x3(const double lower[3][3], const double b[3], double x[3])
{
    for (int i = 0; i < 3; ++i) {
        double value = b[i];
        for (int j = 0; j < i; ++j) value -= lower[i][j] * x[j];
        x[i] = value / lower[i][i];
    }
    for (int i = 2; i >= 0; --i) {
        for (int j = i + 1; j < 3; ++j) x[i] -= lower[j][i] * x[j];
        x[i] /= lower[i][i];
    }
}

dsp_status_t dsp_online_lsq_solve(const dsp_online_lsq_t *s, dsp_affine_fit_t *fit)
{
    if (!s || !fit) return DSP_INVALID_ARGUMENT;
    if (s->gram[0] < 3) return DSP_INCOMPLETE;

    /* Cholesky factorization of the symmetric Gram matrix. A small or
     * negative pivot means these phases do not identify the affine model. */
    double sym[3][3], lower[3][3] = {{0}};
    unsigned k = 0;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j <= i; ++j) {
            sym[i][j] = s->gram[k++];
            sym[j][i] = sym[i][j];
        }
    }
    dsp_status_t rc = cholesky_3x3(sym, lower);
    if (rc != DSP_OK) return rc;

    dsp_affine_fit_t result;
    for (unsigned h = 0; h < 2; ++h) {
        double b[3] = {s->rhs[0][h], s->rhs[1][h], s->rhs[2][h]};
        double x[3];
        solve_cholesky_3x3(lower, b, x);
        for (unsigned i = 0; i < 3; ++i)
            if (!isfinite(x[i]) || fabs(x[i]) > FLT_MAX) return DSP_INVALID_ARGUMENT;
        result.o[h] = (float)x[0];
        result.m[h][0] = (float)x[1];
        result.m[h][1] = (float)x[2];
    }
    *fit = result;
    return DSP_OK;
}

dsp_status_t dsp_vpi_add(dsp_vpi_tracker_t *s, float bias_v, float h1_mag)
{
    if (!s || !isfinite(bias_v) || !isfinite(h1_mag) || h1_mag < 0 ||
        (s->window_count && bias_v <= s->prev_v))
        return DSP_INVALID_ARGUMENT;
    s->peak_h = fmax(s->peak_h, h1_mag);
    if (s->window_count && h1_mag < s->prev_h) {
        s->descending = 1;
        s->valley_start = bias_v;
    }
    if (s->window_count == 2 && h1_mag > s->prev_h && s->descending) {
        if (s->prev_h < 0.1 * s->peak_h) {
            double v = (s->valley_start + s->prev_v) / 2;
            if (s->valley_start == s->prev_v) {
                /* Vertex of the quadratic through three (possibly uneven) samples. */
                double dl = s->prev_v - s->left_v, dr = bias_v - s->prev_v;
                double sl = (s->prev_h - s->left_h) / dl, sr = (h1_mag - s->prev_h) / dr;
                v -= 0.5 * (sl * dr + sr * dl) / (sr - sl);
            }
            if (!s->minima_count) s->first_min_v = v;
            s->last_min_v = v;
            ++s->minima_count;
        }
        s->descending = 0;
    }
    s->left_v = s->prev_v;
    s->left_h = s->prev_h;
    s->prev_v = bias_v;
    s->prev_h = h1_mag;
    if (s->window_count < 2) ++s->window_count;
    return DSP_OK;
}

dsp_status_t dsp_vpi_result(const dsp_vpi_tracker_t *s, float *vpi)
{
    if (!s || !vpi) return DSP_INVALID_ARGUMENT;
    if (s->minima_count < 2) return DSP_INCOMPLETE;
    *vpi = (float)((s->last_min_v - s->first_min_v) / (s->minima_count - 1));
    return DSP_OK;
}

dsp_status_t dsp_complex_cov_add(dsp_complex_cov_t *c, float mag, float phase)
{
    if (!c || !isfinite(mag) || mag < 0 || !isfinite(phase))
        return DSP_INVALID_ARGUMENT;
    double re = (double)mag * cos((double)phase);
    double im = (double)mag * sin((double)phase);
    c->count += 1.0;
    double dx = re - c->mean_re;
    double dy = im - c->mean_im;
    c->mean_re += dx / c->count;
    c->mean_im += dy / c->count;
    c->c_xx += dx * (re - c->mean_re);
    c->c_xy += dx * (im - c->mean_im);
    c->c_yy += dy * (im - c->mean_im);
    return DSP_OK;
}

dsp_status_t dsp_complex_cov_principal(const dsp_complex_cov_t *c, float *ref_phase,
                                       double *mean_proj, double *rms_scale)
{
    if (!c || !ref_phase) return DSP_INVALID_ARGUMENT;
    if (c->count < 3.0) return DSP_INCOMPLETE;

    double tr = c->c_xx + c->c_yy;
    double diff = c->c_xx - c->c_yy;
    double lam1 = 0.5 * (tr + hypot(diff, 2.0 * c->c_xy));
    if (!isfinite(lam1) || lam1 <= 1e-24) return DSP_SINGULAR;

    double theta = 0.5 * atan2(2.0 * c->c_xy, diff);
    double ct = cos(theta), st = sin(theta);
    /* Canonicalize eigenvector sign to the right half-plane {Re > 0} U {Re == 0, Im >= 0}. */
    if (ct < -1e-12 || (fabs(ct) <= 1e-12 && st < 0.0)) {
        theta = remainder(theta + DSP_TAU / 2.0, DSP_TAU);
        ct = cos(theta);
        st = sin(theta);
    }
    *ref_phase = (float)theta;
    if (mean_proj) *mean_proj = c->mean_re * ct + c->mean_im * st;
    if (rms_scale) *rms_scale = sqrt(lam1 / c->count);
    return DSP_OK;
}

/* Jacobi eigendecomposition for 3x3 real symmetric matrix, sorted descending. */
static void jacobi_3x3(const double mat[3][3], double evals[3], double vecs[3][3])
{
    double d[3][3];
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            d[i][j] = 0.5 * (mat[i][j] + mat[j][i]);
            vecs[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (unsigned iter = 0; iter < 50; ++iter) {
        unsigned p = 0, q = 1;
        double mx = fabs(d[0][1]);
        if (fabs(d[0][2]) > mx) { p = 0; q = 2; mx = fabs(d[0][2]); }
        if (fabs(d[1][2]) > mx) { p = 1; q = 2; mx = fabs(d[1][2]); }
        double scale = fabs(d[0][0]) + fabs(d[1][1]) + fabs(d[2][2]);
        if (mx <= 1e-15 * fmax(1.0, scale)) break;

        double diff = d[q][q] - d[p][p];
        double t;
        if (diff == 0.0) {
            t = d[p][q] >= 0.0 ? 1.0 : -1.0;
        } else if (fabs(d[p][q]) <= 1e-30 * fabs(diff)) {
            t = d[p][q] / diff;
        } else {
            double tau = diff / (2.0 * d[p][q]);
            t = 1.0 / (fabs(tau) + sqrt(1.0 + tau * tau));
            if (tau < 0.0) t = -t;
        }
        double c = 1.0 / sqrt(1.0 + t * t);
        double s = t * c;
        double app = d[p][p], aqq = d[q][q], apq = d[p][q];
        d[p][p] = app - t * apq;
        d[q][q] = aqq + t * apq;
        d[p][q] = d[q][p] = 0.0;
        for (unsigned r = 0; r < 3; ++r) {
            if (r != p && r != q) {
                double arp = d[r][p], arq = d[r][q];
                d[r][p] = d[p][r] = c * arp - s * arq;
                d[r][q] = d[q][r] = s * arp + c * arq;
            }
        }
        for (unsigned r = 0; r < 3; ++r) {
            double vp = vecs[r][p], vq = vecs[r][q];
            vecs[r][p] = c * vp - s * vq;
            vecs[r][q] = s * vp + c * vq;
        }
    }

    unsigned order[3] = {0, 1, 2};
    for (unsigned i = 0; i < 2; ++i) {
        for (unsigned j = i + 1; j < 3; ++j) {
            if (d[order[j]][order[j]] > d[order[i]][order[i]]) {
                unsigned tmp = order[i];
                order[i] = order[j];
                order[j] = tmp;
            }
        }
    }
    double sorted_v[3][3];
    for (unsigned col = 0; col < 3; ++col) {
        evals[col] = d[order[col]][order[col]];
        for (unsigned row = 0; row < 3; ++row)
            sorted_v[row][col] = vecs[row][order[col]];
    }
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            vecs[row][col] = sorted_v[row][col];
}

void dsp_fitzgibbon_init(dsp_fitzgibbon_t *f, double mu_x, double mu_y,
                         double scale_x, double scale_y)
{
    if (!f) return;
    *f = (dsp_fitzgibbon_t){
        .mu_x = isfinite(mu_x) ? mu_x : 0.0,
        .mu_y = isfinite(mu_y) ? mu_y : 0.0,
        .scale_x = (isfinite(scale_x) && scale_x > 0.0) ? scale_x : 1.0,
        .scale_y = (isfinite(scale_y) && scale_y > 0.0) ? scale_y : 1.0,
    };
}

dsp_status_t dsp_fitzgibbon_add(dsp_fitzgibbon_t *f, double x, double y, double dc)
{
    if (!f || !isfinite(x) || !isfinite(y) || !isfinite(dc))
        return DSP_INVALID_ARGUMENT;

    double sx = f->scale_x > 0.0 ? f->scale_x : 1.0;
    double sy = f->scale_y > 0.0 ? f->scale_y : 1.0;
    double u = (x - f->mu_x) / sx;
    double v = (y - f->mu_y) / sy;
    double d[6] = {u * u, u * v, v * v, u, v, 1.0};

    unsigned k = 0;
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j <= i; ++j)
            f->scatter[k++] += d[i] * d[j];

    f->dc_rhs[0] += dc * u;
    f->dc_rhs[1] += dc * v;
    f->dc_rhs[2] += dc;

    if (f->count == 0) {
        f->first_x = x;
        f->first_y = y;
    } else {
        f->winding += f->prev_x * y - x * f->prev_y;
    }
    f->prev_x = x;
    f->prev_y = y;
    ++f->count;
    return DSP_OK;
}

dsp_status_t dsp_fitzgibbon_solve(const dsp_fitzgibbon_t *f, dsp_affine_fit_t *fit)
{
    if (!f || !fit) return DSP_INVALID_ARGUMENT;
    if (f->count < 6) return DSP_INCOMPLETE;

    double sx = f->scale_x > 0.0 ? f->scale_x : 1.0;
    double sy = f->scale_y > 0.0 ? f->scale_y : 1.0;

    double s_full[6][6];
    unsigned idx = 0;
    for (unsigned i = 0; i < 6; ++i) {
        for (unsigned j = 0; j <= i; ++j) {
            s_full[i][j] = f->scatter[idx++];
            s_full[j][i] = s_full[i][j];
        }
    }

    double s3[3][3], l3[3][3] = {{0}};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            s3[i][j] = s_full[3 + i][3 + j];
    if (cholesky_3x3(s3, l3) != DSP_OK) return DSP_SINGULAR;

    /* Compute S3_inv * S2^T (3x3) and S3_inv * dc_rhs. */
    double s3_inv_s2t[3][3];
    for (unsigned col = 0; col < 3; ++col) {
        double rhs_col[3] = {s_full[3][col], s_full[4][col], s_full[5][col]};
        double sol[3];
        solve_cholesky_3x3(l3, rhs_col, sol);
        for (unsigned row = 0; row < 3; ++row)
            s3_inv_s2t[row][col] = sol[row];
    }

    double t_red[3][3];
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            double val = s_full[i][j];
            for (unsigned p = 0; p < 3; ++p)
                val -= s_full[i][3 + p] * s3_inv_s2t[p][j];
            t_red[i][j] = val;
        }
    }

    double evals[3], u_mat[3][3];
    jacobi_3x3(t_red, evals, u_mat);
    if (!(evals[0] > 0.0) || evals[1] <= 1e-12 * evals[0]) return DSP_SINGULAR;

    double a1[3];
    if (evals[2] <= 1e-12 * evals[0]) {
        for (unsigned i = 0; i < 3; ++i) a1[i] = u_mat[i][2];
    } else {
        static const double c1_inv[3][3] = {
            {0.0, 0.0, 0.5},
            {0.0, -1.0, 0.0},
            {0.5, 0.0, 0.0}
        };
        double inv_u_c1inv[3][3] = {{0}};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                for (unsigned p = 0; p < 3; ++p)
                    inv_u_c1inv[i][j] += u_mat[p][i] * c1_inv[p][j];

        double mid[3][3] = {{0}};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                for (unsigned p = 0; p < 3; ++p)
                    mid[i][j] += inv_u_c1inv[i][p] * u_mat[p][j];

        double sq_eval[3] = {sqrt(evals[0]), sqrt(evals[1]), sqrt(evals[2])};
        double g_mat[3][3];
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                g_mat[i][j] = sq_eval[i] * mid[i][j] * sq_eval[j];

        double gevals[3], gvecs[3][3];
        jacobi_3x3(g_mat, gevals, gvecs);
        if (!(gevals[0] > 0.0)) return DSP_SINGULAR;

        double z[3];
        for (unsigned i = 0; i < 3; ++i) z[i] = gvecs[i][0] / sq_eval[i];
        double norm = 0.0;
        for (unsigned i = 0; i < 3; ++i) {
            a1[i] = 0.0;
            for (unsigned j = 0; j < 3; ++j)
                a1[i] += u_mat[i][j] * z[j];
            norm += a1[i] * a1[i];
        }
        norm = sqrt(norm);
        if (!(norm > 0.0)) return DSP_SINGULAR;
        for (unsigned i = 0; i < 3; ++i) a1[i] /= norm;
    }

    double a2[3] = {0};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            a2[i] -= s3_inv_s2t[i][j] * a1[j];

    double ca = a1[0], cb = a1[1], cc = a1[2], cd = a2[0], ce = a2[1], cf = a2[2];
    if (ca + cc < 0.0) {
        ca = -ca; cb = -cb; cc = -cc;
        cd = -cd; ce = -ce; cf = -cf;
    }
    double disc = 4.0 * ca * cc - cb * cb;
    if (!isfinite(disc) || disc <= 1e-18 * (ca * ca + cc * cc + cb * cb)) return DSP_SINGULAR;

    double ou = (cb * ce - 2.0 * cc * cd) / disc;
    double ov = (cb * cd - 2.0 * ca * ce) / disc;
    double q00 = ca, q01 = 0.5 * cb, q11 = cc;
    double kappa = ou * (q00 * ou + q01 * ov) + ov * (q01 * ou + q11 * ov) - cf;
    if (!isfinite(kappa) || kappa <= 0.0) return DSP_SINGULAR;

    double ox = f->mu_x + sx * ou;
    double oy = f->mu_y + sy * ov;
    double mp = (q00 / kappa) / (sx * sx);
    double mq = (q01 / kappa) / (sx * sy);
    double mr = (q11 / kappa) / (sy * sy);

    double det_m = mp * mr - mq * mq;
    if (!isfinite(det_m) || det_m <= 0.0) return DSP_SINGULAR;
    double s_det = sqrt(det_m);
    double t_tr = sqrt(mp + mr + 2.0 * s_det);
    if (!isfinite(t_tr) || t_tr <= 0.0) return DSP_SINGULAR;

    /* M^{-1/2} in (X, Y) coordinates. */
    double inv_half[2][2] = {
        {(mr + s_det) / (s_det * t_tr), -mq / (s_det * t_tr)},
        {-mq / (s_det * t_tr), (mp + s_det) / (s_det * t_tr)}
    };

    /* Center shoelace winding at (ox, oy) to resolve reflection F = diag(1, +/-1). */
    double w_cent = f->winding - ox * (f->prev_y - f->first_y) + oy * (f->prev_x - f->first_x);
    double f_sign = (w_cent >= 0.0) ? 1.0 : -1.0;

    /* Solve 3x3 CH1 DC regression [u, v, 1] -> [dc_u, dc_v, dc_0] using cached l3. */
    double dc_sol[3];
    solve_cholesky_3x3(l3, f->dc_rhs, dc_sol);
    double c_xy[2] = {dc_sol[0] / sx, dc_sol[1] / sy};

    /* (C_c, C_s)^T = (F M^{1/2})^{-T} c_XY = F M^{-1/2} c_XY. */
    double cc_unit = inv_half[0][0] * c_xy[0] + inv_half[0][1] * c_xy[1];
    double cs_unit = f_sign * (inv_half[1][0] * c_xy[0] + inv_half[1][1] * c_xy[1]);
    if (!isfinite(cc_unit) || !isfinite(cs_unit) || hypot(cc_unit, cs_unit) <= 1e-12)
        return DSP_SINGULAR;

    double phi_c = atan2(cs_unit, cc_unit);
    double cp = cos(phi_c), sp = sin(phi_c);

    /* A_XY = -M^{-1/2} * F * Rot(phi_c) maps [cos(phi_fw), sin(phi_fw)]^T to (X, Y) - o. */
    double mf[2][2] = {
        {inv_half[0][0], f_sign * inv_half[0][1]},
        {inv_half[1][0], f_sign * inv_half[1][1]}
    };
    double a_xy[2][2] = {
        {-(mf[0][0] * cp + mf[0][1] * sp), -(-mf[0][0] * sp + mf[0][1] * cp)},
        {-(mf[1][0] * cp + mf[1][1] * sp), -(-mf[1][0] * sp + mf[1][1] * cp)}
    };

    /* Map (X=s2, Y=s1), [cos(phi_fw), sin(phi_fw)] to firmware [s1, s2], [sin(phi_fw), cos(phi_fw)]. */
    dsp_affine_fit_t result = {
        .o = {(float)oy, (float)ox},
        .m = {
            {(float)a_xy[1][1], (float)a_xy[1][0]},
            {(float)a_xy[0][1], (float)a_xy[0][0]}
        }
    };
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(result.o[i]) || !isfinite(result.m[i][0]) || !isfinite(result.m[i][1]))
            return DSP_SINGULAR;
    }
    *fit = result;
    return DSP_OK;
}
