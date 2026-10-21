/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Discrete LQR design: servo plant by exact ZOH, DARE by structured doubling.
 */
#include "espFoC/utils/esp_foc_lqr.h"

#include <math.h>
#include <string.h>

#define N_MAX ESP_FOC_LQR_N_MAX
#define SDA_ITERS 64
#define SDA_TOL 1.0e-13
#define DECAY_SQUARINGS 14
#define DECAY_DIVERGED 1.0e12
/* (a - b k)^16384 has to have shrunk: a pole at 0.99996 is ~9 s at 2.5 kHz. */
#define DECAY_MAX 0.9

typedef double mat_t[N_MAX][N_MAX];

/* out = op(x) op(y), op = transpose when tx / ty; out may alias x or y. */
static void mat_mul(uint8_t n, const mat_t x, bool tx, const mat_t y, bool ty, mat_t out)
{
    mat_t t;
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            double s = 0.0;
            for (uint8_t k = 0; k < n; k++) {
                s += (tx ? x[k][r] : x[r][k]) * (ty ? y[c][k] : y[k][c]);
            }
            t[r][c] = s;
        }
    }
    memcpy(out, t, sizeof(t));
}

/* Gauss-Jordan with partial pivoting; destroys a. False on a singular matrix. */
static bool mat_inv(uint8_t n, mat_t a, mat_t inv)
{
    memset(inv, 0, sizeof(mat_t));
    for (uint8_t i = 0; i < n; i++) {
        inv[i][i] = 1.0;
    }
    for (uint8_t c = 0; c < n; c++) {
        uint8_t p = c;
        for (uint8_t r = c + 1u; r < n; r++) {
            if (fabs(a[r][c]) > fabs(a[p][c])) {
                p = r;
            }
        }
        if (!(fabs(a[p][c]) > 1.0e-300)) {
            return false;
        }
        if (p != c) {
            for (uint8_t k = 0; k < n; k++) {
                double t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
                t = inv[c][k];
                inv[c][k] = inv[p][k];
                inv[p][k] = t;
            }
        }
        const double d = 1.0 / a[c][c];
        for (uint8_t k = 0; k < n; k++) {
            a[c][k] *= d;
            inv[c][k] *= d;
        }
        for (uint8_t r = 0; r < n; r++) {
            if (r == c) {
                continue;
            }
            const double f = a[r][c];
            if (f == 0.0) {
                continue;
            }
            for (uint8_t k = 0; k < n; k++) {
                a[r][k] -= f * a[c][k];
                inv[r][k] -= f * inv[c][k];
            }
        }
    }
    return true;
}

static double mat_norm_inf(uint8_t n, const mat_t x)
{
    double m = 0.0;
    for (uint8_t r = 0; r < n; r++) {
        double s = 0.0;
        for (uint8_t c = 0; c < n; c++) {
            s += fabs(x[r][c]);
        }
        m = (s > m) ? s : m;
    }
    return m;
}

esp_err_t esp_foc_lqr_servo_model(const esp_foc_lqr_servo_t *p, esp_foc_lqr_model_t *m)
{
    if ((p == NULL) || (m == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!(p->k_rads2_a > 0.0) || !(p->ts_s > 0.0) || !(p->tau_i_s > 0.0) ||
        (p->position && (p->pole_pairs == 0u)) || !isfinite(p->k_rads2_a) ||
        !isfinite(p->ts_s) || !isfinite(p->tau_i_s)) {
        return ESP_ERR_INVALID_ARG;
    }
    const double t = p->ts_s;
    const double tau = p->tau_i_s;
    const double k = p->k_rads2_a;
    const double a = exp(-t / tau);
    const double c1 = tau * (1.0 - a);

    memset(m, 0, sizeof(*m));
    const uint8_t th = 0u;
    const uint8_t w = p->position ? 1u : 0u;
    const uint8_t i = w + 1u;
    const uint8_t up = w + 2u;
    const uint8_t z = w + 3u;
    m->n = (uint8_t)(up + 1u + (p->integral ? 1u : 0u));

    m->a[w][w] = 1.0;
    m->a[w][i] = k * c1;
    m->a[w][up] = k * (t - c1);
    m->a[i][i] = a;
    m->a[i][up] = 1.0 - a;
    m->b[up] = 1.0;
    if (p->position) {
        const double ipp = 1.0 / (double)p->pole_pairs;
        m->a[th][th] = 1.0;
        m->a[th][w] = t * ipp;
        m->a[th][i] = k * ipp * (tau * t - tau * c1);
        m->a[th][up] = k * ipp * (0.5 * t * t - tau * t + tau * c1);
    }
    if (p->integral) {
        m->a[z][z] = 1.0;
        m->a[z][p->position ? th : w] = t;
    }
    return ESP_OK;
}

double esp_foc_lqr_cl_decay(const esp_foc_lqr_model_t *m, const double *k)
{
    if ((m == NULL) || (k == NULL) || (m->n == 0u) || (m->n > N_MAX)) {
        return DECAY_DIVERGED;
    }
    const uint8_t n = m->n;
    mat_t cl;
    memset(cl, 0, sizeof(cl));
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            cl[r][c] = m->a[r][c] - m->b[r] * k[c];
        }
    }
    for (int s = 0; s < DECAY_SQUARINGS; s++) {
        mat_mul(n, cl, false, cl, false, cl);
        const double nm = mat_norm_inf(n, cl);
        if (!isfinite(nm) || (nm > DECAY_DIVERGED)) {
            return DECAY_DIVERGED;
        }
    }
    return mat_norm_inf(n, cl);
}

esp_err_t esp_foc_lqr_dare(const esp_foc_lqr_model_t *m, const double *q, double r, double *k)
{
    if ((m == NULL) || (q == NULL) || (k == NULL) || (m->n == 0u) || (m->n > N_MAX) ||
        !(r > 0.0) || !isfinite(r)) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t n = m->n;
    for (uint8_t i = 0; i < n; i++) {
        if (!(q[i] >= 0.0) || !isfinite(q[i]) || !isfinite(m->b[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        for (uint8_t j = 0; j < n; j++) {
            if (!isfinite(m->a[i][j])) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    /* A0 = A, G0 = B R^-1 B', H0 = Q; H_k converges to the DARE solution. */
    mat_t ak;
    mat_t gk;
    mat_t hk;
    memset(ak, 0, sizeof(ak));
    memset(gk, 0, sizeof(gk));
    memset(hk, 0, sizeof(hk));
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            ak[i][j] = m->a[i][j];
            gk[i][j] = m->b[i] * m->b[j] / r;
        }
        hk[i][i] = q[i];
    }

    bool done = false;
    /* Five matrices live at once: the design runs on a supervisor task stack. */
    for (int it = 0; (it < SDA_ITERS) && !done; it++) {
        mat_t w;
        mat_t wi;
        mat_mul(n, gk, false, hk, false, w);
        for (uint8_t i = 0; i < n; i++) {
            w[i][i] += 1.0;
        }
        if (!mat_inv(n, w, wi)) {
            return ESP_ERR_INVALID_STATE;
        }
        /* G1 = G + A W^-1 G A' */
        mat_mul(n, wi, false, gk, false, w);
        mat_mul(n, w, false, ak, true, w);
        mat_mul(n, ak, false, w, false, w);
        /* Symmetrise: round-off would otherwise grow an antisymmetric part. */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = i; j < n; j++) {
                const double g = gk[i][j] + 0.5 * (w[i][j] + w[j][i]);
                gk[i][j] = g;
                gk[j][i] = g;
            }
        }
        /* H1 = H + A' H W^-1 A, A1 = A W^-1 A */
        mat_mul(n, wi, false, ak, false, wi);
        mat_mul(n, hk, false, wi, false, w);
        mat_mul(n, ak, true, w, false, w);
        mat_mul(n, ak, false, wi, false, ak);
        double dn = 0.0;
        double hn = 0.0;
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = i; j < n; j++) {
                const double d = 0.5 * (w[i][j] + w[j][i]);
                const double h = hk[i][j] + d;
                hk[i][j] = h;
                hk[j][i] = h;
                const double s = (i == j) ? 1.0 : 2.0;
                dn += s * d * d;
                hn += s * h * h;
            }
        }
        if (!isfinite(dn) || !isfinite(hn)) {
            return ESP_ERR_INVALID_STATE;
        }
        done = (dn <= SDA_TOL * SDA_TOL * hn);
    }
    if (!done) {
        return ESP_ERR_INVALID_STATE;
    }

    double pb[N_MAX];
    double den = r;
    for (uint8_t i = 0; i < n; i++) {
        double s = 0.0;
        for (uint8_t j = 0; j < n; j++) {
            s += hk[i][j] * m->b[j];
        }
        pb[i] = s;
        den += m->b[i] * s;
    }
    if (!(den > 0.0)) {
        return ESP_ERR_INVALID_STATE;
    }
    double kk[N_MAX];
    for (uint8_t c = 0; c < n; c++) {
        double s = 0.0;
        for (uint8_t i = 0; i < n; i++) {
            s += pb[i] * m->a[i][c];
        }
        kk[c] = s / den;
        if (!isfinite(kk[c])) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (!(esp_foc_lqr_cl_decay(m, kk) < DECAY_MAX)) {
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(k, kk, sizeof(double) * n);
    return ESP_OK;
}
