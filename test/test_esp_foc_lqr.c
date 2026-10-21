/*
 * Unit tests for esp_foc_lqr: servo model by ZOH and the DARE by doubling.
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/utils/esp_foc_lqr.h"

/* Bench motor: 13 pp, K from the identification, 2.5 kHz slot, 1 kHz loop. */
static esp_foc_lqr_servo_t bench_servo(bool position, bool integral)
{
    const esp_foc_lqr_servo_t p = {
        .k_rads2_a = 67347.0,
        .pole_pairs = 13u,
        .ts_s = 1.0 / 2500.0,
        .tau_i_s = 1.0 / (2.0 * M_PI * 1000.0),
        .position = position,
        .integral = integral,
    };
    return p;
}

/* Plain Riccati value iteration, the independent reference. */
static void riccati_iterate(const esp_foc_lqr_model_t *m, const double *q, double r, double *k,
                            int iters)
{
    const uint8_t n = m->n;
    double p[ESP_FOC_LQR_N_MAX][ESP_FOC_LQR_N_MAX];
    memset(p, 0, sizeof(p));
    for (uint8_t i = 0; i < n; i++) {
        p[i][i] = q[i];
    }
    for (int it = 0; it < iters; it++) {
        double pa[ESP_FOC_LQR_N_MAX][ESP_FOC_LQR_N_MAX];
        double pb[ESP_FOC_LQR_N_MAX];
        double den = r;
        for (uint8_t i = 0; i < n; i++) {
            pb[i] = 0.0;
            for (uint8_t j = 0; j < n; j++) {
                double s = 0.0;
                for (uint8_t l = 0; l < n; l++) {
                    s += p[i][l] * m->a[l][j];
                }
                pa[i][j] = s;
                pb[i] += p[i][j] * m->b[j];
            }
            den += m->b[i] * pb[i];
        }
        for (uint8_t c = 0; c < n; c++) {
            double s = 0.0;
            for (uint8_t i = 0; i < n; i++) {
                s += pb[i] * m->a[i][c];
            }
            k[c] = s / den;
        }
        double np[ESP_FOC_LQR_N_MAX][ESP_FOC_LQR_N_MAX];
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                double s = 0.0;
                for (uint8_t l = 0; l < n; l++) {
                    s += m->a[l][i] * pa[l][j];
                }
                /* A'PA - A'PB k */
                double bpa = 0.0;
                for (uint8_t l = 0; l < n; l++) {
                    bpa += m->a[l][i] * pb[l];
                }
                np[i][j] = s - bpa * k[j] + ((i == j) ? q[i] : 0.0);
            }
        }
        memcpy(p, np, sizeof(p));
    }
}

TEST_CASE("lqr dare matches the scalar closed form", "[espFoC][lqr]")
{
    const double a_s[] = { 0.5, 1.0, 1.2 };
    for (int t = 0; t < 3; t++) {
        esp_foc_lqr_model_t m;
        memset(&m, 0, sizeof(m));
        m.n = 1u;
        m.a[0][0] = a_s[t];
        m.b[0] = 0.3;
        const double q = 2.0;
        const double r = 0.7;
        double k = 0.0;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_dare(&m, &q, r, &k));
        const double a = a_s[t];
        const double b = 0.3;
        const double c1 = r * (1.0 - a * a) - q * b * b;
        const double p = (-c1 + sqrt(c1 * c1 + 4.0 * b * b * q * r)) / (2.0 * b * b);
        const double kref = a * b * p / (r + b * b * p);
        TEST_ASSERT_DOUBLE_WITHIN(1.0e-9 * fabs(kref), kref, k);
        TEST_ASSERT_TRUE(fabs(a - b * k) < 1.0);
    }
}

TEST_CASE("lqr dare agrees with riccati iteration on the servo models", "[espFoC][lqr]")
{
    for (int c = 0; c < 4; c++) {
        const bool pos = (c & 1) != 0;
        const bool integ = (c & 2) != 0;
        const esp_foc_lqr_servo_t p = bench_servo(pos, integ);
        esp_foc_lqr_model_t m;
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_servo_model(&p, &m));
        TEST_ASSERT_EQUAL_UINT8((pos ? 4u : 3u) + (integ ? 1u : 0u), m.n);
        double q[ESP_FOC_LQR_N_MAX] = {0};
        uint8_t s = 0;
        if (pos) {
            q[s++] = 1.0 / (0.02 * 0.02);
        }
        q[s++] = 1.0 / (60.0 * 60.0);
        s += 2u;
        if (integ) {
            q[s] = 1.0 / (0.02 * 0.02 * 0.05 * 0.05);
        }
        const double r = 1.0 / (0.7 * 0.7);
        double k[ESP_FOC_LQR_N_MAX];
        double kr[ESP_FOC_LQR_N_MAX];
        TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_dare(&m, q, r, k));
        /* Agrees to 1e-6 from ~2000 iterations on; soft double on the chip. */
        riccati_iterate(&m, q, r, kr, 5000);
        for (uint8_t i = 0; i < m.n; i++) {
            TEST_ASSERT_DOUBLE_WITHIN(1.0e-6 * fabs(kr[i]) + 1.0e-12, kr[i], k[i]);
        }
        TEST_ASSERT_LESS_THAN_DOUBLE(1.0e-6, esp_foc_lqr_cl_decay(&m, k));
    }
}

/* One sample of the continuous plant by fine Euler steps. */
static void plant_fine(const esp_foc_lqr_servo_t *p, double *th, double *w, double *i, double u)
{
    const int sub = 20000;
    const double h = p->ts_s / sub;
    for (int s = 0; s < sub; s++) {
        const double di = (u - *i) / p->tau_i_s;
        *th += h * (*w) / (double)p->pole_pairs;
        *w += h * p->k_rads2_a * (*i);
        *i += h * di;
    }
}

TEST_CASE("lqr servo model is the ZOH of the continuous plant", "[espFoC][lqr]")
{
    const esp_foc_lqr_servo_t p = bench_servo(true, false);
    esp_foc_lqr_model_t m;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_servo_model(&p, &m));
    double x[4] = { 0.1, 30.0, 0.2, -0.3 };
    double th = x[0], w = x[1], i = x[2];
    const double up = x[3];
    plant_fine(&p, &th, &w, &i, up);
    double xn[4] = {0};
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            xn[r] += m.a[r][c] * x[c];
        }
    }
    TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, th, xn[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0e-3, w, xn[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0e-5, i, xn[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0e-12, 0.0, xn[3]);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, m.b[3]);

    const esp_foc_lqr_servo_t ps = bench_servo(false, true);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_servo_model(&ps, &m));
    TEST_ASSERT_EQUAL_DOUBLE(ps.ts_s, m.a[3][0]);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, m.a[3][3]);
}

TEST_CASE("lqr refuses bad arguments and an unstabilisable plant", "[espFoC][lqr]")
{
    esp_foc_lqr_servo_t p = bench_servo(true, true);
    esp_foc_lqr_model_t m;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_servo_model(NULL, &m));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_servo_model(&p, NULL));
    p.tau_i_s = 0.0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_servo_model(&p, &m));
    p = bench_servo(true, false);
    p.pole_pairs = 0u;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_servo_model(&p, &m));
    p = bench_servo(true, false);
    p.k_rads2_a = -1.0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_servo_model(&p, &m));

    p = bench_servo(false, false);
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_servo_model(&p, &m));
    double q[ESP_FOC_LQR_N_MAX] = { 1.0, 0.0, 0.0 };
    double k[ESP_FOC_LQR_N_MAX];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_dare(&m, q, 0.0, k));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_dare(&m, q, -1.0, k));
    q[1] = -1.0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_dare(&m, q, 1.0, k));
    q[1] = NAN;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_dare(&m, q, 1.0, k));
    q[1] = 0.0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_lqr_dare(NULL, q, 1.0, k));

    /* An unstable mode the input cannot reach. */
    esp_foc_lqr_model_t u;
    memset(&u, 0, sizeof(u));
    u.n = 2u;
    u.a[0][0] = 1.05;
    u.a[1][1] = 0.5;
    u.b[1] = 1.0;
    const double qu[2] = { 1.0, 1.0 };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, esp_foc_lqr_dare(&u, qu, 1.0, k));
}

TEST_CASE("lqr servo closed loop settles to zero without ringing", "[espFoC][lqr]")
{
    const esp_foc_lqr_servo_t p = bench_servo(true, true);
    esp_foc_lqr_model_t m;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_servo_model(&p, &m));
    const double q[5] = { 1.0 / (0.02 * 0.02), 1.0 / (60.0 * 60.0), 0.0, 0.0,
                          1.0 / (0.02 * 0.02 * 0.05 * 0.05) };
    double k[5];
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_lqr_dare(&m, q, 1.0 / 0.49, k));

    /* 0.2 rad off, released: the error decays, crosses zero at most once and
     * the tail is flat. */
    double x[5] = { 0.2, 0.0, 0.0, 0.0, 0.0 };
    int sign_changes = 0;
    double prev = x[0];
    double tail = 0.0;
    const int steps = 5000;
    for (int s = 0; s < steps; s++) {
        double u = 0.0;
        for (int i = 0; i < 5; i++) {
            u -= k[i] * x[i];
        }
        double xn[5] = {0};
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 5; c++) {
                xn[r] += m.a[r][c] * x[c];
            }
            xn[r] += m.b[r] * u;
        }
        memcpy(x, xn, sizeof(x));
        if ((x[0] > 0.0) != (prev > 0.0) && (fabs(x[0]) > 1.0e-6)) {
            sign_changes++;
        }
        prev = x[0];
        if (s >= steps - 500) {
            tail = fmax(tail, fabs(x[0]));
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_INT(1, sign_changes);
    TEST_ASSERT_LESS_THAN_DOUBLE(1.0e-5, tail);
}
