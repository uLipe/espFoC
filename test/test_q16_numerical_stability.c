/*
 * Numerical stability tests for the Q16.16 fixed-point pipeline.
 * Covers overflow/underflow boundaries, sustained integration drift,
 * Q16 ↔ IQ31 bridge round-trips, and multi-step pipeline determinism.
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"
#include "espFoC/utils/pid_controller.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/space_vector_modulator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ----- Q16 arithmetic boundary tests ----- */

TEST_CASE("q16_mul: near-max saturation", "[espFoC][q16][stability]")
{
    q16_t a = (q16_t)INT32_MAX;
    q16_t b = Q16_ONE;
    q16_t r = q16_mul(a, b);
    TEST_ASSERT_TRUE(r > 0);
    TEST_ASSERT_TRUE(r <= (q16_t)INT32_MAX);
}

TEST_CASE("q16_mul: negative saturation", "[espFoC][q16][stability]")
{
    q16_t a = (q16_t)INT32_MIN;
    q16_t b = Q16_ONE;
    q16_t r = q16_mul(a, b);
    TEST_ASSERT_TRUE(r < 0);
    TEST_ASSERT_TRUE(r >= (q16_t)INT32_MIN);
}

TEST_CASE("q16_mul: both extremes overflow to positive", "[espFoC][q16][stability]")
{
    q16_t r = q16_mul((q16_t)INT32_MIN, (q16_t)INT32_MIN);
    TEST_ASSERT_EQUAL_INT32((q16_t)INT32_MAX, r);
}

TEST_CASE("q16_add: overflow saturates", "[espFoC][q16][stability]")
{
    q16_t r = q16_add((q16_t)INT32_MAX, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32((q16_t)INT32_MAX, r);
}

TEST_CASE("q16_sub: underflow saturates", "[espFoC][q16][stability]")
{
    q16_t r = q16_sub((q16_t)INT32_MIN, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32((q16_t)INT32_MIN, r);
}

TEST_CASE("q16_from_float: extreme values saturate", "[espFoC][q16][stability]")
{
    TEST_ASSERT_EQUAL_INT32((q16_t)INT32_MAX, q16_from_float(1e10f));
    TEST_ASSERT_EQUAL_INT32((q16_t)INT32_MIN, q16_from_float(-1e10f));
}

TEST_CASE("q16_from_float: NaN becomes zero", "[espFoC][q16][stability]")
{
    volatile float z = 0.0f;
    float nan = z / z;
    TEST_ASSERT_EQUAL_INT32(0, q16_from_float(nan));
}

/* ----- Q16 sin/cos boundary and identity tests ----- */

TEST_CASE("q16_sin_cos: sin^2 + cos^2 = 1 across full range", "[espFoC][q16][stability]")
{
    const int steps = 256;
    for (int i = 0; i < steps; i++) {
        q16_t angle = q16_from_float(2.0f * (float)M_PI * (float)i / (float)steps);
        q16_t s = q16_sin(angle);
        q16_t c = q16_cos(angle);
        float sf = q16_to_float(s);
        float cf = q16_to_float(c);
        float mag_sq = sf * sf + cf * cf;
        TEST_ASSERT_FLOAT_WITHIN(0.03f, 1.0f, mag_sq);
    }
}

TEST_CASE("q16_sin_cos: zero angle", "[espFoC][q16][stability]")
{
    q16_t s = q16_sin(0);
    q16_t c = q16_cos(0);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, q16_to_float(s));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, q16_to_float(c));
}

TEST_CASE("q16_sin_cos: negative angle wraps correctly", "[espFoC][q16][stability]")
{
    q16_t angle = q16_from_float(-1.0f);
    q16_t s = q16_sin(angle);
    float sf = q16_to_float(s);
    float ref = sinf(-1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, ref, sf);
}

/* ----- Clarke/Park roundtrip numerical stability ----- */

TEST_CASE("q16 Clarke/Park roundtrip: small signals", "[espFoC][q16][stability]")
{
    q16_t u = q16_from_float(0.001f);
    q16_t v = q16_from_float(-0.0005f);
    q16_t w = q16_from_float(-0.0005f);
    q16_t alpha, beta;
    q16_clarke(u, v, w, &alpha, &beta);

    q16_t u2, v2, w2;
    q16_inverse_clarke(alpha, beta, &u2, &v2, &w2);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, q16_to_float(u), q16_to_float(u2));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, q16_to_float(v), q16_to_float(v2));
}

TEST_CASE("q16 Clarke/Park roundtrip: large signals", "[espFoC][q16][stability]")
{
    q16_t u = q16_from_float(10.0f);
    q16_t v = q16_from_float(-5.0f);
    q16_t w = q16_from_float(-5.0f);
    q16_t alpha, beta;
    q16_clarke(u, v, w, &alpha, &beta);

    q16_t angle = q16_from_float(1.2f);
    q16_t st = q16_sin(angle);
    q16_t ct = q16_cos(angle);

    q16_t d, q;
    q16_park(st, ct, alpha, beta, &d, &q);

    q16_t a2, b2;
    q16_inverse_park(st, ct, d, q, &a2, &b2);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, q16_to_float(alpha), q16_to_float(a2));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, q16_to_float(beta), q16_to_float(b2));
}

/* ----- limit_voltage edge cases ----- */

TEST_CASE("q16 limit_voltage: zero magnitude is no-op", "[espFoC][q16][stability]")
{
    q16_t vd = 0, vq = 0;
    esp_foc_limit_voltage_q16(&vd, &vq, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32(0, vd);
    TEST_ASSERT_EQUAL_INT32(0, vq);
}

TEST_CASE("q16 limit_voltage: zero dc clamps vector to zero", "[espFoC][q16][stability]")
{
    q16_t vd = Q16_ONE, vq = Q16_ONE;
    esp_foc_limit_voltage_q16(&vd, &vq, 0);
    TEST_ASSERT_EQUAL_INT32(0, vd);
    TEST_ASSERT_EQUAL_INT32(0, vq);
}

TEST_CASE("q16 limit_voltage: within limit unchanged", "[espFoC][q16][stability]")
{
    q16_t vd = q16_from_float(0.1f);
    q16_t vq = q16_from_float(0.1f);
    q16_t vd_before = vd, vq_before = vq;
    esp_foc_limit_voltage_q16(&vd, &vq, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32(vd_before, vd);
    TEST_ASSERT_EQUAL_INT32(vq_before, vq);
}

TEST_CASE("q16 limit_voltage: exceeds limit is scaled", "[espFoC][q16][stability]")
{
    q16_t vd = q16_from_float(8.0f);
    q16_t vq = q16_from_float(8.0f);
    q16_t vdc = q16_from_float(5.0f);
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    float mag = sqrtf(q16_to_float(vd) * q16_to_float(vd) + q16_to_float(vq) * q16_to_float(vq));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, q16_to_float(vdc), mag);
}

/* ----- PID numerical stability ----- */

TEST_CASE("PID q16: sustained zero-error stays zero", "[espFoC][q16][stability]")
{
    esp_foc_pid_controller_t pid = {0};
    esp_foc_pid_init_from_float(&pid, 1.0f, 10.0f, 0.0f, 0.001f, -10.0f, 10.0f, 5.0f);
    for (int i = 0; i < 10000; i++) {
        q16_t out = esp_foc_pid_update(&pid, q16_from_float(1.0f), q16_from_float(1.0f));
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, q16_to_float(out));
    }
}

TEST_CASE("PID q16: integrator windup clamped", "[espFoC][q16][stability]")
{
    esp_foc_pid_controller_t pid = {0};
    const float int_lim = 2.0f;
    esp_foc_pid_init_from_float(&pid, 0.0f, 100.0f, 0.0f, 0.01f, -100.0f, 100.0f, int_lim);
    for (int i = 0; i < 5000; i++) {
        esp_foc_pid_update(&pid, Q16_ONE, 0);
    }
    float int_val = (float)pid.integrator / 65536.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, int_lim, int_val);
}

TEST_CASE("PID q16: output saturation prevents runaway", "[espFoC][q16][stability]")
{
    esp_foc_pid_controller_t pid = {0};
    esp_foc_pid_init_from_float(&pid, 100.0f, 0.0f, 0.0f, 0.001f, -0.5f, 0.5f, 10.0f);
    q16_t out = esp_foc_pid_update(&pid, q16_from_float(100.0f), 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, q16_to_float(out));
}

TEST_CASE("PID q16: alternating sign error stays bounded", "[espFoC][q16][stability]")
{
    esp_foc_pid_controller_t pid = {0};
    esp_foc_pid_init_from_float(&pid, 1.0f, 5.0f, 0.01f, 0.001f, -10.0f, 10.0f, 5.0f);
    for (int i = 0; i < 2000; i++) {
        float ref = (i & 1) ? 0.5f : -0.5f;
        q16_t out = esp_foc_pid_update(&pid, q16_from_float(ref), 0);
        float out_f = q16_to_float(out);
        TEST_ASSERT_TRUE(out_f >= -10.0f && out_f <= 10.0f);
    }
}

/* (EMA-specific stability cases removed when the helper was retired
 * — equivalent coverage now lives in test_biquad_q16.c.) */

/* ----- SVPWM duty bounding ----- */

TEST_CASE("SVPWM q16: extreme inputs produce bounded duties", "[espFoC][q16][stability]")
{
    q16_t cases[][2] = {
        {q16_from_float(100.0f), q16_from_float(100.0f)},
        {q16_from_float(-100.0f), q16_from_float(-100.0f)},
        {q16_from_float(100.0f), q16_from_float(-100.0f)},
        {0, 0},
    };
    for (int i = 0; i < 4; i++) {
        q16_t da, db, dc;
        esp_foc_svm_set(cases[i][0], cases[i][1], q16_from_float(0.1f), &da, &db, &dc);
        TEST_ASSERT_TRUE(q16_to_float(da) >= 0.0f && q16_to_float(da) <= 1.0f);
        TEST_ASSERT_TRUE(q16_to_float(db) >= 0.0f && q16_to_float(db) <= 1.0f);
        TEST_ASSERT_TRUE(q16_to_float(dc) >= 0.0f && q16_to_float(dc) <= 1.0f);
    }
}

/* ----- Full pipeline long-run stability ----- */

TEST_CASE("full pipeline q16: 10000 steps bounded and deterministic", "[espFoC][q16][stability]")
{
    esp_foc_pid_controller_t pid_d = {0}, pid_q = {0};
    esp_foc_biquad_q16_t filt_d, filt_q;

    const float dt = 1.0f / 20000.0f;
    esp_foc_pid_init_from_float(&pid_d, 0.4f, 60.0f, 0.0f, dt, -0.7f, 0.7f, 0.7f / 60.0f);
    esp_foc_pid_init_from_float(&pid_q, 0.4f, 60.0f, 0.0f, dt, -0.7f, 0.7f, 0.7f / 60.0f);
    esp_foc_biquad_butterworth_lpf_design_q16(&filt_d, 60.0f, 20000.0f);
    esp_foc_biquad_butterworth_lpf_design_q16(&filt_q, 60.0f, 20000.0f);

    const int N = 10000;
    const q16_t vmax = q16_from_float(0.7f);

    for (int k = 0; k < N; k++) {
        float t = (float)k / (float)N;
        float angle = 2.0f * (float)M_PI * t * 10.0f;
        q16_t sq = q16_sin(q16_from_float(angle));
        q16_t cq = q16_cos(q16_from_float(angle));

        q16_t iu = q16_from_float(0.3f * sinf(angle));
        q16_t iv = q16_from_float(0.3f * sinf(angle - 2.0f * (float)M_PI / 3.0f));
        q16_t iw = q16_sub(q16_sub(0, iu), iv);

        q16_t ia, ib, id, iq;
        esp_foc_get_dq_currents(sq, cq, iu, iv, iw, &ia, &ib, &iq, &id);

        q16_t uq = esp_foc_pid_update(&pid_q, q16_from_float(0.2f),
                                       esp_foc_biquad_q16_update(&filt_q, iq));
        q16_t ud = esp_foc_pid_update(&pid_d, 0,
                                       esp_foc_biquad_q16_update(&filt_d, id));

        q16_t ua, ub, vu, vv, vw;
        esp_foc_modulate_dq_voltage(sq, cq, ud, uq, &ua, &ub, &vu, &vv, &vw, vmax);

        TEST_ASSERT_TRUE(fabsf(q16_to_float(vu)) < 2.0f);
        TEST_ASSERT_TRUE(fabsf(q16_to_float(vv)) < 2.0f);
        TEST_ASSERT_TRUE(fabsf(q16_to_float(vw)) < 2.0f);
    }
}

/* ----- Q16 ↔ IQ31 bridge round-trip ----- */

TEST_CASE("q16_from_iq31_per_unit: roundtrip consistency", "[espFoC][q16][stability]")
{
    float test_vals[] = {0.0f, 0.5f, -0.5f, 0.999f, -0.999f, 0.001f, -0.001f};
    for (int i = 0; i < (int)(sizeof(test_vals) / sizeof(test_vals[0])); i++) {
        iq31_t iq = iq31_from_float(test_vals[i]);
        q16_t q = q16_from_iq31_per_unit(iq);
        float back = q16_to_float(q);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, test_vals[i], back);
    }
}

/* ----- angle normalization edge cases ----- */

TEST_CASE("q16_normalize_angle_rad: multiple wraps", "[espFoC][q16][stability]")
{
    q16_t three_pi = q16_from_float(3.0f * (float)M_PI);
    q16_t norm = q16_normalize_angle_rad(three_pi);
    float f = q16_to_float(norm);
    float expected = fmodf(3.0f * (float)M_PI, 2.0f * (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected, f);
}

TEST_CASE("q16_normalize_angle_rad: large negative", "[espFoC][q16][stability]")
{
    q16_t neg = q16_from_float(-5.0f * (float)M_PI);
    q16_t norm = q16_normalize_angle_rad(neg);
    float f = q16_to_float(norm);
    TEST_ASSERT_TRUE(f >= 0.0f);
    TEST_ASSERT_TRUE(f < q16_to_float(Q16_TWO_PI) + 0.1f);
}
