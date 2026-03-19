/*
 * Unit tests for FOC math in IQ31: Clarke/Park, normalize_angle, limit_voltage,
 * apply_bias. Trivial cases, edge cases, and convergence vs float reference.
 */
#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_iq31.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TOL_FLOAT  0.02f   /* IQ31 vs float comparison */
#define TOL_STRICT 0.15f   /* roundtrip / limit_voltage magnitude */

/* ========== Clarke ========== */

TEST_CASE("foc_math_iq31: clarke trivial u=1,v=w=0", "[espFoC][foc_math_iq31]")
{
    iq31_t u = IQ31_ONE;
    iq31_t v = 0;
    iq31_t w = 0;
    iq31_t alpha, beta;
    iq31_clarke(u, v, w, &alpha, &beta);
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, alpha);
    /* beta = (1+0)*K3 = 1/sqrt(3) */
    float beta_f = iq31_to_float(beta);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 1.0f / sqrtf(3.0f), beta_f);
}

TEST_CASE("foc_math_iq31: clarke trivial balanced u+v+w=0", "[espFoC][foc_math_iq31]")
{
    iq31_t u = iq31_from_float(0.6f);
    iq31_t v = iq31_from_float(-0.3f);
    iq31_t w = iq31_from_float(-0.3f);
    iq31_t alpha, beta;
    iq31_clarke(u, v, w, &alpha, &beta);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.6f, iq31_to_float(alpha));
    /* (u+2v) = 0 -> beta = 0 */
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.0f, iq31_to_float(beta));
}

TEST_CASE("foc_math_iq31: clarke edge all zero", "[espFoC][foc_math_iq31]")
{
    iq31_t alpha, beta;
    iq31_clarke(0, 0, 0, &alpha, &beta);
    TEST_ASSERT_EQUAL_INT32(0, alpha);
    TEST_ASSERT_EQUAL_INT32(0, beta);
}

TEST_CASE("foc_math_iq31: clarke edge saturated phase", "[espFoC][foc_math_iq31]")
{
    iq31_t u = IQ31_MINUS_ONE;
    iq31_t v = IQ31_ONE;
    iq31_t w = 0;
    iq31_t alpha, beta;
    iq31_clarke(u, v, w, &alpha, &beta);
    TEST_ASSERT_EQUAL_INT32(IQ31_MINUS_ONE, alpha);
    float beta_f = iq31_to_float(beta);
    /* (-1 + 2*1)*K3 = 1/sqrt(3) */
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 1.0f / sqrtf(3.0f), beta_f);
}

/* ========== Inverse Clarke ========== */

TEST_CASE("foc_math_iq31: inverse_clarke trivial alpha=1 beta=0", "[espFoC][foc_math_iq31]")
{
    iq31_t u, v, w;
    iq31_inverse_clarke(IQ31_ONE, 0, &u, &v, &w);
    TEST_ASSERT_EQUAL_INT32(IQ31_ONE, u);
    /* v = -alpha/2 = -0.5, w = -u - v = -0.5 */
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, -0.5f, iq31_to_float(v));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, -0.5f, iq31_to_float(w));
}

TEST_CASE("foc_math_iq31: inverse_clarke sum to zero", "[espFoC][foc_math_iq31]")
{
    iq31_t alpha = iq31_from_float(0.5f);
    iq31_t beta  = iq31_from_float(0.3f);
    iq31_t u, v, w;
    iq31_inverse_clarke(alpha, beta, &u, &v, &w);
    float sum = iq31_to_float(u) + iq31_to_float(v) + iq31_to_float(w);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.0f, sum);
}

TEST_CASE("foc_math_iq31: inverse_clarke roundtrip", "[espFoC][foc_math_iq31]")
{
    iq31_t u = iq31_from_float(0.6f);
    iq31_t v = iq31_from_float(-0.3f);
    iq31_t w = iq31_from_float(-0.3f);
    iq31_t alpha, beta;
    iq31_clarke(u, v, w, &alpha, &beta);
    iq31_t u2, v2, w2;
    iq31_inverse_clarke(alpha, beta, &u2, &v2, &w2);
    TEST_ASSERT_FLOAT_WITHIN(TOL_STRICT, iq31_to_float(u), iq31_to_float(u2));
    TEST_ASSERT_FLOAT_WITHIN(TOL_STRICT, iq31_to_float(v), iq31_to_float(v2));
    TEST_ASSERT_FLOAT_WITHIN(TOL_STRICT, iq31_to_float(w), iq31_to_float(w2));
}

/* ========== Park / Inverse Park ========== */

TEST_CASE("foc_math_iq31: park trivial angle 0", "[espFoC][foc_math_iq31]")
{
    iq31_t sin_t = 0;
    iq31_t cos_t = IQ31_ONE;
    iq31_t alpha = IQ31_ONE;
    iq31_t beta  = 0;
    iq31_t d, q;
    iq31_park(sin_t, cos_t, alpha, beta, &d, &q);
    /* d = alpha*cos_t + beta*sin_t = 1*1 + 0; Q1.31*Q1.31 can round to ONE or ONE-1 */
    TEST_ASSERT_TRUE(d == IQ31_ONE || d == IQ31_ONE - 1);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

TEST_CASE("foc_math_iq31: park inverse_park roundtrip", "[espFoC][foc_math_iq31]")
{
    iq31_t alpha = iq31_from_float(0.6f);
    iq31_t beta  = iq31_from_float(0.2f);
    iq31_t sin_t = iq31_from_float(0.6f);
    iq31_t cos_t = iq31_from_float(0.8f);
    iq31_t d, q;
    iq31_park(sin_t, cos_t, alpha, beta, &d, &q);
    iq31_t alpha2, beta2;
    iq31_inverse_park(sin_t, cos_t, d, q, &alpha2, &beta2);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, iq31_to_float(alpha), iq31_to_float(alpha2));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, iq31_to_float(beta), iq31_to_float(beta2));
}

TEST_CASE("foc_math_iq31: park edge zero alpha beta", "[espFoC][foc_math_iq31]")
{
    iq31_t sin_t = iq31_from_float(0.5f);
    iq31_t cos_t = iq31_from_float(0.866f);
    iq31_t d, q;
    iq31_park(sin_t, cos_t, 0, 0, &d, &q);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

/* ========== Normalize angle ========== */

TEST_CASE("foc_math_iq31: normalize_angle trivial 0 and quarter", "[espFoC][foc_math_iq31]")
{
    iq31_t a0 = 0;
    iq31_t out0 = iq31_normalize_angle(a0);
    TEST_ASSERT_EQUAL_INT32(0, out0);

    uint32_t quarter = (uint32_t)IQ31_ONE >> 2;
    iq31_t out_q = iq31_normalize_angle((iq31_t)quarter);
    TEST_ASSERT_EQUAL_INT32((iq31_t)quarter, out_q);
}

TEST_CASE("foc_math_iq31: normalize_angle 2pi wraps to 0", "[espFoC][foc_math_iq31]")
{
    iq31_t two_pi = IQ31_ONE;
    iq31_t out = iq31_normalize_angle(two_pi);
    TEST_ASSERT_EQUAL_INT32(0, out);
}

TEST_CASE("foc_math_iq31: normalize_angle negative equivalent", "[espFoC][foc_math_iq31]")
{
    /* -pi/2 in unsigned mod period = 3*pi/2 equivalent; we treat angle as unsigned mod 2^31 */
    uint32_t three_quarter = (uint32_t)IQ31_ONE / 4 * 3;
    iq31_t a = (iq31_t)three_quarter;
    iq31_t out = iq31_normalize_angle(a);
    TEST_ASSERT_EQUAL_INT32((iq31_t)three_quarter, out);
}

TEST_CASE("foc_math_iq31: normalize_angle multiple periods", "[espFoC][foc_math_iq31]")
{
    uint32_t period = (uint32_t)IQ31_ONE + 1u;
    uint32_t two_periods = period * 2;
    iq31_t out = iq31_normalize_angle((iq31_t)two_periods);
    /* two_periods % period = period-1 or 0 depending on exact value; IQ31_ONE is 0x7FFFFFFF */
    TEST_ASSERT_TRUE((out >= 0 && (uint32_t)out < period) || out == 0);
}

/* ========== Limit voltage ========== */

TEST_CASE("foc_math_iq31: limit_voltage within limit unchanged", "[espFoC][foc_math_iq31]")
{
    iq31_t v_d = iq31_from_float(0.3f);
    iq31_t v_q = iq31_from_float(0.2f);
    iq31_t v_dc = iq31_from_float(1.0f);
    esp_foc_limit_voltage_iq31(&v_d, &v_q, v_dc);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.3f, iq31_to_float(v_d));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.2f, iq31_to_float(v_q));
}

TEST_CASE("foc_math_iq31: limit_voltage exceeds limit scaled down", "[espFoC][foc_math_iq31]")
{
    /* Vector (0.8, 0.8) has magnitude > 1; scale to unit circle */
    iq31_t v_d = iq31_from_float(0.8f);
    iq31_t v_q = iq31_from_float(0.8f);
    iq31_t v_dc = iq31_from_float(1.0f);
    esp_foc_limit_voltage_iq31(&v_d, &v_q, v_dc);
    float mag_sq = iq31_to_float(v_d) * iq31_to_float(v_d) + iq31_to_float(v_q) * iq31_to_float(v_q);
    /* Should be ~1.0 (on circle) */
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 1.0f, sqrtf(mag_sq));
}

TEST_CASE("foc_math_iq31: limit_voltage edge zero vector", "[espFoC][foc_math_iq31]")
{
    iq31_t v_d = 0;
    iq31_t v_q = 0;
    iq31_t v_dc = IQ31_ONE;
    esp_foc_limit_voltage_iq31(&v_d, &v_q, v_dc);
    TEST_ASSERT_EQUAL_INT32(0, v_d);
    TEST_ASSERT_EQUAL_INT32(0, v_q);
}

TEST_CASE("foc_math_iq31: limit_voltage edge v_q zero", "[espFoC][foc_math_iq31]")
{
    iq31_t v_d = iq31_from_float(0.6f);
    iq31_t v_q = 0;
    iq31_t v_dc = iq31_from_float(0.5f);  /* limit below 0.6 -> scale down */
    esp_foc_limit_voltage_iq31(&v_d, &v_q, v_dc);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.5f, iq31_to_float(v_d));
    TEST_ASSERT_EQUAL_INT32(0, v_q);
}

TEST_CASE("foc_math_iq31: limit_voltage convergence vs float", "[espFoC][foc_math_iq31]")
{
    float v_d_f = 0.9f;
    float v_q_f = 0.4f;
    float v_dc_f = 0.5f;
    float mag = sqrtf(v_d_f * v_d_f + v_q_f * v_q_f);
    float scale = (mag > 1e-12f && mag > v_dc_f) ? (v_dc_f / mag) : 1.0f;
    float expect_d = v_d_f * scale;
    float expect_q = v_q_f * scale;

    iq31_t v_d = iq31_from_float(v_d_f);
    iq31_t v_q = iq31_from_float(v_q_f);
    iq31_t v_dc = iq31_from_float(v_dc_f);
    esp_foc_limit_voltage_iq31(&v_d, &v_q, v_dc);
    TEST_ASSERT_FLOAT_WITHIN(TOL_STRICT, expect_d, iq31_to_float(v_d));
    TEST_ASSERT_FLOAT_WITHIN(TOL_STRICT, expect_q, iq31_to_float(v_q));
}

/* ========== Apply bias ========== */

TEST_CASE("foc_math_iq31: apply_bias trivial", "[espFoC][foc_math_iq31]")
{
    iq31_t v_a = 0;
    iq31_t v_b = IQ31_ONE;
    esp_foc_apply_bias_iq31(&v_a, &v_b);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.5f, iq31_to_float(v_a));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 1.0f, iq31_to_float(v_b));
}

TEST_CASE("foc_math_iq31: apply_bias minus one to zero", "[espFoC][foc_math_iq31]")
{
    iq31_t v_a = IQ31_MINUS_ONE;
    iq31_t v_b = IQ31_MINUS_ONE;
    esp_foc_apply_bias_iq31(&v_a, &v_b);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.0f, iq31_to_float(v_a));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, 0.0f, iq31_to_float(v_b));
}

TEST_CASE("foc_math_iq31: apply_bias convergence vs float", "[espFoC][foc_math_iq31]")
{
    float a = 0.2f;
    float b = -0.4f;
    float expect_a = a * 0.5f + 0.5f;
    float expect_b = b * 0.5f + 0.5f;
    iq31_t v_a = iq31_from_float(a);
    iq31_t v_b = iq31_from_float(b);
    esp_foc_apply_bias_iq31(&v_a, &v_b);
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, expect_a, iq31_to_float(v_a));
    TEST_ASSERT_FLOAT_WITHIN(TOL_FLOAT, expect_b, iq31_to_float(v_b));
}
