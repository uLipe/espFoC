/*
 * Unit tests for FOC math (Clarke, Park, normalize, clamp, limit, rsqrt).
 * No hardware; pure math only.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/foc_math.h"

#define FLOAT_TOL 1e-5f

TEST_CASE("esp_foc_clamp: in range", "[espFoC][foc_math]")
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, esp_foc_clamp(0.5f, 0.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -1.0f, esp_foc_clamp(-1.0f, -2.0f, 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, esp_foc_clamp(0.0f, -1.0f, 1.0f));
}

TEST_CASE("esp_foc_clamp: below min", "[espFoC][foc_math]")
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, esp_foc_clamp(-0.5f, 0.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -2.0f, esp_foc_clamp(-5.0f, -2.0f, 2.0f));
}

TEST_CASE("esp_foc_clamp: above max", "[espFoC][foc_math]")
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, esp_foc_clamp(1.5f, 0.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 2.0f, esp_foc_clamp(10.0f, -2.0f, 2.0f));
}

TEST_CASE("esp_foc_normalize_angle: [0, 2pi)", "[espFoC][foc_math]")
{
    float out = esp_foc_normalize_angle(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out);
    out = esp_foc_normalize_angle((float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, (float)M_PI, out);
    out = esp_foc_normalize_angle(2.0f * (float)M_PI - 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f * (float)M_PI - 0.001f, out);
}

TEST_CASE("esp_foc_normalize_angle: negative wraps to [0, 2pi)", "[espFoC][foc_math]")
{
    float out = esp_foc_normalize_angle(-0.5f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 2.0f * (float)M_PI - 0.5f, out);
    out = esp_foc_normalize_angle(-2.0f * (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out);
}

TEST_CASE("esp_foc_mechanical_to_elec_angle", "[espFoC][foc_math]")
{
    float elec = esp_foc_mechanical_to_elec_angle(0.0f, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, elec);
    elec = esp_foc_mechanical_to_elec_angle((float)M_PI / 2.0f, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 2.0f * (float)M_PI, elec);
}

TEST_CASE("esp_foc_clarke_transform: two-phase equivalent", "[espFoC][foc_math]")
{
    float uvw[3] = { 1.0f, 0.0f, 0.0f };
    float alpha, beta;
    esp_foc_clarke_transform(uvw, &alpha, &beta);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, alpha);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f / ESP_FOC_CLARKE_PARK_SQRT3, beta);
}

TEST_CASE("esp_foc_park_transform: dq from alpha beta", "[espFoC][foc_math]")
{
    float sin_t = 0.0f, cos_t = 1.0f;
    float v_ab[2] = { 1.0f, 0.0f };
    float v_d, v_q;
    esp_foc_park_transform(sin_t, cos_t, v_ab, &v_d, &v_q);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, v_d);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, v_q);
}

TEST_CASE("esp_foc_inverse_park: roundtrip", "[espFoC][foc_math]")
{
    float angle = 0.3f;
    float s = sinf(angle), c = cosf(angle);
    float v_dq[2] = { 1.0f, 0.5f };
    float v_alpha, v_beta;
    esp_foc_inverse_park_transform(s, c, v_dq, &v_alpha, &v_beta);
    float v_ab2[2] = { v_alpha, v_beta };
    float v_d2, v_q2;
    esp_foc_park_transform(s, c, v_ab2, &v_d2, &v_q2);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, v_dq[0], v_d2);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, v_dq[1], v_q2);
}

TEST_CASE("esp_foc_inverse_clarke: sum to zero", "[espFoC][foc_math]")
{
    float v_ab[2] = { 0.5f, 0.3f };
    float v_u, v_v, v_w;
    esp_foc_inverse_clarke_transform(v_ab, &v_u, &v_v, &v_w);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, v_u + v_v + v_w);
}

TEST_CASE("esp_foc_limit_voltage: within limit unchanged", "[espFoC][foc_math]")
{
    float v_d = 1.0f, v_q = 0.5f;
    float v_dc = 24.0f;
    esp_foc_limit_voltage(&v_d, &v_q, v_dc);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, v_d);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.5f, v_q);
}

TEST_CASE("esp_foc_limit_voltage: exceeds limit scaled", "[espFoC][foc_math]")
{
    float v_d = 20.0f, v_q = 20.0f;
    float v_dc = 24.0f;
    esp_foc_limit_voltage(&v_d, &v_q, v_dc);
    float mag_sq = v_d * v_d + v_q * v_q;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, v_dc * v_dc, mag_sq);
}

TEST_CASE("esp_foc_rsqrt_fast: approximate 1/sqrt", "[espFoC][foc_math]")
{
    float x = 4.0f;
    float r = esp_foc_rsqrt_fast(x);
    float expected = 1.0f / sqrtf(x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, r);
}

TEST_CASE("esp_foc_sine_cosine: standard lib", "[espFoC][foc_math]")
{
    float a = 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, sinf(a), esp_foc_sine(a));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, cosf(a), esp_foc_cosine(a));
}
