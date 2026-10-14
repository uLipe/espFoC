/*
 * Unit tests for dq circle voltage limiter.
 */
#include <math.h>

#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"

#define VLIM_ERR ((q16_t)600)

static q16_t mag_dq(q16_t d, q16_t q)
{
    return esp_foc_sqrt(q16_add(q16_mul(d, d), q16_mul(q, q)));
}

TEST_CASE("vlim under limit unchanged", "[espFoC][vlim]")
{
    q16_t d = q16_from_float(0.1f);
    q16_t q = q16_from_float(-0.05f);
    esp_foc_vlim_dq(&d, &q, Q16_INV_SQRT3);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(0.1f), d);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(-0.05f), q);
}

TEST_CASE("vlim over limit mag equals vmax same angle", "[espFoC][vlim]")
{
    q16_t d = q16_from_float(0.8f);
    q16_t q = q16_from_float(0.6f);
    q16_t vmax = Q16_INV_SQRT3;
    esp_foc_vlim_dq(&d, &q, vmax);
    TEST_ASSERT_INT32_WITHIN(VLIM_ERR, vmax, mag_dq(d, q));
    TEST_ASSERT_TRUE(d > 0 && q > 0);
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.08f), q16_div(q, d),
                             q16_div(q16_from_float(0.6f), q16_from_float(0.8f)));
}

TEST_CASE("vlim zero vector", "[espFoC][vlim]")
{
    q16_t d = 0;
    q16_t q = 0;
    esp_foc_vlim_dq(&d, &q, Q16_INV_SQRT3);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

TEST_CASE("vlim vmax zero clears", "[espFoC][vlim]")
{
    q16_t d = q16_from_float(0.4f);
    q16_t q = q16_from_float(-0.3f);
    esp_foc_vlim_dq(&d, &q, 0);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

TEST_CASE("vlim negative vmax clears", "[espFoC][vlim]")
{
    q16_t d = Q16_ONE;
    q16_t q = Q16_ONE;
    esp_foc_vlim_dq(&d, &q, Q16_MINUS_ONE);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

TEST_CASE("vlim repeated clamp does not grow", "[espFoC][vlim]")
{
    q16_t d = q16_from_float(1.2f);
    q16_t q = q16_from_float(-0.9f);
    q16_t vmax = Q16_INV_SQRT3;
    for (int i = 0; i < 50; i++) {
        esp_foc_vlim_dq(&d, &q, vmax);
    }
    TEST_ASSERT_INT32_WITHIN(VLIM_ERR, vmax, mag_dq(d, q));
    TEST_ASSERT_TRUE(mag_dq(d, q) <= q16_add(vmax, VLIM_ERR));
}

TEST_CASE("vlim_mag with the soft magnitude matches vlim_dq", "[espFoC][vlim]")
{
    const q16_t vmax = Q16_INV_SQRT3;
    for (int k = 0; k < 64; k++) {
        const q16_t th = q16_from_float(-3.1f + 0.0968f * (float)k);
        const q16_t r = q16_from_float(0.05f + 0.03f * (float)k);
        q16_t s;
        q16_t c;
        esp_foc_sincos(th, &s, &c);
        q16_t d0 = q16_mul(r, c);
        q16_t q0 = q16_mul(r, s);
        q16_t d1 = d0;
        q16_t q1 = q0;
        esp_foc_vlim_dq(&d0, &q0, vmax);
        esp_foc_vlim_dq_mag(&d1, &q1, mag_dq(d1, q1), vmax);
        TEST_ASSERT_EQUAL_INT32(d0, d1);
        TEST_ASSERT_EQUAL_INT32(q0, q1);
    }
}

TEST_CASE("vlim_mag under limit, zero and vmax edges", "[espFoC][vlim]")
{
    q16_t d = q16_from_float(0.2f);
    q16_t q = q16_from_float(-0.1f);
    esp_foc_vlim_dq_mag(&d, &q, mag_dq(d, q), Q16_INV_SQRT3);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(0.2f), d);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(-0.1f), q);

    d = 0;
    q = 0;
    esp_foc_vlim_dq_mag(&d, &q, 0, Q16_INV_SQRT3);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);

    d = Q16_ONE;
    q = Q16_ONE;
    esp_foc_vlim_dq_mag(&d, &q, mag_dq(d, q), 0);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);

    d = Q16_ONE;
    q = Q16_ONE;
    esp_foc_vlim_dq_mag(&d, &q, mag_dq(d, q), Q16_MINUS_ONE);
    TEST_ASSERT_EQUAL_INT32(0, d);
    TEST_ASSERT_EQUAL_INT32(0, q);
}

TEST_CASE("vlim_mag scale matches the q16 divide on both paths", "[espFoC][vlim]")
{
    static const q16_t vmaxes[] = {
        1, 4096, Q16_INV_SQRT3, Q16_ONE - 1, Q16_ONE, 2 * Q16_ONE, 300 * Q16_ONE,
    };
    uint32_t x = 0xcafef00du;
    for (unsigned k = 0; k < sizeof(vmaxes) / sizeof(vmaxes[0]); k++) {
        const q16_t vmax = vmaxes[k];
        for (int i = 0; i < 2000; i++) {
            x = x * 1664525u + 1013904223u;
            const q16_t d0 = (q16_t)x >> 8;
            x = x * 1664525u + 1013904223u;
            const q16_t q0 = (q16_t)x >> 8;
            x = x * 1664525u + 1013904223u;
            const q16_t mag = q16_add(vmax, (q16_t)((x >> 4) | 1u));
            q16_t d = d0;
            q16_t q = q0;
            esp_foc_vlim_dq_mag(&d, &q, mag, vmax);
            const q16_t scale = q16_div(vmax, mag);
            TEST_ASSERT_EQUAL_INT32(q16_mul(d0, scale), d);
            TEST_ASSERT_EQUAL_INT32(q16_mul(q0, scale), q);
        }
    }
}

TEST_CASE("vlim_mag exact magnitude lands on vmax and does not grow", "[espFoC][vlim]")
{
    const q16_t vmax = Q16_INV_SQRT3;
    q16_t d = q16_from_float(1.5f);
    q16_t q = q16_from_float(-2.0f);
    for (int i = 0; i < 50; i++) {
        const q16_t mag = q16_from_float(sqrtf(q16_to_float(d) * q16_to_float(d) +
                                               q16_to_float(q) * q16_to_float(q)));
        esp_foc_vlim_dq_mag(&d, &q, mag, vmax);
    }
    const float m = sqrtf(q16_to_float(d) * q16_to_float(d) + q16_to_float(q) * q16_to_float(q));
    TEST_ASSERT_FLOAT_WITHIN(4.0f / 65536.0f, q16_to_float(vmax), m);
    TEST_ASSERT_TRUE(d > 0 && q < 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.0f / 1.5f, q16_to_float(q) / q16_to_float(d));
}
