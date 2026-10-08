/*
 * Unit tests for dq circle voltage limiter.
 */
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
