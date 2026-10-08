/*
 * Unit tests for Park / inverse Park.
 */
#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"

#define PARK_ERR ((q16_t)400)

TEST_CASE("park theta 0 is identity", "[espFoC][park]")
{
    q16_t d, q;
    esp_foc_park(0, Q16_ONE, q16_from_float(0.3f), q16_from_float(-0.2f), &d, &q);
    TEST_ASSERT_INT32_WITHIN(8, q16_from_float(0.3f), d);
    TEST_ASSERT_INT32_WITHIN(8, q16_from_float(-0.2f), q);
}

TEST_CASE("park 90 deg swaps with sign", "[espFoC][park]")
{
    q16_t d, q;
    q16_t a = q16_from_float(0.4f);
    q16_t b = q16_from_float(0.1f);
    esp_foc_park(Q16_ONE, 0, a, b, &d, &q);
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, b, d);
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, q16_neg(a), q);
}

TEST_CASE("park round-trip ab", "[espFoC][park]")
{
    q16_t s, c;
    esp_foc_sincos(q16_from_float(1.1f), &s, &c);
    q16_t a0 = q16_from_float(0.25f);
    q16_t b0 = q16_from_float(-0.18f);
    q16_t d, q, a1, b1;
    esp_foc_park(s, c, a0, b0, &d, &q);
    esp_foc_inv_park(s, c, d, q, &a1, &b1);
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, a0, a1);
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, b0, b1);
}

TEST_CASE("park rotating ab of const mag is const dq", "[espFoC][park]")
{
    q16_t d_ref = 0;
    q16_t q_ref = 0;
    for (int k = 0; k < 16; k++) {
        q16_t th = (q16_t)(((int64_t)k * (int64_t)Q16_TWO_PI) / 16);
        q16_t s, c;
        esp_foc_sincos(th, &s, &c);
        q16_t d, q;
        esp_foc_park(s, c, c, s, &d, &q);
        if (k == 0) {
            d_ref = d;
            q_ref = q;
            TEST_ASSERT_INT32_WITHIN(PARK_ERR, Q16_ONE, d);
            TEST_ASSERT_INT32_WITHIN(PARK_ERR, 0, q);
        } else {
            TEST_ASSERT_INT32_WITHIN(PARK_ERR, d_ref, d);
            TEST_ASSERT_INT32_WITHIN(PARK_ERR, q_ref, q);
        }
    }
}

TEST_CASE("park inv then park settles, no grow", "[espFoC][park]")
{
    q16_t s, c;
    esp_foc_sincos(q16_from_float(0.7f), &s, &c);
    q16_t d = q16_from_float(0.2f);
    q16_t q = q16_from_float(-0.15f);
    q16_t a, b;
    for (int i = 0; i < 200; i++) {
        esp_foc_inv_park(s, c, d, q, &a, &b);
        esp_foc_park(s, c, a, b, &d, &q);
    }
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, q16_from_float(0.2f), d);
    TEST_ASSERT_INT32_WITHIN(PARK_ERR, q16_from_float(-0.15f), q);
}
