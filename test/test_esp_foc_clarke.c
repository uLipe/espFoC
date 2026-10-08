/*
 * Unit tests for amplitude-invariant Clarke / inverse Clarke.
 */
#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_clarke.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"

#define CLARKE_ERR ((q16_t)400)

TEST_CASE("clarke zeros stay zeros", "[espFoC][clarke]")
{
    q16_t a, b;
    esp_foc_clarke(0, 0, 0, &a, &b);
    TEST_ASSERT_EQUAL_INT32(0, a);
    TEST_ASSERT_EQUAL_INT32(0, b);
}

TEST_CASE("clarke balanced 120 deg |ab| constant", "[espFoC][clarke]")
{
    q16_t mag0 = 0;
    for (int k = 0; k < 12; k++) {
        q16_t th = (q16_t)(((int64_t)k * (int64_t)Q16_TWO_PI) / 12);
        q16_t su, cu, sv, cv, sw, cw;
        esp_foc_sincos(th, &su, &cu);
        esp_foc_sincos(q16_sub(th, q16_div(Q16_TWO_PI, q16_from_float(3.0f))), &sv, &cv);
        esp_foc_sincos(q16_add(th, q16_div(Q16_TWO_PI, q16_from_float(3.0f))), &sw, &cw);
        (void)cu;
        (void)cv;
        (void)cw;
        q16_t a, b;
        esp_foc_clarke(su, sv, sw, &a, &b);
        q16_t mag = esp_foc_sqrt(q16_add(q16_mul(a, a), q16_mul(b, b)));
        if (k == 0) {
            mag0 = mag;
            TEST_ASSERT_INT32_WITHIN(CLARKE_ERR, Q16_ONE, mag);
        } else {
            TEST_ASSERT_INT32_WITHIN(CLARKE_ERR, mag0, mag);
        }
    }
}

TEST_CASE("clarke kirchhoff reconstruction matches 3-wire", "[espFoC][clarke]")
{
    q16_t u = q16_from_float(0.4f);
    q16_t v = q16_from_float(-0.15f);
    q16_t w = q16_neg(q16_add(u, v));
    q16_t a3, b3, a2, b2;
    esp_foc_clarke(u, v, w, &a3, &b3);
    esp_foc_clarke(u, v, q16_neg(q16_add(u, v)), &a2, &b2);
    TEST_ASSERT_EQUAL_INT32(a3, a2);
    TEST_ASSERT_EQUAL_INT32(b3, b2);
}

TEST_CASE("clarke round-trip bounded", "[espFoC][clarke]")
{
    q16_t a0 = q16_from_float(0.31f);
    q16_t b0 = q16_from_float(-0.22f);
    q16_t u, v, w, a1, b1;
    esp_foc_inv_clarke(a0, b0, &u, &v, &w);
    TEST_ASSERT_INT32_WITHIN(64, 0, q16_add(q16_add(u, v), w));
    esp_foc_clarke(u, v, w, &a1, &b1);
    TEST_ASSERT_INT32_WITHIN(CLARKE_ERR, a0, a1);
    TEST_ASSERT_INT32_WITHIN(CLARKE_ERR, b0, b1);
}

TEST_CASE("clarke inv sum to zero", "[espFoC][clarke]")
{
    q16_t u, v, w;
    esp_foc_inv_clarke(Q16_ONE, 0, &u, &v, &w);
    TEST_ASSERT_INT32_WITHIN(8, Q16_ONE, u);
    TEST_ASSERT_INT32_WITHIN(64, 0, q16_add(q16_add(u, v), w));
}

TEST_CASE("clarke huge inputs stay finite over loop", "[espFoC][clarke]")
{
    q16_t a = Q16_ONE;
    q16_t b = Q16_ONE;
    q16_t u = q16_from_float(20.0f);
    q16_t v = q16_from_float(-15.0f);
    q16_t w = q16_from_float(-5.0f);
    for (int i = 0; i < 200; i++) {
        esp_foc_clarke(u, v, w, &a, &b);
        esp_foc_inv_clarke(a, b, &u, &v, &w);
    }
    TEST_ASSERT_TRUE(a < q16_from_float(40.0f) && a > q16_from_float(-40.0f));
    TEST_ASSERT_TRUE(b < q16_from_float(40.0f) && b > q16_from_float(-40.0f));
}
