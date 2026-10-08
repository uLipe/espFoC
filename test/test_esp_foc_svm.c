/*
 * Unit tests for min-max common-mode SVM.
 */
#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_park.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_svm.h"
#include "espFoC/utils/esp_foc_trig.h"
#include "espFoC/utils/esp_foc_vlim.h"

#define DUTY_ERR ((q16_t)800)

static void assert_duty_unit(q16_t d)
{
    TEST_ASSERT_TRUE(d >= 0 && d <= Q16_ONE);
}

TEST_CASE("svm zero vector mid duties", "[espFoC][svm]")
{
    q16_t du, dv, dw;
    esp_foc_svm(0, 0, &du, &dv, &dw);
    TEST_ASSERT_INT32_WITHIN(DUTY_ERR, Q16_HALF, du);
    TEST_ASSERT_INT32_WITHIN(DUTY_ERR, Q16_HALF, dv);
    TEST_ASSERT_INT32_WITHIN(DUTY_ERR, Q16_HALF, dw);
}

TEST_CASE("svm linear disk duties in unit interval", "[espFoC][svm]")
{
    q16_t vmax = Q16_INV_SQRT3;
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
            q16_t a = q16_from_float((float)i * 0.12f);
            q16_t b = q16_from_float((float)j * 0.12f);
            q16_t mag = esp_foc_sqrt(q16_add(q16_mul(a, a), q16_mul(b, b)));
            if (mag > vmax) {
                continue;
            }
            q16_t du, dv, dw;
            esp_foc_svm(a, b, &du, &dv, &dw);
            assert_duty_unit(du);
            assert_duty_unit(dv);
            assert_duty_unit(dw);
        }
    }
}

TEST_CASE("svm min-max property max+min ~ 1", "[espFoC][svm]")
{
    q16_t a = q16_from_float(0.3f);
    q16_t b = q16_from_float(-0.2f);
    q16_t du, dv, dw;
    esp_foc_svm(a, b, &du, &dv, &dw);
    q16_t mx = q16_max(q16_max(du, dv), dw);
    q16_t mn = q16_min(q16_min(du, dv), dw);
    TEST_ASSERT_INT32_WITHIN(DUTY_ERR, Q16_ONE, q16_add(mx, mn));
}

TEST_CASE("svm overmod still clamped", "[espFoC][svm]")
{
    q16_t du, dv, dw;
    esp_foc_svm(Q16_ONE, Q16_ONE, &du, &dv, &dw);
    assert_duty_unit(du);
    assert_duty_unit(dv);
    assert_duty_unit(dw);
}

TEST_CASE("svm inv-park vlim pipeline finite", "[espFoC][svm]")
{
    q16_t vd = q16_from_float(0.05f);
    q16_t vq = q16_from_float(0.35f);
    for (int k = 0; k < 24; k++) {
        q16_t th = (q16_t)(((int64_t)k * (int64_t)Q16_TWO_PI) / 24);
        q16_t s, c;
        esp_foc_sincos(th, &s, &c);
        q16_t d = vd;
        q16_t q = vq;
        esp_foc_vlim_dq(&d, &q, Q16_INV_SQRT3);
        q16_t a, b;
        esp_foc_inv_park(s, c, d, q, &a, &b);
        q16_t du, dv, dw;
        esp_foc_svm(a, b, &du, &dv, &dw);
        assert_duty_unit(du);
        assert_duty_unit(dv);
        assert_duty_unit(dw);
    }
}

TEST_CASE("svm loop does not explode", "[espFoC][svm]")
{
    q16_t a = q16_from_float(0.2f);
    q16_t b = q16_from_float(-0.15f);
    q16_t du = Q16_HALF;
    q16_t dv = Q16_HALF;
    q16_t dw = Q16_HALF;
    for (int i = 0; i < 200; i++) {
        esp_foc_svm(a, b, &du, &dv, &dw);
    }
    assert_duty_unit(du);
    assert_duty_unit(dv);
    assert_duty_unit(dw);
}
