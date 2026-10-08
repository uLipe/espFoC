/*
 * Unit tests for esp_foc_trig (CORDIC Q16.16).
 */
#include <math.h>
#include <stdio.h>

#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_trig.h"

#define TRIG_ABS_ERR 0.002f
#define TRIG_CIRCLE_ERR 0.003f
#define ATAN_ABS_ERR 0.003f
#define SQRT_ABS_ERR 0.002f
#define ATAN_RECOVER_ERR 0.01f

TEST_CASE("sincos happy path cardinals", "[espFoC][trig]")
{
    q16_t s, c;

    esp_foc_sincos(0, &s, &c);
    TEST_ASSERT_INT32_WITHIN(64, 0, s);
    TEST_ASSERT_INT32_WITHIN(64, Q16_ONE, c);

    esp_foc_sincos(Q16_PI_2, &s, &c);
    TEST_ASSERT_INT32_WITHIN(128, Q16_ONE, s);
    TEST_ASSERT_INT32_WITHIN(128, 0, c);

    esp_foc_sincos(Q16_PI, &s, &c);
    TEST_ASSERT_INT32_WITHIN(128, 0, s);
    TEST_ASSERT_INT32_WITHIN(128, Q16_MINUS_ONE, c);

    esp_foc_sincos(Q16_MINUS_PI_2, &s, &c);
    TEST_ASSERT_INT32_WITHIN(128, Q16_MINUS_ONE, s);
    TEST_ASSERT_INT32_WITHIN(128, 0, c);
}

TEST_CASE("sincos wrappers match joint call", "[espFoC][trig]")
{
    q16_t a = q16_from_float(1.2f);
    q16_t s, c;
    esp_foc_sincos(a, &s, &c);
    TEST_ASSERT_EQUAL_INT32(s, esp_foc_sin(a));
    TEST_ASSERT_EQUAL_INT32(c, esp_foc_cos(a));
}

TEST_CASE("sincos wrap across +/- pi", "[espFoC][trig]")
{
    q16_t s0, c0, s1, c1;
    /* π+ε and −π−ε wrap to opposite sides of the branch cut but same trig values after wrap. */
    esp_foc_sincos(q16_add(Q16_PI, Q16_ONE), &s0, &c0);
    esp_foc_sincos(q16_wrap_pi(q16_add(Q16_PI, Q16_ONE)), &s1, &c1);
    TEST_ASSERT_INT32_WITHIN(64, s0, s1);
    TEST_ASSERT_INT32_WITHIN(64, c0, c1);
}

TEST_CASE("sincos unit circle sweep", "[espFoC][trig]")
{
    float max_err = 0.0f;
    for (int i = 0; i <= 64; i++) {
        float af = (-(float)M_PI) + (2.0f * (float)M_PI * (float)i / 64.0f);
        q16_t s, c;
        esp_foc_sincos(q16_from_float(af), &s, &c);
        float sf = q16_to_float(s);
        float cf = q16_to_float(c);
        float r2 = sf * sf + cf * cf;
        float err = fabsf(r2 - 1.0f);
        if (err > max_err) {
            max_err = err;
        }
    }
    TEST_ASSERT_TRUE(max_err <= TRIG_CIRCLE_ERR);
}

TEST_CASE("sincos vs float golden", "[espFoC][trig]")
{
    float max_s = 0.0f;
    float max_c = 0.0f;
    for (int i = 0; i <= 128; i++) {
        float af = (-(float)M_PI) + (2.0f * (float)M_PI * (float)i / 128.0f);
        q16_t s, c;
        esp_foc_sincos(q16_from_float(af), &s, &c);
        float es = fabsf(q16_to_float(s) - sinf(af));
        float ec = fabsf(q16_to_float(c) - cosf(af));
        if (es > max_s) {
            max_s = es;
        }
        if (ec > max_c) {
            max_c = ec;
        }
    }
    TEST_ASSERT_TRUE(max_s <= TRIG_ABS_ERR);
    TEST_ASSERT_TRUE(max_c <= TRIG_ABS_ERR);
}

TEST_CASE("atan2 axes and quadrants", "[espFoC][trig]")
{
    TEST_ASSERT_INT32_WITHIN(128, 0, esp_foc_atan2(0, 0));
    TEST_ASSERT_INT32_WITHIN(128, Q16_PI_2, esp_foc_atan2(Q16_ONE, 0));
    TEST_ASSERT_INT32_WITHIN(128, Q16_MINUS_PI_2, esp_foc_atan2(Q16_MINUS_ONE, 0));
    TEST_ASSERT_INT32_WITHIN(128, 0, esp_foc_atan2(0, Q16_ONE));
    TEST_ASSERT_INT32_WITHIN(256, Q16_PI, esp_foc_atan2(0, Q16_MINUS_ONE));

    q16_t a = esp_foc_atan2(Q16_ONE, Q16_ONE);
    TEST_ASSERT_FLOAT_WITHIN(ATAN_ABS_ERR, (float)M_PI / 4.0f, q16_to_float(a));
}

TEST_CASE("atan2 vs float golden", "[espFoC][trig]")
{
    float max_err = 0.0f;
    const float vals[] = { -2.f, -1.f, -0.5f, 0.f, 0.5f, 1.f, 2.f };
    for (unsigned yi = 0; yi < sizeof(vals) / sizeof(vals[0]); yi++) {
        for (unsigned xi = 0; xi < sizeof(vals) / sizeof(vals[0]); xi++) {
            float yf = vals[yi];
            float xf = vals[xi];
            if (yf == 0.f && xf == 0.f) {
                continue;
            }
            float got = q16_to_float(esp_foc_atan2(q16_from_float(yf), q16_from_float(xf)));
            float exp = atan2f(yf, xf);
            float err = fabsf(got - exp);
            /* wrap distance */
            if (err > (float)M_PI) {
                err = fabsf(err - 2.0f * (float)M_PI);
            }
            if (err > max_err) {
                max_err = err;
            }
        }
    }
    TEST_ASSERT_TRUE(max_err <= ATAN_ABS_ERR);
}

TEST_CASE("atan2 recovers sincos angle", "[espFoC][trig]")
{
    for (int i = 1; i < 32; i++) {
        float af = (-0.9f * (float)M_PI) + (1.8f * (float)M_PI * (float)i / 32.0f);
        q16_t s, c;
        esp_foc_sincos(q16_from_float(af), &s, &c);
        float back = q16_to_float(esp_foc_atan2(s, c));
        float err = fabsf(back - af);
        if (err > (float)M_PI) {
            err = fabsf(err - 2.0f * (float)M_PI);
        }
        TEST_ASSERT_TRUE(err <= ATAN_RECOVER_ERR);
    }
}

TEST_CASE("sqrt happy path and edges", "[espFoC][trig]")
{
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_sqrt(0));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_sqrt(Q16_MINUS_ONE));
    TEST_ASSERT_INT32_WITHIN(64, Q16_ONE, esp_foc_sqrt(Q16_ONE));
    TEST_ASSERT_INT32_WITHIN(128, Q16_HALF, esp_foc_sqrt(Q16_ONE / 4));
}

TEST_CASE("sqrt vs float golden", "[espFoC][trig]")
{
    float max_rel = 0.0f;
    const float vals[] = { 0.f, 0.01f, 0.25f, 0.5f, 1.f, 2.f, 4.f, 9.f, 16.f, 100.f, 1000.f };
    for (unsigned i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        float vf = vals[i];
        float got = q16_to_float(esp_foc_sqrt(q16_from_float(vf)));
        float exp = sqrtf(vf);
        float err = fabsf(got - exp);
        float rel = (exp > 1e-3f) ? (err / exp) : err;
        if (rel > max_rel) {
            max_rel = rel;
        }
        TEST_ASSERT_TRUE(err <= fmaxf(SQRT_ABS_ERR, 0.02f * fmaxf(exp, 1.0f)));
    }
    TEST_ASSERT_TRUE(max_rel <= 0.02f);
}
