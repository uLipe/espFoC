/*
 * esp_foc_limit_voltage_q16 — geometry vs float hypot + edge cases.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"

static float mag_float(q16_t vd, q16_t vq)
{
    double d = (double)q16_to_float(vd);
    double q = (double)q16_to_float(vq);
    return (float)sqrt(d * d + q * q);
}

TEST_CASE("voltage_limit_q16: zero vector with dc>0 unchanged", "[espFoC][voltage_limit]")
{
    q16_t vd = 0;
    q16_t vq = 0;
    esp_foc_limit_voltage_q16(&vd, &vq, q16_from_float(10.0f));
    TEST_ASSERT_EQUAL_INT32(0, vd);
    TEST_ASSERT_EQUAL_INT32(0, vq);
}

TEST_CASE("voltage_limit_q16: zero dc clamps output", "[espFoC][voltage_limit]")
{
    q16_t vd = q16_from_float(3.0f);
    q16_t vq = q16_from_float(4.0f);
    esp_foc_limit_voltage_q16(&vd, &vq, 0);
    TEST_ASSERT_EQUAL_INT32(0, vd);
    TEST_ASSERT_EQUAL_INT32(0, vq);
}

TEST_CASE("voltage_limit_q16: inside circle unchanged", "[espFoC][voltage_limit]")
{
    q16_t vd = q16_from_float(0.2f);
    q16_t vq = q16_from_float(0.3f);
    q16_t exp_d = vd;
    q16_t exp_q = vq;
    esp_foc_limit_voltage_q16(&vd, &vq, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32(exp_d, vd);
    TEST_ASSERT_EQUAL_INT32(exp_q, vq);
}

TEST_CASE("voltage_limit_q16: scaled to dc magnitude (axis-aligned)", "[espFoC][voltage_limit]")
{
    q16_t vdc = q16_from_float(5.0f);
    q16_t vd = q16_from_float(10.0f);
    q16_t vq = 0;
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 5.0f, mag_float(vd, vq));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 5.0f, q16_to_float(vd));
    TEST_ASSERT_EQUAL_INT32(0, vq);
}

TEST_CASE("voltage_limit_q16: scaled to dc magnitude (diagonal)", "[espFoC][voltage_limit]")
{
    q16_t vdc = q16_from_float(5.0f);
    q16_t vd = q16_from_float(40.0f);
    q16_t vq = q16_from_float(40.0f);
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 5.0f, mag_float(vd, vq));
}

TEST_CASE("voltage_limit_q16: negative dc uses absolute limit", "[espFoC][voltage_limit]")
{
    q16_t vdc = q16_from_float(-6.0f);
    q16_t vd = q16_from_float(20.0f);
    q16_t vq = 0;
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 6.0f, mag_float(vd, vq));
}

TEST_CASE("voltage_limit_q16: large values stay bounded", "[espFoC][voltage_limit]")
{
    q16_t vdc = q16_from_float(12.0f);
    q16_t vd = q16_from_float(8000.0f);
    q16_t vq = q16_from_float(-6000.0f);
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    TEST_ASSERT_TRUE(mag_float(vd, vq) <= 12.5f);
}

TEST_CASE("voltage_limit_q16: tiny overflow still scales", "[espFoC][voltage_limit]")
{
    q16_t vdc = q16_from_float(1.0f);
    q16_t vd = q16_from_float(1.05f);
    q16_t vq = 0;
    esp_foc_limit_voltage_q16(&vd, &vq, vdc);
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 1.0f, mag_float(vd, vq));
}
