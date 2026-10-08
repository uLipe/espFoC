/*
 * Unit tests for esp_foc_angle (portable Q16 wrap / delta).
 */
#include "unity.h"
#include "espFoC/utils/esp_foc_angle.h"
#include "espFoC/utils/esp_foc_q16.h"

TEST_CASE("q16_wrap_pi: identity inside range", "[espFoC][angle]")
{
    TEST_ASSERT_EQUAL_INT32(0, q16_wrap_pi(0));
    TEST_ASSERT_EQUAL_INT32(Q16_HALF, q16_wrap_pi(Q16_HALF));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, q16_wrap_pi(Q16_MINUS_ONE));
}

TEST_CASE("q16_wrap_pi: above +pi wraps down", "[espFoC][angle]")
{
    q16_t x = q16_add(Q16_PI, Q16_ONE);
    q16_t w = q16_wrap_pi(x);
    TEST_ASSERT_TRUE(w <= Q16_PI);
    TEST_ASSERT_TRUE(w > Q16_MINUS_PI);
    /* +π + 1 → near −π */
    TEST_ASSERT_INT32_WITHIN(Q16_ONE * 2, Q16_MINUS_PI, w);
}

TEST_CASE("q16_wrap_pi: below -pi wraps up", "[espFoC][angle]")
{
    q16_t x = q16_sub(Q16_MINUS_PI, Q16_ONE);
    q16_t w = q16_wrap_pi(x);
    TEST_ASSERT_TRUE(w <= Q16_PI);
    TEST_ASSERT_TRUE(w > Q16_MINUS_PI);
}

TEST_CASE("q16_wrap_pi: multi-turn", "[espFoC][angle]")
{
    q16_t x = q16_add(Q16_TWO_PI, q16_add(Q16_TWO_PI, Q16_HALF));
    q16_t w = q16_wrap_pi(x);
    TEST_ASSERT_INT32_WITHIN(64, Q16_HALF, w);
}

TEST_CASE("q16_angle_delta: near zero", "[espFoC][angle]")
{
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, q16_angle_delta(0, Q16_ONE));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, q16_angle_delta(Q16_ONE, 0));
}

TEST_CASE("q16_angle_delta: wrap positive", "[espFoC][angle]")
{
    /* from nearly +π to nearly −π → small positive step */
    q16_t a = q16_sub(Q16_PI, Q16_ONE);
    q16_t b = q16_add(Q16_MINUS_PI, Q16_ONE * 2);
    q16_t d = q16_angle_delta(a, b);
    TEST_ASSERT_TRUE(d > 0);
    TEST_ASSERT_TRUE(d < Q16_PI);
}

TEST_CASE("q16_angle_delta: wrap negative", "[espFoC][angle]")
{
    q16_t a = q16_add(Q16_MINUS_PI, Q16_ONE);
    q16_t b = q16_sub(Q16_PI, Q16_ONE * 2);
    q16_t d = q16_angle_delta(a, b);
    TEST_ASSERT_TRUE(d < 0);
    TEST_ASSERT_TRUE(d > Q16_MINUS_PI);
}

TEST_CASE("velocity from delta * inv_dt is stable", "[espFoC][angle]")
{
    q16_t dt = q16_from_float(0.001f);
    q16_t inv_dt = q16_from_float(1000.0f);
    q16_t delta = q16_from_float(0.01f); /* 0.01 rad */
    q16_t omega = q16_mul(delta, inv_dt); /* ~10 rad/s */
    float w = q16_to_float(omega);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, w);
    (void)dt;
}
