/*
 * Unit tests for the scalar Q16 helpers (abs, sat64, slew, to_milli).
 */
#include <stdint.h>

#include "unity.h"
#include "espFoC/utils/esp_foc_q16.h"

TEST_CASE("q16_abs: sign and INT32_MIN saturate", "[espFoC][q16]")
{
    TEST_ASSERT_EQUAL_INT32(0, q16_abs(0));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, q16_abs(Q16_ONE));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, q16_abs(Q16_MINUS_ONE));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, q16_abs((q16_t)INT32_MIN));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, q16_abs((q16_t)-INT32_MAX));
}

TEST_CASE("q16_sat64: passes in range, clamps symmetric", "[espFoC][q16]")
{
    TEST_ASSERT_EQUAL_INT32(12345, q16_sat64(12345));
    TEST_ASSERT_EQUAL_INT32(-12345, q16_sat64(-12345));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, q16_sat64((int64_t)INT32_MAX));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, q16_sat64((int64_t)INT32_MAX + 1));
    TEST_ASSERT_EQUAL_INT32(-INT32_MAX, q16_sat64((int64_t)INT32_MIN));
    TEST_ASSERT_EQUAL_INT32(-INT32_MAX, q16_sat64(INT64_MIN));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, q16_sat64(INT64_MAX));
}

TEST_CASE("q16_slew: bounded step, lands exactly on target", "[espFoC][q16]")
{
    const q16_t step = Q16_ONE / 4;
    q16_t x = 0;
    int n = 0;
    while (x != Q16_ONE && n < 100) {
        const q16_t prev = x;
        x = q16_slew(x, Q16_ONE, step);
        TEST_ASSERT_TRUE(x - prev <= step);
        TEST_ASSERT_TRUE(x >= prev);
        n++;
    }
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, x);
    TEST_ASSERT_EQUAL(4, n);

    x = q16_slew(x, Q16_MINUS_ONE, step);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE - step, x);

    TEST_ASSERT_EQUAL_INT32(Q16_ONE + 3, q16_slew(Q16_ONE, Q16_ONE + 3, step));
}

TEST_CASE("q16_slew: step 0 jumps, extremes do not overflow", "[espFoC][q16]")
{
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, q16_slew(Q16_ONE, Q16_MINUS_ONE, 0));
    const q16_t hi = (q16_t)INT32_MAX;
    const q16_t lo = (q16_t)-INT32_MAX;
    TEST_ASSERT_EQUAL_INT32(hi - Q16_ONE, q16_slew(hi, lo, Q16_ONE));
    TEST_ASSERT_EQUAL_INT32(lo + Q16_ONE, q16_slew(lo, hi, Q16_ONE));
}

TEST_CASE("q16_to_milli: scale and floor rounding", "[espFoC][q16]")
{
    TEST_ASSERT_EQUAL_INT32(1000, q16_to_milli(Q16_ONE));
    TEST_ASSERT_EQUAL_INT32(-1000, q16_to_milli(Q16_MINUS_ONE));
    TEST_ASSERT_EQUAL_INT32(500, q16_to_milli(Q16_HALF));
    TEST_ASSERT_EQUAL_INT32(0, q16_to_milli(1));
    TEST_ASSERT_EQUAL_INT32(-1, q16_to_milli(-1));
    TEST_ASSERT_EQUAL_INT32(32767999, q16_to_milli((q16_t)INT32_MAX));
}
