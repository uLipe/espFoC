/*
 * Unit tests for esp_foc_design (design-time float/fixed conversions and
 * bandwidth bounds).
 */
#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "espFoC/utils/esp_foc_design.h"
#include "espFoC/utils/esp_foc_q16.h"

TEST_CASE("design: q32/q16 from double round to nearest", "[espFoC][design]")
{
    TEST_ASSERT_TRUE((1LL << 32) == esp_foc_q32_from_double(1.0));
    TEST_ASSERT_TRUE((-(1LL << 31)) == esp_foc_q32_from_double(-0.5));
    TEST_ASSERT_TRUE(0 == esp_foc_q32_from_double(0.0));
    TEST_ASSERT_TRUE(1 == esp_foc_q32_from_double(0.6 / ESP_FOC_Q32_ONE));
    TEST_ASSERT_TRUE(32768 == esp_foc_q16_from_double(0.5));
    TEST_ASSERT_TRUE((-65536LL * 40000) == esp_foc_q16_from_double(-40000.0));
}

TEST_CASE("design: step_q16 scales per tick, never 0", "[espFoC][design]")
{
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, esp_foc_step_q16(1000.0f, 1000.0f));
    TEST_ASSERT_EQUAL_INT32(Q16_HALF, esp_foc_step_q16(10000.0f, 20000.0f));
    TEST_ASSERT_EQUAL_INT32(1, esp_foc_step_q16(1e-6f, 20000.0f));
    TEST_ASSERT_EQUAL_INT32(1, esp_foc_step_q16(0.0f, 20000.0f));
}

TEST_CASE("design: base speed from Vdc and flux", "[espFoC][design]")
{
    const float expect = 12.0f / sqrtf(3.0f) / (2.0f * 3.14159265f * 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f * expect, expect, esp_foc_base_speed_hz(12.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f * expect, 2.0f * expect,
                             esp_foc_base_speed_hz(24.0f, 0.01f));
}

TEST_CASE("design: speed bandwidth bounds", "[espFoC][design]")
{
    const float noise = esp_foc_speed_bw_noise_hz(0.5f, 2.0f, 100.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 25.0f / (4.0f * 3.14159265f), noise);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, noise * 0.5f,
                             esp_foc_speed_bw_noise_hz(0.5f, 4.0f, 100.0f, 1.0f));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, esp_foc_speed_bw_phase_hz(100.0f, 5.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.5f, esp_foc_speed_bw_phase_hz(100.0f, 5.0f, 2.0f));
}
