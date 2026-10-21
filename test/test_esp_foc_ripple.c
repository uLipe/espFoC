/*
 * Unit tests for esp_foc_ripple (residual sigma about a least-squares line).
 */
#include <math.h>
#include <stdint.h>

#include "unity.h"
#include "espFoC/utils/esp_foc_ripple.h"
#include "espFoC/utils/esp_foc_q16.h"

static uint32_t s_lcg;

/* Uniform in [-1, 1). */
static float lcg_uniform(void)
{
    s_lcg = s_lcg * 1664525u + 1013904223u;
    return ((float)(s_lcg >> 8) / 8388608.0f) - 1.0f;
}

TEST_CASE("ripple: too few samples gives 0", "[espFoC][ripple]")
{
    esp_foc_ripple_t r;
    esp_foc_ripple_arm(&r, 1000u);
    for (int i = 0; i < 10; i++) {
        esp_foc_ripple_add(&r, (q16_t)(i * Q16_ONE));
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, esp_foc_ripple_sigma(&r, 64u));
}

TEST_CASE("ripple: budget caps accumulation, stop freezes it", "[espFoC][ripple]")
{
    esp_foc_ripple_t r;
    esp_foc_ripple_arm(&r, 50u);
    for (int i = 0; i < 80; i++) {
        esp_foc_ripple_add(&r, (q16_t)(i * Q16_ONE));
    }
    TEST_ASSERT_EQUAL_UINT32(50u, r.n);
    esp_foc_ripple_arm(&r, 1000u);
    TEST_ASSERT_EQUAL_UINT32(0u, r.n);
    esp_foc_ripple_add(&r, Q16_ONE);
    esp_foc_ripple_stop(&r);
    esp_foc_ripple_add(&r, Q16_ONE);
    TEST_ASSERT_EQUAL_UINT32(1u, r.n);
}

TEST_CASE("ripple: pure ramp on a large offset stays at quantisation floor", "[espFoC][ripple]")
{
    esp_foc_ripple_t r;
    esp_foc_ripple_arm(&r, 4000u);
    const q16_t base = q16_from_float(20000.0f);
    for (int i = 0; i < 4000; i++) {
        esp_foc_ripple_add(&r, base + i * 37);
    }
    /* Samples are kept at 2^-8 Q16 LSB: floor ~ 2^-8 / sqrt(12) = 1.1e-3. */
    TEST_ASSERT_TRUE(esp_foc_ripple_sigma(&r, 64u) < 1.5e-3f);
}

TEST_CASE("ripple: ramp plus uniform noise recovers sigma", "[espFoC][ripple]")
{
    const float amp = 0.05f;
    esp_foc_ripple_t r;
    esp_foc_ripple_arm(&r, 4000u);
    s_lcg = 12345u;
    for (int i = 0; i < 4000; i++) {
        const float x = 10.0f + 0.002f * (float)i + amp * lcg_uniform();
        esp_foc_ripple_add(&r, q16_from_float(x));
    }
    const float expect = amp / sqrtf(3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f * expect, expect, esp_foc_ripple_sigma(&r, 64u));
}

TEST_CASE("ripple: negative ramp is symmetric", "[espFoC][ripple]")
{
    const float amp = 0.02f;
    esp_foc_ripple_t up;
    esp_foc_ripple_t dn;
    esp_foc_ripple_arm(&up, 2000u);
    esp_foc_ripple_arm(&dn, 2000u);
    s_lcg = 777u;
    for (int i = 0; i < 2000; i++) {
        const float n = amp * lcg_uniform();
        esp_foc_ripple_add(&up, q16_from_float(0.003f * (float)i + n));
        esp_foc_ripple_add(&dn, q16_from_float(-0.003f * (float)i - n));
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, esp_foc_ripple_sigma(&up, 64u),
                             esp_foc_ripple_sigma(&dn, 64u));
}
