/*
 * Unit tests for esp_foc_pi_q32 (Q32-gain PI with conditional integration).
 */
#include <stdint.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/utils/esp_foc_pi_q32.h"
#include "espFoC/utils/esp_foc_q16.h"

static void pi_make(esp_foc_pi_q32_t *pi, float kp, float ki, float fs, q16_t i_lim)
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pi_q32_set_gains(pi, kp, ki, fs));
    esp_foc_pi_q32_set_limit(pi, i_lim);
    esp_foc_pi_q32_reset(pi, 0);
}

TEST_CASE("pi_q32: set_gains rejects bad args", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_pi_q32_set_gains(NULL, 1.0f, 1.0f, 1000.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_pi_q32_set_gains(&pi, 1.0f, 1.0f, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_pi_q32_set_gains(&pi, -1.0f, 1.0f, 1000.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_pi_q32_set_gains(&pi, 1.0f, -1.0f, 1000.0f));
}

TEST_CASE("pi_q32: gains scale to Q32 per sample", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pi_q32_set_gains(&pi, 2.0f, 100.0f, 1000.0f));
    TEST_ASSERT_TRUE((2LL << 32) == pi.kp_q32);
    TEST_ASSERT_TRUE(pi.ki_ts_q32 > 429496729LL - 2 && pi.ki_ts_q32 < 429496729LL + 2);
}

TEST_CASE("pi_q32: P only, ff adds, reset seeds output", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    pi_make(&pi, 2.0f, 0.0f, 1000.0f, Q16_ONE);
    const q16_t lim = q16_from_float(10.0f);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, esp_foc_pi_q32_update(&pi, Q16_HALF, 0, lim));
    TEST_ASSERT_EQUAL_INT32(Q16_ONE + Q16_HALF,
                            esp_foc_pi_q32_update(&pi, Q16_HALF, Q16_HALF, lim));
    esp_foc_pi_q32_reset(&pi, Q16_HALF);
    TEST_ASSERT_EQUAL_INT32(Q16_HALF, esp_foc_pi_q32_update(&pi, 0, 0, lim));
}

TEST_CASE("pi_q32: output clamped to +-lim", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    pi_make(&pi, 100.0f, 0.0f, 1000.0f, Q16_ONE);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, esp_foc_pi_q32_update(&pi, q16_from_float(5.0f), 0, Q16_ONE));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE,
                            esp_foc_pi_q32_update(&pi, q16_from_float(-5.0f), 0, Q16_ONE));
}

TEST_CASE("pi_q32: integrator clamped to i_lim", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    pi_make(&pi, 0.0f, 1000.0f, 1000.0f, Q16_HALF);
    const q16_t lim = q16_from_float(100.0f);
    q16_t u = 0;
    for (int i = 0; i < 100; i++) {
        u = esp_foc_pi_q32_update(&pi, Q16_ONE, 0, lim);
    }
    TEST_ASSERT_EQUAL_INT32(Q16_HALF, u);
    TEST_ASSERT_TRUE(((int64_t)Q16_HALF << 16) == pi.i_q32);
    for (int i = 0; i < 100; i++) {
        u = esp_foc_pi_q32_update(&pi, Q16_MINUS_ONE, 0, lim);
    }
    TEST_ASSERT_EQUAL_INT32(-Q16_HALF, u);
}

TEST_CASE("pi_q32: no windup while saturated, leaves on sign flip", "[espFoC][pi_q32]")
{
    esp_foc_pi_q32_t pi;
    pi_make(&pi, 0.1f, 50.0f, 1000.0f, q16_from_float(100.0f));
    const q16_t lim = Q16_ONE;
    for (int i = 0; i < 2000; i++) {
        TEST_ASSERT_EQUAL_INT32(lim, esp_foc_pi_q32_update(&pi, q16_from_float(20.0f), 0, lim));
    }
    TEST_ASSERT_TRUE(pi.i_q32 < ((int64_t)lim << 16));
    const q16_t u = esp_foc_pi_q32_update(&pi, q16_from_float(-0.5f), 0, lim);
    TEST_ASSERT_TRUE(u < lim);
}

TEST_CASE("pi_q32: closed loop on integrator plant settles, no oscillation", "[espFoC][pi_q32]")
{
    /* y[k+1] = y[k] + b·u[k], b = 0.01 → plant gain 10 /s at 1 kHz. */
    esp_foc_pi_q32_t pi;
    pi_make(&pi, 5.0f, 20.0f, 1000.0f, q16_from_float(10.0f));
    const q16_t b = q16_from_float(0.01f);
    const q16_t ref = q16_from_float(3.0f);
    const q16_t lim = q16_from_float(10.0f);
    q16_t y = 0;
    q16_t peak = 0;
    for (int k = 0; k < 6000; k++) {
        const q16_t u = esp_foc_pi_q32_update(&pi, q16_sub(ref, y), 0, lim);
        y = q16_add(y, q16_mul(b, u));
        if (y > peak) {
            peak = y;
        }
    }
    /* Poles -4.4 and -45.6 rad/s, PI zero at -4: ~8 % overshoot expected. */
    TEST_ASSERT_TRUE(peak < q16_add(ref, q16_from_float(0.4f)));
    q16_t y_min = y;
    q16_t y_max = y;
    for (int k = 0; k < 1000; k++) {
        const q16_t u = esp_foc_pi_q32_update(&pi, q16_sub(ref, y), 0, lim);
        y = q16_add(y, q16_mul(b, u));
        y_min = q16_min(y_min, y);
        y_max = q16_max(y_max, y);
    }
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.01f), ref, y);
    TEST_ASSERT_TRUE(y_max - y_min < q16_from_float(0.005f));
}
