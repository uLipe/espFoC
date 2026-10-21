/*
 * Unit tests for esp_foc_iir (TDF-II Butterworth LPF).
 */
#include <math.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/utils/esp_foc_iir.h"
#include "espFoC/utils/esp_foc_q16.h"

TEST_CASE("iir LPF DC gain ~ 1", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_iir_design_lpf(&f, 1000.0f, 100.0f));
    q16_t x = Q16_ONE;
    q16_t y = 0;
    for (int i = 0; i < 400; i++) {
        y = esp_foc_iir_update(&f, x);
    }
    TEST_ASSERT_INT32_WITHIN(256, Q16_ONE, y);
}

TEST_CASE("iir reset clears delays", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_iir_design_lpf(&f, 1000.0f, 80.0f));
    (void)esp_foc_iir_update(&f, Q16_ONE);
    (void)esp_foc_iir_update(&f, Q16_ONE);
    TEST_ASSERT_TRUE(f.w1 != 0 || f.w2 != 0);
    esp_foc_iir_reset(&f);
    TEST_ASSERT_EQUAL_INT32(0, f.w1);
    TEST_ASSERT_EQUAL_INT32(0, f.w2);
}

TEST_CASE("iir design rejects invalid fc", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_iir_design_lpf(&f, 1000.0f, 500.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_iir_design_lpf(&f, 1000.0f, 0.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_foc_iir_design_lpf(NULL, 1000.0f, 100.0f));
}

TEST_CASE("iir negative input is signed", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_iir_design_lpf(&f, 1000.0f, 100.0f));
    q16_t y = 0;
    for (int i = 0; i < 400; i++) {
        y = esp_foc_iir_update(&f, Q16_MINUS_ONE);
    }
    TEST_ASSERT_INT32_WITHIN(256, Q16_MINUS_ONE, y);
}

TEST_CASE("iir step settles without growing oscillation", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_iir_design_lpf(&f, 1000.0f, 100.0f));
    q16_t y = 0;
    q16_t peak = 0;
    for (int i = 0; i < 250; i++) {
        y = esp_foc_iir_update(&f, Q16_ONE);
        if (y > peak) {
            peak = y;
        }
    }
    TEST_ASSERT_TRUE(peak < q16_from_float(1.25f));
    q16_t y0 = y;
    for (int i = 0; i < 50; i++) {
        y = esp_foc_iir_update(&f, Q16_ONE);
    }
    int32_t e0 = y0 > Q16_ONE ? (y0 - Q16_ONE) : (Q16_ONE - y0);
    int32_t e1 = y > Q16_ONE ? (y - Q16_ONE) : (Q16_ONE - y);
    TEST_ASSERT_TRUE(e1 <= e0);
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.03f), Q16_ONE, y);
}

TEST_CASE("iir impulse energy decays", "[espFoC][iir]")
{
    esp_foc_iir_t f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_iir_design_lpf(&f, 1000.0f, 80.0f));
    q16_t y = esp_foc_iir_update(&f, Q16_ONE);
    int64_t e_early = (int64_t)y * (int64_t)y;
    for (int i = 0; i < 8; i++) {
        y = esp_foc_iir_update(&f, 0);
        e_early += (int64_t)y * (int64_t)y;
    }
    int64_t e_late = 0;
    for (int i = 0; i < 8; i++) {
        y = esp_foc_iir_update(&f, 0);
        e_late += (int64_t)y * (int64_t)y;
    }
    TEST_ASSERT_TRUE(e_late < e_early);
}

TEST_CASE("lpf1 coef: 0 at DC, monotonic, below 1", "[espFoC][iir]")
{
    const float ts = 1.0f / 20000.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, esp_foc_lpf1_coef(0.0f, ts));
    const float b100 = esp_foc_lpf1_coef(100.0f, ts);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f - expf(-2.0f * 3.14159265f * 100.0f * ts), b100);
    TEST_ASSERT_TRUE(esp_foc_lpf1_coef(1000.0f, ts) > b100);
    TEST_ASSERT_TRUE(esp_foc_lpf1_coef(1e6f, ts) <= 1.0f);
}

TEST_CASE("lpf1: b0 edges hold or pass through", "[espFoC][iir]")
{
    TEST_ASSERT_EQUAL_INT32(Q16_HALF, esp_foc_lpf1(Q16_HALF, Q16_ONE, 0));
    TEST_ASSERT_EQUAL_INT32(Q16_MINUS_ONE, esp_foc_lpf1(Q16_HALF, Q16_MINUS_ONE, Q16_ONE));
}

TEST_CASE("lpf1: step tracks first-order curve and settles", "[espFoC][iir]")
{
    const float b0f = esp_foc_lpf1_coef(100.0f, 1.0f / 20000.0f);
    const q16_t b0 = q16_from_float(b0f);
    q16_t y = 0;
    q16_t prev = 0;
    for (int k = 1; k <= 200; k++) {
        y = esp_foc_lpf1(y, Q16_ONE, b0);
        TEST_ASSERT_TRUE(y >= prev);
        TEST_ASSERT_TRUE(y <= Q16_ONE);
        prev = y;
    }
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.01f),
                             q16_from_float(1.0f - powf(1.0f - b0f, 200.0f)), y);
    for (int k = 0; k < 4000; k++) {
        y = esp_foc_lpf1(y, Q16_ONE, b0);
    }
    /* Truncated b0·e leaves a dead band of 1/b0 LSB. */
    TEST_ASSERT_INT32_WITHIN(40, Q16_ONE, y);
    const q16_t y_end = esp_foc_lpf1(y, Q16_ONE, b0);
    TEST_ASSERT_EQUAL_INT32(y, y_end);
}

TEST_CASE("lpf1: negative full scale does not overflow", "[espFoC][iir]")
{
    q16_t y = q16_from_float(30000.0f);
    for (int k = 0; k < 2000; k++) {
        y = esp_foc_lpf1(y, q16_from_float(-30000.0f), q16_from_float(0.1f));
    }
    TEST_ASSERT_INT32_WITHIN(16, q16_from_float(-30000.0f), y);
}
