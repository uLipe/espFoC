/*
 * Unit tests for FOC low-pass filter (EMA, inline in ema_low_pass_filter.h).
 * No hardware; pure difference equation.
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/ema_low_pass_filter.h"

#define FLOAT_TOL 1e-5f

TEST_CASE("esp_foc_lp_filter: init clamps alpha to [0,1]", "[espFoC][lp_filter]")
{
    esp_foc_lp_filter_t f;
    esp_foc_low_pass_filter_init(&f, 1.5f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, f.alpha);
    esp_foc_low_pass_filter_init(&f, -0.1f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, f.alpha);
}

TEST_CASE("esp_foc_lp_filter: alpha=1 passes input through", "[espFoC][lp_filter]")
{
    esp_foc_lp_filter_t f;
    esp_foc_low_pass_filter_init(&f, 1.0f);
    float out = esp_foc_low_pass_filter_update(&f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 5.0f, out);
    out = esp_foc_low_pass_filter_update(&f, -3.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -3.0f, out);
}

TEST_CASE("esp_foc_lp_filter: alpha=0 holds initial zero", "[espFoC][lp_filter]")
{
    esp_foc_lp_filter_t f;
    esp_foc_low_pass_filter_init(&f, 0.0f);
    float out = esp_foc_low_pass_filter_update(&f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out);
}

TEST_CASE("esp_foc_lp_filter: step response converges", "[espFoC][lp_filter]")
{
    esp_foc_lp_filter_t f;
    esp_foc_low_pass_filter_init(&f, 0.1f);
    for (int i = 0; i < 200; i++) {
        esp_foc_low_pass_filter_update(&f, 1.0f);
    }
    float out = esp_foc_low_pass_filter_update(&f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, out);
}

TEST_CASE("esp_foc_lp_filter_set_cutoff: sets alpha from cutoff and fs", "[espFoC][lp_filter]")
{
    esp_foc_lp_filter_t f;
    float cutoff = 10.0f;
    float fs = 1000.0f;
    esp_foc_low_pass_filter_set_cutoff(&f, cutoff, fs);
    float wc = (2.0f * (float)M_PI * (cutoff / fs));
    float expected_alpha = wc / (1.0f + wc);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_alpha, f.alpha);
}
