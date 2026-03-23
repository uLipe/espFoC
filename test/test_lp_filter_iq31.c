/*
 * Unit tests for EMA low-pass filter in IQ31 (ema_low_pass_filter_iq31.h).
 */
#include <math.h>
#include <unity.h>
#include "espFoC/utils/ema_low_pass_filter_iq31.h"

#define TOL 0.02f

TEST_CASE("esp_foc_lp_filter_iq31: init clamps alpha to [0,1]", "[espFoC][lp_filter_iq31]")
{
    esp_foc_lp_filter_iq31_t f;
    /* iq31_from_float(1.5) saturates to ONE; path alpha>ONE is for raw iq31_t > ONE if ever used */
    esp_foc_low_pass_filter_init_iq31(&f, iq31_from_float(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, iq31_to_float(f.alpha));
    esp_foc_low_pass_filter_init_iq31(&f, iq31_from_float(-0.1f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, iq31_to_float(f.alpha));
}

TEST_CASE("esp_foc_lp_filter_iq31: alpha=1 passes input through", "[espFoC][lp_filter_iq31]")
{
    esp_foc_lp_filter_iq31_t f;
    esp_foc_low_pass_filter_init_iq31(&f, IQ31_ONE);
    iq31_t out = esp_foc_low_pass_filter_update_iq31(&f, iq31_from_float(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5f, iq31_to_float(out));
    out = esp_foc_low_pass_filter_update_iq31(&f, iq31_from_float(-0.3f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.3f, iq31_to_float(out));
}

TEST_CASE("esp_foc_lp_filter_iq31: alpha=0 holds initial zero", "[espFoC][lp_filter_iq31]")
{
    esp_foc_lp_filter_iq31_t f;
    esp_foc_low_pass_filter_init_iq31(&f, 0);
    iq31_t out = esp_foc_low_pass_filter_update_iq31(&f, iq31_from_float(0.8f));
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, iq31_to_float(out));
}

TEST_CASE("esp_foc_lp_filter_iq31: step response converges", "[espFoC][lp_filter_iq31]")
{
    esp_foc_lp_filter_iq31_t f;
    esp_foc_low_pass_filter_init_iq31(&f, iq31_from_float(0.1f));
    iq31_t one = IQ31_ONE;
    for (int i = 0; i < 200; i++) {
        esp_foc_low_pass_filter_update_iq31(&f, one);
    }
    iq31_t out = esp_foc_low_pass_filter_update_iq31(&f, one);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 1.0f, iq31_to_float(out));
}

TEST_CASE("esp_foc_lp_filter_set_cutoff_iq31: alpha matches float formula", "[espFoC][lp_filter_iq31]")
{
    esp_foc_lp_filter_iq31_t f;
    float cutoff = 10.0f;
    float fs = 1000.0f;
    esp_foc_low_pass_filter_set_cutoff_iq31(&f, cutoff, fs);
    float wc = (2.0f * (float)M_PI * (cutoff / fs));
    float expected_alpha = wc / (1.0f + wc);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, expected_alpha, iq31_to_float(f.alpha));
}
