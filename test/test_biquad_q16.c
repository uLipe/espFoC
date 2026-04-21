/*
 * MIT License
 *
 * Tests for the discrete biquad and the 2nd-order Butterworth designer.
 *
 * Strategy:
 *   - DC gain: drive a constant input, expect the steady-state output
 *     to converge to the same constant (Butterworth has unity DC gain
 *     by construction, plus the bilinear transform preserves that).
 *   - Nyquist gain: drive an alternating +A/-A signal (frequency = fs/2),
 *     expect the output to converge to ~0 (Butterworth LPF has a perfect
 *     zero at Nyquist, H(-1) = 0).
 *   - Cutoff position: drive a sinusoid at the design fc, expect ~-3 dB
 *     (~0.707x amplitude) attenuation in steady state.
 *   - Below cutoff: ~0 dB; above cutoff: -40 dB/decade roll-off.
 *   - Coefficient designer guards: bypass on invalid (fc, fs).
 *
 * Tolerances are intentionally a bit loose to accommodate Q16 quantisation
 * noise on the coefficients (1 LSB ≈ 1.5e-5).
 */

#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/esp_foc_q16.h"

/* Settle long enough that the impulse response has effectively died.
 * 8000 samples at any fc above ~fs/100 gets us deep into steady state. */
#define SETTLE_SAMPLES 8000

TEST_CASE("biquad: bypass design passes input through", "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_q16_set_bypass(&f);
    q16_t x = q16_from_float(1.234f);
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_EQUAL_INT32(x, esp_foc_biquad_q16_update(&f, x));
    }
}

TEST_CASE("biquad: invalid (fc, fs) falls back to bypass",
          "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    /* Negative fs */
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 100.0f, -1000.0f);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, f.b0);
    /* fc beyond Nyquist */
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 600.0f, 1000.0f);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, f.b0);
    /* Zero fc */
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 0.0f, 1000.0f);
    TEST_ASSERT_EQUAL_INT32(Q16_ONE, f.b0);
}

TEST_CASE("biquad: DC gain is exactly unity", "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 200.0f, 20000.0f);

    q16_t x = q16_from_float(0.7f);
    q16_t y = 0;
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        y = esp_foc_biquad_q16_update(&f, x);
    }
    /* 200 LSB tolerance ≈ 0.003 — covers Q16 quantisation on the
     * coefficients propagating through the recursion. */
    TEST_ASSERT_INT32_WITHIN(200, x, y);
}

TEST_CASE("biquad: Nyquist input is fully rejected", "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 200.0f, 20000.0f);

    q16_t amp = q16_from_float(0.5f);
    q16_t y = 0;
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        q16_t x = (i & 1) ? -amp : amp;
        y = esp_foc_biquad_q16_update(&f, x);
    }
    TEST_ASSERT_INT32_WITHIN(200, 0, y);
}

/* Drive a sinusoid at frequency `f_hz` and measure the steady-state
 * peak amplitude. Returns the gain (output peak / input peak). */
static float measure_gain(float fc_hz, float fs_hz, float f_hz)
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_butterworth_lpf_design_q16(&f, fc_hz, fs_hz);

    const q16_t amp_q16 = q16_from_float(0.5f);
    /* Burn enough samples for steady state. */
    int settle = SETTLE_SAMPLES;
    for (int i = 0; i < settle; ++i) {
        float angle = 2.0f * 3.14159265358979f * f_hz * (float)i / fs_hz;
        q16_t x = q16_from_float(0.5f * sinf(angle));
        (void)esp_foc_biquad_q16_update(&f, x);
    }

    /* Now sweep one period and capture the peak. */
    int per_period = (int)(fs_hz / f_hz);
    if (per_period < 16) {
        per_period = 16;
    }
    float peak = 0.0f;
    int total = per_period * 8;
    for (int i = 0; i < total; ++i) {
        float angle = 2.0f * 3.14159265358979f * f_hz * (float)(settle + i) / fs_hz;
        q16_t x = q16_from_float(0.5f * sinf(angle));
        q16_t y = esp_foc_biquad_q16_update(&f, x);
        float yf = q16_to_float(y);
        if (yf > peak) {
            peak = yf;
        }
    }
    return peak / q16_to_float(amp_q16);
}

TEST_CASE("biquad: -3 dB landing at cutoff (within 5%)",
          "[espFoC][biquad]")
{
    /* Standard 2nd-order Butterworth: gain at fc is 1/sqrt(2) ≈ 0.707.
     * Q16 quantisation gives a few percent slack. */
    float gain = measure_gain(500.0f, 20000.0f, 500.0f);
    /* Allow [0.62, 0.78]; theoretical 0.707. */
    TEST_ASSERT_TRUE(gain > 0.62f);
    TEST_ASSERT_TRUE(gain < 0.78f);
}

TEST_CASE("biquad: passband is essentially unity",
          "[espFoC][biquad]")
{
    /* One decade below cutoff: gain should be very close to 1.
     * Standard 2nd-order Butterworth has -0.04 dB at fc/3,
     * so > 0.95 is comfortable. */
    float gain = measure_gain(2000.0f, 40000.0f, 200.0f);
    TEST_ASSERT_TRUE(gain > 0.95f);
    TEST_ASSERT_TRUE(gain < 1.05f);
}

TEST_CASE("biquad: stopband attenuation tracks 40 dB/decade",
          "[espFoC][biquad]")
{
    /* One decade above cutoff: attenuation should be roughly 40 dB
     * (gain ≈ 0.01). Quantisation widens the floor; require < 0.05. */
    float gain = measure_gain(200.0f, 20000.0f, 2000.0f);
    TEST_ASSERT_TRUE(gain < 0.05f);
}

TEST_CASE("biquad: long-run stability with white-ish input",
          "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 500.0f, 20000.0f);
    /* 100 k samples of LCG-pseudo-noise; ensure no overflow / NaN
     * (q16 has no NaN representation but a runaway would saturate to
     * INT32_MIN/MAX which we can detect). */
    uint32_t s = 1u;
    q16_t y = 0;
    for (int i = 0; i < 100000; ++i) {
        s = s * 1103515245u + 12345u;
        int32_t r = (int32_t)(s & 0x3fff) - 0x2000; /* +-8192 raw q16, ~+-0.125 */
        y = esp_foc_biquad_q16_update(&f, (q16_t)r);
    }
    TEST_ASSERT_NOT_EQUAL(INT32_MAX, y);
    TEST_ASSERT_NOT_EQUAL(INT32_MIN, y);
}

TEST_CASE("biquad: reset clears state without touching coefficients",
          "[espFoC][biquad]")
{
    esp_foc_biquad_q16_t f;
    esp_foc_biquad_butterworth_lpf_design_q16(&f, 100.0f, 10000.0f);
    q16_t b0 = f.b0, b1 = f.b1, a2 = f.a2;
    /* Wind state up. */
    for (int i = 0; i < 50; ++i) {
        (void)esp_foc_biquad_q16_update(&f, q16_from_float(0.9f));
    }
    TEST_ASSERT_NOT_EQUAL(0, f.s1);
    esp_foc_biquad_q16_reset(&f);
    TEST_ASSERT_EQUAL_INT32(0, f.s1);
    TEST_ASSERT_EQUAL_INT32(0, f.s2);
    TEST_ASSERT_EQUAL_INT32(b0, f.b0);
    TEST_ASSERT_EQUAL_INT32(b1, f.b1);
    TEST_ASSERT_EQUAL_INT32(a2, f.a2);
}
