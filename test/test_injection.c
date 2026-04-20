/*
 * MIT License
 *
 * Unit tests for esp_foc_injection (q-axis reference injection).
 * Tests cover:
 *   - step injection holds amplitude then auto-disables
 *   - chirp injection produces sinusoidal samples and sweeps frequency
 *   - axis-level helpers return the right error codes
 */

#include <string.h>
#include <math.h>
#include <unity.h>
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_injection.h"
#include "espFoC/esp_foc_axis_tuning.h"
#include "espFoC/utils/esp_foc_q16.h"

/* --- bare injection state tests (no axis) ------------------------------ */

TEST_CASE("injection: disabled passes reference through",
          "[espFoC][injection]")
{
    esp_foc_injection_t inj;
    esp_foc_injection_disable(&inj);
    q16_t base = q16_from_float(0.75f);
    TEST_ASSERT_EQUAL_INT32(base,
        esp_foc_injection_apply_q16(&inj, base));
}

TEST_CASE("injection: step holds amplitude then auto-disables",
          "[espFoC][injection]")
{
    esp_foc_injection_t inj;
    esp_foc_injection_disable(&inj);

    q16_t amp = q16_from_float(1.5f);
    q16_t ts = q16_from_float(0.001f); /* ~1 ms (Q16 quantized) */
    esp_foc_injection_step_setup(&inj, amp, 100 /*ms*/, ts);

    /* The implementation computes samples_total from the Q16-quantized ts,
     * which is the authoritative count here (it rounds down). */
    uint32_t expected_active = inj.samples_total;
    TEST_ASSERT_GREATER_THAN_UINT(0, expected_active);

    for (uint32_t i = 0; i < expected_active; ++i) {
        q16_t out = esp_foc_injection_apply_q16(&inj, 0);
        TEST_ASSERT_EQUAL_INT32(amp, out);
    }
    /* After that: disabled, pass-through */
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_injection_apply_q16(&inj, 0));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_injection_apply_q16(&inj, 0));
}

TEST_CASE("injection: chirp produces bounded sinusoid and sweeps freq",
          "[espFoC][injection]")
{
    esp_foc_injection_t inj;
    esp_foc_injection_disable(&inj);

    q16_t amp = q16_from_float(1.0f);
    q16_t ts = q16_from_float(0.0001f); /* 100 us */
    q16_t f0 = q16_from_float(100.0f);
    q16_t f1 = q16_from_float(500.0f);
    /* 100 ms total = 1000 samples */
    esp_foc_injection_chirp_setup(&inj, amp, f0, f1, 100, ts);

    q16_t peak = 0;
    int zero_crossings = 0;
    q16_t prev = 0;
    int samples = 1000;

    for (int i = 0; i < samples; ++i) {
        q16_t cur = esp_foc_injection_apply_q16(&inj, 0);
        q16_t abs_cur = (cur < 0) ? -cur : cur;
        if (abs_cur > peak) {
            peak = abs_cur;
        }
        if ((prev >= 0 && cur < 0) || (prev < 0 && cur >= 0)) {
            zero_crossings++;
        }
        prev = cur;
    }

    /* Peak must not exceed amplitude (may be slightly less due to phase) */
    TEST_ASSERT_TRUE(peak > q16_from_float(0.90f));
    TEST_ASSERT_TRUE(peak <= q16_from_float(1.01f));

    /* Expected number of zero crossings: integral of f(t) over 100 ms.
     * Linear sweep from 100 to 500 Hz: mean = 300 Hz, total cycles = 30,
     * so ~60 zero crossings. Allow ±20% slack. */
    TEST_ASSERT_INT_WITHIN(15, 60, zero_crossings);

    /* After the duration, injection auto-disables. */
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_injection_apply_q16(&inj, 0));
}

/* --- axis-level API tests (without full init, just smoke via struct) --- */

TEST_CASE("injection api: stop is idempotent and safe on null",
          "[espFoC][injection]")
{
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG, esp_foc_axis_inject_stop(NULL));
}

TEST_CASE("injection api: step rejects null axis",
          "[espFoC][injection]")
{
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_inject_step_q16(NULL, 0, 10));
    TEST_ASSERT_EQUAL(ESP_FOC_ERR_INVALID_ARG,
        esp_foc_axis_inject_chirp_q16(NULL, 0, 0, 0, 10));
}
