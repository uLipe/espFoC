/*
 * MIT License
 *
 * Tests for the alpha-beta electrical-angle tracker. Strategy:
 *   - synthesise a ground-truth angle that walks at a known electrical
 *     frequency, sample it at the "encoder" rate (2 kHz), call
 *     update() with the noisy / quantised samples and check that the
 *     intermediate predict() values stay close to ground truth.
 *   - exercise wrap-around, step-frequency convergence, and the
 *     "encoder hung" degradation path.
 *
 * Tolerances are picked to stay above Q16 quantisation (one LSB at
 * the integrator outputs) while still catching real algorithmic
 * regressions.
 */

#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "espFoC/utils/angle_predictor_q16.h"
#include "espFoC/utils/esp_foc_q16.h"

static const float TWO_PI = 6.283185307179586f;

static q16_t wrap_2pi_float(float a)
{
    while (a < 0.0f) {
        a += TWO_PI;
    }
    while (a >= TWO_PI) {
        a -= TWO_PI;
    }
    return q16_from_float(a);
}

static float angle_diff_rad(q16_t a_q16, float b)
{
    float a = q16_to_float(a_q16);
    float d = a - b;
    while (d > 3.14159265f) {
        d -= TWO_PI;
    }
    while (d <= -3.14159265f) {
        d += TWO_PI;
    }
    return d;
}

TEST_CASE("predictor: untouched returns 0",
          "[espFoC][angle_predictor]")
{
    esp_foc_angle_predictor_q16_t p;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);
    /* Before the first update, predict() returns 0 — sentinel value
     * so the consumer never feeds a stale extrapolation into the PI. */
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_angle_predictor_predict_q16(&p, 1000u));
}

TEST_CASE("predictor: first update seeds without spurious omega",
          "[espFoC][angle_predictor]")
{
    esp_foc_angle_predictor_q16_t p;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);

    q16_t seed = q16_from_float(1.5f);
    esp_foc_angle_predictor_update_q16(&p, seed, 1000u);
    /* Right after the seeding update, predict() at the same instant
     * must equal the seed and omega must still be zero. */
    TEST_ASSERT_INT32_WITHIN(50, seed,
                             esp_foc_angle_predictor_predict_q16(&p, 1000u));
    TEST_ASSERT_EQUAL_INT32(0, esp_foc_angle_predictor_speed_q16(&p));
}

TEST_CASE("predictor: tracks a steady 60 Hz electrical rotation",
          "[espFoC][angle_predictor]")
{
    /* 60 Hz electrical -> omega = 376.99 rad/s. Sample at 2 kHz
     * (encoder cap). Predict at 40 kHz between samples. After ~40 ms
     * of settling, the predicted angle must be within 0.05 rad
     * (~3 degrees) of ground truth. */
    esp_foc_angle_predictor_q16_t p;
    const float fe_hz = 60.0f;
    const float omega_true = TWO_PI * fe_hz;
    const uint64_t enc_dt_us = 500u;       /* 2 kHz */
    const uint64_t pred_dt_us = 25u;        /* 40 kHz */
    const uint64_t total_us = 200000u;      /* 200 ms */

    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);

    uint64_t t_us = 0u;
    uint64_t next_enc = 0u;
    float worst_err_late = 0.0f;
    while (t_us <= total_us) {
        if (t_us >= next_enc) {
            float th = (omega_true * (float)t_us * 1e-6f);
            esp_foc_angle_predictor_update_q16(&p, wrap_2pi_float(th), t_us);
            next_enc += enc_dt_us;
        }
        if (t_us > 50000u) { /* skip the first 50 ms of settling */
            float th_true = omega_true * (float)t_us * 1e-6f;
            float err = fabsf(angle_diff_rad(
                esp_foc_angle_predictor_predict_q16(&p, t_us), th_true));
            if (err > worst_err_late) {
                worst_err_late = err;
            }
        }
        t_us += pred_dt_us;
    }
    TEST_ASSERT_TRUE_MESSAGE(worst_err_late < 0.05f,
        "predictor lagged truth by more than 0.05 rad in steady state");
    /* And the omega estimate should be close to the truth. */
    float omega_est = q16_to_float(esp_foc_angle_predictor_speed_q16(&p));
    TEST_ASSERT_FLOAT_WITHIN(omega_true * 0.02f, omega_true, omega_est);
}

TEST_CASE("predictor: converges on a step frequency change",
          "[espFoC][angle_predictor]")
{
    /* Run at 60 Hz for 100 ms, then jump to 120 Hz. The tracker
     * should re-acquire the new omega within ~30 ms of update calls
     * (60 update cycles at 2 kHz). */
    esp_foc_angle_predictor_q16_t p;
    const uint64_t enc_dt_us = 500u;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);

    /* First leg: 60 Hz for 100 ms */
    float omega1 = TWO_PI * 60.0f;
    uint64_t t = 0u;
    float th = 0.0f;
    while (t <= 100000u) {
        th += omega1 * (float)enc_dt_us * 1e-6f;
        esp_foc_angle_predictor_update_q16(&p, wrap_2pi_float(th), t);
        t += enc_dt_us;
    }

    /* Step to 120 Hz, run another 60 ms, sample omega at the end. */
    float omega2 = TWO_PI * 120.0f;
    while (t <= 160000u) {
        th += omega2 * (float)enc_dt_us * 1e-6f;
        esp_foc_angle_predictor_update_q16(&p, wrap_2pi_float(th), t);
        t += enc_dt_us;
    }
    float omega_est = q16_to_float(esp_foc_angle_predictor_speed_q16(&p));
    TEST_ASSERT_FLOAT_WITHIN(omega2 * 0.05f, omega2, omega_est);
}

TEST_CASE("predictor: wrap-around does not produce omega glitches",
          "[espFoC][angle_predictor]")
{
    /* Drive the angle past 2*pi back into [0, 2*pi) at every update.
     * The residual wrap to (-pi, +pi] must keep the omega estimate
     * locked at the true 60 Hz rate even when theta_meas jumps from
     * ~6.27 rad to ~0.01 rad between samples. */
    esp_foc_angle_predictor_q16_t p;
    const uint64_t enc_dt_us = 500u;
    const float omega_true = TWO_PI * 60.0f;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);
    uint64_t t = 0u;
    /* Run for 500 ms => ~30 wrap events at 60 Hz. */
    for (int i = 0; i <= 1000; ++i) {
        float th_true = omega_true * (float)t * 1e-6f;
        esp_foc_angle_predictor_update_q16(&p, wrap_2pi_float(th_true), t);
        t += enc_dt_us;
    }
    float omega_est = q16_to_float(esp_foc_angle_predictor_speed_q16(&p));
    TEST_ASSERT_FLOAT_WITHIN(omega_true * 0.02f, omega_true, omega_est);
    /* And predict() in the middle of a wrap stays in [0, 2*pi). */
    q16_t pred = esp_foc_angle_predictor_predict_q16(&p, t + 250u);
    float pred_f = q16_to_float(pred);
    TEST_ASSERT_TRUE(pred_f >= 0.0f && pred_f < TWO_PI + 0.01f);
}

TEST_CASE("predictor: degrades gracefully when encoder hangs",
          "[espFoC][angle_predictor]")
{
    /* If the I2C bus glitches and the outer task stops calling
     * update() for 50 ms while predict() keeps firing in the ISR, the
     * extrapolated angle must not blow up. At 60 Hz the true angle
     * advances by ~18.85 rad (3 full revolutions); modulo 2*pi the
     * error stays bounded by the omega_est noise floor. */
    esp_foc_angle_predictor_q16_t p;
    const uint64_t enc_dt_us = 500u;
    const float omega_true = TWO_PI * 60.0f;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);
    /* Settle the tracker for 100 ms. */
    uint64_t t = 0u;
    while (t <= 100000u) {
        esp_foc_angle_predictor_update_q16(
            &p, wrap_2pi_float(omega_true * (float)t * 1e-6f), t);
        t += enc_dt_us;
    }

    /* Then go silent for 50 ms but keep predicting at 40 kHz. */
    uint64_t silence_start = t;
    float worst = 0.0f;
    for (uint64_t dt = 25u; dt <= 50000u; dt += 25u) {
        uint64_t t_now = silence_start + dt;
        float th_true = omega_true * (float)t_now * 1e-6f;
        float err = fabsf(angle_diff_rad(
            esp_foc_angle_predictor_predict_q16(&p, t_now), th_true));
        if (err > worst) {
            worst = err;
        }
    }
    /* With a perfect omega_est, drift would be zero; allow 0.5 rad
     * worst-case to cover Q16 quantisation propagating over 50 ms. */
    TEST_ASSERT_TRUE_MESSAGE(worst < 0.5f,
        "predictor drifted more than 0.5 rad with no updates for 50 ms");
}

TEST_CASE("predictor: set_gains updates without losing state",
          "[espFoC][angle_predictor]")
{
    esp_foc_angle_predictor_q16_t p;
    esp_foc_angle_predictor_init_q16(&p, 0.3f, 0.05f, 0u);
    esp_foc_angle_predictor_update_q16(&p, q16_from_float(1.0f), 1000u);
    q16_t omega_before = p.omega_est;
    q16_t theta_before = p.theta_est;
    esp_foc_angle_predictor_set_gains_q16(&p, 0.5f, 0.10f);
    TEST_ASSERT_EQUAL_INT32(omega_before, p.omega_est);
    TEST_ASSERT_EQUAL_INT32(theta_before, p.theta_est);
    /* New gains take effect immediately: alpha=1.0 makes the next
     * update snap theta_est exactly to the measurement. */
    esp_foc_angle_predictor_set_gains_q16(&p, 1.0f, 0.0f);
    q16_t target = q16_from_float(2.5f);
    esp_foc_angle_predictor_update_q16(&p, target, 2000u);
    TEST_ASSERT_INT32_WITHIN(200, target, p.theta_est);
}
