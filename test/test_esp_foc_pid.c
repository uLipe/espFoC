/*
 * Unit tests for esp_foc_pid (2p2z).
 */
#include <math.h>

#include "unity.h"
#include "esp_err.h"
#include "espFoC/utils/esp_foc_pid.h"
#include "espFoC/utils/esp_foc_q16.h"

static float wrap_pi_f_local(float d)
{
    while (d > (float)M_PI) {
        d -= 2.0f * (float)M_PI;
    }
    while (d < -(float)M_PI) {
        d += 2.0f * (float)M_PI;
    }
    return d;
}

TEST_CASE("pid PI tracks first-order plant step", "[espFoC][pid]")
{
    esp_foc_pid_t pid;
    const float ts = 0.01f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, 0.6f, 6.0f, 0.0f, 0.0f, ts));

    q16_t y = 0;
    q16_t sp = Q16_ONE;
    const q16_t a = q16_from_float(0.90f);
    const q16_t b = q16_from_float(0.10f);
    q16_t y_prev = 0;
    int growing = 0;
    for (int i = 0; i < 300; i++) {
        q16_t u = esp_foc_pid_update(&pid, sp, y);
        y = q16_add(q16_mul(a, y), q16_mul(b, u));
        q16_t e = q16_sub(sp, y);
        q16_t e_prev = q16_sub(sp, y_prev);
        int32_t ae = e < 0 ? -e : e;
        int32_t aep = e_prev < 0 ? -e_prev : e_prev;
        if (i > 80 && ae > aep + 128) {
            growing++;
        }
        y_prev = y;
    }
    TEST_ASSERT_TRUE(growing < 25);
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.08f), sp, y);
}

TEST_CASE("pid P-only leftover offset", "[espFoC][pid]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, 0.5f, 0.0f, 0.0f, 0.0f, 0.01f));
    q16_t y = 0;
    q16_t sp = Q16_ONE;
    const q16_t a = q16_from_float(0.90f);
    const q16_t b = q16_from_float(0.10f);
    for (int i = 0; i < 200; i++) {
        q16_t u = esp_foc_pid_update(&pid, sp, y);
        y = q16_add(q16_mul(a, y), q16_mul(b, u));
    }
    q16_t e = q16_sub(sp, y);
    if (e < 0) {
        e = q16_neg(e);
    }
    TEST_ASSERT_TRUE(e > q16_from_float(0.05f));
}

TEST_CASE("pid reset zeros effort at e=0", "[espFoC][pid]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, 0.5f, 4.0f, 0.0f, 0.0f, 0.01f));
    for (int i = 0; i < 20; i++) {
        (void)esp_foc_pid_update(&pid, Q16_ONE, 0);
    }
    esp_foc_pid_reset(&pid);
    q16_t u = esp_foc_pid_update(&pid, 0, 0);
    TEST_ASSERT_INT32_WITHIN(8, 0, u);
}

TEST_CASE("pid bypass is sp + ff", "[espFoC][pid]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, 1.0f, 1.0f, 0.0f, 0.0f, 0.01f));
    esp_foc_pid_set_ff(&pid, Q16_HALF);
    esp_foc_pid_set_bypass(&pid, true);
    q16_t u = esp_foc_pid_update(&pid, Q16_ONE, q16_from_float(99.0f));
    TEST_ASSERT_INT32_WITHIN(8, q16_add(Q16_ONE, Q16_HALF), u);
}

TEST_CASE("pid feedforward adds after 2p2z", "[espFoC][pid]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f));
    q16_t u0 = esp_foc_pid_update(&pid, Q16_ONE, 0);
    esp_foc_pid_reset(&pid);
    esp_foc_pid_set_ff(&pid, Q16_HALF);
    q16_t u1 = esp_foc_pid_update(&pid, Q16_ONE, 0);
    TEST_ASSERT_INT32_WITHIN(16, q16_add(u0, Q16_HALF), u1);
}

TEST_CASE("pid applied output holds windup", "[espFoC][pid]")
{
    esp_foc_pid_t no_aw;
    esp_foc_pid_t aw;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&no_aw, 0.2f, 40.0f, 0.0f, 0.0f, 0.01f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&aw, 0.2f, 40.0f, 0.0f, 0.0f, 0.01f));
    const q16_t umax = q16_from_float(0.15f);
    q16_t u_free = 0;
    q16_t u_held = 0;
    for (int i = 0; i < 80; i++) {
        u_free = esp_foc_pid_update(&no_aw, Q16_ONE, 0);
        u_held = esp_foc_pid_update(&aw, Q16_ONE, 0);
        q16_t sat = q16_clamp(u_held, q16_neg(umax), umax);
        esp_foc_pid_set_applied(&aw, sat);
    }
    q16_t af = u_free < 0 ? q16_neg(u_free) : u_free;
    q16_t ah = u_held < 0 ? q16_neg(u_held) : u_held;
    TEST_ASSERT_TRUE(af > q16_from_float(8.0f));
    TEST_ASSERT_TRUE(ah < q16_from_float(1.0f));
    TEST_ASSERT_TRUE(af > q16_mul(ah, q16_from_float(4.0f)));
}

TEST_CASE("speed PI applied seed makes handoff bumpless", "[espFoC][pid][speed]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(
        ESP_OK,
        esp_foc_pid_init(&pid, 0.0048f, 0.050f, 0.0f, 0.0f, 0.0005f));

    const q16_t omega = q16_from_float(2.0f * (float)M_PI * 50.0f);
    const q16_t iq_seed = q16_from_float(0.35f);
    esp_foc_pid_reset(&pid);
    esp_foc_pid_set_applied(&pid, iq_seed);

    q16_t iq_ref = esp_foc_pid_update(&pid, omega, omega);
    TEST_ASSERT_INT32_WITHIN(8, iq_seed, iq_ref);
}

TEST_CASE("speed PI cascade reaches 300 Hz without oscillation",
          "[espFoC][pid][speed]")
{
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(
        ESP_OK,
        esp_foc_pid_init(&pid, 0.0048f, 0.050f, 0.0f, 0.0f, 0.0005f));

    const q16_t omega_start = q16_from_float(2.0f * (float)M_PI * 50.0f);
    const q16_t omega_final = q16_from_float(2.0f * (float)M_PI * 300.0f);
    const q16_t iq_load = q16_from_float(0.03f);
    const q16_t iq_max = q16_from_float(0.45f);
    const q16_t current_alpha = q16_from_float(0.20f);
    const q16_t plant_step = q16_from_float(6500.0f * 0.0005f);
    const int ramp_steps = 16000;
    const int hold_steps = 8000;
    q16_t omega = omega_start;
    q16_t iq = iq_load;
    q16_t max_tail_error = 0;
    q16_t min_tail_error = INT32_MAX;

    esp_foc_pid_reset(&pid);
    esp_foc_pid_set_applied(&pid, iq_load);

    for (int i = 1; i <= ramp_steps + hold_steps; i++) {
        q16_t omega_ref = omega_final;
        if (i <= ramp_steps) {
            omega_ref = (q16_t)((int64_t)omega_start +
                                (((int64_t)omega_final - omega_start) * i) /
                                    ramp_steps);
        }
        q16_t iq_ref = esp_foc_pid_update(&pid, omega_ref, omega);
        iq_ref = q16_clamp(iq_ref, q16_neg(iq_max), iq_max);
        esp_foc_pid_set_applied(&pid, iq_ref);
        iq = q16_add(iq, q16_mul(current_alpha, q16_sub(iq_ref, iq)));
        omega = q16_add(omega, q16_mul(plant_step, q16_sub(iq, iq_load)));

        if (i > ramp_steps + hold_steps - 1000) {
            q16_t error = q16_sub(omega_final, omega);
            if (error > max_tail_error) {
                max_tail_error = error;
            }
            if (error < min_tail_error) {
                min_tail_error = error;
            }
        }
    }

    TEST_ASSERT_INT32_WITHIN(q16_from_float(1.0f), omega_final, omega);
    TEST_ASSERT_TRUE(q16_sub(max_tail_error, min_tail_error) <
                     q16_from_float(0.5f));
}

TEST_CASE("speed PI 400 Hz design holds 150 Hz without hunting",
          "[espFoC][pid][speed]")
{
    const float ts = 1.0f / 4000.0f;
    const float k_plant = 6500.0f;
    const float w_c = 2.0f * (float)M_PI * 400.0f;
    const float kp = w_c / k_plant;
    const float ki = kp * w_c / 6.0f;
    const float i_alpha_f = 1.0f - expf(-2.0f * (float)M_PI * 1000.0f * ts);
    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, kp, ki, 0.0f, 0.0f, ts));

    const q16_t omega_start = q16_from_float(2.0f * (float)M_PI * 81.0f);
    const q16_t omega_final = q16_from_float(2.0f * (float)M_PI * 150.0f);
    const q16_t iq_load = q16_from_float(0.03f);
    const q16_t iq_max = q16_from_float(0.45f);
    const q16_t current_alpha = q16_from_float(i_alpha_f);
    const q16_t plant_step = q16_from_float(k_plant * ts);
    const int ramp_steps = 800;
    const int hold_steps = 2000;
    q16_t omega = omega_start;
    q16_t iq = q16_from_float(0.22f);
    q16_t max_tail_error = 0;
    q16_t min_tail_error = INT32_MAX;

    esp_foc_pid_reset(&pid);
    esp_foc_pid_set_applied(&pid, iq);

    for (int i = 1; i <= ramp_steps + hold_steps; i++) {
        q16_t omega_ref = omega_final;
        if (i <= ramp_steps) {
            omega_ref = (q16_t)((int64_t)omega_start +
                                (((int64_t)omega_final - omega_start) * i) /
                                    ramp_steps);
        }
        q16_t iq_ref = esp_foc_pid_update(&pid, omega_ref, omega);
        iq_ref = q16_clamp(iq_ref, q16_neg(iq_max), iq_max);
        esp_foc_pid_set_applied(&pid, iq_ref);
        iq = q16_add(iq, q16_mul(current_alpha, q16_sub(iq_ref, iq)));
        omega = q16_add(omega, q16_mul(plant_step, q16_sub(iq, iq_load)));

        if (i > ramp_steps + hold_steps - 400) {
            q16_t error = q16_sub(omega_final, omega);
            if (error > max_tail_error) {
                max_tail_error = error;
            }
            if (error < min_tail_error) {
                min_tail_error = error;
            }
        }
    }

    TEST_ASSERT_INT32_WITHIN(q16_from_float(2.0f * (float)M_PI * 2.0f),
                             omega_final, omega);
    TEST_ASSERT_TRUE(q16_sub(max_tail_error, min_tail_error) <
                     q16_from_float(2.0f * (float)M_PI * 1.0f));
}

TEST_CASE("pid runtime set_kp changes response", "[espFoC][pid]")
{
    esp_foc_pid_t lo;
    esp_foc_pid_t hi;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&lo, 0.2f, 0.0f, 0.0f, 0.0f, 0.01f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&hi, 0.2f, 0.0f, 0.0f, 0.0f, 0.01f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_set_kp(&hi, 1.5f));
    q16_t u_lo = esp_foc_pid_update(&lo, Q16_ONE, 0);
    q16_t u_hi = esp_foc_pid_update(&hi, Q16_ONE, 0);
    TEST_ASSERT_TRUE(u_hi > u_lo);
}

TEST_CASE("pid imc_zoh rejects bad args", "[espFoC][pid]")
{
    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_imc_zoh(0.0f, 0.001f, 20000.0f, 1000.0f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_imc_zoh(6.3f, 0.0f, 20000.0f, 1000.0f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_imc_zoh(6.3f, 0.001f, 20000.0f, 12000.0f, &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_imc_zoh(6.3f, 0.001f, 20000.0f, 1000.0f, NULL, &ki));
}

TEST_CASE("pid imc_zoh Tustin matches C(z)=Kc(z-a)/(z-1)", "[espFoC][pid]")
{
    const float k = 12.0f / 1.9f;
    const float tau = 0.0005f / 1.9f;
    const float fs = 20000.0f;
    const float bw = 1000.0f;
    const float ts = 1.0f / fs;
    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_design_imc_zoh(k, tau, fs, bw, &kp, &ki));

    float a = expf(-ts / tau);
    float p = expf(-2.0f * (float)M_PI * bw * ts);
    float kc = (1.0f - p) / (k * (1.0f - a));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, kc * (1.0f + a) * 0.5f, kp);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-2f, kc * (1.0f - a) / ts, ki);

    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, kp, ki, 0.0f, 0.0f, ts));
    TEST_ASSERT_INT32_WITHIN(8, q16_from_float(kc), pid.b0);
    TEST_ASSERT_INT32_WITHIN(8, q16_from_float(-kc * a), pid.b1);
    TEST_ASSERT_EQUAL_INT32(q16_from_float(-1.0f), pid.a1);
}

TEST_CASE("pid imc_zoh closed loop settles without hunt", "[espFoC][pid]")
{
    const float k = 12.0f / 1.9f;
    const float tau = 0.0005f / 1.9f;
    const float fs = 20000.0f;
    const float bw = 1000.0f;
    const float ts = 1.0f / fs;
    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_design_imc_zoh(k, tau, fs, bw, &kp, &ki));

    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, kp, ki, 0.0f, 0.0f, ts));

    const q16_t a = q16_from_float(expf(-ts / tau));
    const q16_t b = q16_from_float(k * (1.0f - expf(-ts / tau)));
    q16_t y = 0;
    q16_t y_prev = 0;
    q16_t sp = q16_from_float(0.35f);
    int growing = 0;
    for (int i = 0; i < 200; i++) {
        q16_t u = esp_foc_pid_update(&pid, sp, y);
        y = q16_add(q16_mul(a, y), q16_mul(b, u));
        q16_t e = q16_sub(sp, y);
        q16_t e_prev = q16_sub(sp, y_prev);
        int32_t ae = e < 0 ? -e : e;
        int32_t aep = e_prev < 0 ? -e_prev : e_prev;
        if (i > 40 && ae > aep + 64) {
            growing++;
        }
        y_prev = y;
    }
    TEST_ASSERT_TRUE(growing < 8);
    TEST_ASSERT_INT32_WITHIN(q16_from_float(0.02f), sp, y);
}

TEST_CASE("pid integrator design rejects bad args", "[espFoC][pid]")
{
    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_integrator(0.0f, 20000.0f, 1000.0f, 0.70f,
                                                    &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_integrator(1.0f, 20000.0f, 12000.0f, 0.70f,
                                                    &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_integrator(1.0f, 20000.0f, 1000.0f, 0.20f,
                                                    &kp, &ki));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      esp_foc_pid_design_integrator(1.0f, 20000.0f, 1000.0f, 0.70f,
                                                    NULL, &ki));
}

TEST_CASE("pid integrator 1 kHz VCO tracks 50 Hz ramp without hunt",
          "[espFoC][pid]")
{
    const float fs = 20000.0f;
    const float ts = 1.0f / fs;
    const float bw = 1000.0f;
    float kp = 0.0f;
    float ki = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK,
                      esp_foc_pid_design_integrator(1.0f, fs, bw, 0.70f, &kp, &ki));
    TEST_ASSERT_TRUE(kp > 8000.0f && kp < 10000.0f);
    TEST_ASSERT_TRUE(ki > 3.5e7f && ki < 4.5e7f);

    esp_foc_pid_t pid;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pid, kp, ki, 0.0f, 0.0f, ts));

    const float w_tgt = 2.0f * (float)M_PI * 50.0f;
    const int ramp_n = (int)(0.05f * fs);
    float th_true = 0.0f;
    float th_hat = 0.0f;
    q16_t w_hat = 0;
    q16_t w_prev = 0;
    int growing = 0;
    for (int i = 0; i < 4000; i++) {
        float w_true = (i < ramp_n) ? (w_tgt * (float)i / (float)ramp_n) : w_tgt;
        float e = wrap_pi_f_local(th_true - th_hat);
        q16_t u = esp_foc_pid_update(&pid, q16_from_float(e), 0);
        u = q16_clamp(u, q16_from_float(-4000.0f), q16_from_float(4000.0f));
        esp_foc_pid_set_applied(&pid, u);
        w_hat = u;
        if (i > 2500) {
            q16_t dw = q16_sub(w_hat, w_prev);
            if (dw < 0) {
                dw = q16_neg(dw);
            }
            if (dw > q16_from_float(4.0f)) {
                growing++;
            }
        }
        w_prev = w_hat;
        th_hat += q16_to_float(w_hat) * ts;
        th_true += w_true * ts;
        while (th_hat > (float)M_PI) {
            th_hat -= 2.0f * (float)M_PI;
        }
        while (th_hat < -(float)M_PI) {
            th_hat += 2.0f * (float)M_PI;
        }
        while (th_true > (float)M_PI) {
            th_true -= 2.0f * (float)M_PI;
        }
        while (th_true < -(float)M_PI) {
            th_true += 2.0f * (float)M_PI;
        }
    }
    TEST_ASSERT_TRUE(growing < 20);
    TEST_ASSERT_FLOAT_WITHIN(25.0f, w_tgt, q16_to_float(w_hat));
}

TEST_CASE("pid PMSM decoupling ff is BEMF plus weL in pu of Vdc", "[espFoC][pid]")
{
    esp_foc_pid_t pd;
    esp_foc_pid_t pq;
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pd, 1.0f, 0.0f, 0.0f, 0.0f, 5e-5f));
    TEST_ASSERT_EQUAL(ESP_OK, esp_foc_pid_init(&pq, 1.0f, 0.0f, 0.0f, 0.0f, 5e-5f));

    const q16_t we = q16_from_float(2.0f * (float)M_PI * 144.0f);
    const q16_t id = q16_from_float(0.30f);
    const q16_t iq = q16_from_float(0.20f);
    const q16_t ls = q16_from_float(0.0005f);
    const q16_t psi = q16_from_float(0.002564f);
    const q16_t inv_vdc = q16_from_float(1.0f / 12.0f);
    esp_foc_pid_set_pmsm_ff(&pd, &pq, we, id, iq, ls, psi, inv_vdc);

    const float we_f = 2.0f * (float)M_PI * 144.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.002f, -we_f * 0.0005f * 0.20f / 12.0f,
                             q16_to_float(pd.ff));
    TEST_ASSERT_FLOAT_WITHIN(0.005f,
                             (we_f * 0.0005f * 0.30f + we_f * 0.002564f) / 12.0f,
                             q16_to_float(pq.ff));
}
