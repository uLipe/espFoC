/*
 * Smoke tests: observers run update() with Q16 inputs only (API + link coverage).
 */
#include <unity.h>
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_simu_observer.h"
#include "espFoC/observer/esp_foc_pll_observer.h"
#include "espFoC/observer/esp_foc_kf_observer.h"
#include "espFoC/observer/esp_foc_pmsm_model_observer.h"

static void fill_inputs(esp_foc_observer_inputs_t *in, float dt_s)
{
    q16_t dt_q = q16_from_float(dt_s);
    q16_t inv_q = q16_from_float(1.0f / dt_s);
    in->u_dq[0] = 0;
    in->u_dq[1] = q16_from_float(0.1f);
    in->u_alpha_beta[0] = q16_from_float(0.05f);
    in->u_alpha_beta[1] = q16_from_float(0.05f);
    in->i_dq[0] = 0;
    in->i_dq[1] = q16_from_float(0.2f);
    in->i_alpha_beta[0] = q16_from_float(0.1f);
    in->i_alpha_beta[1] = q16_from_float(0.1f);
    in->dt = dt_q;
    in->inv_dt = inv_q;
}

TEST_CASE("simu_observer: Q16 update smoke", "[espFoC][observer]")
{
    const float dt = 1.0e-4f;
    esp_foc_simu_observer_settings_t s = {
        .phase_resistance = 0.5f,
        .phase_inductance = 0.001f,
        .pole_pairs = 4.0f,
        .dt = dt,
    };
    esp_foc_observer_t *obs = simu_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    esp_foc_observer_inputs_t in;
    fill_inputs(&in, dt);
    for (int i = 0; i < 20; i++) {
        (void)obs->update(obs, &in);
    }
    (void)obs->get_angle(obs);
    (void)obs->get_speed(obs);
    obs->reset(obs, q16_from_float(0.25f));
}

TEST_CASE("pll_observer: Q16 update smoke", "[espFoC][observer]")
{
    const float dt = 1.0e-4f;
    esp_foc_pll_observer_settings_t s = {
        .pll_kp = 100.0f,
        .pll_ki = 2000.0f,
        .phase_resistance = 0.5f,
        .phase_inductance = 0.001f,
        .pole_pairs = 4.0f,
        .flux_linkage = 0.015f,
        .inertia = 0.0002f,
        .friction = 0.001f,
        .dt = dt,
    };
    esp_foc_observer_t *obs = pll_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    esp_foc_observer_inputs_t in;
    fill_inputs(&in, dt);
    for (int i = 0; i < 20; i++) {
        (void)obs->update(obs, &in);
    }
    (void)obs->get_angle(obs);
    (void)obs->get_speed(obs);
    obs->reset(obs, 0);
}

TEST_CASE("kf_observer: Q16 update smoke", "[espFoC][observer]")
{
    const float dt = 1.0e-4f;
    esp_foc_kf_observer_settings_t s = {
        .phase_resistance = 0.5f,
        .phase_inductance = 0.001f,
        .pole_pairs = 4.0f,
        .dt = dt,
        .sigma_theta_meas = 0.05f,
        .sigma_accel = 500.0f,
        .omega_limit = 0.0f,
        .current_lpf_frac_fs = 0.2f,
    };
    esp_foc_observer_t *obs = kf_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    esp_foc_observer_inputs_t in;
    fill_inputs(&in, dt);
    for (int i = 0; i < 20; i++) {
        (void)obs->update(obs, &in);
    }
    (void)obs->get_angle(obs);
    (void)obs->get_speed(obs);
    obs->reset(obs, q16_from_float(0.5f));
}

TEST_CASE("pmsm_model_observer: Q16 update smoke", "[espFoC][observer]")
{
    const float dt = 1.0e-4f;
    esp_foc_pmsm_model_observer_settings_t s = {
        .phase_resistance = 0.5f,
        .Ld = 0.001f,
        .Lq = 0.0012f,
        .lambda = 0.015f,
        .inertia = 0.0002f,
        .friction = 0.001f,
        .load_torque = 0.0f,
        .pole_pairs = 4.0f,
        .dt = dt,
        .current_limit = 30.0f,
        .omega_limit = 500.0f,
    };
    esp_foc_observer_t *obs = pmsm_model_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    esp_foc_observer_inputs_t in;
    fill_inputs(&in, dt);
    for (int i = 0; i < 20; i++) {
        (void)obs->update(obs, &in);
    }
    (void)obs->get_angle(obs);
    (void)obs->get_speed(obs);
    obs->reset(obs, 0);
}
