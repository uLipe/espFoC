/*
 * Trajectory tests: reference model in float (same physics as observers) stepped in
 * parallel with IQ31 observers; asserts outputs stay coherent vs reference.
 *
 * FoC context: PLL/KF use voltage model (u - R*i - L*di/dt); we synthesize alpha-beta
 * waveforms from a known electrical angle and dq setpoints so the observer can lock.
 */

#include <math.h>
#include <unity.h>
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_simu_observer.h"
#include "espFoC/observer/esp_foc_pll_observer.h"
#include "espFoC/observer/esp_foc_kf_observer.h"
#include "espFoC/observer/esp_foc_pmsm_model_observer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --- Must match esp_foc_simu_observer.c --- */
#define REF_SIMUL_FLUX_LINKAGE 0.015f
#define REF_SIMUL_INERTIA       0.00024f
#define REF_SIMUL_FRICTION      0.001f
#define REF_SIMU_V_FULL         24.0f
#define REF_SIMU_I_FULL         20.0f
#define REF_SIMU_T_NOM_NM       10.0f

static float wrap_pi(float x)
{
    while (x > (float)M_PI) {
        x -= 2.0f * (float)M_PI;
    }
    while (x < -(float)M_PI) {
        x += 2.0f * (float)M_PI;
    }
    return x;
}

/** Error between two angles on [0,1) circle (normalized revolutions). */
static float angle_norm_err01(float a, float b)
{
    float e = fabsf(a - b);
    if (e > 0.5f) {
        e = 1.0f - e;
    }
    return e;
}

/** iq31 angle [0,1) -> radians [0, 2pi) */
static float iq31_angle_to_rad(iq31_t a)
{
    return iq31_to_float(a) * 2.0f * (float)M_PI;
}

static void simu_float_ref_step(float uq_norm, float dt, float R, float L, float pp,
                                float *estim_iq, float *omega, float *angle_norm)
{
    const float Vb = REF_SIMU_V_FULL;
    const float Ib = REF_SIMU_I_FULL;
    float ri = (R * Ib / Vb) * (*estim_iq);
    float fw = (REF_SIMUL_FLUX_LINKAGE * ESP_FOC_OBS_OMEGA_MAX_RAD_S / Vb) * (*omega);
    float num = uq_norm - ri - fw;
    float d_iq = (dt / L) * (Vb / Ib) * num;
    *estim_iq += d_iq;

    float Te_coeff = 1.5f * pp * REF_SIMUL_FLUX_LINKAGE * Ib;
    float Te_u = (Te_coeff * (*estim_iq)) / REF_SIMU_T_NOM_NM;
    float Tf_u = ((REF_SIMUL_FRICTION * ESP_FOC_OBS_OMEGA_MAX_RAD_S) / REF_SIMU_T_NOM_NM) * (*omega);
    float dw = ((REF_SIMU_T_NOM_NM * dt) / (REF_SIMUL_INERTIA * ESP_FOC_OBS_OMEGA_MAX_RAD_S)) * (Te_u - Tf_u);
    *omega += dw;

    float k_ang = (ESP_FOC_OBS_OMEGA_MAX_RAD_S * dt) / (2.0f * (float)M_PI);
    *angle_norm += (*omega) * k_ang;
    *angle_norm -= floorf(*angle_norm);
}

TEST_CASE("simu_observer: tracks float reference (constant Uq trajectory)", "[espFoC][observer][trajectory]")
{
    const float dt = 1.0e-4f;
    const float R = 0.5f;
    const float L = 0.001f;
    const float pp = 4.0f;
    const float uq = 0.18f;

    esp_foc_simu_observer_settings_t s = {
        .phase_resistance = R,
        .phase_inductance = L,
        .pole_pairs = pp,
        .dt = dt,
    };
    esp_foc_observer_t *obs = simu_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    float ref_iq = 0.0f;
    float ref_w = 0.0f;
    float ref_ang = 0.0f;

    esp_foc_observer_inputs_t in;
    iq31_t dt_q = iq31_from_float(dt);
    iq31_t inv_q = iq31_from_float(1.0f / dt);

    const int n = 800;
    for (int i = 0; i < n; i++) {
        in.u_dq[0] = 0;
        in.u_dq[1] = iq31_from_float(uq);
        in.u_alpha_beta[0] = 0;
        in.u_alpha_beta[1] = 0;
        in.i_dq[0] = 0;
        in.i_dq[1] = 0;
        in.i_alpha_beta[0] = 0;
        in.i_alpha_beta[1] = 0;
        in.dt = dt_q;
        in.inv_dt = inv_q;

        simu_float_ref_step(uq, dt, R, L, pp, &ref_iq, &ref_w, &ref_ang);
        (void)obs->update(obs, &in);
    }

    float obs_w = iq31_to_float(obs->get_speed(obs));
    float ang_q = iq31_to_float(obs->get_angle(obs));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, ref_w, obs_w);
    TEST_ASSERT_LESS_THAN_FLOAT(0.02f, angle_norm_err01(ref_ang, ang_q));

    /* Ramp Uq down and check both stay close */
    const float uq2 = 0.05f;
    for (int i = 0; i < 400; i++) {
        in.u_dq[1] = iq31_from_float(uq2);
        simu_float_ref_step(uq2, dt, R, L, pp, &ref_iq, &ref_w, &ref_ang);
        (void)obs->update(obs, &in);
    }
    obs_w = iq31_to_float(obs->get_speed(obs));
    ang_q = iq31_to_float(obs->get_angle(obs));
    TEST_ASSERT_FLOAT_WITHIN(0.03f, ref_w, obs_w);
    TEST_ASSERT_LESS_THAN_FLOAT(0.03f, angle_norm_err01(ref_ang, ang_q));
}

/** Inverse Park d,q -> alpha, beta */
static void inv_park_dq_to_ab(float s, float c, float d, float q, float *a, float *b)
{
    *a = d * c - q * s;
    *b = d * s + q * c;
}

/**
 * Synthesizes motor-consistent u_ab, i_ab for constant electrical speed omega_e_phys [rad/s],
 * constant id, iq in amperes. theta_e advances each step.
 */
static void synth_stationary_motor_ab(float theta_e_rad, float omega_e_phys,
                                      float R, float Ld, float Lq, float lambda,
                                      float id_a, float iq_a,
                                      float *ua, float *ub, float *ia, float *ib)
{
    float c = cosf(theta_e_rad);
    float s = sinf(theta_e_rad);
    float vd = R * id_a - omega_e_phys * Lq * iq_a;
    float vq = R * iq_a + omega_e_phys * Ld * id_a + omega_e_phys * lambda;
    inv_park_dq_to_ab(s, c, vd, vq, ua, ub);
    inv_park_dq_to_ab(s, c, id_a, iq_a, ia, ib);
}

TEST_CASE("pll_observer: locks to synthetic rotating trajectory", "[espFoC][observer][trajectory]")
{
    const float dt = 1.0e-4f;
    const float R = 0.5f;
    const float L = 0.001f;
    const float omega_e_phys = 350.0f;
    const float id_a = 0.0f;
    const float iq_a = 4.0f;
    const float Ld = L;
    const float Lq = L;
    const float lambda = 0.015f;

    esp_foc_pll_observer_settings_t ps = {
        .pll_kp = 180.0f,
        .pll_ki = 8000.0f,
        .phase_resistance = R,
        .phase_inductance = L,
        .pole_pairs = 4.0f,
        .flux_linkage = lambda,
        .inertia = 0.0002f,
        .friction = 0.001f,
        .dt = dt,
    };
    esp_foc_observer_t *obs = pll_observer_new(0, ps);
    TEST_ASSERT_NOT_NULL(obs);

    iq31_t dt_q = iq31_from_float(dt);
    iq31_t inv_q = iq31_from_float(1.0f / dt);
    esp_foc_observer_inputs_t in;

    float theta = 0.0f;
    const int warmup = 2500;
    const int run = 500;
    float max_err_rad = 0.0f;
    float max_w_err = 0.0f;

    for (int i = 0; i < warmup + run; i++) {
        float ua, ub, ia, ib;
        synth_stationary_motor_ab(theta, omega_e_phys, R, Ld, Lq, lambda, id_a, iq_a, &ua, &ub, &ia, &ib);

        in.u_alpha_beta[0] = iq31_from_float(ua / REF_SIMU_V_FULL);
        in.u_alpha_beta[1] = iq31_from_float(ub / REF_SIMU_V_FULL);
        in.i_alpha_beta[0] = iq31_from_float(ia / REF_SIMU_I_FULL);
        in.i_alpha_beta[1] = iq31_from_float(ib / REF_SIMU_I_FULL);
        in.u_dq[0] = 0;
        in.u_dq[1] = 0;
        in.i_dq[0] = 0;
        in.i_dq[1] = 0;
        in.dt = dt_q;
        in.inv_dt = inv_q;

        (void)obs->update(obs, &in);
        theta += omega_e_phys * dt;
        theta -= floorf(theta / (2.0f * (float)M_PI)) * (2.0f * (float)M_PI);

        if (i >= warmup) {
            float th_obs = iq31_angle_to_rad(obs->get_angle(obs));
            float err = fabsf(wrap_pi(th_obs - theta));
            if (err > max_err_rad) {
                max_err_rad = err;
            }
            float w_obs = iq31_to_float(obs->get_speed(obs)) * ESP_FOC_OBS_OMEGA_MAX_RAD_S;
            float we = fabsf(w_obs - omega_e_phys);
            if (we > max_w_err) {
                max_w_err = we;
            }
        }
    }
    TEST_ASSERT_LESS_THAN_FLOAT(0.35f, max_err_rad);
    TEST_ASSERT_LESS_THAN_FLOAT(80.0f, max_w_err);
}

TEST_CASE("kf_observer: tracks synthetic trajectory (relaxed)", "[espFoC][observer][trajectory]")
{
    const float dt = 1.0e-4f;
    const float R = 0.5f;
    const float L = 0.001f;
    const float omega_e_phys = 280.0f;
    const float id_a = 0.0f;
    const float iq_a = 3.5f;
    const float Ld = L;
    const float Lq = L;
    const float lambda = 0.015f;

    esp_foc_kf_observer_settings_t ks = {
        .phase_resistance = R,
        .phase_inductance = L,
        .pole_pairs = 4.0f,
        .dt = dt,
        .sigma_theta_meas = 0.08f,
        .sigma_accel = 1200.0f,
        .omega_limit = 0.0f,
        .current_lpf_frac_fs = 0.25f,
    };
    esp_foc_observer_t *obs = kf_observer_new(0, ks);
    TEST_ASSERT_NOT_NULL(obs);

    iq31_t dt_q = iq31_from_float(dt);
    iq31_t inv_q = iq31_from_float(1.0f / dt);
    esp_foc_observer_inputs_t in;

    float theta = 0.0f;
    const int warmup = 3000;

    for (int i = 0; i < warmup; i++) {
        float ua, ub, ia, ib;
        synth_stationary_motor_ab(theta, omega_e_phys, R, Ld, Lq, lambda, id_a, iq_a, &ua, &ub, &ia, &ib);
        in.u_alpha_beta[0] = iq31_from_float(ua / REF_SIMU_V_FULL);
        in.u_alpha_beta[1] = iq31_from_float(ub / REF_SIMU_V_FULL);
        in.i_alpha_beta[0] = iq31_from_float(ia / REF_SIMU_I_FULL);
        in.i_alpha_beta[1] = iq31_from_float(ib / REF_SIMU_I_FULL);
        in.u_dq[0] = 0;
        in.u_dq[1] = 0;
        in.i_dq[0] = 0;
        in.i_dq[1] = 0;
        in.dt = dt_q;
        in.inv_dt = inv_q;
        (void)obs->update(obs, &in);
        theta += omega_e_phys * dt;
        theta -= floorf(theta / (2.0f * (float)M_PI)) * (2.0f * (float)M_PI);
    }

    float th_obs = iq31_angle_to_rad(obs->get_angle(obs));
    float err = fabsf(wrap_pi(th_obs - theta));
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, err);
    float w_obs = iq31_to_float(obs->get_speed(obs)) * ESP_FOC_OBS_OMEGA_MAX_RAD_S;
    TEST_ASSERT_FLOAT_WITHIN(120.0f, omega_e_phys, w_obs);
}

static void pmsm_float_ref_step(float ua_n, float ub_n, float dt,
                                float R, float Ld, float Lq, float lambda,
                                float J, float B, float Tl, float pp,
                                float Vb, float Ib,
                                float *id, float *iq, float *omega_e, float *theta_e)
{
    float th = *theta_e;
    float c = cosf(th);
    float s = sinf(th);
    float alpha = ua_n * Vb;
    float beta = ub_n * Vb;
    float vd = alpha * c + beta * s;
    float vq = beta * c - alpha * s;

    float id_p = *id * Ib;
    float iq_p = *iq * Ib;
    float we = (*omega_e) * ESP_FOC_OBS_OMEGA_MAX_RAD_S;

    float did_dt = (vd - R * id_p + we * Lq * iq_p) / Ld;
    float diq_dt = (vq - R * iq_p - we * Ld * id_p - we * lambda) / Lq;
    id_p += did_dt * dt;
    iq_p += diq_dt * dt;
    *id = id_p / Ib;
    *iq = iq_p / Ib;

    float Te = 1.5f * pp * (lambda * iq_p + (Ld - Lq) * id_p * iq_p);
    float dwe = dt * ((pp / J) * (Te - Tl) - (B / J) * we);
    we += dwe;
    *omega_e = we / ESP_FOC_OBS_OMEGA_MAX_RAD_S;

    th += we * dt;
    th -= floorf(th / (2.0f * (float)M_PI)) * (2.0f * (float)M_PI);
    *theta_e = th;
}

TEST_CASE("pmsm_model_observer: matches float Euler reference", "[espFoC][observer][trajectory]")
{
    const float dt = 5.0e-5f;
    esp_foc_pmsm_model_observer_settings_t s = {
        .phase_resistance = 0.5f,
        .Ld = 0.001f,
        .Lq = 0.0011f,
        .lambda = 0.015f,
        .inertia = 0.0002f,
        .friction = 0.001f,
        .load_torque = 0.02f,
        .pole_pairs = 4.0f,
        .dt = dt,
        .current_limit = 0.0f,
        .omega_limit = 0.0f,
    };
    const float Vb = 24.0f;
    const float Ib = 20.0f;

    esp_foc_observer_t *obs = pmsm_model_observer_new(0, s);
    TEST_ASSERT_NOT_NULL(obs);

    float rid = 0.0f;
    float riq = 0.0f;
    float rw = 0.0f;
    float rth = 0.0f;

    iq31_t dt_q = iq31_from_float(dt);
    iq31_t inv_q = iq31_from_float(1.0f / dt);
    esp_foc_observer_inputs_t in;

    const int n = 1200;
    for (int i = 0; i < n; i++) {
        float t = (float)i * dt;
        float ua_n = 0.12f * cosf(2.0f * (float)M_PI * 30.0f * t);
        float ub_n = 0.12f * sinf(2.0f * (float)M_PI * 30.0f * t);

        in.u_alpha_beta[0] = iq31_from_float(ua_n);
        in.u_alpha_beta[1] = iq31_from_float(ub_n);
        in.u_dq[0] = 0;
        in.u_dq[1] = 0;
        in.i_dq[0] = 0;
        in.i_dq[1] = 0;
        in.i_alpha_beta[0] = 0;
        in.i_alpha_beta[1] = 0;
        in.dt = dt_q;
        in.inv_dt = inv_q;

        pmsm_float_ref_step(ua_n, ub_n, dt, s.phase_resistance, s.Ld, s.Lq, s.lambda,
                            s.inertia, s.friction, s.load_torque, s.pole_pairs, Vb, Ib,
                            &rid, &riq, &rw, &rth);
        (void)obs->update(obs, &in);
    }

    float obs_w = iq31_to_float(obs->get_speed(obs));
    float obs_th = iq31_angle_to_rad(obs->get_angle(obs));

    TEST_ASSERT_FLOAT_WITHIN(0.05f, rw, obs_w);
    TEST_ASSERT_LESS_THAN_FLOAT(0.25f, fabsf(wrap_pi(obs_th - rth)));
}
