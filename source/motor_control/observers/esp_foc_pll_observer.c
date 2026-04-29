/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * PLL angle observer — Q16 in update() (ISR-safe).
 */

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/esp_foc_int_sqrt.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/observer/esp_foc_pll_observer.h"

#define PLL_OBSERVER_CONVERGE_WAIT_TIME     1.0f
#define PLL_LOCK_VAR                        0.2f
#define PLL_INTEGRATOR_FREEZE_TIME          0.5f
#define PLL_MAG2_MIN                        0.5f

#ifndef PLL_DEFAULT_V_FULL
#define PLL_DEFAULT_V_FULL 24.0f
#endif
#ifndef PLL_DEFAULT_I_FULL
#define PLL_DEFAULT_I_FULL 20.0f
#endif

typedef struct {
    q16_t theta_est;
    q16_t omega_est;
    q16_t theta_error;
    esp_foc_biquad_q16_t current_filters[2];
    esp_foc_biquad_q16_t theta_filter;
    q16_t e_mag2;
    q16_t e_alpha;
    q16_t e_beta;
    q16_t i_alpha_prev;
    q16_t i_beta_prev;
    q16_t integral;
    q16_t kp_q16;
    q16_t ki_dt_q16;
    q16_t r_q16;
    q16_t l_q16;
    q16_t k_dtheta_rad_q16;
    q16_t mag_eps_q16;
    q16_t mag2_min_q16;
    q16_t lock_var_q16;
    q16_t integral_decay_q16;
    int converging_count;
    int integrator_freeze_time;
    q16_t prev_err;
    int converge_period_ticks;
    int integrator_freeze_ticks;
    esp_foc_observer_t interface;
} angle_estimator_pll_t;

static const char *TAG = "ESP-FOC-PLL-OBS";
static DRAM_ATTR angle_estimator_pll_t pll_observers[CONFIG_NOOF_AXIS];

static int pll_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);

    q16_t ia = in->i_alpha_beta[0];
    q16_t ib = in->i_alpha_beta[1];
    q16_t ua = in->u_alpha_beta[0];
    q16_t ub = in->u_alpha_beta[1];

    q16_t delta_ia = q16_sub(ia, est->i_alpha_prev);
    q16_t delta_ib = q16_sub(ib, est->i_beta_prev);

    q16_t ldi_a = q16_mul(est->l_q16, delta_ia);
    q16_t ldi_b = q16_mul(est->l_q16, delta_ib);

    q16_t raw_a = q16_sub(ua, q16_add(q16_mul(est->r_q16, ia), ldi_a));
    q16_t raw_b = q16_sub(ub, q16_add(q16_mul(est->r_q16, ib), ldi_b));

    est->e_alpha = esp_foc_biquad_q16_update(&est->current_filters[0], raw_a);
    est->e_beta = esp_foc_biquad_q16_update(&est->current_filters[1], raw_b);

    est->i_alpha_prev = ia;
    est->i_beta_prev = ib;

    q16_t sn = q16_sin(est->theta_est);
    q16_t cs = q16_cos(est->theta_est);
    q16_t mix = q16_add(q16_mul(est->e_alpha, q16_sub(0, sn)), q16_mul(est->e_beta, cs));

    est->e_mag2 = q16_add(q16_mul(est->e_alpha, est->e_alpha), q16_mul(est->e_beta, est->e_beta));

    if (est->e_mag2 < est->mag2_min_q16) {
        est->theta_error = 0;
        est->integral = q16_mul(est->integral, est->integral_decay_q16);
    } else {
        int64_t ea64 = (int64_t)est->e_alpha;
        int64_t eb64 = (int64_t)est->e_beta;
        uint64_t av = (uint64_t)(ea64 >= 0 ? ea64 : -ea64);
        uint64_t aq = (uint64_t)(eb64 >= 0 ? eb64 : -eb64);
        uint64_t m2 = av * av + aq * aq;
        int64_t me64 = (int64_t)est->mag_eps_q16;
        uint64_t me = (uint64_t)(me64 >= 0 ? me64 : -me64);
        uint64_t m2e = m2 + me;

        uint64_t inv_u64 = esp_foc_u64_rsqrt(m2e);
        if (inv_u64 > (uint64_t)INT32_MAX) {
            inv_u64 = (uint64_t)INT32_MAX;
        }
        q16_t inv = (q16_t)inv_u64;
        est->theta_error = q16_mul(mix, inv);
        est->theta_error = esp_foc_biquad_q16_update(&est->theta_filter, est->theta_error);
        est->integral = q16_add(est->integral, q16_mul(est->theta_error, est->ki_dt_q16));
    }

    est->omega_est = q16_add(q16_mul(est->kp_q16, est->theta_error), est->integral);
    est->theta_est = q16_normalize_angle_rad(
        q16_add(est->theta_est, q16_mul(est->omega_est, est->k_dtheta_rad_q16)));

    if ((q16_abs(q16_sub(est->theta_error, est->prev_err)) <= est->lock_var_q16) &&
        (est->e_mag2 >= est->mag2_min_q16)) {
        est->converging_count--;
    } else {
        est->converging_count = est->converge_period_ticks;
    }

    est->prev_err = est->theta_error;

    if (est->integrator_freeze_time) {
        est->integrator_freeze_time--;
    }

    return est->converging_count;
}

static q16_t pll_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->theta_est;
}

static q16_t pll_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->omega_est;
}

static void pll_observer_reset(esp_foc_observer_t *self, q16_t offset)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    est->integral = 0;
    est->theta_est = q16_normalize_angle_rad(q16_add(est->theta_est, offset));
    est->integrator_freeze_time = est->integrator_freeze_ticks;
}

esp_foc_observer_t *pll_observer_new(int unit, esp_foc_pll_observer_settings_t settings)
{
    if (unit >= CONFIG_NOOF_AXIS) {
        ESP_LOGE(TAG, "Invalid unit!");
        return NULL;
    }

    if (settings.dt == 0.0f) {
        ESP_LOGE(TAG, "Invalid dt!");
        return NULL;
    }

    if (settings.phase_resistance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase resistance !");
        return NULL;
    }

    if (settings.phase_inductance <= 0.0f) {
        ESP_LOGE(TAG, "Invalid phase Inductance !");
        return NULL;
    }

    if (settings.pll_kp <= 0.0f) {
        ESP_LOGE(TAG, "Invalid proportional gain !");
        return NULL;
    }

    if (settings.pll_ki < 0.0f) {
        ESP_LOGE(TAG, "Invalid integral gain !");
        return NULL;
    }

    const float Vb = PLL_DEFAULT_V_FULL;
    const float Ib = PLL_DEFAULT_I_FULL;

    angle_estimator_pll_t *est = &pll_observers[unit];
    est->theta_est = 0;
    est->omega_est = 0;
    est->theta_error = 0;
    est->e_alpha = 0;
    est->e_beta = 0;
    est->i_alpha_prev = 0;
    est->i_beta_prev = 0;
    est->integral = 0;
    est->prev_err = 0;

    est->r_q16 = q16_from_float((settings.phase_resistance * Ib) / Vb);
    est->l_q16 = q16_from_float((settings.phase_inductance * Ib) / (Vb * settings.dt));

    est->kp_q16 = q16_from_float(settings.pll_kp / ESP_FOC_OBS_OMEGA_MAX_RAD_S);
    est->ki_dt_q16 = q16_from_float((settings.pll_ki * settings.dt) / ESP_FOC_OBS_OMEGA_MAX_RAD_S);

    est->k_dtheta_rad_q16 = q16_from_float(ESP_FOC_OBS_OMEGA_MAX_RAD_S * settings.dt);

    est->mag_eps_q16 = q16_from_float(0.0001f);
    est->mag2_min_q16 = q16_from_float(PLL_MAG2_MIN);
    est->lock_var_q16 = q16_from_float(PLL_LOCK_VAR);
    est->integral_decay_q16 = q16_from_float(0.9999f);

    {
        float fs_hz = 1.0f / settings.dt;
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[0],
                                                  0.05f * fs_hz, fs_hz);
        esp_foc_biquad_butterworth_lpf_design_q16(&est->current_filters[1],
                                                  0.05f * fs_hz, fs_hz);
        esp_foc_biquad_butterworth_lpf_design_q16(&est->theta_filter,
                                                  ESP_FOC_PLL_BANDWIDTH_HZ, fs_hz);
    }

    est->converge_period_ticks = (int)(PLL_OBSERVER_CONVERGE_WAIT_TIME / settings.dt);
    est->converging_count = est->converge_period_ticks;
    est->integrator_freeze_ticks = (int)(PLL_INTEGRATOR_FREEZE_TIME / settings.dt);

    est->interface.update = pll_observer_update;
    est->interface.get_angle = pll_observer_get_angle;
    est->interface.get_speed = pll_observer_get_speed;
    est->interface.reset = pll_observer_reset;

    ESP_LOGI(TAG, "Observer sample time %f s (Q16)", (double)settings.dt);

    return &est->interface;
}
