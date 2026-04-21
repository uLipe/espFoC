/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * PLL angle observer — IQ31 only in update() (ISR-safe).
 * pll_observer_new() may use float for logging and coefficient conversion.
 */

#include <math.h>
#include <esp_attr.h>
#include <sdkconfig.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/utils/biquad_q16.h"
#include "espFoC/utils/esp_foc_iq31_q16_bridge.h"
#include "espFoC/observer/esp_foc_observer_interface.h"
#include "espFoC/utils/foc_math_iq31.h"
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
    iq31_t theta_est;
    iq31_t omega_est;
    iq31_t theta_error;
    esp_foc_biquad_q16_t current_filters[2];
    esp_foc_biquad_q16_t theta_filter;
    iq31_t e_mag2;
    iq31_t e_alpha;
    iq31_t e_beta;
    iq31_t i_alpha_prev;
    iq31_t i_beta_prev;
    iq31_t integral;
    iq31_t kp_q31;
    iq31_t ki_dt_q31;
    iq31_t r_q31;
    iq31_t l_q31;
    iq31_t dt_q31;
    iq31_t k_theta_q31;
    iq31_t mag_eps_q31;
    iq31_t mag2_min_q31;
    iq31_t lock_var_q31;
    iq31_t integral_decay_q31;
    int converging_count;
    int integrator_freeze_time;
    iq31_t prev_err;
    int converge_period_ticks;
    int integrator_freeze_ticks;
    esp_foc_observer_t interface;
} angle_estimator_pll_t;

static const char *TAG = "ESP-FOC-PLL-OBS";
static DRAM_ATTR angle_estimator_pll_t pll_observers[CONFIG_NOOF_AXIS];

static int pll_observer_update(esp_foc_observer_t *self, esp_foc_observer_inputs_t *in)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);

    iq31_t ia = in->i_alpha_beta[0];
    iq31_t ib = in->i_alpha_beta[1];
    iq31_t ua = in->u_alpha_beta[0];
    iq31_t ub = in->u_alpha_beta[1];

    iq31_t delta_ia = iq31_sub(ia, est->i_alpha_prev);
    iq31_t delta_ib = iq31_sub(ib, est->i_beta_prev);

    iq31_t ldi_a = iq31_mul(est->l_q31, delta_ia);
    iq31_t ldi_b = iq31_mul(est->l_q31, delta_ib);

    iq31_t raw_a = iq31_sub(ua, iq31_add(iq31_mul(est->r_q31, ia), ldi_a));
    iq31_t raw_b = iq31_sub(ub, iq31_add(iq31_mul(est->r_q31, ib), ldi_b));

    q16_t ea_q = esp_foc_biquad_q16_update(&est->current_filters[0],
                                            esp_foc_iq31_per_unit_to_q16(raw_a));
    q16_t eb_q = esp_foc_biquad_q16_update(&est->current_filters[1],
                                            esp_foc_iq31_per_unit_to_q16(raw_b));
    est->e_alpha = esp_foc_q16_per_unit_to_iq31(ea_q);
    est->e_beta = esp_foc_q16_per_unit_to_iq31(eb_q);

    est->i_alpha_prev = ia;
    est->i_beta_prev = ib;

    iq31_t sn = iq31_sin(est->theta_est);
    iq31_t cs = iq31_cos(est->theta_est);
    iq31_t mix = iq31_add(iq31_mul(est->e_alpha, iq31_sub(0, sn)), iq31_mul(est->e_beta, cs));

    est->e_mag2 = iq31_add(iq31_mul(est->e_alpha, est->e_alpha), iq31_mul(est->e_beta, est->e_beta));

    if (est->e_mag2 < est->mag2_min_q31) {
        est->theta_error = 0;
        est->integral = iq31_mul(est->integral, est->integral_decay_q31);
    } else {
        iq31_t rs = iq31_rsqrt_fast(iq31_add(est->e_mag2, est->mag_eps_q31));
        est->theta_error = iq31_mul_q230(mix, rs);
        est->theta_error = esp_foc_q16_per_unit_to_iq31(
            esp_foc_biquad_q16_update(&est->theta_filter,
                                      esp_foc_iq31_per_unit_to_q16(est->theta_error)));
        est->integral = iq31_add(est->integral, iq31_mul(est->theta_error, est->ki_dt_q31));
    }

    est->omega_est = iq31_add(iq31_mul(est->kp_q31, est->theta_error), est->integral);
    est->theta_est = iq31_normalize_angle(iq31_add(est->theta_est, iq31_mul(est->omega_est, est->k_theta_q31)));

    if ((iq31_abs(iq31_sub(est->theta_error, est->prev_err)) <= est->lock_var_q31) &&
        (est->e_mag2 >= est->mag2_min_q31)) {
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

static iq31_t pll_observer_get_angle(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->theta_est;
}

static iq31_t pll_observer_get_speed(esp_foc_observer_t *self)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    return est->omega_est;
}

static void pll_observer_reset(esp_foc_observer_t *self, iq31_t offset)
{
    angle_estimator_pll_t *est = __containerof(self, angle_estimator_pll_t, interface);
    est->integral = 0;
    est->theta_est = iq31_normalize_angle(iq31_add(est->theta_est, offset));
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

    est->dt_q31 = iq31_from_float(settings.dt);

    est->r_q31 = iq31_from_float((settings.phase_resistance * Ib) / Vb);
    est->l_q31 = iq31_from_float((settings.phase_inductance * Ib) / (Vb * settings.dt));

    est->kp_q31 = iq31_from_float(settings.pll_kp / ESP_FOC_OBS_OMEGA_MAX_RAD_S);
    est->ki_dt_q31 = iq31_from_float((settings.pll_ki * settings.dt) / ESP_FOC_OBS_OMEGA_MAX_RAD_S);

    est->k_theta_q31 = iq31_from_float((ESP_FOC_OBS_OMEGA_MAX_RAD_S * settings.dt) / (2.0f * (float)M_PI));

    est->mag_eps_q31 = iq31_from_float(0.0001f);
    est->mag2_min_q31 = iq31_from_float(PLL_MAG2_MIN);
    est->lock_var_q31 = iq31_from_float(PLL_LOCK_VAR);
    est->integral_decay_q31 = iq31_from_float(0.9999f);

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

    ESP_LOGI(TAG, "Observer sample time %f s (IQ31)", (double)settings.dt);

    return &est->interface;
}
