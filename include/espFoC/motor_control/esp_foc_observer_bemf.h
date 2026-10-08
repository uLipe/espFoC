/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#pragma once

#include "esp_err.h"
#include "espFoC/motor_control/esp_foc_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Voltage-model BEMF observer (ê = v − Rs i − Ls di/dt, LPF'd) plus atan2/PLL.
 *
 * This is the original mid/high-speed estimator. A high-gain current-model ê
 * copies the inverter voltage; do not use those poles as the angle source.
 * θ̂ is electrical. ω* does not enter the VCO.
 */
typedef struct {
    float rs_ohm;
    float ls_h;
    float psi_f_wb;
    float ts_s;
    float current_model_hz;
    float emf_lpf_hz;
    float pll_bw_hz;
    float pll_zeta;
    float pll_kp;
    float pll_ki;
    float e_lock_min_v;
    float w_max_rads;
    uint16_t lock_count;
    uint16_t unlock_count;
    esp_foc_angle_extract_t extract;
} esp_foc_observer_bemf_config_t;

typedef struct {
    esp_foc_observer_t iface;
    q16_t alpha;
    q16_t beta;
    q16_t ts_over_l;
    q16_t current_corr;
    q16_t emf_corr;
    q16_t current_wc;
    q16_t e_lock_min;
    q16_t inv_ts;
    q16_t ts;
    q16_t pll_kp;
    q16_t pll_ki_ts;
    q16_t w_max;
    q16_t lpf_wc;
    q16_t comp_b0;
    q16_t w_slow;
    q16_t theta_comp;
    q16_t w_sign_hyst;
    q16_t i_hat_a;
    q16_t i_hat_b;
    q16_t emf_hat_a;
    q16_t emf_hat_b;
    q16_t e_a;
    q16_t e_b;
    q16_t theta;
    q16_t omega;
    q16_t pll_integ;
    q16_t theta_prev;
    q16_t lpf_b0;
    q16_t lpf_w_a;
    q16_t lpf_w_b;
    uint16_t lock_need;
    uint16_t unlock_need;
    uint16_t lock_acc;
    uint16_t unlock_acc;
    bool locked;
    bool reverse;
    bool have_theta_prev;
    bool have_i_prev;
    bool pll_enable;
    esp_foc_angle_extract_t extract;
} esp_foc_observer_bemf_t;

esp_err_t esp_foc_observer_bemf_init(esp_foc_observer_bemf_t *o,
                                     const esp_foc_observer_bemf_config_t *cfg);

#ifdef __cplusplus
}
#endif
