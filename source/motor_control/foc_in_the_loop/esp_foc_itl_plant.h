/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Discretized PMSM plant (dq state, abc terminal voltages). Hot path is Q16-only.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FOC_ITL_CPR  4096u

typedef struct {
    float r_ohm;
    float l_henry;
    float j_kgm2;
    float b_nms;
    float kt_nm_per_a;
    int pole_pairs;
    float vdc_volts;
    float i_max_a;
    bool locked_rotor;
    bool delta_connection;
} esp_foc_itl_plant_params_t;

typedef struct {
    bool locked_rotor;
    bool delta_connection;
    int pole_pairs;
    q16_t dt_q16;
    q16_t dt_half_q16;
    q16_t dt_over_6_q16;
    q16_t r_q16;
    q16_t l_q16;
    q16_t inv_l_q16;
    q16_t psi_f_q16;
    q16_t kt_q16;
    q16_t b_q16;
    q16_t j_q16;
    q16_t inv_j_q16;
    q16_t i_max_q16;
    q16_t vdc_q16;
    q16_t half_vdc_q16;
    q16_t delta_scale_q16;
} esp_foc_itl_plant_coeff_t;

typedef struct {
    esp_foc_itl_plant_coeff_t c;
    q16_t id_q16;
    q16_t iq_q16;
    q16_t omega_m_q16;
    q16_t theta_m_q16;
    q16_t iu_q16;
    q16_t iv_q16;
} esp_foc_itl_plant_t;

void esp_foc_itl_plant_init(esp_foc_itl_plant_t *plant, const esp_foc_itl_plant_params_t *params);
void esp_foc_itl_plant_set_dt(esp_foc_itl_plant_t *plant, uint32_t pwm_hz);
void esp_foc_itl_plant_reset_parked(esp_foc_itl_plant_t *plant);
void esp_foc_itl_plant_duties_to_phase_volts(q16_t duty_u, q16_t duty_v, q16_t duty_w,
                                              const esp_foc_itl_plant_t *plant,
                                              q16_t *vu, q16_t *vv, q16_t *vw);
void esp_foc_itl_plant_step(esp_foc_itl_plant_t *plant, q16_t vu, q16_t vv, q16_t vw);
int32_t esp_foc_itl_plant_encoder_ticks(const esp_foc_itl_plant_t *plant);

#ifdef __cplusplus
}
#endif
