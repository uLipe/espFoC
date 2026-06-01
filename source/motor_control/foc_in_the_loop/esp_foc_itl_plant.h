/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Discretized PMSM plant (dq state, abc terminal voltages). Host-testable.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

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
    esp_foc_itl_plant_params_t p;
    float id_a;
    float iq_a;
    float omega_m_rad_s;
    float theta_m_rad;
    float iu_a;
    float iv_a;
    float iw_a;
} esp_foc_itl_plant_t;

void esp_foc_itl_plant_init(esp_foc_itl_plant_t *plant, const esp_foc_itl_plant_params_t *params);
void esp_foc_itl_plant_reset_parked(esp_foc_itl_plant_t *plant);
void esp_foc_itl_plant_duties_to_phase_volts(float duty_u, float duty_v, float duty_w,
                                              const esp_foc_itl_plant_t *plant,
                                              float *vu, float *vv, float *vw);
void esp_foc_itl_plant_step(esp_foc_itl_plant_t *plant, float vu, float vv, float vw, float dt_s);
int32_t esp_foc_itl_plant_encoder_ticks(const esp_foc_itl_plant_t *plant);

#ifdef __cplusplus
}
#endif
