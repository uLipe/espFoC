/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "espFoC/drivers/esp_foc_inverter.h"
#include "espFoC/drivers/esp_foc_encoder.h"
#include "espFoC/esp_foc_err.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float r_ohm;
    float l_henry;
    float j_kgm2;
    float b_nms;
    float kt_nm_per_a;
    int pole_pairs;
    q16_t vdc_q16;
    float i_max_a;
    bool locked_rotor;
    bool connection_delta;
    uint32_t pwm_hz;
} esp_foc_in_the_loop_config_t;

typedef struct {
    esp_foc_inverter_t *inverter;
    esp_foc_encoder_t *encoder;
} esp_foc_in_the_loop_handles_t;

esp_foc_err_t esp_foc_in_the_loop_create(const esp_foc_in_the_loop_config_t *cfg,
                                         esp_foc_in_the_loop_handles_t *out);
void esp_foc_in_the_loop_destroy(void);

#ifdef __cplusplus
}
#endif
