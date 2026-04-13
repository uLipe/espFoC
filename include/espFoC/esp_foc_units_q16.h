/*
 * MIT License
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

typedef struct { q16_t raw; } esp_foc_q_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_d_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_alpha_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_beta_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_u_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_v_voltage_q16_t;
typedef struct { q16_t raw; } esp_foc_w_voltage_q16_t;

typedef struct { q16_t raw; } esp_foc_q_current_q16_t;
typedef struct { q16_t raw; } esp_foc_d_current_q16_t;
typedef struct { q16_t raw; } esp_foc_alpha_current_q16_t;
typedef struct { q16_t raw; } esp_foc_beta_current_q16_t;
typedef struct { q16_t raw; } esp_foc_u_current_q16_t;
typedef struct { q16_t raw; } esp_foc_v_current_q16_t;
typedef struct { q16_t raw; } esp_foc_w_current_q16_t;

typedef struct { q16_t raw; } esp_foc_radians_per_second_q16_t;
typedef struct { q16_t raw; } esp_foc_radians_q16_t;
typedef struct { q16_t raw; } esp_foc_seconds_q16_t;
