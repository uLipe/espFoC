/*
 * Backward compatibility: include the canonical Q16 units header.
 * New code should include espFoC/esp_foc_units_q16.h directly.
 */
#pragma once

#include "espFoC/esp_foc_units_q16.h"

typedef esp_foc_q_voltage_q16_t   esp_foc_q_voltage_iq31_t;
typedef esp_foc_d_voltage_q16_t   esp_foc_d_voltage_iq31_t;
typedef esp_foc_alpha_voltage_q16_t esp_foc_alpha_voltage_iq31_t;
typedef esp_foc_beta_voltage_q16_t  esp_foc_beta_voltage_iq31_t;
typedef esp_foc_u_voltage_q16_t   esp_foc_u_voltage_iq31_t;
typedef esp_foc_v_voltage_q16_t   esp_foc_v_voltage_iq31_t;
typedef esp_foc_w_voltage_q16_t   esp_foc_w_voltage_iq31_t;

typedef esp_foc_q_current_q16_t   esp_foc_q_current_iq31_t;
typedef esp_foc_d_current_q16_t   esp_foc_d_current_iq31_t;
typedef esp_foc_alpha_current_q16_t esp_foc_alpha_current_iq31_t;
typedef esp_foc_beta_current_q16_t  esp_foc_beta_current_iq31_t;
typedef esp_foc_u_current_q16_t   esp_foc_u_current_iq31_t;
typedef esp_foc_v_current_q16_t   esp_foc_v_current_iq31_t;
typedef esp_foc_w_current_q16_t   esp_foc_w_current_iq31_t;

typedef esp_foc_radians_per_second_q16_t esp_foc_radians_per_second_iq31_t;
typedef esp_foc_radians_q16_t   esp_foc_radians_iq31_t;
typedef esp_foc_seconds_q16_t   esp_foc_seconds_iq31_t;
