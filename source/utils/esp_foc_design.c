/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "espFoC/utils/esp_foc_design.h"

#include <math.h>

#include "espFoC/utils/esp_foc_angle.h"

int64_t esp_foc_q32_from_double(double x)
{
    return (int64_t)llround(x * ESP_FOC_Q32_ONE);
}

int64_t esp_foc_q16_from_double(double x)
{
    return (int64_t)llround(x * 65536.0);
}

q16_t esp_foc_step_q16(float per_s, float tick_hz)
{
    const q16_t step = q16_from_float(per_s / tick_hz);
    return (step > 0) ? step : 1;
}

float esp_foc_base_speed_hz(float vdc_v, float psi_wb)
{
    return vdc_v * ESP_FOC_INV_SQRT3_F / (ESP_FOC_TWO_PI_F * psi_wb);
}

float esp_foc_speed_bw_noise_hz(float iq_budget_a, float ripple_rads, float k_rads2_a,
                                float zeta)
{
    return (iq_budget_a / ripple_rads) * k_rads2_a / (2.0f * zeta * ESP_FOC_TWO_PI_F);
}

float esp_foc_speed_bw_phase_hz(float pll_bw_hz, float sep_min, float zeta)
{
    return 0.5f * pll_bw_hz / sep_min / (2.0f * zeta);
}
