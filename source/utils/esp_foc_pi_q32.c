/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 */
#include "espFoC/utils/esp_foc_pi_q32.h"

#include <stddef.h>

#include "espFoC/utils/esp_foc_design.h"

esp_err_t esp_foc_pi_q32_set_gains(esp_foc_pi_q32_t *pi, float kp, float ki, float fs_hz)
{
    if ((pi == NULL) || !(fs_hz > 0.0f) || !(kp >= 0.0f) || !(ki >= 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    pi->kp_q32 = esp_foc_q32_from_double((double)kp);
    pi->ki_ts_q32 = esp_foc_q32_from_double((double)ki / (double)fs_hz);
    return ESP_OK;
}
