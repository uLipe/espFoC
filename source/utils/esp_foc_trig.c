/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public trig API — software CORDIC by default; optional HW accel with soft fallback.
 */
#include "espFoC/utils/esp_foc_trig.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_foc_trig_soft.h"

#if CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO
void esp_foc_trig_hw_sincos(q16_t angle, q16_t *s_out, q16_t *c_out);
q16_t esp_foc_trig_hw_atan2(q16_t y, q16_t x);
q16_t esp_foc_trig_hw_sqrt(q16_t x);
bool esp_foc_trig_hw_available(void);
#endif

void esp_foc_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO
    if (esp_foc_trig_hw_available()) {
        esp_foc_trig_hw_sincos(angle, s_out, c_out);
        return;
    }
#endif
    esp_foc_trig_soft_sincos(angle, s_out, c_out);
}

q16_t esp_foc_sin(q16_t angle)
{
    q16_t s = 0;
    esp_foc_sincos(angle, &s, NULL);
    return s;
}

q16_t esp_foc_cos(q16_t angle)
{
    q16_t c = 0;
    esp_foc_sincos(angle, NULL, &c);
    return c;
}

q16_t esp_foc_atan2(q16_t y, q16_t x)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO
    if (esp_foc_trig_hw_available()) {
        return esp_foc_trig_hw_atan2(y, x);
    }
#endif
    return esp_foc_trig_soft_atan2(y, x);
}

q16_t esp_foc_sqrt(q16_t x)
{
#if CONFIG_ESP_FOC_USE_HW_ACCEL_TRIGO
    if (esp_foc_trig_hw_available()) {
        return esp_foc_trig_hw_sqrt(x);
    }
#endif
    return esp_foc_trig_soft_sqrt(x);
}
