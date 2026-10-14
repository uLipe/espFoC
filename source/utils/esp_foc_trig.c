/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Public trig API — software CORDIC.
 */
#include "espFoC/utils/esp_foc_trig.h"

#include <stddef.h>

#include "esp_foc_trig_soft.h"

void esp_foc_sincos(q16_t angle, q16_t *s_out, q16_t *c_out)
{
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
    return esp_foc_trig_soft_atan2(y, x);
}

q16_t esp_foc_sqrt(q16_t x)
{
    return esp_foc_trig_soft_sqrt(x);
}
