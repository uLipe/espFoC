/*
 * MIT License
 */
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_q16.h"

static inline iq31_t q16_angle_rad_to_iq31_lut(q16_t angle_rad)
{
    q16_t a = q16_normalize_angle_rad(angle_rad);
    return (iq31_t)(((int64_t)a * (int64_t)IQ31_ONE) / (int64_t)Q16_TWO_PI);
}

q16_t q16_sin(q16_t angle_rad)
{
    iq31_t iq = q16_angle_rad_to_iq31_lut(angle_rad);
    return q16_from_iq31_per_unit(iq31_sin(iq));
}

q16_t q16_cos(q16_t angle_rad)
{
    iq31_t iq = q16_angle_rad_to_iq31_lut(angle_rad);
    return q16_from_iq31_per_unit(iq31_cos(iq));
}
