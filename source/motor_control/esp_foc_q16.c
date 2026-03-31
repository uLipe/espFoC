/*
 * MIT License
 */
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_iq31.h"

static inline iq31_t q16_angle_rad_to_iq31(q16_t angle_rad)
{
    iq31_t iq = (iq31_t)(((int64_t)angle_rad * (int64_t)IQ31_ONE) / (int64_t)Q16_TWO_PI);
    return iq31_normalize_angle(iq);
}

q16_t q16_sin(q16_t angle_rad)
{
    iq31_t iq = q16_angle_rad_to_iq31(angle_rad);
    return q16_from_iq31_per_unit(iq31_sin(iq));
}

q16_t q16_cos(q16_t angle_rad)
{
    iq31_t iq = q16_angle_rad_to_iq31(angle_rad);
    return q16_from_iq31_per_unit(iq31_cos(iq));
}
