/*
 * MIT License
 */
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/esp_foc_iq31.h"
#include "espFoC/utils/foc_math_q16.h"

q16_t q16_sin(q16_t angle_rad)
{
    iq31_t iq = iq31_from_q16_per_unit(angle_rad);
    return q16_from_iq31_per_unit(iq31_sin(iq));
}

q16_t q16_cos(q16_t angle_rad)
{
    iq31_t iq = iq31_from_q16_per_unit(angle_rad);
    return q16_from_iq31_per_unit(iq31_cos(iq));
}
