/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "espFoC/utils/esp_foc_iq31.h"
#include <stdint.h>

#define IQ31_SIN_LUT_SIZE     8192
#define IQ31_SIN_LUT_MASK     (IQ31_SIN_LUT_SIZE - 1)
#define IQ31_SIN_LUT_QUARTER  (IQ31_SIN_LUT_SIZE >> 2)

static const iq31_t iq31_sin_lut[IQ31_SIN_LUT_SIZE] = {
#include "esp_foc_iq31_sin_lut.inc"
};

iq31_t iq31_sin(iq31_t angle_q31)
{
    uint32_t u = (uint32_t)angle_q31 & 0x7FFFFFFFU;
    uint32_t idx = ((uint64_t)u * (uint64_t)IQ31_SIN_LUT_SIZE) >> 31;
    idx &= IQ31_SIN_LUT_MASK;
    return iq31_sin_lut[idx];
}

iq31_t iq31_cos(iq31_t angle_q31)
{
    uint32_t u = (uint32_t)angle_q31 & 0x7FFFFFFFU;
    uint32_t idx = ((uint64_t)u * (uint64_t)IQ31_SIN_LUT_SIZE) >> 31;
    idx = (idx + IQ31_SIN_LUT_QUARTER) & IQ31_SIN_LUT_MASK;
    return iq31_sin_lut[idx];
}
