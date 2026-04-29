/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

/**
 * @file esp_foc_iq31.h
 * @brief Minimal Q1.31 surface: LUT sin/cos + float conversion for legacy bridge paths.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t iq31_t;

#define IQ31_ONE       ((iq31_t)0x7FFFFFFF)
#define IQ31_MINUS_ONE ((iq31_t)0x80000000)

#define IQ31_SCALE (2147483648.0)

static inline iq31_t iq31_from_float(float x)
{
    if (x >= 1.0f) {
        return IQ31_ONE;
    }
    if (x <= -1.0f) {
        return IQ31_MINUS_ONE;
    }
    if (x != x) {
        return 0;
    }
    return (iq31_t)((double)x * (double)0x80000000ULL);
}

static inline float iq31_to_float(iq31_t x)
{
    return (float)x / (float)(1ULL << 31);
}

#define IQ31_FROM_FLOAT(x) iq31_from_float(x)
#define IQ31_TO_FLOAT(x)   iq31_to_float(x)

iq31_t iq31_sin(iq31_t angle_q31);
iq31_t iq31_cos(iq31_t angle_q31);

#ifdef __cplusplus
}
#endif
