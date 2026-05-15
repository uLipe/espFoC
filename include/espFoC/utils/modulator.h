/*
 * MIT License
 */
/**
 * @file modulator.h
 * @brief DQ voltage modulation and ABC->DQ current conversion (Q16.16).
 */
#pragma once

#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/space_vector_modulator.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Inverse Park + SVPWM: Vdq/αβ in pu, duties in [0, Q16_ONE]. */
static inline void esp_foc_modulate_dq_to_duties(q16_t sin,
                                                 q16_t cos,
                                                 q16_t v_d,
                                                 q16_t v_q,
                                                 q16_t *v_alpha,
                                                 q16_t *v_beta,
                                                 q16_t *duty_a,
                                                 q16_t *duty_b,
                                                 q16_t *duty_c,
                                                 q16_t vmax_pu)
{
    q16_t vd = v_d;
    q16_t vq = v_q;
    esp_foc_limit_voltage_q16(&vd, &vq, vmax_pu);
    q16_inverse_park(sin, cos, vd, vq, v_alpha, v_beta);
    esp_foc_svm_set(*v_alpha, *v_beta, Q16_ONE, duty_a, duty_b, duty_c);
}

static inline void esp_foc_get_dq_currents(q16_t sin,
                                           q16_t cos,
                                           q16_t i_u,
                                           q16_t i_v,
                                           q16_t i_w,
                                           q16_t *i_alpha,
                                           q16_t *i_beta,
                                           q16_t *i_q,
                                           q16_t *i_d)
{
    q16_t d;
    q16_t q;
    q16_clarke(i_u, i_v, i_w, i_alpha, i_beta);
    q16_park(sin, cos, *i_alpha, *i_beta, &d, &q);
    *i_d = d;
    *i_q = q;
}

#ifdef __cplusplus
}
#endif
