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

/** Inverse Park → three phase voltages (volts Q16). MCPWM drivers convert to duties. */
static inline void esp_foc_modulate_dq_voltage(q16_t sin,
                                               q16_t cos,
                                               q16_t v_d,
                                               q16_t v_q,
                                               q16_t *v_alpha,
                                               q16_t *v_beta,
                                               q16_t *v_u,
                                               q16_t *v_v,
                                               q16_t *v_w,
                                               q16_t vmax)
{
    q16_t vd = v_d;
    q16_t vq = v_q;
    esp_foc_limit_voltage_q16(&vd, &vq, vmax);
    q16_inverse_park(sin, cos, vd, vq, v_alpha, v_beta);
    esp_foc_svm_alpha_beta_to_phase_volts(*v_alpha, *v_beta, v_u, v_v, v_w);
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
