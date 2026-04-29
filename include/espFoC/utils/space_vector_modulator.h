/*
 * MIT License
 */
/**
 * @file space_vector_modulator.h
 * @brief Space-vector PWM (SVPWM) in Q16.16: alpha/beta → phase volts → duties [0,1].
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline q16_t esp_foc_q16_max3(q16_t a, q16_t b, q16_t c)
{
    return q16_max(q16_max(a, b), c);
}

static inline q16_t esp_foc_q16_min3(q16_t a, q16_t b, q16_t c)
{
    return q16_min(q16_min(a, b), c);
}

static inline q16_t esp_foc_q16_avg2(q16_t a, q16_t b)
{
    return (q16_t)(((int64_t)a + (int64_t)b) >> 1);
}

/** Alpha/beta (volts) → three phase voltages v_a,v_b,v_c (same units). */
static inline void esp_foc_svm_alpha_beta_to_phase_volts(q16_t v_alpha,
                                                        q16_t v_beta,
                                                        q16_t *v_a,
                                                        q16_t *v_b,
                                                        q16_t *v_c)
{
    q16_t zero = 0;
    q16_t neg_half_alpha = q16_sub(zero, q16_mul(v_alpha, Q16_HALF));
    q16_t sqrt3_beta = q16_mul(v_beta, Q16_SQRT3_OVER_2);

    *v_a = v_alpha;
    *v_b = q16_add(neg_half_alpha, sqrt3_beta);
    *v_c = q16_sub(neg_half_alpha, sqrt3_beta);
}

/** Phase voltages and 1/Vdc → PWM duties Q16 [0, 1]. */
static inline void esp_foc_svm_phase_volts_to_duties(q16_t v_a,
                                                     q16_t v_b,
                                                     q16_t v_c,
                                                     q16_t inv_vbus,
                                                     q16_t *duty_a,
                                                     q16_t *duty_b,
                                                     q16_t *duty_c)
{
    q16_t v_max = esp_foc_q16_max3(v_a, v_b, v_c);
    q16_t v_min = esp_foc_q16_min3(v_a, v_b, v_c);
    q16_t v_cm = esp_foc_q16_avg2(v_max, v_min);
    q16_t zero = 0;

    q16_t da = q16_add(Q16_HALF, q16_mul(q16_sub(v_a, v_cm), inv_vbus));
    q16_t db = q16_add(Q16_HALF, q16_mul(q16_sub(v_b, v_cm), inv_vbus));
    q16_t dc = q16_add(Q16_HALF, q16_mul(q16_sub(v_c, v_cm), inv_vbus));

    *duty_a = q16_clamp(da, zero, Q16_ONE);
    *duty_b = q16_clamp(db, zero, Q16_ONE);
    *duty_c = q16_clamp(dc, zero, Q16_ONE);
}

static inline void esp_foc_svm_set(q16_t v_alpha,
                                   q16_t v_beta,
                                   q16_t inv_vbus,
                                   q16_t *duty_a,
                                   q16_t *duty_b,
                                   q16_t *duty_c)
{
    q16_t v_a, v_b, v_c;
    esp_foc_svm_alpha_beta_to_phase_volts(v_alpha, v_beta, &v_a, &v_b, &v_c);
    esp_foc_svm_phase_volts_to_duties(v_a, v_b, v_c, inv_vbus,
                                      duty_a, duty_b, duty_c);
}

#ifdef __cplusplus
}
#endif
