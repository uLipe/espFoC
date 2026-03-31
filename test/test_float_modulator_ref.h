/*
 * Float SVPWM / modulator reference (tests only) — not part of the espFoC control API.
 */
#pragma once

#include <math.h>
#include "espFoC/utils/foc_math.h"

static inline void esp_foc_svm_set_float(float v_alpha,
                                         float v_beta,
                                         float inv_vbus,
                                         float *duty_a,
                                         float *duty_b,
                                         float *duty_c)
{
    const float SQRT3_OVER_2 = 0.8660254037844386f;

    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float v_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    float v_max = v_a;
    if (v_b > v_max) {
        v_max = v_b;
    }
    if (v_c > v_max) {
        v_max = v_c;
    }

    float v_min = v_a;
    if (v_b < v_min) {
        v_min = v_b;
    }
    if (v_c < v_min) {
        v_min = v_c;
    }

    float v_cm = 0.5f * (v_max + v_min);

    float d_a = 0.5f + (v_a - v_cm) * inv_vbus;
    float d_b = 0.5f + (v_b - v_cm) * inv_vbus;
    float d_c = 0.5f + (v_c - v_cm) * inv_vbus;

    if (d_a < 0.0f) {
        d_a = 0.0f;
    } else if (d_a > 1.0f) {
        d_a = 1.0f;
    }
    if (d_b < 0.0f) {
        d_b = 0.0f;
    } else if (d_b > 1.0f) {
        d_b = 1.0f;
    }
    if (d_c < 0.0f) {
        d_c = 0.0f;
    } else if (d_c > 1.0f) {
        d_c = 1.0f;
    }

    *duty_a = d_a;
    *duty_b = d_b;
    *duty_c = d_c;
}

static inline void esp_foc_modulate_dq_voltage_float(float sin,
                                                     float cos,
                                                     float v_d,
                                                     float v_q,
                                                     float *v_alpha,
                                                     float *v_beta,
                                                     float *v_u,
                                                     float *v_v,
                                                     float *v_w,
                                                     float vmax,
                                                     float bias,
                                                     float normalization_scale)
{
    (void)bias;
    float dq_frame[2] = {v_d, v_q};
    float ab_frame[2];

    esp_foc_limit_voltage(&dq_frame[0], &dq_frame[1], vmax);
    esp_foc_inverse_park_transform(sin, cos, dq_frame, &ab_frame[0], &ab_frame[1]);
    esp_foc_svm_set_float(ab_frame[0], ab_frame[1], normalization_scale, v_u, v_v, v_w);
    *v_alpha = ab_frame[0];
    *v_beta = ab_frame[1];
}

static inline void esp_foc_get_dq_currents_float(float sin,
                                                 float cos,
                                                 float i_u,
                                                 float i_v,
                                                 float i_w,
                                                 float *i_alpha,
                                                 float *i_beta,
                                                 float *i_q,
                                                 float *i_d)
{
    float phase_current_frame[3] = {i_u, i_v, i_w};
    float ab_frame[2];
    esp_foc_clarke_transform(phase_current_frame, &ab_frame[0], &ab_frame[1]);
    esp_foc_park_transform(sin, cos, ab_frame, i_d, i_q);
    *i_alpha = ab_frame[0];
    *i_beta = ab_frame[1];
}
