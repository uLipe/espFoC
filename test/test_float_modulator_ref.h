/*
 * Float SVPWM / Clarke–Park reference for unit tests only — not the espFoC API.
 * Self-contained (no install tree header) so the library can stay Q16/IQ31-only.
 */
#pragma once

#include <math.h>
#include <stdint.h>

#define TFR_FOC_CLARKE_PARK_SQRT3  1.7320508075688772f
#define TFR_FOC_CLARKE_K3          (1.0f / TFR_FOC_CLARKE_PARK_SQRT3)

static inline float tfr_foc_rsqrt(float x)
{
    float xhalf = 0.5f * x;
    union {
        float    f;
        uint32_t u;
    } conv;
    conv.f = x;
    conv.u = 0x5f3759dfu - (conv.u >> 1);
    float y = conv.f;
    y = y * (1.5f - xhalf * y * y);
    return y;
}

static inline void tfr_foc_clarke(const float v_uvw[3], float *v_alpha, float *v_beta)
{
    *v_alpha = v_uvw[0];
    *v_beta = (v_uvw[0] + 2.0f * v_uvw[1]) * TFR_FOC_CLARKE_K3;
}

static inline void tfr_foc_park(float sin, float cos, const float v_ab[2],
                                 float *v_d, float *v_q)
{
    *v_d = v_ab[0] * cos + v_ab[1] * sin;
    *v_q = -v_ab[0] * sin + v_ab[1] * cos;
}

static inline void tfr_foc_inverse_park(float sin, float cos, const float v_dq[2],
                                        float *v_alpha, float *v_beta)
{
    *v_alpha = v_dq[0] * cos - v_dq[1] * sin;
    *v_beta = v_dq[0] * sin + v_dq[1] * cos;
}

static inline void tfr_foc_limit_voltage(float *v_d, float *v_q, float v_dc)
{
    float v_max_sq = v_dc * v_dc;
    float v_mag_sq = (*v_d) * (*v_d) + (*v_q) * (*v_q);
    if (v_mag_sq > v_max_sq && v_mag_sq > 1e-12f) {
        float scale = v_dc * tfr_foc_rsqrt(v_mag_sq);
        *v_d *= scale;
        *v_q *= scale;
    }
}

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

    tfr_foc_limit_voltage(&dq_frame[0], &dq_frame[1], vmax);
    tfr_foc_inverse_park(sin, cos, dq_frame, &ab_frame[0], &ab_frame[1]);
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
    tfr_foc_clarke(phase_current_frame, &ab_frame[0], &ab_frame[1]);
    tfr_foc_park(sin, cos, ab_frame, i_d, i_q);
    *i_alpha = ab_frame[0];
    *i_beta = ab_frame[1];
}
