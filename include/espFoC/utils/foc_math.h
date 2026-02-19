/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#ifdef CONFIG_ESP_FOC_CUSTOM_MATH
extern const float ESP_FOC_FAST_PI;
extern const float ESP_FOC_FAST_2PI;
extern const float ESP_FOC_SIN_COS_APPROX_B;
extern const float ESP_FOC_SIN_COS_APPROX_C;
extern const float ESP_FOC_SIN_COS_APPROX_P;
extern const float ESP_FOC_SIN_COS_APPROX_D;
#endif

extern const float ESP_FOC_CLARKE_K1;
extern const float ESP_FOC_CLARKE_K2;
extern const float ESP_FOC_CLARKE_PARK_SQRT3;
extern const float ESP_FOC_CLARKE_K3;
extern const float ESP_FOC_SQRT3_TWO;

static inline float esp_foc_sine(float x)
{
#ifdef CONFIG_ESP_FOC_CUSTOM_MATH
    extern float esp_foc_fast_sine(float angle);
    return esp_foc_fast_sine(x);
#else
    return sinf(x);
#endif
}

static inline float esp_foc_cosine(float x)
{
#ifdef CONFIG_ESP_FOC_CUSTOM_MATH
    extern float esp_foc_fast_cosine(float angle);
    return esp_foc_fast_cosine(x);
#else
    return cosf(x);
#endif
}

static inline float esp_foc_sqrtf(float x)
{
    return sqrtf(x);
}

/* Fast rsqrt (Quake-style), then one NR iteration for accuracy */
static inline float esp_foc_rsqrt_fast(float x)
{
    float xhalf = 0.5f * x;

    union {
        float    f;
        uint32_t u;
    } conv;

    conv.f = x;                                  // load float bits
    conv.u = 0x5f3759dfu - (conv.u >> 1);        // initial guess
    float y = conv.f;

    // One Newton-Raphson iteration
    y = y * (1.5f - xhalf * y * y);

    return y;
}

static inline float esp_foc_mechanical_to_elec_angle(float mech_angle,
                                                    float pole_pairs)
{
    return(mech_angle * pole_pairs);
}

static inline float esp_foc_normalize_angle(float angle)
{
    const float full2pi = M_PI * 2.0f;
    float result =  fmod(angle, full2pi);

    if(result < 0) {
        result += full2pi;
    }
    return result;
}

static inline void esp_foc_clarke_transform (float v_uvw[3],
                                            float * v_aplha,
                                            float *v_beta)
{
    *v_aplha = v_uvw[0];
    *v_beta = (v_uvw[0] + 2.0f * v_uvw[1]) * ESP_FOC_CLARKE_K3;
}

static inline void esp_foc_park_transform (float sin,
                                        float cos,
                                        float v_ab[2],
                                        float *v_d,
                                        float *v_q)
{
    *v_d = v_ab[0] * cos + v_ab[1] * sin;
    *v_q = -v_ab[0] * sin + v_ab[1] * cos;
}

static inline void esp_foc_inverse_park_transform (float sin,
                                                float cos,
                                                float v_dq[2],
                                                float *v_alpha,
                                                float *v_beta)
{
    *v_alpha = v_dq[0] * cos - v_dq[1] * sin;
    *v_beta = v_dq[0] * sin + v_dq[1] * cos;
}


static inline void esp_foc_inverse_clarke_transform (float v_ab[2],
                                                    float *v_u,
                                                    float *v_v,
                                                    float *v_w)
{
    float a,b,c;

    a = v_ab[0];
    b = ((ESP_FOC_CLARKE_PARK_SQRT3 * v_ab[1]) - v_ab[0]) * 0.5f;
    c = -a - b;

    *v_u = a;
    *v_v = b;
    *v_w = c;
}

static inline void esp_foc_limit_voltage(float *v_d, float *v_q, float v_dc)
{
    float v_max = v_dc;
    float v_max_sq = v_dc * v_dc;
    float v_mag_sq = (*v_d) * (*v_d) + (*v_q) * (*v_q);

    if (v_mag_sq > v_max_sq && v_mag_sq > 1e-12f) {
        float scale = v_max * esp_foc_rsqrt_fast(v_mag_sq);
        *v_d *= scale;
        *v_q *= scale;
    }
}

static inline void esp_foc_apply_bias(float *v_alpha, float *v_beta)
{
    *v_alpha = (*v_alpha * 0.5f) + 0.5f;
    *v_beta = (*v_beta * 0.5f) + 0.5f;
}

static inline void esp_foc_third_harmonic_injection(float *v_alpha, float *v_beta)
{

#ifdef CONFIG_ESP_FOC_COMP_THI
    float v_max = fmaxf(fmaxf(*v_alpha, *v_beta), 0.0f);
    float v_min = fminf(fminf(*v_alpha, *v_beta), 0.0f);
    float v_offset = (v_max + v_min) * 0.5f;

    *v_alpha -= v_offset;
    *v_beta  -= v_offset;

    *v_alpha *= 0.95f;
    *v_beta  *= 0.95f;
#endif

}

static inline float esp_foc_clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}