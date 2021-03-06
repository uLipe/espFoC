#pragma once 

extern const float ESP_FOC_FAST_PI;
extern const float ESP_FOC_FAST_2PI;
extern const float ESP_FOC_SIN_COS_APPROX_B;
extern const float ESP_FOC_SIN_COS_APPROX_C;
extern const float ESP_FOC_SIN_COS_APPROX_P;
extern const float ESP_FOC_SIN_COS_APPROX_D; 
extern const float ESP_FOC_CLARKE_K1;
extern const float ESP_FOC_CLARKE_K2;
extern const float ESP_FOC_CLARKE_PARK_SQRT3;
extern const float ESP_FOC_CLARKE_K3;

static inline float esp_foc_sine(float x) 
{
    float y = ESP_FOC_SIN_COS_APPROX_B * x + 
        ESP_FOC_SIN_COS_APPROX_C * x * (x < 0 ? -x : x);
    return ESP_FOC_SIN_COS_APPROX_P * (y * (y < 0 ? -y : y) - y) + y;
}

static inline float esp_foc_cosine(float x)
{
    x = (x > 0) ? -x : x;
    x += ESP_FOC_SIN_COS_APPROX_D;

    return esp_foc_sine(x);
}

static inline float esp_foc_mechanical_to_elec_angle(float mech_angle, 
                                                    float pole_pairs)
{
    return(mech_angle * pole_pairs);
}

static inline float esp_foc_normalize_angle(float angle)
{
    float result =  fmod(angle, ESP_FOC_FAST_2PI);
    if(result > ESP_FOC_FAST_PI) {
        result -= ESP_FOC_FAST_2PI;  
    }
    
    return result;
}

static inline float esp_foc_saturate(float value, float limit) 
{
    float result = value;
    if(value > limit) {
        result = limit;
    } else if (value < -limit) {
        result = -limit;
    }

    return result;
}

#ifdef ESP_FOC_USE_TORQUE_CONTROLLER

static inline void esp_foc_clarke_transform (float v_uvw[3], 
                                            float * v_aplha, 
                                            float *v_beta)
{
    *v_aplha = ESP_FOC_CLARKE_K1 * v_uvw[0] - 
        ESP_FOC_CLARKE_K2 * (v_uvw[1] - v_uvw[2]);

    *v_beta = ESP_FOC_CLARKE_K3  * (v_uvw[1] - v_uvw[2]);
}

static inline void esp_foc_park_transform (float theta,
                                        float v_ab[2],
                                        float *v_d,
                                        float *v_q)
{
    *v_d = v_ab[0] * esp_foc_cosine(theta) +
        v_ab[1] * esp_foc_sine(theta);
    
    *v_q = v_ab[1] * esp_foc_cosine(theta) -
        v_ab[0] * esp_foc_sine(theta);    
}

#endif

static inline void esp_foc_inverse_clarke_transform (float v_ab[2],
                                                    float *v_u,
                                                    float *v_v,
                                                    float *v_w)
{
    *v_u = v_ab[0];
    *v_v = (-v_ab[0] + ESP_FOC_CLARKE_PARK_SQRT3 * v_ab[1]) * 0.5f;
    *v_w = (-v_ab[0] - ESP_FOC_CLARKE_PARK_SQRT3 * v_ab[1]) * 0.5f;  
}

static inline void esp_foc_inverse_park_transform (float theta,
                                                float v_dq[2],
                                                float *v_alpha,
                                                float *v_beta)
{
    *v_alpha = v_dq[0] * esp_foc_cosine(theta) -
        v_dq[1] * esp_foc_sine(theta);
    
    *v_beta = v_dq[1] * esp_foc_cosine(theta) +
        v_dq[0] * esp_foc_sine(theta);    
}
