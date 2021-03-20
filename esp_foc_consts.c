#include <math.h>

const float ESP_FOC_FAST_PI = 3.14159265358f;
const float ESP_FOC_FAST_2PI = ESP_FOC_FAST_PI * 2.0f;
const float ESP_FOC_SIN_COS_APPROX_B = 4.0f / ESP_FOC_FAST_PI;
const float ESP_FOC_SIN_COS_APPROX_C = -4.0f / (ESP_FOC_FAST_PI * ESP_FOC_FAST_PI);
const float ESP_FOC_SIN_COS_APPROX_P = 0.225f;
const float ESP_FOC_SIN_COS_APPROX_D = ESP_FOC_FAST_PI/2.0f; 
const float ESP_FOC_CLARKE_K1 = 2.0/3.0f;
const float ESP_FOC_CLARKE_K2 = 1.0/3.0f;
const float ESP_FOC_CLARKE_PARK_SQRT3 = 1.73205080757f;
const float ESP_FOC_CLARKE_K3 = 2.0f / ESP_FOC_CLARKE_PARK_SQRT3;
