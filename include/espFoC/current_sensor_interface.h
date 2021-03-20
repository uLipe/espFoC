#pragma once

typedef struct esp_foc_isensor_s esp_foc_isensor_t;

struct esp_foc_isensor_s {
    int (*get_noof_isensors)(esp_foc_isensor_t *self);
    void (*get_isensors)(esp_foc_isensor_t *self,
                    float *isensor_values);
};