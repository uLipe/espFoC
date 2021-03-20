#pragma once

typedef struct esp_foc_inverter_s esp_foc_inverter_t;

struct esp_foc_inverter_s {
    float (*get_dc_link_voltage)(esp_foc_inverter_t *self);
    void (*set_voltages)(esp_foc_inverter_t *self,
                        float v_u,
                        float v_v,
                        float v_w);

};