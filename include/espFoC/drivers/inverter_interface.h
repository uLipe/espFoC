#pragma once
#include <stdint.h>
#include "espFoC/utils/esp_foc_q16.h"

typedef void (*esp_foc_inverter_callback_t)(void *argument);
typedef struct esp_foc_inverter_s esp_foc_inverter_t;

struct esp_foc_inverter_s {
    void (*set_inverter_callback)(esp_foc_inverter_t *self,
                        esp_foc_inverter_callback_t callback,
                        void *argument);
    q16_t (*get_dc_link_voltage)(esp_foc_inverter_t *self);
    void (*set_voltages)(esp_foc_inverter_t *self,
                        q16_t v_u, q16_t v_v, q16_t v_w);
    uint32_t (*get_inverter_pwm_rate)(esp_foc_inverter_t *self);
    void (*enable)(esp_foc_inverter_t *self);
    void (*disable)(esp_foc_inverter_t *self);
};
