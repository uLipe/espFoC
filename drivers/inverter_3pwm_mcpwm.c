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

#include <sys/cdefs.h>
#include "espFoC/inverter_3pwm_mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

typedef struct {
    int enable_gpio;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operators[3];
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_gen_handle_t  generators[3];

    float voltage_to_duty_ratio;
    float dc_link_voltage;
    esp_foc_inverter_callback_t notifier;
    void *arg;
    esp_foc_inverter_t interface;

}esp_foc_mcpwm_inverter_t;

#define MCPWM_PERIOD_TOP      1000
#define MCPWM_PERIOD_TOP_HALF 500
#define MCPWM_RATE_HZ         10000

DRAM_ATTR static esp_foc_mcpwm_inverter_t mcpwms[CONFIG_NOOF_AXIS];

static mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 10000000,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    .period_ticks = MCPWM_PERIOD_TOP,
};

static mcpwm_operator_config_t operator_config = {
    .group_id = 0,
};

static mcpwm_comparator_config_t compare_config = {
    .flags.update_cmp_on_tez = true,
};

static const char *TAG = "INVERTER_MCPWM";

static IRAM_ATTR bool inverter_isr(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_data)
{
    esp_foc_fpu_isr_enter();

    esp_foc_mcpwm_inverter_t *inverter = (esp_foc_mcpwm_inverter_t *)user_data;

    if(inverter->notifier) {
        inverter->notifier(inverter->arg);
    }

    esp_foc_fpu_isr_leave();
    return false;
}

const static mcpwm_timer_event_callbacks_t driver_cb = {
    .on_full = inverter_isr,
};

IRAM_ATTR static float get_dc_link_voltage (esp_foc_inverter_t *self)
{
    esp_foc_mcpwm_inverter_t *obj =
    __containerof(self, esp_foc_mcpwm_inverter_t, interface);

    return obj->dc_link_voltage;
}

IRAM_ATTR static void set_voltages(esp_foc_inverter_t *self,
                    float v_u,
                    float v_v,
                    float v_w)
{
    esp_foc_mcpwm_inverter_t *obj =
        __containerof(self, esp_foc_mcpwm_inverter_t, interface);

    if(v_u > 1.0f) {
        v_u = 1.0f;
    } else if (v_u < 0.0f) {
        v_u = 0.0f;
    }

    if(v_v > 1.0f) {
        v_v = 1.0f;
    } else if (v_v < 0.0f) {
        v_v = 0.0f;
    }

    if(v_w > 1.0f) {
        v_w = 1.0f;
    } else if (v_w < 0.0f) {
        v_w = 0.0f;
    }

    v_u *= obj->voltage_to_duty_ratio;
    v_v *= obj->voltage_to_duty_ratio;
    v_w *= obj->voltage_to_duty_ratio;

    mcpwm_comparator_set_compare_value(obj->comparators[0], (uint16_t)v_u);
    mcpwm_comparator_set_compare_value(obj->comparators[1], (uint16_t)v_v);
    mcpwm_comparator_set_compare_value(obj->comparators[2], (uint16_t)v_w);
}

IRAM_ATTR static void set_inverter_callback(esp_foc_inverter_t *self,
                        esp_foc_inverter_callback_t callback,
                        void *argument)
{
    esp_foc_mcpwm_inverter_t *obj =
    __containerof(self, esp_foc_mcpwm_inverter_t, interface);

    obj->notifier = callback;
    obj->arg = argument;
}

IRAM_ATTR static void phase_remap(esp_foc_inverter_t *self)
{
    (void)self;
}

IRAM_ATTR static float get_inverter_pwm_rate (esp_foc_inverter_t *self)
{
    (void)self;
    return (float)MCPWM_RATE_HZ;
}

IRAM_ATTR static void inverter_enable(esp_foc_inverter_t *self)
{
    esp_foc_mcpwm_inverter_t *obj =
    __containerof(self, esp_foc_mcpwm_inverter_t, interface);

    gpio_set_level(obj->enable_gpio, true);
}

IRAM_ATTR static void inverter_disable(esp_foc_inverter_t *self)
{
    esp_foc_mcpwm_inverter_t *obj =
    __containerof(self, esp_foc_mcpwm_inverter_t, interface);

    gpio_set_level(obj->enable_gpio, false);
}


esp_foc_inverter_t *inverter_3pwm_mpcwm_new(int gpio_u, int gpio_v, int gpio_w, int gpio_enable,
                                        float dc_link_voltage, int port)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << gpio_enable,
    };

    gpio_config(&drv_en_config);
    gpio_set_level(gpio_enable, false);

    /* the PWM arguments now are limited to range 0 -- 1.0 */
    mcpwms[port].enable_gpio = gpio_enable;
    mcpwms[port].dc_link_voltage = dc_link_voltage;
    mcpwms[port].interface.get_dc_link_voltage = get_dc_link_voltage;
    mcpwms[port].interface.set_voltages = set_voltages;
    mcpwms[port].interface.set_inverter_callback = set_inverter_callback;
    mcpwms[port].interface.phase_remap = phase_remap;
    mcpwms[port].interface.get_inverter_pwm_rate = get_inverter_pwm_rate;
    mcpwms[port].interface.enable = inverter_enable;
    mcpwms[port].interface.disable = inverter_disable;
    mcpwms[port].voltage_to_duty_ratio = MCPWM_PERIOD_TOP_HALF;
    mcpwms[port].notifier = NULL;

    timer_config.group_id = port;
    mcpwm_new_timer(&timer_config, &mcpwms[port].timer);

    operator_config.group_id = port;
    for (int i = 0; i < 3; i++) {
        mcpwm_new_operator(&operator_config, &mcpwms[port].operators[i]);
        mcpwm_operator_connect_timer(mcpwms[port].operators[i], mcpwms[port].timer);
    }

    for (int i = 0; i < 3; i++) {
        mcpwm_new_comparator(mcpwms[port].operators[i], &compare_config, &mcpwms[port].comparators[i]);
        mcpwm_comparator_set_compare_value(mcpwms[port].comparators[i], 0);
    }

    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = gpio_u;
    mcpwm_new_generator(mcpwms[port].operators[0], &gen_config, &mcpwms[port].generators[0]);
    gen_config.gen_gpio_num = gpio_v;
    mcpwm_new_generator(mcpwms[port].operators[1], &gen_config, &mcpwms[port].generators[1]);
    gen_config.gen_gpio_num = gpio_w;
    mcpwm_new_generator(mcpwms[port].operators[2], &gen_config, &mcpwms[port].generators[2]);

    for (int i = 0; i < 3; i++) {

        mcpwm_generator_set_actions_on_compare_event(mcpwms[port].generators[i],
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                        mcpwms[port].comparators[i],
                                                        MCPWM_GEN_ACTION_LOW),
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                                        mcpwms[port].comparators[i],
                                                        MCPWM_GEN_ACTION_HIGH),
                            MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    }

    mcpwm_timer_register_event_callbacks(mcpwms[port].timer, &driver_cb, &mcpwms[port]);
    mcpwm_timer_enable(mcpwms[port].timer);
    mcpwm_timer_start_stop(mcpwms[port].timer, MCPWM_TIMER_START_NO_STOP);

    ESP_LOGI(TAG,"Inverter driver is ready for use, intance %p", &mcpwms[port].interface);

    return &mcpwms[port].interface;
}
