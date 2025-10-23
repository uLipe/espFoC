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
#include <sdkconfig.h>
#include "espFoC/inverter_3pwm_ledc.h"
#include "hal/ledc_hal.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "INVERTER_LEDC";

#ifdef  CONFIG_ESP_FOC_ENABLE_LEGACY_LEDC_PWM

#define LEDC_FREQUENCY_HZ       ESP_FOC_PWM_RATE_HZ
#define LEDC_RESOLUTION_STEPS   255.0

typedef struct {
    int enable_gpio;
    ledc_dev_t *hw;
    float voltage_to_duty_ratio;
    float dc_link_voltage;
    ledc_channel_t ledc_channel[3];
    esp_foc_inverter_callback_t notifier;
    void *arg;
    esp_foc_inverter_t interface;
}esp_foc_ledc_inverter;

static const ledc_timer_t ledc_timers[4] = {LEDC_TIMER_0,LEDC_TIMER_1,LEDC_TIMER_2,LEDC_TIMER_3};

static bool ledc_driver_configured = false;

DRAM_ATTR static esp_foc_ledc_inverter ledc[CONFIG_NOOF_AXIS];

static const char *TAG = "INVERTER_LEDC";

IRAM_ATTR static void ledc_isr(void *arg)
{
    esp_foc_fpu_isr_enter();

    esp_foc_ledc_inverter *obj = (esp_foc_ledc_inverter *)arg;

    obj->hw->int_clr.val = (LEDC_LSTIMER0_OVF_INT_ENA |
                        LEDC_LSTIMER1_OVF_INT_ENA |
                        LEDC_LSTIMER2_OVF_INT_ENA |
                        LEDC_LSTIMER3_OVF_INT_ENA );

    if(obj->notifier) {
        obj->notifier(obj->arg);
    }

    esp_foc_fpu_isr_leave();
}

/* This function is required because the ledc driver does not support update from
 * ISR
 */
IRAM_ATTR static void ledc_update(esp_foc_ledc_inverter *obj, ledc_channel_t channel, float duty)
{
    /* set duty parameters */
    ledc_ll_set_hpoint(obj->hw, LEDC_LOW_SPEED_MODE, channel, 0);
    ledc_ll_set_duty_int_part(obj->hw, LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_ll_set_duty_direction(obj->hw, LEDC_LOW_SPEED_MODE, channel, LEDC_DUTY_DIR_INCREASE);
    ledc_ll_set_duty_num(obj->hw, LEDC_LOW_SPEED_MODE, channel, 1);
    ledc_ll_set_duty_cycle(obj->hw, LEDC_LOW_SPEED_MODE, channel, 1);
    ledc_ll_set_duty_scale(obj->hw, LEDC_LOW_SPEED_MODE, channel, 0);
    ledc_ll_ls_channel_update(obj->hw, LEDC_LOW_SPEED_MODE, channel);

    /* trigger the duty update */
    ledc_ll_set_sig_out_en(obj->hw, LEDC_LOW_SPEED_MODE, channel, true);
    ledc_ll_set_duty_start(obj->hw, LEDC_LOW_SPEED_MODE, channel, true);
}


IRAM_ATTR static float get_dc_link_voltage (esp_foc_inverter_t *self)
{
    esp_foc_ledc_inverter *obj =
    __containerof(self, esp_foc_ledc_inverter, interface);

    return obj->dc_link_voltage;
}

IRAM_ATTR static void set_voltages(esp_foc_inverter_t *self,
                    float v_u,
                    float v_v,
                    float v_w)
{
    esp_foc_ledc_inverter *obj =
        __containerof(self, esp_foc_ledc_inverter, interface);

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

    ledc_update(obj, obj->ledc_channel[0], obj->voltage_to_duty_ratio * v_u);
    ledc_update(obj, obj->ledc_channel[1], obj->voltage_to_duty_ratio * v_v);
    ledc_update(obj, obj->ledc_channel[2], obj->voltage_to_duty_ratio * v_w);
}

IRAM_ATTR static void set_inverter_callback(esp_foc_inverter_t *self,
                        esp_foc_inverter_callback_t callback,
                        void *argument)
{
    esp_foc_ledc_inverter *obj =
    __containerof(self, esp_foc_ledc_inverter, interface);

    obj->notifier = callback;
    obj->arg = argument;

    ledc_isr_register(ledc_isr, obj, ESP_INTR_FLAG_IRAM, NULL);
    obj->hw->int_ena.val |= (LEDC_LSTIMER0_OVF_INT_ENA |
                        LEDC_LSTIMER1_OVF_INT_ENA |
                        LEDC_LSTIMER2_OVF_INT_ENA |
                        LEDC_LSTIMER3_OVF_INT_ENA );
}

IRAM_ATTR static void phase_remap(esp_foc_inverter_t *self)
{
    (void)self;
}

IRAM_ATTR static float get_inverter_pwm_rate (esp_foc_inverter_t *self)
{
    (void)self;
    return (float)LEDC_FREQUENCY_HZ;
}

IRAM_ATTR static void inverter_enable(esp_foc_inverter_t *self)
{
    esp_foc_ledc_inverter *obj =
    __containerof(self, esp_foc_ledc_inverter, interface);

    gpio_set_level(obj->enable_gpio, true);
}

IRAM_ATTR static void inverter_disable(esp_foc_inverter_t *self)
{
    esp_foc_ledc_inverter *obj =
    __containerof(self, esp_foc_ledc_inverter, interface);

    gpio_set_level(obj->enable_gpio, false);
}


static esp_err_t inverter_3pwm_ledc_init()
{
    for (int i = 0; i < CONFIG_NOOF_AXIS; i++) {
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = LEDC_FREQUENCY_HZ,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = ledc_timers[i],
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_timer_config(&ledc_timer);
    }

    return ESP_OK;
}

esp_foc_inverter_t *inverter_3pwm_ledc_new(ledc_channel_t ch_u,
                                        ledc_channel_t ch_v,
                                        ledc_channel_t ch_w,
                                        int gpio_u,
                                        int gpio_v,
                                        int gpio_w,
                                        int gpio_enable,
                                        float dc_link_voltage,
                                        int port)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    if(!ledc_driver_configured) {
        inverter_3pwm_ledc_init();
        ledc_driver_configured = true;
    }

    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << gpio_enable,
    };

    gpio_config(&drv_en_config);
    gpio_set_level(gpio_enable, false);

    /* the PWM arguments now are limited to range 0 -- 1.0 */
    ledc[port].enable_gpio = gpio_enable;
    ledc[port].dc_link_voltage = dc_link_voltage;
    ledc[port].interface.get_dc_link_voltage = get_dc_link_voltage;
    ledc[port].interface.set_voltages = set_voltages;
    ledc[port].interface.set_inverter_callback = set_inverter_callback;
    ledc[port].interface.phase_remap = phase_remap;
    ledc[port].interface.get_inverter_pwm_rate = get_inverter_pwm_rate;
    ledc[port].interface.enable = inverter_enable;
    ledc[port].interface.disable = inverter_disable;

    ledc[port].ledc_channel[0] = ch_u;
    ledc[port].ledc_channel[1] = ch_v;
    ledc[port].ledc_channel[2] = ch_w;

    ledc_channel_config_t ledc_channel[3] =  {
        {
            .channel    = ch_u,
            .duty       = 0,
            .gpio_num   = gpio_u,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = ledc_timers[port]
        },

        {
            .channel    = ch_v,
            .duty       = 0,
            .gpio_num   = gpio_v,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = ledc_timers[port]
        },

        {
            .channel    = ch_w,
            .duty       = 0,
            .gpio_num   = gpio_w,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = ledc_timers[port]
        },
    };

    for (int ch = 0; ch < 3; ch++) {
        esp_err_t err = ledc_channel_config(&ledc_channel[ch]);
        if(err != ESP_OK) {
            return NULL;
        }
    }

    ledc[port].hw = LEDC_LL_GET_HW();

    ledc[port].voltage_to_duty_ratio = LEDC_RESOLUTION_STEPS;
    ESP_LOGI(TAG,"Inverter driver is ready for use, intance %p", &ledc[port].interface);

    return &ledc[port].interface;
}

#else
esp_foc_inverter_t *inverter_3pwm_ledc_new(ledc_channel_t ch_u,
                                        ledc_channel_t ch_v,
                                        ledc_channel_t ch_w,
                                        int gpio_u,
                                        int gpio_v,
                                        int gpio_w,
                                        int gpio_enable,
                                        float dc_link_voltage,
                                        int port)
{
    ESP_LOGE(TAG,"LEDC driver is not enabled, please set CONFIG_ESP_FOC_ENABLE_LEGACY_LEDC_PWM=y on your sdkconfig.defaults");
    return NULL;
}
#endif