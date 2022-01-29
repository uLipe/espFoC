#include <sys/cdefs.h>
#include "espFoC/inverter_3pwm_ledc.h"
#include "esp_attr.h"

#define LEDC_FREQUENCY_HZ       25000
#define LEDC_RESOLUTION_STEPS   255.0

typedef struct {
    float voltage_to_duty_ratio;
    float dc_link_voltage;
    ledc_channel_t ledc_channel[3];
    esp_foc_inverter_t interface;
}esp_foc_ledc_inverter;

DRAM_ATTR static esp_foc_ledc_inverter ledc[CONFIG_NOOF_AXIS];

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

    if(v_u > obj->dc_link_voltage) {
        v_u = obj->dc_link_voltage;
    } else if (v_u < 0.0f) {
        v_u = 0.0f;
    }

    if(v_v > obj->dc_link_voltage) {
        v_v = obj->dc_link_voltage;
    } else if (v_v < 0.0f) {
        v_v = 0.0f;
    }

    if(v_w > obj->dc_link_voltage) {
        v_w = obj->dc_link_voltage;
    } else if (v_w < 0.0f) {
        v_w = 0.0f;
    }

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE, 
        obj->ledc_channel[0], 
        obj->voltage_to_duty_ratio * v_u
    );
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, obj->ledc_channel[0]);

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE, 
        obj->ledc_channel[1], 
        obj->voltage_to_duty_ratio * v_v
    );
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, obj->ledc_channel[1]);

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE, 
        obj->ledc_channel[2], 
        obj->voltage_to_duty_ratio * v_w
    );
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, obj->ledc_channel[2]);
}

IRAM_ATTR static void set_inverter_callback(esp_foc_inverter_t *self,
                        esp_foc_inverter_callback_t callback)
{
    (void)self;
    (void)callback;
}

esp_err_t inverter_3pwm_ledc_init()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    return ESP_OK;
}

esp_foc_inverter_t *inverter_3pwm_ledc_new(ledc_channel_t ch_u,
                                        ledc_channel_t ch_v,
                                        ledc_channel_t ch_w,
                                        int gpio_u,
                                        int gpio_v,
                                        int gpio_w,
                                        float dc_link_voltage,
                                        int port)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }


    ledc[port].dc_link_voltage = dc_link_voltage;
    ledc[port].interface.get_dc_link_voltage = get_dc_link_voltage;
    ledc[port].interface.set_voltages = set_voltages;
    ledc[port].interface.set_inverter_callback = set_inverter_callback;
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
            .timer_sel  = LEDC_TIMER_0
        },

        {
            .channel    = ch_v,
            .duty       = 0,
            .gpio_num   = gpio_v,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },

        {
            .channel    = ch_w,
            .duty       = 0,
            .gpio_num   = gpio_w,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
    };

    for (int ch = 0; ch < 3; ch++) {
        esp_err_t err = ledc_channel_config(&ledc_channel[ch]);
        if(err != ESP_OK) {
            return NULL;
        }
    }

    ledc[port].voltage_to_duty_ratio = LEDC_RESOLUTION_STEPS / ledc[port].dc_link_voltage;

    return &ledc[port].interface;
}