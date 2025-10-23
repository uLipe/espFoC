#include <math.h>
#include "espFoC/rotor_sensor_pcnt.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "SENSOR_PCNT";

#ifdef CONFIG_ESP_FOC_ENABLE_PCNT_DRIVER

typedef struct {
    float accumulated;
    float previous;
    float pulses_per_revolution;
    uint32_t pcnt_unit;
    esp_foc_rotor_sensor_t interface;
}esp_foc_pcnt_t;

static bool pcnt_configured = false;

DRAM_ATTR static esp_foc_pcnt_t rotor_sensors[CONFIG_NOOF_AXIS];

static void pcnt_overflow_handler(void *arg)
{
    esp_foc_pcnt_t *obj = (esp_foc_pcnt_t *)arg;
    uint32_t status = 0;
    pcnt_get_event_status(obj->pcnt_unit, &status);

    if (status & PCNT_EVT_H_LIM) {
        obj->accumulated += obj->pulses_per_revolution;
    } else if (status & PCNT_EVT_L_LIM) {
        obj->accumulated -= obj->pulses_per_revolution;
    }
}

IRAM_ATTR static float read_accumulated_counts(esp_foc_rotor_sensor_t *self)
{
    int16_t raw_count;
    esp_foc_pcnt_t *obj =
        __containerof(self,esp_foc_pcnt_t, interface);

    pcnt_get_counter_value(obj->pcnt_unit, &raw_count);
    return obj->accumulated + fabs((float)raw_count);
}

IRAM_ATTR  static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_pcnt_t *obj =
        __containerof(self,esp_foc_pcnt_t, interface);

    pcnt_counter_clear(obj->pcnt_unit);
}

IRAM_ATTR static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    esp_foc_pcnt_t *obj =
        __containerof(self,esp_foc_pcnt_t, interface);

    return obj->pulses_per_revolution;
}

IRAM_ATTR static float read_counts(esp_foc_rotor_sensor_t *self)
{
    int16_t raw_count;
    esp_foc_pcnt_t *obj =
        __containerof(self,esp_foc_pcnt_t, interface);

    pcnt_get_counter_value(obj->pcnt_unit, &raw_count);
    obj->previous = (float)raw_count;

    return(fabs(obj->previous));
}

esp_foc_rotor_sensor_t *rotor_sensor_pcnt_new(int pin_a,
                                            int pin_b,
                                            int port,
                                            int16_t pulses_per_revolution)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.read_accumulated_counts = read_accumulated_counts;
    rotor_sensors[port].previous = 0;
    rotor_sensors[port].accumulated = 0;
    rotor_sensors[port].pulses_per_revolution = (float)pulses_per_revolution;
    rotor_sensors[port].pcnt_unit = port;

    if(!pcnt_configured) {

        pcnt_config_t dev_config = {
            .pulse_gpio_num = pin_a,
            .ctrl_gpio_num = pin_b,
            .channel = PCNT_CHANNEL_0,
            .unit = rotor_sensors[port].pcnt_unit,
            .pos_mode = PCNT_COUNT_DEC,
            .neg_mode = PCNT_COUNT_INC,
            .lctrl_mode = PCNT_MODE_REVERSE,
            .hctrl_mode = PCNT_MODE_KEEP,
            .counter_h_lim = pulses_per_revolution,
            .counter_l_lim = -pulses_per_revolution,
        };
        pcnt_unit_config(&dev_config);

        dev_config.pulse_gpio_num = pin_b;
        dev_config.ctrl_gpio_num = pin_a;
        dev_config.channel = PCNT_CHANNEL_1;
        dev_config.pos_mode = PCNT_COUNT_INC;
        dev_config.neg_mode = PCNT_COUNT_DEC;
        pcnt_unit_config(&dev_config);

        pcnt_counter_pause(rotor_sensors[port].pcnt_unit);
        pcnt_counter_clear(rotor_sensors[port].pcnt_unit);
        pcnt_isr_handler_add(rotor_sensors[port].pcnt_unit,
            pcnt_overflow_handler,
            &rotor_sensors[port]);

        pcnt_event_enable(rotor_sensors[port].pcnt_unit, PCNT_EVT_H_LIM);
        pcnt_event_enable(rotor_sensors[port].pcnt_unit, PCNT_EVT_L_LIM);

        pcnt_configured = true;
    }

    return &rotor_sensors[port].interface;
}
#else
esp_foc_rotor_sensor_t *rotor_sensor_pcnt_new(int pin_a,
    int pin_b,
    int port,
    int16_t pulses_per_revolution)
{
    ESP_LOGE(TAG,"PCNT driver is not enabled, please set CONFIG_ESP_FOC_ENABLE_PCNT_DRIVER=y on your sdkconfig.defaults");
    return NULL;
}

#endif