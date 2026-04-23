#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/driver_iq31_local.h"
#include "espFoC/rotor_sensor_pcnt.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "SENSOR_PCNT";

#ifdef CONFIG_ESP_FOC_ENABLE_PCNT_DRIVER

typedef struct {
    int64_t accum_i64;
    uint32_t ppr_u32;
    uint32_t pcnt_unit;
    esp_foc_rotor_sensor_t interface;
} esp_foc_pcnt_t;

static bool pcnt_configured = false;
static esp_foc_pcnt_t rotor_sensors[CONFIG_NOOF_AXIS];

static void pcnt_overflow_handler(void *arg)
{
    esp_foc_pcnt_t *obj = (esp_foc_pcnt_t *)arg;
    uint32_t status = 0;
    pcnt_get_event_status(obj->pcnt_unit, &status);

    if (status & PCNT_EVT_H_LIM) {
        obj->accum_i64 += (int64_t)obj->ppr_u32;
    } else if (status & PCNT_EVT_L_LIM) {
        obj->accum_i64 -= (int64_t)obj->ppr_u32;
    }
}

static int64_t read_accumulated_counts_i64(esp_foc_rotor_sensor_t *self)
{
    int16_t raw_count;
    esp_foc_pcnt_t *obj = __containerof(self, esp_foc_pcnt_t, interface);

    pcnt_get_counter_value(obj->pcnt_unit, &raw_count);
    int32_t rc = (int32_t)raw_count;
    uint32_t absv = (uint32_t)(rc < 0 ? -rc : rc);
    return obj->accum_i64 + (int64_t)absv;
}

static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_pcnt_t *obj = __containerof(self, esp_foc_pcnt_t, interface);
    pcnt_counter_clear(obj->pcnt_unit);
}

static uint32_t get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    esp_foc_pcnt_t *obj = __containerof(self, esp_foc_pcnt_t, interface);
    return obj->ppr_u32 ? obj->ppr_u32 : 1u;
}

static q16_t read_counts(esp_foc_rotor_sensor_t *self)
{
    int16_t raw_count;
    esp_foc_pcnt_t *obj = __containerof(self, esp_foc_pcnt_t, interface);

    pcnt_get_counter_value(obj->pcnt_unit, &raw_count);

    int32_t rc = (int32_t)raw_count;
    uint32_t absv = (uint32_t)(rc < 0 ? -rc : rc);
    return esp_foc_q16_from_counts_mod(absv, obj->ppr_u32 ? obj->ppr_u32 : 1u);
}

static void set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment_normalized)
{
    esp_foc_pcnt_t *obj = __containerof(self, esp_foc_pcnt_t, interface);
    int64_t dt = ((int64_t)increment_normalized * (int64_t)obj->ppr_u32) >> 31;
    obj->accum_i64 += dt;
}

esp_foc_rotor_sensor_t *rotor_sensor_pcnt_new(int pin_a,
                                            int pin_b,
                                            int port,
                                            int16_t pulses_per_revolution)
{
    if (port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.read_accumulated_counts_i64 = read_accumulated_counts_i64;
    rotor_sensors[port].interface.set_simulation_count = set_simulation_count;
    rotor_sensors[port].interface.set_zero_offset_raw_12b = NULL;
    rotor_sensors[port].interface.get_zero_offset_12b = NULL;
    rotor_sensors[port].accum_i64 = 0;
    rotor_sensors[port].ppr_u32 = (uint32_t)(int32_t)pulses_per_revolution;
    rotor_sensors[port].pcnt_unit = port;

    if (!pcnt_configured) {

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
    (void)pin_a;
    (void)pin_b;
    (void)port;
    (void)pulses_per_revolution;
    return NULL;
}

#endif
