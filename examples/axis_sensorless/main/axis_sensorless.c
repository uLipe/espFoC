/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/current_sensor_adc_one_shot.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#endif

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;

static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs = 4,
    .motor_inductance = 0,
    .motor_resistance = 0,
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
    .motor_unit = 0,
};


#ifdef CONFIG_IDF_TARGET_ESP32P4
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void esp_foc_init_bus_callback(void)
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

}

void esp_foc_send_buffer_callback(const uint8_t *buffer, int size)
{
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, buffer, size);
    esp_err_t err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
    }
}
#endif

static void regulation_callback(esp_foc_axis_t *axis_cb,
                                esp_foc_d_current_q16_t *id_ref,
                                esp_foc_q_current_q16_t *iq_ref,
                                esp_foc_d_voltage_q16_t *ud_forward,
                                esp_foc_q_voltage_q16_t *uq_forward)
{
    (void)axis_cb;

    uq_forward->raw = q16_from_float(0.0f);
    ud_forward->raw = q16_from_float(0.0f);
    iq_ref->raw = q16_from_float(2.0f);
    id_ref->raw = q16_from_float(0.0f);
}

static void initialize_foc_drivers(void)
{
    inverter = inverter_6pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_UL_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_VL_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_WL_PIN,
        -47,
        12.0f,
        0
    );
    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

#ifdef CONFIG_IDF_TARGET_ESP32P4
    esp_foc_isensor_adc_oneshot_config_t shunt_oneshot_cfg = {
        .axis_channels = {ADC_CHANNEL_7, ADC_CHANNEL_6},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 20.0f,
        .shunt_resistance = 0.005f,
        .number_of_channels = 2,
        .enable_analog_encoder = false,
    };

    shunts = isensor_adc_oneshot_new(&shunt_oneshot_cfg, NULL);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
#elif defined (CONFIG_IDF_TARGET_ESP32S3)
    esp_foc_isensor_adc_config_t shunt_cfg = {
        .axis_channels = {ADC_CHANNEL_1, ADC_CHANNEL_5},
        .units = {ADC_UNIT_1, ADC_UNIT_1},
        .amp_gain = 50.0f,
        .shunt_resistance = 0.01f,
        .number_of_channels = 2,
    };

    shunts = isensor_adc_new(&shunt_cfg);
    if(shunts == NULL) {
        ESP_LOGE(TAG, "failed to create the shunt sensor driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }

#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing the espFoC sensorless controller");
    ESP_LOGW(TAG, "Sensorless mode requires observer integration (WIP)");

    settings.motor_inductance = q16_from_float(0.0018f);
    settings.motor_resistance = q16_from_float(1.08f);

    initialize_foc_drivers();

    /*
     * Sensorless requires a rotor position observer instead of a physical
     * encoder.  Full observer integration is WIP; this example shows the
     * API shape but will not run a closed loop until the observer chain
     * is connected to the axis.
     */

#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_scope_add_channel(&axis.i_u, 1);
    esp_foc_scope_add_channel(&axis.i_v, 2);
    esp_foc_scope_add_channel(&axis.i_w, 3);
#endif

    ESP_LOGI(TAG, "Sensorless startup placeholder complete");
}
