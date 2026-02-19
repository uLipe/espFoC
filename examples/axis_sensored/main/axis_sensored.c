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

#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/current_sensor_adc_one_shot.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/esp_foc.h"


#ifdef CONFIG_IDF_TARGET_ESP32P4
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#endif

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;
static esp_foc_rotor_sensor_t *sensor;

static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    /*Motor part number: QBL4208 - 64 - 013 */
    .motor_pole_pairs = 4,
    .motor_inductance = 0.0018f,
    .motor_resistance = 1.08f,
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};


#ifdef CONFIG_IDF_TARGET_ESP32P4
/* USB backed for espfoc scope */
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
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    /* the second way to register a callback */
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

static void regulation_callback (esp_foc_axis_t *axis, esp_foc_d_current  *id_ref, esp_foc_q_current *iq_ref,
                                                    esp_foc_d_voltage *ud_forward, esp_foc_q_voltage *uq_forward)
{

    float vq_base = 0.0f;
    float vd_base = 0.0f;

    float iq_base = 2.0f;
    float id_base = 0.0f;

    uq_forward->raw = vq_base;
    ud_forward->raw = vd_base;
    iq_ref->raw = iq_base;
    id_ref->raw = id_base;

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
    sensor = rotor_sensor_as5600_new(
        CONFIG_FOC_ENCODER_SDA_PIN,
        CONFIG_FOC_ENCODER_SCL_PIN,
        0
    );

    if(sensor == NULL) {
        ESP_LOGE(TAG, "failed to create as5600 encoder driver");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    float vramp = 5.0f;
    float step = 0.2f;

    esp_foc_q_voltage uq = {.raw = 0.0f};
    esp_foc_q_current iq = {.raw = 0.0f};

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        sensor,
        shunts,
        settings
    );

    /* Add some channels to monitor the currents */
#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_scope_add_channel(&axis.i_u, 1);
    esp_foc_scope_add_channel(&axis.i_v, 2);
    esp_foc_scope_add_channel(&axis.i_w, 3);
#endif

    esp_foc_align_axis(&axis);
    esp_foc_run(&axis);
    esp_foc_set_regulation_callback(&axis, regulation_callback);
}