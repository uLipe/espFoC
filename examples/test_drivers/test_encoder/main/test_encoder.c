/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Encoder characterization: real rotor sensor on a dummy axis (mock inverter +
 * mock isensor) exposed to the link layer for counts/degrees verification.
 *
 * Workflow:
 *   1. Flash and connect espFoC Studio / tunerctl on UART.
 *   2. CONNECT → SCOPE_START, rotate the shaft by hand.
 *   3. Scope channels: raw counts, mech turns, degrees, PLL omega, elec angle.
 *   4. READ ENC_COUNTS / ENC_DEG / ENC_TURNS for spot checks.
 *   5. EXEC ENC_SET_ZERO to re-zero the encoder.
 */

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "espFoC/esp_foc.h"
#include "espFoC/gui_link/esp_foc_tuner.h"
#include "espFoC/gui_link/esp_foc_link_session.h"
#include "espFoC/esp_foc_estimator_q16.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "mock_drivers.h"

#if defined(CONFIG_TEST_ENCODER_AS5600)
#include "espFoC/rotor_sensor_as5600.h"
#elif defined(CONFIG_TEST_ENCODER_AS5048)
#include "espFoC/rotor_sensor_as5048.h"
#endif

static const char *TAG = "encoder_char";

static esp_foc_axis_t s_axis;
static mock_inverter_t s_mock_inv;
static mock_isensor_t s_mock_isen;
static esp_foc_rotor_sensor_t *s_rotor;

static q16_t s_enc_deg_q16;
static q16_t s_enc_elec_deg_q16;

uint32_t esp_foc_tuner_firmware_type(void)
{
    return ESP_FOC_TUNER_FIRMWARE_TYPE_ENCHAR;
}

static esp_foc_rotor_sensor_t *encoder_new(void)
{
#if defined(CONFIG_TEST_ENCODER_AS5600)
    return rotor_sensor_as5600_new(
        CONFIG_TEST_ENCODER_ENC_SDA,
        CONFIG_TEST_ENCODER_ENC_SCL,
        0);
#elif defined(CONFIG_TEST_ENCODER_AS5048)
    return rotor_sensor_as5048_new(
        CONFIG_TEST_ENCODER_MOSI_PIN,
        CONFIG_TEST_ENCODER_MISO_PIN,
        CONFIG_TEST_ENCODER_SCK_PIN,
        CONFIG_TEST_ENCODER_CS_PIN,
        0,
        0);
#else
    return NULL;
#endif
}

static void encoder_axis_poll(esp_foc_axis_t *axis)
{
    if (axis->rotor_sensor_driver == NULL ||
        axis->rotor_sensor_driver->read_counts == NULL) {
        return;
    }

    q16_t ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_shaft_ticks = ticks;
    axis->rotor_position = ticks;

    q16_t theta_mech = q16_normalize_angle(q16_mul(ticks, axis->encoder_inv_cpr_q16));
    esp_foc_estimator_q16_set_meas(&axis->rotor_estimator, theta_mech);
    esp_foc_estimator_q16_step(&axis->rotor_estimator);

    s_enc_deg_q16 = q16_mul(theta_mech, q16_from_float(360.0f));
    q16_t theta_elec = esp_foc_estimator_q16_theta_elec(&axis->rotor_estimator);
    s_enc_elec_deg_q16 = q16_mul(theta_elec, q16_from_float(360.0f));
}

static void wire_scope_channels(void)
{
#if defined(CONFIG_ESP_FOC_SCOPE)
    esp_foc_scope_add_channel(&s_axis.rotor_shaft_ticks, 0);
    esp_foc_scope_add_channel(&s_axis.rotor_estimator.theta_meas_mech, 1);
    esp_foc_scope_add_channel(&s_enc_deg_q16, 2);
    esp_foc_scope_add_channel(&s_axis.rotor_estimator.omega_est_mech, 3);
    esp_foc_scope_add_channel(&s_enc_elec_deg_q16, 4);
    esp_foc_scope_add_channel(
        (const q16_t *)(const volatile void *)&s_axis.rotor_estimator.pll_err, 5);
    esp_foc_scope_initalize();
#endif
}

#if defined(CONFIG_ESP_FOC_SCOPE)
static void pump_scope_idle(void)
{
    for (int n = 0; n < CONFIG_ESP_FOC_SCOPE_BUFFER_SIZE; n++) {
        esp_foc_scope_data_push();
    }
}
#endif

static void encoder_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1);
    while (1) {
        encoder_axis_poll(&s_axis);
        vTaskDelay(period);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "encoder characterization — connect host, SCOPE, rotate shaft");

    mock_inverter_init(&s_mock_inv, 12.0f, (float)CONFIG_ESP_FOC_PWM_RATE_HZ);
    mock_isensor_init(&s_mock_isen);

    s_rotor = encoder_new();
    if (s_rotor == NULL) {
        ESP_LOGE(TAG, "encoder driver init failed");
        return;
    }

    esp_foc_motor_control_settings_t settings = {
        .motor_pole_pairs = CONFIG_TEST_ENCODER_POLE_PAIRS,
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_unit = 0,
    };

    esp_foc_err_t err = esp_foc_initialize_axis(
        &s_axis,
        mock_inverter_interface(&s_mock_inv),
        s_rotor,
        mock_isensor_interface(&s_mock_isen),
        settings);
    if (err != ESP_FOC_OK) {
        ESP_LOGE(TAG, "dummy axis init failed: %d", err);
        return;
    }

    wire_scope_channels();
#if defined(CONFIG_ESP_FOC_SCOPE)
    pump_scope_idle();
#endif

    ESP_ERROR_CHECK(esp_foc_link_attach_axis(0, &s_axis));

    if (esp_foc_task_spawn(encoder_task, NULL, 4096, 5, NULL) != 0) {
        ESP_LOGE(TAG, "encoder poll task spawn failed");
        return;
    }

    ESP_LOGW(TAG,
             ">>> Rotate the shaft. CONNECT + SCOPE_START on UART. "
             "READ 0x0057=counts 0x0058=deg 0x0059=turns. EXEC 0x00A6=zero.");

    while (1) {
#if defined(CONFIG_ESP_FOC_SCOPE)
        pump_scope_idle();
#endif
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
