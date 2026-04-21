/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * tuner_studio_target — service-mode firmware for live tuning with
 * espFoC TunerStudio. Initialises the inverter, encoder and shunts
 * from the Kconfig pin map, attaches the axis to the runtime tuner
 * and parks the motor at zero current. Everything else (alignment,
 * motion, persistence, app generation) is driven from the host.
 *
 * Use the Kconfig pin entries under "TunerStudio target — pin map"
 * to match your hardware before flashing.
 */

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_tuner.h"
#include "espFoC/inverter_6pwm_mcpwm.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "espFoC/utils/esp_foc_q16.h"

static const char *TAG = "tuner-target";

static esp_foc_axis_t s_axis;
static esp_foc_inverter_t *s_inverter;
static esp_foc_rotor_sensor_t *s_rotor;

/* Advertise as a TunerStudio target so the GUI can light up its
 * Generate App tab on connect. */
uint32_t esp_foc_tuner_firmware_type(void)
{
    return ESP_FOC_TUNER_FIRMWARE_TYPE_TSGX;
}

/* The default regulation callback while no tuner override is active:
 * keep the axis parked at zero current so the motor sits idle. */
static void parked_regulation_cb(esp_foc_axis_t *axis,
                                 esp_foc_d_current_q16_t *id_ref,
                                 esp_foc_q_current_q16_t *iq_ref,
                                 esp_foc_d_voltage_q16_t *ud_ff,
                                 esp_foc_q_voltage_q16_t *uq_ff)
{
    (void)axis;
    id_ref->raw = 0;
    iq_ref->raw = 0;
    ud_ff->raw = 0;
    uq_ff->raw = 0;
}

static void wire_scope_channels(void)
{
#if defined(CONFIG_ESP_FOC_SCOPE)
    /* Convention enforced across the host: ch0/1/2 carry the SVPWM
     * three-phase voltages, so the GUI's SVM hexagon picks them up
     * unchanged. Extra channels can be added by user code later. */
    esp_foc_scope_add_channel(&s_axis.u_u.raw, 0);
    esp_foc_scope_add_channel(&s_axis.u_v.raw, 1);
    esp_foc_scope_add_channel(&s_axis.u_w.raw, 2);
    esp_foc_scope_add_channel(&s_axis.target_i_q.raw, 3);
    esp_foc_scope_add_channel(&s_axis.i_q.raw, 4);
    esp_foc_scope_add_channel(&s_axis.u_q.raw, 5);
    esp_foc_scope_initalize();
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot — tuner_studio_target");

    s_inverter = inverter_6pwm_mpcwm_new(
        CONFIG_TUNER_TARGET_PWM_U_HI,
        CONFIG_TUNER_TARGET_PWM_U_LO,
        CONFIG_TUNER_TARGET_PWM_V_HI,
        CONFIG_TUNER_TARGET_PWM_V_LO,
        CONFIG_TUNER_TARGET_PWM_W_HI,
        CONFIG_TUNER_TARGET_PWM_W_LO,
        -1,
        (float)CONFIG_TUNER_TARGET_DC_LINK_V,
        0);
    if (s_inverter == NULL) {
        ESP_LOGE(TAG, "inverter init failed");
        return;
    }

    s_rotor = rotor_sensor_as5600_new(
        CONFIG_TUNER_TARGET_ENC_SDA,
        CONFIG_TUNER_TARGET_ENC_SCL,
        0);
    if (s_rotor == NULL) {
        ESP_LOGE(TAG, "rotor sensor init failed");
        return;
    }

    esp_foc_motor_control_settings_t settings = {
        .motor_pole_pairs  = CONFIG_TUNER_TARGET_POLE_PAIRS,
        .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
        .motor_unit        = 0,
    };

    /* Current sensor wiring is left out of this service firmware on
     * purpose: voltage-mode tuning is enough to dial the loop in,
     * and shipping a fixed shunt config would clash with whatever
     * board the operator is using. The Generate App flow lets the
     * user pick the ADC mode for the production firmware. */
    esp_foc_err_t err = esp_foc_initialize_axis(&s_axis, s_inverter,
                                                s_rotor, NULL, settings);
    if (err != ESP_FOC_OK) {
        ESP_LOGE(TAG, "axis init failed: %d", err);
        return;
    }

    wire_scope_channels();

    esp_foc_set_regulation_callback(&s_axis, parked_regulation_cb);
    ESP_ERROR_CHECK(esp_foc_tuner_attach_axis(0, &s_axis));

    /* Do NOT call esp_foc_align_axis() here — the operator triggers
     * alignment from TunerStudio (CMD_ALIGN_AXIS). esp_foc_run() is
     * also deferred until alignment completes, otherwise the run
     * call would refuse with ESP_FOC_ERR_NOT_ALIGNED. */
    ESP_LOGI(TAG, "ready — connect TunerStudio and click Align");

    while (1) {
        if (s_axis.rotor_aligned == ESP_FOC_OK) {
            ESP_LOGI(TAG, "alignment detected — starting control loop");
            esp_foc_run(&s_axis);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Keep the main task alive so RTOS doesn't reap it. */
    vTaskDelete(NULL);
}
