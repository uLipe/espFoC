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

#include <math.h>
#include "esp_log.h"
#include "espFoC/esp_foc_iq31_api.h"
#include "espFoC/esp_foc_controls.h"
#include "espFoC/esp_foc.h"

#if CONFIG_ESP_FOC_USE_FIXED_POINT
static const char *tag = "ESP_FOC_CORE_IQ31";

static void do_foc_outer_loop_iq31(void *arg)
{
    esp_foc_axis_iq31_t *axis = (esp_foc_axis_iq31_t *)arg;
    axis->regulator_ev = esp_foc_get_event_handle();

    while (1) {
        esp_foc_wait_notifier();
        if (axis->regulator_cb) {
            axis->regulator_cb(axis, &axis->target_i_d, &axis->target_i_q, &axis->target_u_d, &axis->target_u_q);
        }
    }
}

esp_foc_err_t esp_foc_initialize_axis_iq31(esp_foc_axis_iq31_t *axis,
                                           esp_foc_inverter_t *inverter,
                                           esp_foc_rotor_sensor_t *rotor,
                                           esp_foc_isensor_t *isensor,
                                           esp_foc_motor_control_settings_iq31_t settings)
{
    float current_control_analog_bandwith;
    float dc_link_voltage_f;
    float pwm_rate_hz_f;
    float dt_f;
    float motor_r_f;
    float motor_l_f;
    float motor_pp_f;

    if (axis == NULL || inverter == NULL || rotor == NULL) {
        ESP_LOGE(tag, "invalid args for iq31 axis initialization");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    axis->rotor_aligned = ESP_FOC_ERR_AXIS_INVALID_STATE;
    axis->inverter_driver = inverter;
    axis->rotor_sensor_driver = rotor;
    axis->isensor_driver = isensor;

    dc_link_voltage_f = axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);
    axis->dc_link_voltage = iq31_from_float(dc_link_voltage_f);
    axis->biased_dc_link_voltage = iq31_from_float(dc_link_voltage_f / 2.0f);
    axis->dc_link_to_normalized = iq31_from_float((dc_link_voltage_f > 1e-9f) ? (1.0f / dc_link_voltage_f) : 0.0f);

#ifdef CONFIG_ESP_FOC_USE_SINE_PWM
    axis->max_voltage = iq31_from_float(dc_link_voltage_f / 2.0f);
#else
    axis->max_voltage = iq31_from_float(dc_link_voltage_f / ESP_FOC_CLARKE_PARK_SQRT3);
#endif

    axis->inverter_driver->set_voltages_iq31(axis->inverter_driver, 0, 0, 0);

    pwm_rate_hz_f = inverter->get_inverter_pwm_rate(inverter);
    dt_f = (pwm_rate_hz_f > 1e-9f) ? (1.0f / pwm_rate_hz_f) : 0.0f;
    axis->dt = iq31_from_float(dt_f);
    axis->inv_dt = iq31_from_float((dt_f > 1e-9f) ? (1.0f / dt_f) : 0.0f);

    axis->i_d.raw = 0;
    axis->i_q.raw = 0;
    axis->u_d.raw = 0;
    axis->u_q.raw = 0;
    axis->target_i_d.raw = 0;
    axis->target_i_q.raw = 0;
    axis->target_u_d.raw = 0;
    axis->target_u_q.raw = 0;
    axis->skip_torque_control = 0;

    if (isensor != NULL) {
        axis->isensor_driver->calibrate_isensors(axis->isensor_driver, ESP_FOC_ISENSOR_CALIBRATION_ROUNDS);
    }

    current_control_analog_bandwith = (2.0f * (float)M_PI * 150.0f);
    motor_r_f = iq31_to_float(settings.motor_resistance);
    motor_l_f = iq31_to_float(settings.motor_inductance);
    motor_pp_f = (float)settings.motor_pole_pairs;

    axis->torque_controller[0].dt = dt_f * ESP_FOC_LOW_SPEED_DOWNSAMPLING;
    axis->torque_controller[0].inv_dt = (axis->torque_controller[0].dt > 1e-9f) ? (1.0f / axis->torque_controller[0].dt) : 0.0f;
    axis->torque_controller[0].kp = motor_l_f * current_control_analog_bandwith;
    axis->torque_controller[0].ki = motor_r_f * current_control_analog_bandwith;
    axis->torque_controller[0].kd = 0.0f;
    axis->torque_controller[0].integrator_limit = iq31_to_float(axis->max_voltage) / axis->torque_controller[0].ki;
    axis->torque_controller[0].max_output_value = iq31_to_float(axis->max_voltage);
    axis->torque_controller[0].min_output_value = -iq31_to_float(axis->max_voltage);
    esp_foc_pid_iq31_reset(&axis->torque_controller[0]);
    esp_foc_low_pass_filter_set_cutoff_iq31(&axis->current_filters[0],
                                            0.1f * axis->torque_controller[0].inv_dt,
                                            axis->torque_controller[0].inv_dt);

    axis->torque_controller[1].dt = dt_f * ESP_FOC_LOW_SPEED_DOWNSAMPLING;
    axis->torque_controller[1].inv_dt = (axis->torque_controller[1].dt > 1e-9f) ? (1.0f / axis->torque_controller[1].dt) : 0.0f;
    axis->torque_controller[1].kp = motor_l_f * current_control_analog_bandwith;
    axis->torque_controller[1].ki = motor_r_f * current_control_analog_bandwith;
    axis->torque_controller[1].kd = 0.0f;
    axis->torque_controller[1].integrator_limit = (iq31_to_float(axis->max_voltage) * 0.1f) / axis->torque_controller[1].ki;
    axis->torque_controller[1].max_output_value = iq31_to_float(axis->max_voltage) * 0.1f;
    axis->torque_controller[1].min_output_value = -(iq31_to_float(axis->max_voltage) * 0.1f);
    esp_foc_pid_iq31_reset(&axis->torque_controller[1]);
    esp_foc_low_pass_filter_set_cutoff_iq31(&axis->current_filters[1],
                                            0.1f * axis->torque_controller[1].inv_dt,
                                            axis->torque_controller[1].inv_dt);

    axis->motor_pole_pairs = iq31_from_float(motor_pp_f);
    axis->shaft_ticks_to_radians_ratio = iq31_from_float(((2.0f * (float)M_PI)) /
                                                         axis->rotor_sensor_driver->get_counts_per_revolution(axis->rotor_sensor_driver));
    axis->natural_direction = (settings.natural_direction == ESP_FOC_MOTOR_NATURAL_DIRECTION_CW) ? IQ31_ONE : IQ31_MINUS_ONE;

    axis->high_speed_loop_cb = do_current_mode_sensored_high_speed_loop_iq31;
    axis->low_speed_loop_cb = do_current_mode_sensored_low_speed_loop_iq31;
    axis->outer_loop_cb = do_foc_outer_loop_iq31;

    esp_foc_sleep_ms(250);
    axis->rotor_aligned = ESP_FOC_ERR_NOT_ALIGNED;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_align_axis_iq31(esp_foc_axis_iq31_t *axis)
{
    iq31_t e_sin, e_cos;
    iq31_t a, b, u, v, w;

    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->rotor_aligned != ESP_FOC_ERR_NOT_ALIGNED) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->rotor_aligned = ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;
    axis->inverter_driver->set_voltages_iq31(axis->inverter_driver, 0, 0, 0);

    e_sin = iq31_sin(0);
    e_cos = iq31_cos(0);

    axis->inverter_driver->enable(axis->inverter_driver);
    esp_foc_sleep_ms(500);

    esp_foc_modulate_dq_voltage_iq31(e_sin,
                                     e_cos,
                                     IQ31_ONE,
                                     0,
                                     &a, &b, &u, &v, &w,
                                     axis->max_voltage,
                                     axis->biased_dc_link_voltage,
                                     axis->dc_link_to_normalized);
    axis->inverter_driver->set_voltages_iq31(axis->inverter_driver, u, v, w);
    esp_foc_sleep_ms(500);

    axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);
    axis->rotor_aligned = ESP_FOC_OK;
    axis->inverter_driver->set_voltages_iq31(axis->inverter_driver, 0, 0, 0);
    esp_foc_sleep_ms(500);
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_run_iq31(esp_foc_axis_iq31_t *axis)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->rotor_aligned != ESP_FOC_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    esp_foc_create_runner(axis->outer_loop_cb, axis, 2);
    esp_foc_create_runner(axis->low_speed_loop_cb, axis, 1);
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_set_regulation_callback_iq31(esp_foc_axis_iq31_t *axis,
                                                   esp_foc_motor_regulation_callback_iq31_t callback)
{
    if (axis == NULL || callback == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    esp_foc_critical_enter();
    axis->regulator_cb = callback;
    esp_foc_critical_leave();
    return ESP_FOC_OK;
}
#endif

