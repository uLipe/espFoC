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

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_log.h"
#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
#include "driver/gpio.h"
#endif
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_simu_observer.h"
#include "espFoC/esp_foc_pll_observer.h"

#define ESP_FOC_ISENSOR_CALIBRATION_ROUNDS 100
#define ESP_FOC_VELOCITY_PID_DOWNSAMPLING  10
#define ESP_FOC_POSITION_PID_DOWNSAMPLING  100
#define ESP_FOC_ESTIMATORS_DOWNSAMPLING    4
#define ESP_FOC_DEBUG_PIN                  22

static const char * tag = "ESP_FOC";

static inline float esp_foc_ticks_to_radians_normalized(esp_foc_axis_t *axis)
{
    if(axis->rotor_sensor_driver) {
        axis->rotor_shaft_ticks =
            axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);

        axis->rotor_position =
            axis->rotor_shaft_ticks * axis->shaft_ticks_to_radians_ratio * axis->natural_direction;

        return esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(axis->rotor_position, axis->motor_pole_pairs)
        );

    } else {
        axis->rotor_position = axis->observer->get_angle(axis->observer);

        return esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(
                axis->rotor_position, 1.0f            )
        );
    }
}

static inline void esp_foc_motor_speed_estimator(esp_foc_axis_t * axis)
{
    if(axis->rotor_sensor_driver) {
        float raw_speed =  (axis->rotor_position - axis->rotor_position_prev) * axis->inv_dt;
        axis->current_speed = esp_foc_low_pass_filter_update( &axis->velocity_filter, raw_speed);
        axis->rotor_position_prev = axis->rotor_position;
    } else {
        axis->current_speed = axis->observer->get_speed(axis->observer);
    }
}

static inline void esp_foc_position_control_loop(esp_foc_axis_t *axis)
{
    /* position control is disabled */
    if(!axis->downsampling_position) return;

    axis->downsampling_position--;

    if(axis->downsampling_position == 0) {
        axis->downsampling_position = ESP_FOC_POSITION_PID_DOWNSAMPLING;

        axis->target_speed = esp_foc_pid_update( &axis->position_controller,
                                            axis->target_position,
                                            axis->rotor_position);
    }
}

static inline void esp_foc_velocity_control_loop(esp_foc_axis_t *axis)
{
    /* speed control is disabled */
    if(!axis->downsampling_speed) return;

    axis->downsampling_speed--;

    if(axis->downsampling_speed == 0) {

        axis->downsampling_speed = ESP_FOC_VELOCITY_PID_DOWNSAMPLING;

        axis->target_i_q.raw = esp_foc_pid_update( &axis->velocity_controller,
                                            axis->target_speed,
                                            axis->current_speed) ;
        axis->target_i_d.raw = 0.0f;
    }
}


static inline void esp_foc_current_control_loop(esp_foc_axis_t *axis)
{

    if(axis->enable_torque_control) {
        axis->u_q.raw =  esp_foc_pid_update( &axis->torque_controller[0],
                                            axis->target_i_q.raw,
                                            esp_foc_low_pass_filter_update(
                                                &axis->current_filters[0], axis->i_q.raw));

        axis->u_d.raw = esp_foc_pid_update( &axis->torque_controller[1],
                                            axis->target_i_d.raw,
                                            esp_foc_low_pass_filter_update(
                                                &axis->current_filters[1], axis->i_d.raw));
    } else {
        axis->u_q.raw =  axis->target_u_q.raw;
        axis->u_d.raw =  axis->target_u_d.raw;
    }
}

IRAM_ATTR static void esp_foc_sensors_loop(esp_foc_axis_t *axis)
{
    if(axis->downsampling_estimators > 0) {
        axis->downsampling_estimators--;
        if(axis->downsampling_estimators) {
            return;
        }
    }

#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_control_data_t control_data;
#endif

    esp_foc_observer_inputs_t in = {
            .i_dq[0] = axis->i_d.raw,
            .i_dq[1]  =  axis->i_q.raw,
            .u_dq[0] = axis->u_d.raw,
            .u_dq[1]  =  axis->u_q.raw,
            .u_alpha_beta[0] = axis->u_alpha.raw,
            .u_alpha_beta[1] = axis->u_beta.raw,
            .i_alpha_beta[0] = axis->i_alpha.raw,
            .i_alpha_beta[1] = axis->i_beta.raw,
    };

    axis->downsampling_estimators = ESP_FOC_ESTIMATORS_DOWNSAMPLING;

    if(axis->rotor_sensor_driver == NULL) {
        /* observer only when rotor sensor is not provided */
        axis->open_loop_observer->update(axis->open_loop_observer, &in);
        if(axis->isensor_driver != NULL) {
            int not_conv = axis->current_observer->update(axis->current_observer, &in);
            if(!not_conv) {
                //float obs = axis->current_observer->get_angle(axis->current_observer);
                // axis->current_observer->reset(axis->current_observer, axis->rotor_position);
               // axis->observer = axis->current_observer;
            }
        }
    }

    if(axis->isensor_driver != NULL) {
        isensor_values_t val;
        axis->isensor_driver->fetch_isensors(axis->isensor_driver, &val);
        axis->isensor_driver->sample_isensors(axis->isensor_driver);

        axis->i_u = val.iu_axis_0;
        axis->i_v = val.iv_axis_0;
        axis->i_w = val.iw_axis_0;
    }

    axis->rotor_elec_angle = esp_foc_ticks_to_radians_normalized(axis);
    esp_foc_motor_speed_estimator(axis);

#ifdef CONFIG_ESP_FOC_SCOPE
    esp_foc_get_control_data(axis, &control_data);
    esp_foc_scope_data_push(&control_data);
#endif
}

IRAM_ATTR static void esp_foc_core_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    float e_sin;
    float e_cos;

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    gpio_set_level(ESP_FOC_DEBUG_PIN, true);
#endif

    esp_foc_sensors_loop(axis);
    e_sin = esp_foc_sine(axis->rotor_elec_angle);
    e_cos = esp_foc_cosine(axis->rotor_elec_angle);

    esp_foc_get_dq_currents(e_sin,
                            e_cos,
                            axis->i_u,
                            axis->i_v,
                            axis->i_w,
                            &axis->i_alpha.raw,
                            &axis->i_beta.raw,
                            &axis->i_q.raw,
                            &axis->i_d.raw);

    esp_foc_position_control_loop(axis);
    esp_foc_velocity_control_loop(axis);
    esp_foc_current_control_loop(axis);


    esp_foc_modulate_dq_voltage(e_sin,
                    e_cos,
                    axis->u_d.raw,
                    axis->u_q.raw,
                    &axis->u_alpha.raw,
                    &axis->u_beta.raw,
                    &axis->u_u.raw,
                    &axis->u_v.raw,
                    &axis->u_w.raw,
                    axis->biased_dc_link_voltage,
                    axis->dc_link_to_normalized);

    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                        axis->u_u.raw,
                                        axis->u_v.raw,
                                        axis->u_w.raw);

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    gpio_set_level(ESP_FOC_DEBUG_PIN, false);
#endif
}

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                    esp_foc_inverter_t *inverter,
                                    esp_foc_rotor_sensor_t *rotor,
                                    esp_foc_isensor_t *isensor,
                                    esp_foc_motor_control_settings_t settings)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    if(inverter == NULL) {
        ESP_LOGE(tag, "invalid inverter driver!");
        return ESP_FOC_ERR_INVALID_ARG;
    }

#ifdef CONFIG_ESP_FOC_SENSORLESS
    if(isensor == NULL) {
        ESP_LOGE(tag, "Current sensor is mandatory on sensorless");
        return ESP_FOC_ERR_INVALID_ARG;
    }
#endif

#ifdef CONFIG_ESP_FOC_CUSTOM_MATH
    extern void esp_foc_fast_init_sqrt_table(void);
    esp_foc_fast_init_sqrt_table();
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        gpio_config_t drv_en_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << ESP_FOC_DEBUG_PIN,
        };
        gpio_config(&drv_en_config);
        gpio_set_level(ESP_FOC_DEBUG_PIN, false);
#endif

    float current_control_analog_bandwith;

    axis->rotor_aligned = ESP_FOC_ERR_AXIS_INVALID_STATE;
    axis->inverter_driver = inverter;
    axis->isensor_driver = isensor;

    axis->dc_link_voltage =
        axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);

#ifdef CONFIG_ESP_FOC_USE_SINE_PWM
    axis->biased_dc_link_voltage = axis->dc_link_voltage / 4.0f;
    axis->dc_link_to_normalized = 1.0f / axis->dc_link_voltage;
#else
    axis->biased_dc_link_voltage = (axis->dc_link_voltage * ESP_FOC_SQRT3_TWO) / 2.0f;
    axis->dc_link_to_normalized = 1.0f / (axis->dc_link_voltage * ESP_FOC_SQRT3_TWO);
#endif
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0.0, 0.0, 0.0);

    ESP_LOGI(tag,"inverter dc-link voltage: %f[V]", axis->dc_link_voltage);
    ESP_LOGI(tag, "FoC max voltage: %f",  axis->biased_dc_link_voltage);
    ESP_LOGI(tag, "FoC normalizer scale: %f",  axis->dc_link_to_normalized);

    axis->dt = (1.0f / inverter->get_inverter_pwm_rate(inverter));
    axis->inv_dt = (1.0f / (axis->dt * ESP_FOC_ESTIMATORS_DOWNSAMPLING));
    axis->target_speed = 0.0;
    axis->target_position = 0.0f;

    axis->downsampling_position = settings.enable_position_control ? ESP_FOC_POSITION_PID_DOWNSAMPLING : 0;
    axis->downsampling_speed = settings.enable_velocity_control ? ESP_FOC_VELOCITY_PID_DOWNSAMPLING : 0;
    axis->downsampling_estimators = ESP_FOC_ESTIMATORS_DOWNSAMPLING;

    axis->i_d.raw = 0.0f;
    axis->i_q.raw = 0.0f;
    axis->u_d.raw = 0.0f;
    axis->u_q.raw = 0.0f;
    axis->target_u_d.raw = 0.0f;
    axis->target_u_q.raw = 0.0f;
    axis->target_i_d.raw = 0.0f;
    axis->target_i_q.raw = 0.0f;

    axis->position_controller.kp = settings.position_control_settings.kp;
    axis->position_controller.ki = settings.position_control_settings.ki;
    axis->position_controller.kd = settings.position_control_settings.kd;
    axis->position_controller.integrator_limit = settings.position_control_settings.integrator_limit;
    axis->position_controller.max_output_value = settings.position_control_settings.max_output_value;
    axis->position_controller.dt = axis->dt * ESP_FOC_POSITION_PID_DOWNSAMPLING;
    axis->position_controller.inv_dt = (1.0f / axis->position_controller.dt);
    esp_foc_pid_reset(&axis->position_controller);

    axis->velocity_controller.kp = settings.velocity_control_settings.kp;
    axis->velocity_controller.ki = settings.velocity_control_settings.ki;
    axis->velocity_controller.kd = settings.velocity_control_settings.kd;
    axis->velocity_controller.integrator_limit = settings.velocity_control_settings.integrator_limit;
    axis->velocity_controller.max_output_value = settings.velocity_control_settings.max_output_value;
    axis->velocity_controller.dt = axis->dt * ESP_FOC_VELOCITY_PID_DOWNSAMPLING;
    axis->velocity_controller.inv_dt = (1.0f / axis->velocity_controller.dt);
    esp_foc_pid_reset(&axis->velocity_controller);
    esp_foc_low_pass_filter_init(&axis->velocity_filter, 0.9f);

    current_control_analog_bandwith = (2.0f * M_PI * (inverter->get_inverter_pwm_rate(inverter) / 200.0f));

    axis->torque_controller[0].kp = settings.motor_inductance * current_control_analog_bandwith;
    axis->torque_controller[0].ki = settings.motor_resistance * current_control_analog_bandwith;
    axis->torque_controller[0].kd = 0.0f;
    axis->torque_controller[0].integrator_limit = 1e+6f;
    axis->torque_controller[0].max_output_value = axis->biased_dc_link_voltage/*settings.torque_control_settings[0].max_output_value*/;
    axis->torque_controller[0].dt = axis->dt;
    axis->torque_controller[0].inv_dt = (1.0f / axis->torque_controller[0].dt);
    esp_foc_pid_reset(&axis->torque_controller[0]);
    esp_foc_low_pass_filter_init(&axis->current_filters[0], 1.0);

    axis->torque_controller[1].kp = settings.motor_inductance * current_control_analog_bandwith;
    axis->torque_controller[1].ki = settings.motor_resistance * current_control_analog_bandwith;
    axis->torque_controller[1].kd = 0.0f;
    axis->torque_controller[1].integrator_limit = 1e+6f;
    axis->torque_controller[1].max_output_value = axis->biased_dc_link_voltage /* settings.torque_control_settings[1].max_output_value */;
    axis->torque_controller[1].dt = axis->dt;
    axis->torque_controller[1].inv_dt = (1.0f / axis->torque_controller[1].dt);
    esp_foc_pid_reset(&axis->torque_controller[1]);
    esp_foc_low_pass_filter_init(&axis->current_filters[1], 1.0);

    ESP_LOGI(tag, "Position controller is: %s",  settings.enable_position_control ? "on" : "off");
    ESP_LOGI(tag, "Speed controller is: %s",  settings.enable_velocity_control ? "on" : "off");
    ESP_LOGI(tag, "Position controller is: %s",  settings.enable_torque_control ? "on" : "off");

    axis->enable_torque_control = settings.enable_torque_control;
    axis->motor_pole_pairs = (float)settings.motor_pole_pairs;
    ESP_LOGI(tag, "Motor poler pairs: %f",  axis->motor_pole_pairs);

    if(rotor != NULL) {
        axis->rotor_sensor_driver = rotor;
    } else {

        axis->open_loop_observer = simu_observer_new(settings.motor_unit,
            (esp_foc_simu_observer_settings_t) {
                .phase_resistance = settings.motor_resistance,
                .phase_inductance = settings.motor_inductance,
                .pole_pairs = (float)settings.motor_pole_pairs,
                .dt = axis->dt * ESP_FOC_ESTIMATORS_DOWNSAMPLING,
            }
        );

        if(axis->open_loop_observer == NULL) {
            ESP_LOGE(tag, "FAiled to setup the observer");
            return ESP_FOC_ERR_INVALID_ARG;
        }

        axis->observer = axis->open_loop_observer;
    }

    if(axis->isensor_driver != NULL) {
        axis->isensor_driver->calibrate_isensors(axis->isensor_driver,
                                                ESP_FOC_ISENSOR_CALIBRATION_ROUNDS);

        #define PLL_BANDWIDTH_HZ 200.0f
        #define PLL_ZETA 0.707f

        axis->current_observer = pll_observer_new(settings.motor_unit,
                                (esp_foc_pll_observer_settings_t) {
                                .pll_kp = 2.0f * PLL_ZETA * 2.0f * M_PI * PLL_BANDWIDTH_HZ,
                                .pll_ki = (2.0f * M_PI * PLL_BANDWIDTH_HZ ) *
                                          (2.0f * M_PI * PLL_BANDWIDTH_HZ),
                                .phase_resistance = settings.motor_resistance,
                                .phase_inductance = settings.motor_inductance,
                                .pole_pairs = axis->motor_pole_pairs,
                                .flux_linkage = settings.flux_linkage,
                                .inertia = settings.inertia,
                                .friction = settings.friction,
                                .dt = axis->dt * ESP_FOC_ESTIMATORS_DOWNSAMPLING
        });
    }

    if(axis->rotor_sensor_driver) {
        axis->shaft_ticks_to_radians_ratio = ((2.0 * M_PI)) /
            axis->rotor_sensor_driver->get_counts_per_revolution(axis->rotor_sensor_driver);
        ESP_LOGI(tag, "Shaft to ticks ratio: %f", axis->shaft_ticks_to_radians_ratio);
    }

    esp_foc_sleep_ms(250);
    axis->rotor_aligned = ESP_FOC_ERR_NOT_ALIGNED;
    axis->natural_direction = (settings.natural_direction == ESP_FOC_MOTOR_NATURAL_DIRECTION_CW) ?
                                1.0f : -1.0f;

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis)
{
    float a,b,u, v, w, e_sin, e_cos;

    if(axis == NULL) {
        ESP_LOGE(tag, "Invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_ERR_NOT_ALIGNED) {
        ESP_LOGE(tag, "This rotor was aligned already!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    axis->rotor_aligned = ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;

    ESP_LOGI(tag, "Starting to align the rotor");

    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                        0.0f,
                                        0.0f,
                                        0.0f);

    e_sin = esp_foc_sine(0.0f);
    e_cos = esp_foc_cosine(0.0f);

    axis->inverter_driver->enable(axis->inverter_driver);
    esp_foc_sleep_ms(500);

    esp_foc_modulate_dq_voltage(e_sin, e_cos, 1.0f, 0.0f,&a, &b, &u, &v,&w, axis->biased_dc_link_voltage,
                            axis->dc_link_to_normalized);

    axis->inverter_driver->set_voltages(axis->inverter_driver, u, v, w);

    esp_foc_sleep_ms(500);

    if(axis->rotor_sensor_driver) {
        float current_ticks;
        current_ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
        ESP_LOGI(tag, "rotor ticks offset: %f [ticks] for Coil U", current_ticks);

        axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);
    } else {
        axis->open_loop_observer->reset(axis->open_loop_observer, 0.0f);
        axis->current_observer->reset(axis->current_observer, 0.0f);
    }

    axis->rotor_aligned = ESP_FOC_OK;
    ESP_LOGI(tag, "Done, rotor aligned!");

    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                        0.0f,
                                        0.0f,
                                        0.0f);

    esp_foc_sleep_ms(500);

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_set_target_voltage(esp_foc_axis_t *axis,
                                        esp_foc_q_voltage uq,
                                        esp_foc_d_voltage ud)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        ESP_LOGE(tag, "align rotor first!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    if(uq.raw > (axis->biased_dc_link_voltage)) {
        uq.raw = (axis->biased_dc_link_voltage);
    } else if (uq.raw < -(axis->biased_dc_link_voltage)){
        uq.raw = -(axis->biased_dc_link_voltage);
    }

    if(ud.raw > (axis->biased_dc_link_voltage)) {
        ud.raw = (axis->biased_dc_link_voltage);
    } else if (ud.raw < -(axis->biased_dc_link_voltage)){
        ud.raw = -(axis->biased_dc_link_voltage);
    }

    esp_foc_critical_enter();

    axis->target_u_q = uq;
    axis->target_u_d = ud;

    esp_foc_critical_leave();

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_set_target_current(esp_foc_axis_t *axis, esp_foc_q_current iq, esp_foc_d_current id)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        ESP_LOGE(tag, "align rotor first!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    esp_foc_critical_enter();

    axis->target_i_q = iq;
    axis->target_i_d = id;

    esp_foc_critical_leave();

    return ESP_FOC_OK;
}



IRAM_ATTR esp_foc_err_t esp_foc_set_target_speed(esp_foc_axis_t *axis,
                                                esp_foc_radians_per_second speed)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        ESP_LOGE(tag, "align rotor first!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    esp_foc_critical_enter();
    axis->target_speed = speed.raw;
    esp_foc_critical_leave();

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_set_target_position(esp_foc_axis_t *axis,
                                                    esp_foc_radians position)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        ESP_LOGE(tag, "align rotor first!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    esp_foc_critical_enter();
    axis->target_position = position.raw;
    esp_foc_critical_leave();

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        ESP_LOGE(tag, "align rotor first!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

#ifdef CONFIG_ESP_FOC_SCOPE
    /* Start the scope if it has not been yet: */
    esp_foc_scope_initalize();
#endif

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
                                    esp_foc_core_loop,
                                    axis);

    ESP_LOGI(tag, "Starting foc loop task for axis: %p", axis);
    ESP_LOGI(tag, "FoC core base rate [Samples/S]: %f", axis->inverter_driver->get_inverter_pwm_rate(axis->inverter_driver));

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_get_control_data(esp_foc_axis_t *axis, esp_foc_control_data_t *control_data)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    if(control_data == NULL) {
        ESP_LOGE(tag, "invalid control data object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();

    control_data->u_alpha = axis->u_alpha;
    control_data->u_beta = axis->u_beta;

    control_data->i_alpha = axis->i_alpha;
    control_data->i_beta = axis->i_beta;

    control_data->u = axis->u_u;
    control_data->v = axis->u_v;
    control_data->w = axis->u_w;

    control_data->out_q = axis->u_q;
    control_data->out_d = axis->u_d;
    control_data->dt.raw = axis->dt;

    control_data->speed.raw = axis->current_speed;
    control_data->target_position.raw = axis->target_position;
    control_data->target_speed.raw = axis->target_speed;

    if(axis->rotor_sensor_driver == NULL) {
        control_data->rotor_position.raw =  axis->open_loop_observer->get_angle(axis->open_loop_observer);
        control_data->observer_angle.raw = axis->current_observer->get_angle(axis->current_observer);
    } else {
        control_data->rotor_position.raw =  axis->rotor_position;
        control_data->observer_angle.raw = 0.0f;
    }

    control_data->i_u.raw = axis->i_u;
    control_data->i_v.raw = axis->i_v;
    control_data->i_w.raw = axis->i_w;

    control_data->i_q = axis->i_q;
    control_data->i_d = axis->i_d;

    esp_foc_critical_leave();

    return ESP_FOC_OK;
}
