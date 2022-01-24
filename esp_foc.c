#include <string.h>
#include <math.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/esp_foc.h"

static const char * tag = "ESP_FOC";

static const float ALIGN_ANGLE_CONSTANT = ((3.0f * M_PI) / 2.0f) + (2.0f * M_PI); 

static inline float esp_foc_ticks_to_radians_normalized(esp_foc_axis_t *axis)
{
    axis->rotor_shaft_ticks = 
        axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_position = 
        axis->rotor_shaft_ticks * axis->shaft_ticks_to_radians_ratio;
    
    return esp_foc_normalize_angle(
        esp_foc_mechanical_to_elec_angle(
            axis->rotor_position, axis->motor_pole_pairs
        )
    );
}

static inline void esp_foc_position_control_loop(esp_foc_axis_t *axis)
{
    /* position control is disabled */
    if(!axis->downsampling_position_reload_value) return;

    axis->downsampling_position--;

    if(axis->downsampling_position == 0) {

        axis->target_speed = esp_foc_pid_update( &axis->position_controller,
                                            axis->target_position,
                                            axis->rotor_position);
        axis->downsampling_speed = axis->downsampling_speed_reload_value;
    }
}

static inline void esp_foc_velocity_control_loop(esp_foc_axis_t *axis)
{
    /* speed control is disabled */
    if(!axis->downsampling_speed_reload_value) return;

    axis->downsampling_speed--;
    axis->current_speed = 
        (axis->rotor_position - axis->rotor_position_prev) / axis->dt;
    axis->rotor_position_prev = axis->rotor_position;

    if(axis->downsampling_speed == 0) {

        axis->target_i_q.raw = esp_foc_pid_update( &axis->velocity_controller,
                                            axis->target_speed,
                                            esp_foc_low_pass_filter_update(
                                                &axis->velocity_filter, axis->current_speed));
        axis->target_i_d.raw = 0.0f;
        axis->downsampling_speed = axis->downsampling_speed_reload_value;
    }
}


static inline void esp_foc_torque_control_loop(esp_foc_axis_t *axis)
{

    axis->u_q.raw = esp_foc_pid_update( &axis->torque_controller[0],
                                        axis->target_i_q.raw,
                                        esp_foc_low_pass_filter_update(
                                            &axis->current_filters[0], axis->i_q.raw));
   
    axis->u_d.raw = esp_foc_pid_update( &axis->torque_controller[1],
                                        axis->target_i_d.raw,
                                        esp_foc_low_pass_filter_update(
                                            &axis->current_filters[1], axis->i_d.raw));
}

IRAM_ATTR void esp_foc_loop(void *arg)
{
    float now;
    float theta;
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

    ESP_LOGI(tag, "starting foc loop task for axis: %p", axis);

    for(;;) {
        now = (float) esp_foc_now_seconds();
        axis->dt =  now - axis->last_timestamp;
        axis->last_timestamp = now;

        theta = esp_foc_ticks_to_radians_normalized(axis); 

        ESP_LOGD(tag, "rotor position: %f [ticks] at time: %f [s]", axis->rotor_shaft_ticks, now);

        esp_foc_position_control_loop(axis);
        esp_foc_velocity_control_loop(axis);
        esp_foc_torque_control_loop(axis);

        esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                        theta, 
                        axis->u_d.raw, 
                        axis->u_q.raw, 
                        &axis->u_u.raw, 
                        &axis->u_v.raw, 
                        &axis->u_w.raw);

        ESP_LOGD(tag, "Calculated voltage U: %f [V]", axis->u_u.raw);
        ESP_LOGD(tag, "Calculated voltage V: %f [V]", axis->u_v.raw);
        ESP_LOGD(tag, "Calculated voltage W: %f [V]", axis->u_w.raw);

        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                            axis->u_u.raw, 
                                            axis->u_v.raw, 
                                            axis->u_w.raw);

        esp_foc_runner_yield();
    }
}

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                    esp_foc_inverter_t *inverter,
                                    esp_foc_rotor_sensor_t *rotor,
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

    if(rotor == NULL) {
        ESP_LOGE(tag, "invalid rotor sensor driver!");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    axis->inverter_driver = inverter;
    axis->rotor_sensor_driver = rotor;


    axis->dc_link_voltage = 
        axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);
    axis->biased_dc_link_voltage = axis->dc_link_voltage * 0.5;
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0.0, 0.0, 0.0);
    ESP_LOGI(tag,"inverter dc-link voltage: %f[V]", axis->dc_link_voltage);

    axis->dt = 0.0;
    axis->last_timestamp = 0.0;
    axis->target_speed = 0.0;
    axis->target_position = 0.0f;

    axis->downsampling_speed_reload_value = 0;
    axis->downsampling_position_reload_value = 0;

    axis->i_d.raw = 0.0f;
    axis->i_q.raw = 0.0f;
    axis->u_d.raw = 0.0f;
    axis->u_q.raw = 0.0f;
    axis->target_i_d.raw = 0.0f;
    axis->target_i_q.raw = 0.0f;

    axis->position_controller.kp = settings.position_control_settings.kp;
    axis->position_controller.ki = settings.position_control_settings.ki;
    axis->position_controller.kd = settings.position_control_settings.kd;
    axis->position_controller.integrator_limit = settings.position_control_settings.integrator_limit;
    axis->position_controller.max_output_value = settings.position_control_settings.max_output_value;
    esp_foc_pid_reset(&axis->position_controller);
    axis->downsampling_position = settings.downsampling_position_rate;
    axis->downsampling_position_reload_value = settings.downsampling_position_rate;;

    axis->velocity_controller.kp = settings.velocity_control_settings.kp;
    axis->velocity_controller.ki = settings.velocity_control_settings.ki;
    axis->velocity_controller.kd = settings.velocity_control_settings.kd;
    axis->velocity_controller.integrator_limit = settings.velocity_control_settings.integrator_limit;
    axis->velocity_controller.max_output_value = settings.velocity_control_settings.max_output_value;
    esp_foc_pid_reset(&axis->velocity_controller);
    esp_foc_low_pass_filter_init(&axis->velocity_filter, 0.9);
    axis->downsampling_speed = settings.downsampling_speed_rate;
    axis->downsampling_speed_reload_value = settings.downsampling_speed_rate;

    axis->torque_controller[0].kp = 1.0f;
    axis->torque_controller[0].ki = 0.0f;
    axis->torque_controller[0].kd = 0.0f;
    axis->torque_controller[0].integrator_limit = 0.0f;
    axis->torque_controller[0].max_output_value = settings.torque_control_settings[0].max_output_value;
    esp_foc_pid_reset(&axis->torque_controller[0]);
    esp_foc_low_pass_filter_init(&axis->current_filters[0], 0.9);

    axis->torque_controller[1].kp = 1.0f;
    axis->torque_controller[1].ki = 0.0f;
    axis->torque_controller[1].kd = 0.0f;
    axis->torque_controller[1].integrator_limit = 0.0f;
    axis->torque_controller[1].max_output_value = settings.torque_control_settings[1].max_output_value;
    esp_foc_pid_reset(&axis->torque_controller[1]);
    esp_foc_low_pass_filter_init(&axis->current_filters[1], 0.9);

    axis->motor_pole_pairs = (float)settings.motor_pole_pairs;
    ESP_LOGI(tag, "Motor poler pairs: %f",  axis->motor_pole_pairs);

    axis->shaft_ticks_to_radians_ratio = ((2.0 * M_PI)) /
        axis->rotor_sensor_driver->get_counts_per_revolution(axis->rotor_sensor_driver);
    ESP_LOGI(tag, "Shaft to ticks ratio: %f", axis->shaft_ticks_to_radians_ratio);

    esp_foc_sleep_ms(250);
    axis->rotor_aligned = ESP_FOC_ERR_NOT_ALIGNED;

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis)
{
    if(axis == NULL) {
        ESP_LOGE(tag, "Invalid axis object!");
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_ERR_NOT_ALIGNED) {
        ESP_LOGE(tag, "This rotor was aligned already!");
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->rotor_aligned = ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;

    float theta = 0.0;
    axis->u_q.raw = 0.1f * axis->biased_dc_link_voltage;
    axis->u_d.raw = 0.0f;
    
    ESP_LOGI(tag, "Starting to align the rotor");

    for (float i = 0.0f; i < 500.0f; i += 1.0f) {
        theta = esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(
                (ALIGN_ANGLE_CONSTANT * i) / 500.0f, axis->motor_pole_pairs
            )
        );

        esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                        theta, 
                        axis->u_d.raw, 
                        axis->u_q.raw, 
                        &axis->u_u.raw, 
                        &axis->u_v.raw, 
                        &axis->u_w.raw);

        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                            axis->u_u.raw, 
                                            axis->u_v.raw, 
                                            axis->u_w.raw);
        esp_foc_sleep_ms(CONFIG_ESP_FOC_ALIGN_STEP_DELAY_MS);
    }

    for (float i = 500.0f; i != 0.0f; i -= 1.0f) {
        theta = esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(
                (ALIGN_ANGLE_CONSTANT * i) / 500.0f, axis->motor_pole_pairs
            )
        );

        esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                        theta, 
                        axis->u_d.raw, 
                        axis->u_q.raw, 
                        &axis->u_u.raw, 
                        &axis->u_v.raw, 
                        &axis->u_w.raw);

        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                            axis->u_u.raw, 
                                            axis->u_v.raw, 
                                            axis->u_w.raw);
        esp_foc_sleep_ms(CONFIG_ESP_FOC_ALIGN_STEP_DELAY_MS);
    }

    esp_foc_sleep_ms(250);
    axis->inverter_driver->set_voltages(axis->inverter_driver, axis->dc_link_voltage * 0.2f, 0.0, 0.0);
    esp_foc_sleep_ms(250);
    axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);
    axis->rotor_aligned = ESP_FOC_OK;

    ESP_LOGI(tag, "Done, rotor aligned!");

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

    axis->u_q = uq;
    axis->u_d = ud;

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

    axis->target_speed = speed.raw;

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

    axis->target_position = position.raw;

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
    int ret = esp_foc_create_runner(esp_foc_loop, axis);

    if (ret < 0) {
        ESP_LOGE(tag, "Check os interface, the runner creation has failed!");
        return ESP_FOC_ERR_UNKNOWN;
    }

    return ESP_FOC_OK;
}