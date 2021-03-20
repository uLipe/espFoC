#include <string.h>
#include <math.h>
#include "esp_attr.h"
#include "espFoC/esp_foc.h"

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

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                    esp_foc_inverter_t *inverter,
                                    esp_foc_rotor_sensor_t *rotor,
                                    float motor_pole_pairs)
{
    if(axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    if(inverter == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    if(rotor == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    axis->dt = 0.0;
    axis->last_timestamp = 0.0;
    axis->v_qd[0] = 0.0;
    axis->v_qd[1] = 0.0;
    axis->target_speed = 0.0;

    axis->inverter_driver = inverter;
    axis->rotor_sensor_driver = rotor;

    #if CONFIG_ESP_FOC_USE_VELOCITY_CONTROLLER
        axis->velocity_controller.kp = CONFIG_ESP_FOC_VELOCITY_CONTROLLER_KP;
        axis->velocity_controller.ki = CONFIG_ESP_FOC_VELOCITY_CONTROLLER_KI;
        axis->velocity_controller.kd = CONFIG_ESP_FOC_VELOCITY_CONTROLLER_KD;
        axis->velocity_controller.integrator_limit = CONFIG_ESP_FOC_VELOCITY_CONTROL_LIMIT;
        esp_foc_pid_reset(&axis->velocity_controller);
        esp_foc_low_pass_filter_init(&axis->velocity_filter, 0.9);
        axis->inner_control_runs = CONFIG_ESP_FOC_VELOCITY_CONTROLLER_RATE;
    #endif

    axis->motor_pole_pairs = motor_pole_pairs;

    axis->shaft_ticks_to_radians_ratio = ((2.0 * M_PI)) /
        axis->rotor_sensor_driver->get_counts_per_revolution(axis->rotor_sensor_driver);

    axis->dc_link_voltage = 
        axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);
    axis->biased_dc_link_voltage = axis->dc_link_voltage * 0.5;
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0.0, 0.0, 0.0);

    axis->rotor_sensor_driver->delay_ms(axis->rotor_sensor_driver, 250);
    axis->rotor_aligned = ESP_FOC_ERR_NOT_ALIGNED;

    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis)
{
    if(axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_ERR_NOT_ALIGNED) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->rotor_aligned = ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;

    float theta = 0.0;
    axis->v_qd[0] = 0.1f * axis->biased_dc_link_voltage;
    axis->v_qd[1] = 0.0f;
    
    for (float i = 0.0f; i < 500.0f; i += 1.0f) {
        theta = esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(
                (ALIGN_ANGLE_CONSTANT * i) / 500.0f, axis->motor_pole_pairs
            )
        );
        esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                        theta, 
                        axis->v_qd[1], 
                        axis->v_qd[0], 
                        &axis->v_uvw[0], 
                        &axis->v_uvw[1], 
                        &axis->v_uvw[2]);
        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                             axis->v_uvw[0], 
                                             axis->v_uvw[1], 
                                             axis->v_uvw[2]);
        axis->rotor_sensor_driver->delay_ms(axis->rotor_sensor_driver, 
                                            CONFIG_ESP_FOC_ALIGN_STEP_DELAY_MS);
    }

    for (float i = 500.0f; i != 0.0f; i -= 1.0f) {
        theta = esp_foc_normalize_angle(
            esp_foc_mechanical_to_elec_angle(
                (ALIGN_ANGLE_CONSTANT * i) / 500.0f, axis->motor_pole_pairs
            )
        );
        esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                        theta, 
                        axis->v_qd[1], 
                        axis->v_qd[0], 
                        &axis->v_uvw[0], 
                        &axis->v_uvw[1], 
                        &axis->v_uvw[2]);
        axis->inverter_driver->set_voltages(axis->inverter_driver,
                                             axis->v_uvw[0], 
                                             axis->v_uvw[1], 
                                             axis->v_uvw[2]);
        axis->rotor_sensor_driver->delay_ms(axis->rotor_sensor_driver, 
                                            CONFIG_ESP_FOC_ALIGN_STEP_DELAY_MS);
    }

    axis->rotor_sensor_driver->delay_ms(axis->rotor_sensor_driver, 250);
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0.0, 0.0, 0.0);
    axis->rotor_sensor_driver->delay_ms(axis->rotor_sensor_driver, 250);
    axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);
    axis->rotor_aligned = ESP_FOC_OK;

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_set_target_voltage(esp_foc_axis_t *axis,
                                        esp_foc_q_voltage *vq,
                                        esp_foc_d_voltage *vd)
{
    if(axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(vq == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(vd == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->v_qd[0] = esp_foc_saturate(vq->raw, axis->biased_dc_link_voltage);
    axis->v_qd[1] = esp_foc_saturate(vd->raw, axis->biased_dc_link_voltage);

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_set_target_velocity(esp_foc_axis_t *axis, float radians)
{
    if(axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    #if CONFIG_ESP_FOC_USE_VELOCITY_CONTROLLER
        axis->target_speed = esp_foc_saturate(radians, CONFIG_ESP_FOC_MAX_VELOCITY);
    #endif

    return ESP_FOC_OK;
}

IRAM_ATTR esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis, float now)
{
    if(axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if(axis->rotor_aligned != ESP_FOC_OK) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    float dt = now - axis->last_timestamp;
    axis->velocity_dt += dt;
    if(dt == 0.0 || dt < 0.0f) {
        return ESP_FOC_ERR_TIMESTEP_TOO_SMALL;
    }

    axis->last_timestamp = now;

    float theta = esp_foc_ticks_to_radians_normalized(axis);
    
    #if CONFIG_ESP_FOC_USE_VELOCITY_CONTROLLER
        axis->inner_control_runs--;

        if(axis->inner_control_runs == 0) {
            float measured_velocity = 
                (axis->rotor_position - axis->rotor_position_prev) / dt;
            axis->rotor_position_prev = axis->rotor_position;

            axis->v_qd[0] = esp_foc_saturate(
                esp_foc_pid_update(
                    &axis->velocity_controller,
                    esp_foc_low_pass_filter_update(
                        &axis->velocity_filter, axis->target_speed
                    ),
                    measured_velocity
                ),
                axis->biased_dc_link_voltage;
            )
            
            axis->v_qd[1] = 0.0f;
            axis->inner_control_runs = CONFIG_CONFIG_ESP_FOC_VELOCITY_CONTROLLER_RATE;
            axis->velocity_dt = 0.0f;
        }
    #endif

    esp_foc_modulate_dq_voltage(axis->biased_dc_link_voltage, 
                theta, 
                axis->v_qd[1], 
                axis->v_qd[0], 
                &axis->v_uvw[0], 
                &axis->v_uvw[1], 
                &axis->v_uvw[2]);

    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                            axis->v_uvw[0], 
                                            axis->v_uvw[1], 
                                            axis->v_uvw[2]);

    return ESP_FOC_OK;
}