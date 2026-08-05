/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Bench / characterization axis: open-loop Vdq at fixed theta, no FOC loops.
 */

#include "esp_log.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/foc_math_q16.h"
#include "espFoC/utils/modulator.h"

static const char *TAG = "ESP_FOC_BENCH";

static void bench_park_inverter_safe(esp_foc_axis_t *axis)
{
    if (axis->inverter_driver == NULL) {
        return;
    }
    axis->inverter_driver->set_duties(axis->inverter_driver, 0, 0, 0);
    axis->inverter_driver->disable(axis->inverter_driver);
}

static void bench_apply_filter(esp_foc_axis_t *axis, float fc_hz, float fs_hz)
{
    if (axis->isensor_driver == NULL ||
        axis->isensor_driver->set_filter_cutoff == NULL) {
        return;
    }
    axis->isensor_driver->set_filter_cutoff(axis->isensor_driver, fc_hz, fs_hz);
    axis->current_filter_fc_hz_q16 = q16_from_float(fc_hz);
    axis->current_filter_fs_hz_q16 = q16_from_float(fs_hz);
}

esp_foc_err_t esp_foc_initialize_axis_bench(esp_foc_axis_t *axis,
                                            esp_foc_inverter_t *inverter,
                                            esp_foc_isensor_t *isensor,
                                            const esp_foc_axis_bench_config_t *config)
{
    if (axis == NULL || inverter == NULL || isensor == NULL || config == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    float pwm_rate_hz_f = (float)inverter->get_inverter_pwm_rate(inverter);
    float dt_f = (pwm_rate_hz_f > 1e-9f) ? (1.0f / pwm_rate_hz_f) : 0.0f;
    float loop_fs_hz = (dt_f > 1e-9f) ? (1.0f / dt_f) : 0.0f;

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
    axis->magic = ESP_FOC_AXIS_MAGIC;
#endif

    axis->mode = ESP_FOC_AXIS_MODE_BENCH;
    axis->bench_theta_e = config->bench_theta_e;
    axis->state = ESP_FOC_AXIS_STATE_IDLE;
    axis->runner_shutdown = false;
    axis->runner_low_speed_hdl = NULL;
    axis->runner_outer_hdl = NULL;
    axis->regulator_ev = NULL;
    axis->low_speed_ev = NULL;
    axis->inverter_driver = inverter;
    axis->rotor_sensor_driver = NULL;
    axis->isensor_driver = isensor;
    axis->motor_pole_pairs = config->motor.motor_pole_pairs;
    axis->natural_direction = (config->motor.natural_direction ==
                               ESP_FOC_MOTOR_NATURAL_DIRECTION_CW)
                                  ? Q16_ONE
                                  : Q16_MINUS_ONE;

    esp_foc_calibration_axis_init_store(axis, &config->motor);

    axis->vdc_q16 = inverter->get_dc_link_voltage(inverter);
    axis->mod_index_limit_q16 = ESP_FOC_MOD_INDEX_LIMIT_Q16;
    axis->dt = q16_from_float(dt_f);
    axis->inv_dt = q16_from_float(loop_fs_hz);

    axis->u_d.raw = 0;
    axis->u_q.raw = 0;
    axis->i_d.raw = 0;
    axis->i_q.raw = 0;
    axis->target_i_d.raw = 0;
    axis->target_i_q.raw = 0;
    axis->latest_i_alpha = 0;
    axis->latest_i_beta = 0;

    inverter->set_duties(inverter, 0, 0, 0);

    if (config->calibrate_isensor_at_init) {
        isensor->calibrate_isensors(isensor, CONFIG_ESP_FOC_ISENSOR_CALIBRATION_ROUNDS);
    }

    float fc_hz = (float)CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ;
    esp_foc_calibration_axis_boot_apply(axis, &fc_hz, loop_fs_hz);
    bench_apply_filter(axis, fc_hz, loop_fs_hz);

    if (isensor->set_publish_targets != NULL) {
        isensor->set_publish_targets(isensor,
                                     (q16_t *)&axis->latest_i_alpha,
                                     (q16_t *)&axis->latest_i_beta,
                                     &axis->i_u,
                                     &axis->i_v);
    }

    ESP_LOGI(TAG, "bench axis ready (theta_e=%f rad)",
             (double)q16_to_float(axis->bench_theta_e));
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_bench_arm(esp_foc_axis_t *axis)
{
    if (axis == NULL || axis->mode != ESP_FOC_AXIS_MODE_BENCH) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_BENCH) {
        return ESP_FOC_OK;
    }
    if (axis->state != ESP_FOC_AXIS_STATE_IDLE) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    axis->inverter_driver->enable(axis->inverter_driver);
    axis->state = ESP_FOC_AXIS_STATE_BENCH;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_bench_disarm(esp_foc_axis_t *axis)
{
    if (axis == NULL || axis->mode != ESP_FOC_AXIS_MODE_BENCH) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state != ESP_FOC_AXIS_STATE_BENCH) {
        return ESP_FOC_OK;
    }
    bench_park_inverter_safe(axis);
    axis->u_d.raw = 0;
    axis->u_q.raw = 0;
    axis->state = ESP_FOC_AXIS_STATE_IDLE;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_bench_step(esp_foc_axis_t *axis)
{
    if (axis == NULL || axis->mode != ESP_FOC_AXIS_MODE_BENCH) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state != ESP_FOC_AXIS_STATE_BENCH) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    if (axis->isensor_driver != NULL) {
        axis->isensor_driver->sample_isensors(axis->isensor_driver);
        isensor_values_t val;
        axis->isensor_driver->fetch_isensors(axis->isensor_driver, &val);
        axis->i_u = val.iu_axis_0;
        axis->i_v = val.iv_axis_0;
        axis->i_w = val.iw_axis_0;

        q16_t sin_t = q16_sin(axis->bench_theta_e);
        q16_t cos_t = q16_cos(axis->bench_theta_e);
        q16_park(sin_t, cos_t, axis->latest_i_alpha, axis->latest_i_beta,
                 &axis->i_d.raw, &axis->i_q.raw);
    }

    q16_t sin_t = q16_sin(axis->bench_theta_e);
    q16_t cos_t = q16_cos(axis->bench_theta_e);
    q16_t alpha, beta, da, db, dc;
    esp_foc_modulate_dq_to_duties(sin_t, cos_t,
                                  axis->u_d.raw, axis->u_q.raw,
                                  &alpha, &beta, &da, &db, &dc,
                                  axis->mod_index_limit_q16);
    axis->u_alpha.raw = alpha;
    axis->u_beta.raw = beta;
    axis->inverter_driver->set_duties(axis->inverter_driver, da, db, dc);
    return ESP_FOC_OK;
}
