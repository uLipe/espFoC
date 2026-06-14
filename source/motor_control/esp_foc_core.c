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

#include <stdio.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "espFoC/esp_foc.h"

/* Minimum encoder deflection considered conclusive when probing for the
 * natural direction. Expressed in raw encoder counts; AS5600 has 4096
 * counts/rev so 50 counts is ~4.4 degrees — well above noise but small
 * enough that a stuck rotor will obviously fail to clear the bar. */
#define ESP_FOC_DIR_PROBE_MIN_COUNTS 50

/* Alignment: fractions of ESP_FOC_VPU_ONE_Q16 (linear SVPWM ceiling in pu). */
#define ESP_FOC_ALIGN_PARK_VD_VMAX_FRAC      (0.15f)
/* Open-loop sweep: needs enough torque to overcome friction; keep below
 * the single-shot probe levels that overheat idling. */
#define ESP_FOC_ALIGN_SWEEP_VQ_VMAX_FRAC     (0.15f)
#define ESP_FOC_DIR_SWEEP_STEPS              80
#define ESP_FOC_DIR_SWEEP_MS_PER_STEP        22


static const char *tag = "ESP_FOC_CORE";


void esp_foc_axis_refresh_encoder_q16_scales(esp_foc_axis_t *axis)
{
    if (axis == NULL || axis->encoder_driver == NULL ||
        axis->encoder_driver->get_counts_per_revolution == NULL) {
        return;
    }
    uint32_t cpr =
        axis->encoder_driver->get_counts_per_revolution(
            axis->encoder_driver);
    if (cpr == 0u) {
        cpr = 1u;
    }
    axis->encoder_inv_cpr_q16 = q16_from_float(1.0f / (float)cpr);
    esp_foc_estimator_q16_set_pole_pairs(&axis->rotor_estimator,
                                         axis->motor_pole_pairs);
}

static void apply_bypass_current_pid(esp_foc_pid_controller_t *pid,
                                   q16_t out_max)
{
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    pid->kff = Q16_ONE;
    pid->integrator_limit = out_max;
    pid->max_output_value = out_max;
    pid->min_output_value = q16_from_float(-q16_to_float(out_max));
    pid->ke = Q16_ONE;
    esp_foc_pid_reset(pid);
}

void esp_foc_axis_apply_bypass_current_loop_gains(esp_foc_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }
    for (int i = 0; i < 2; ++i) {
        apply_bypass_current_pid(&axis->torque_controller[i],
                                 ESP_FOC_VPU_ONE_Q16);
    }
}

static void park_inverter_safe(esp_foc_axis_t *axis)
{
    if (axis->inverter_driver == NULL) {
        return;
    }
    axis->inverter_driver->set_duties(axis->inverter_driver, 0, 0, 0);
    axis->inverter_driver->disable(axis->inverter_driver);
    if (axis->inverter_driver->set_inverter_callback != NULL) {
        esp_foc_critical_enter();
        axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
                                                     NULL, NULL);
        esp_foc_critical_leave();
    }
}

static void wait_runner_dead(void *task_handle)
{
    while (esp_foc_runner_is_alive(task_handle)) {
        esp_foc_sleep_ms(1);
    }
}

/* -------------------------------------------------------------------------
 * PWM ISR hot path: read published Clarke currents, current PI, SVPWM.
 * ------------------------------------------------------------------------- */
static inline q16_t axis_theta_meas_mech(const esp_foc_axis_t *axis)
{
    return q16_normalize_angle(q16_mul(axis->rotor_position, axis->encoder_inv_cpr_q16));
}

static void IRAM_ATTR foc_hot_isr(void *data)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)data;
    if (axis == NULL || axis->state != ESP_FOC_AXIS_STATE_RUNNING) {
        return;
    }

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    esp_foc_debug_pin_set();
#endif


    esp_foc_estimator_q16_step(&axis->rotor_estimator);
    axis->rotor_elec_angle =
        esp_foc_estimator_q16_theta_elec(&axis->rotor_estimator);
    axis->current_speed = axis->rotor_estimator.omega_est_mech;

    q16_t i_alpha = axis->latest_i_alpha;
    q16_t i_beta = axis->latest_i_beta;

    if (axis->inverter_driver != NULL) {
        axis->inverter_driver->sample_isensors(axis->inverter_driver);
    }

    q16_t e_sin = q16_sin(q16_normalize_angle(axis->rotor_elec_angle));
    q16_t e_cos = q16_cos(q16_normalize_angle(axis->rotor_elec_angle));

    q16_t i_d, i_q;
    q16_park(e_sin, e_cos, i_alpha, i_beta, &i_d, &i_q);

    axis->i_alpha.raw = i_alpha;
    axis->i_beta.raw = i_beta;
    axis->i_d.raw = i_d;
    axis->i_q.raw = i_q;

    {
        q16_t uq = esp_foc_pid_update(&axis->torque_controller[0],
                                      axis->target_i_q.raw, i_q);
        q16_t ud = esp_foc_pid_update(&axis->torque_controller[1],
                                      axis->target_i_d.raw, i_d);
        axis->u_q.raw = q16_mul(uq, axis->mod_index_limit_q16);
        axis->u_d.raw = q16_mul(ud, axis->mod_index_limit_q16);
    }

    esp_foc_modulate_dq_to_duties(e_sin, e_cos, axis->u_d.raw, axis->u_q.raw,
                                  &axis->u_alpha.raw,
                                  &axis->u_beta.raw,
                                  &axis->u_u.raw,
                                  &axis->u_v.raw,
                                  &axis->u_w.raw,
                                  axis->mod_index_limit_q16);

    axis->inverter_driver->set_duties(axis->inverter_driver,
                                      axis->u_u.raw,
                                      axis->u_v.raw,
                                      axis->u_w.raw);

    if (axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if (!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            if (axis->low_speed_ev != NULL) {
                esp_foc_send_notification_from_isr(axis->low_speed_ev);
            }
        }
    }

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    esp_foc_debug_pin_clear();
#endif
}

/* -------------------------------------------------------------------------
 * Slow loop: encoder measurement and regulator callback trigger.
 * ------------------------------------------------------------------------- */
static void core_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = CONFIG_ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag, "FOC outer loop (PWM ISR hot path)");

    uint32_t cpr = axis->encoder_driver->get_counts_per_revolution(
        axis->encoder_driver);
    if (cpr == 0u) {
        ESP_LOGE(tag, "FATAL, INVALID CPR, CHECK YOUR ROTOR SENSOR!");
        abort();
    }

    axis->rotor_shaft_ticks = axis->encoder_driver->read_counts(axis->encoder_driver);
    axis->rotor_position = q16_mul(axis->rotor_shaft_ticks, axis->natural_direction);
    q16_t theta_meas = axis_theta_meas_mech(axis);
    esp_foc_estimator_q16_snap(&axis->rotor_estimator, theta_meas);
    axis->current_speed = 0;

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
                                                 foc_hot_isr,
                                                 axis);

    while (!axis->runner_shutdown) {
        esp_foc_wait_notifier();
        if (axis->runner_shutdown) {
            break;
        }
        if (axis->state != ESP_FOC_AXIS_STATE_RUNNING) {
            continue;
        }

        axis->rotor_shaft_ticks = axis->encoder_driver->read_counts(axis->encoder_driver);
        axis->rotor_position = q16_mul(axis->rotor_shaft_ticks, axis->natural_direction);
        esp_foc_estimator_q16_set_meas(&axis->rotor_estimator,
                                       axis_theta_meas_mech(axis));

        if (axis->regulator_ev != NULL) {
            esp_foc_send_notification(axis->regulator_ev);
        }
    }
    axis->low_speed_ev = NULL;
    esp_foc_runner_delete_self();
}

static void do_foc_outer_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    axis->regulator_ev = esp_foc_get_event_handle();

    while (!axis->runner_shutdown) {
        esp_foc_wait_notifier();
        if (axis->runner_shutdown) {
            break;
        }
        if (axis->state != ESP_FOC_AXIS_STATE_RUNNING) {
            continue;
        }
        if (axis->regulator_cb == NULL) {
            continue;
        }
        axis->regulator_cb(axis, &axis->target_i_d, &axis->target_i_q);
    }
    axis->regulator_ev = NULL;
    esp_foc_runner_delete_self();
}

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                      esp_foc_inverter_t *inverter,
                                      esp_foc_encoder_t *encoder,
                                      esp_foc_motor_control_settings_t settings)
{
    float pwm_rate_hz_f;
    float dt_f;

    if (axis == NULL || inverter == NULL || encoder == NULL) {
        ESP_LOGE(tag, "invalid args for axis initialization");
        return ESP_FOC_ERR_INVALID_ARG;
    }

    axis->state = ESP_FOC_AXIS_STATE_IDLE;
    axis->mode = ESP_FOC_AXIS_MODE_FOC;
    axis->bench_theta_e = 0;
    axis->runner_shutdown = false;
    axis->runner_low_speed_hdl = NULL;
    axis->runner_outer_hdl = NULL;
    axis->regulator_ev = NULL;
    axis->low_speed_ev = NULL;
    axis->inverter_driver = inverter;
    axis->encoder_driver = encoder;
    esp_foc_calibration_axis_init_store(axis, &settings);

    axis->vdc_q16 = axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);
    axis->mod_index_limit_q16 = ESP_FOC_MOD_INDEX_LIMIT_Q16;

    axis->inverter_driver->set_duties(axis->inverter_driver, 0, 0, 0);

    pwm_rate_hz_f = (float)inverter->get_inverter_pwm_rate(inverter);
    dt_f = (pwm_rate_hz_f > 1e-9f) ? (1.0f / pwm_rate_hz_f) : 0.0f;
    axis->dt = q16_from_float(dt_f);
    axis->inv_dt = q16_from_float((dt_f > 1e-9f) ? (1.0f / dt_f) : 0.0f);

    axis->i_d.raw = 0;
    axis->i_q.raw = 0;
    axis->u_d.raw = 0;
    axis->u_q.raw = 0;
    axis->target_i_d.raw = 0;
    axis->target_i_q.raw = 0;

    if (axis->inverter_driver->calibrate_isensors != NULL) {
        axis->inverter_driver->calibrate_isensors(axis->inverter_driver,
                                                  CONFIG_ESP_FOC_ISENSOR_CALIBRATION_ROUNDS);
    }

    float loop_dt_s = dt_f;
    float loop_fs_hz = (loop_dt_s > 1e-9f) ? (1.0f / loop_dt_s) : 0.0f;
    q16_t loop_dt = q16_from_float(loop_dt_s);
    q16_t loop_inv_dt = q16_from_float(loop_fs_hz);

    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].dt = loop_dt;
        axis->torque_controller[i].inv_dt = loop_inv_dt;
        apply_bypass_current_pid(&axis->torque_controller[i],
                                 ESP_FOC_VPU_ONE_Q16);
    }

    float current_filter_fc_hz = (float)CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ;

    axis->natural_direction =
        (settings.natural_direction == ESP_FOC_MOTOR_NATURAL_DIRECTION_CW) ? Q16_ONE
                                                                         : Q16_MINUS_ONE;
    axis->motor_pole_pairs = settings.motor_pole_pairs;
    esp_foc_calibration_axis_boot_apply(axis, &current_filter_fc_hz, loop_fs_hz);

    if (axis->inverter_driver->set_filter_cutoff != NULL) {
        axis->inverter_driver->set_filter_cutoff(axis->inverter_driver,
                                                 current_filter_fc_hz,
                                                 loop_fs_hz);
    }
    axis->current_filter_fc_hz_q16 = q16_from_float(current_filter_fc_hz);
    axis->current_filter_fs_hz_q16 = q16_from_float(loop_fs_hz);

    axis->latest_i_alpha = 0;
    axis->latest_i_beta = 0;
    if (axis->inverter_driver->set_publish_targets != NULL) {
        axis->inverter_driver->set_publish_targets(
            axis->inverter_driver,
            (q16_t *)&axis->latest_i_alpha,
            (q16_t *)&axis->latest_i_beta,
            &axis->i_u,
            &axis->i_v);
    }

    esp_foc_axis_refresh_encoder_q16_scales(axis);

    {
        esp_foc_estimator_q16_config_t est_cfg = {
            .dt_isr = axis->dt,
            .pll_bw_hz = (float)CONFIG_ESP_FOC_ENCODER_PLL_BW_HZ,
            .omega_max_mech =
                q16_from_float((float)CONFIG_ESP_FOC_ENCODER_PLL_OMEGA_MAX_REV_S),
            .pole_pairs = axis->motor_pole_pairs,
        };
        esp_foc_estimator_q16_init(&axis->rotor_estimator, &est_cfg);
    }

    axis->torque_controller[0].ke = Q16_ONE;
    axis->torque_controller[1].ke = Q16_ONE;

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
    esp_foc_debug_pin_init(CONFIG_ESP_FOC_DEBUG_PIN);
#endif

    esp_foc_sleep_ms(250);
    axis->state = ESP_FOC_AXIS_STATE_IDLE;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis)
{
    q16_t e_sin, e_cos;
    q16_t alpha, beta;
    q16_t da, db, dc;
    bool skip_dir_probe = false;

    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state != ESP_FOC_AXIS_STATE_IDLE) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->state = ESP_FOC_AXIS_STATE_ALIGNING;
    esp_foc_calibration_axis_align_apply_stored_hints(axis, &skip_dir_probe);
    axis->inverter_driver->set_duties(axis->inverter_driver, 0, 0, 0);

    e_sin = q16_sin(0);
    e_cos = q16_cos(0);

    axis->inverter_driver->enable(axis->inverter_driver);
    esp_foc_sleep_ms(500);

    const q16_t park_vd = q16_mul(ESP_FOC_VPU_ONE_Q16,
                                  q16_from_float(ESP_FOC_ALIGN_PARK_VD_VMAX_FRAC));

    /* Step 1 — park the rotor at electrical angle 0 by driving Vd. */
    esp_foc_modulate_dq_to_duties(e_sin, e_cos, park_vd, 0,
                                  &alpha, &beta, &da, &db, &dc,
                                  axis->mod_index_limit_q16);
    axis->inverter_driver->set_duties(axis->inverter_driver, da, db, dc);
    esp_foc_sleep_ms(500);

    /* Step 2 — zero the encoder. The current physical position is now
     * the firmware's electrical zero. */
    axis->encoder_driver->set_to_zero(axis->encoder_driver);
    q16_t ticks_zero = axis->encoder_driver->read_counts(axis->encoder_driver);
    q16_t delta = 0;

    /* Step 3 — Cog the rotor by applying small VQ and check direction change*/
    {
        const q16_t uq = q16_mul(ESP_FOC_VPU_ONE_Q16,
                                 q16_from_float(ESP_FOC_ALIGN_SWEEP_VQ_VMAX_FRAC));
        esp_foc_modulate_dq_to_duties(e_sin, e_cos, 0, uq,
                                      &alpha, &beta, &da, &db, &dc,
                                      axis->mod_index_limit_q16);
        axis->inverter_driver->set_duties(axis->inverter_driver, da, db, dc);

            esp_foc_sleep_ms(ESP_FOC_DIR_SWEEP_MS_PER_STEP);

        q16_t ticks_after_fwd = axis->encoder_driver->read_counts(axis->encoder_driver);
        delta = q16_sub(ticks_after_fwd, ticks_zero);
    }

    if (!skip_dir_probe) {
        const q16_t min_delta = q16_from_int(ESP_FOC_DIR_PROBE_MIN_COUNTS);
        if (delta >= min_delta) {
            axis->natural_direction = Q16_ONE; /* CW */
            ESP_LOGI(
                tag,
                "alignment: natural direction = CW (delta from sweep=%lld)",
                (long long)delta);
        } else if (delta <= q16_from_int(-ESP_FOC_DIR_PROBE_MIN_COUNTS)) {
            axis->natural_direction = Q16_MINUS_ONE; /* CCW */
            ESP_LOGI(
                tag,
                "alignment: natural direction = CCW (delta from sweep=%lld)",
                (long long)delta);
        } else if (delta == 0) {
#ifdef CONFIG_ESP_FOC_FITL
            axis->natural_direction = Q16_ONE;
            ESP_LOGW(
                tag,
                "alignment: delta=0; assuming CW (FITL simulated plant)");
#else
            ESP_LOGE(
                tag,
                "alignment: error! failed to find the direction of the motor, aborting!");
            abort();
#endif
        } else {
            ESP_LOGI(
                tag,
                "alignment: direction inconclusive (delta=%lld counts); keeping hint",
                (long long)delta);
        }
    } else {
        ESP_LOGI(
            tag,
            "alignment: using stored natural direction (NVS; sweep enc delta %lld, not re-applied)",
            (long long)delta);
    }

    axis->inverter_driver->set_duties(axis->inverter_driver, 0, 0, 0);
    esp_foc_sleep_ms(500);

    axis->state = ESP_FOC_AXIS_STATE_ALIGNED;
    esp_foc_calibration_axis_align_persist_snapshot(axis);
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_RUNNING) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    if (axis->state != ESP_FOC_AXIS_STATE_ALIGNED) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->runner_shutdown = false;
    axis->state = ESP_FOC_AXIS_STATE_RUNNING;

    if (esp_foc_create_runner(do_foc_outer_loop, axis, 2,
                              &axis->runner_outer_hdl) != 0) {
        axis->state = ESP_FOC_AXIS_STATE_ALIGNED;
        axis->runner_outer_hdl = NULL;
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    if (esp_foc_create_runner(core_low_speed_loop, axis, 1,
                              &axis->runner_low_speed_hdl) != 0) {
        axis->runner_shutdown = true;
        esp_foc_runner_wake(axis->runner_outer_hdl);
        wait_runner_dead(axis->runner_outer_hdl);
        axis->runner_outer_hdl = NULL;
        axis->state = ESP_FOC_AXIS_STATE_ALIGNED;
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_stop(esp_foc_axis_t *axis)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (!esp_foc_in_task_context()) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_IDLE) {
        return ESP_FOC_OK;
    }
    if (axis->state == ESP_FOC_AXIS_STATE_ALIGNING) {
        return ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;
    }

    const bool was_running = (axis->state == ESP_FOC_AXIS_STATE_RUNNING);

    if (was_running) {
        axis->runner_shutdown = true;
    }
    axis->state = ESP_FOC_AXIS_STATE_IDLE;

    park_inverter_safe(axis);

    if (was_running) {
        esp_foc_runner_wake(axis->runner_low_speed_hdl);
        esp_foc_runner_wake(axis->runner_outer_hdl);
        wait_runner_dead(axis->runner_low_speed_hdl);
        wait_runner_dead(axis->runner_outer_hdl);
        axis->runner_low_speed_hdl = NULL;
        axis->runner_outer_hdl = NULL;
    }

    axis->runner_shutdown = false;
    axis->regulator_ev = NULL;
    axis->low_speed_ev = NULL;
    esp_foc_estimator_q16_reset(&axis->rotor_estimator);
    axis->rotor_elec_angle = 0;
    axis->current_speed = 0;
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_set_regulation_callback(esp_foc_axis_t *axis,
                                              esp_foc_motor_regulation_callback_t callback)
{
    if (axis == NULL || callback == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    esp_foc_critical_enter();
    axis->regulator_cb = callback;
    esp_foc_critical_leave();
    return ESP_FOC_OK;
}

static void apply_loop_gains_locked(esp_foc_axis_t *axis,
                                     q16_t kp,
                                     q16_t ki,
                                     q16_t kd,
                                     q16_t kff,
                                     q16_t integrator_limit)
{
    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].kp = kp;
        axis->torque_controller[i].ki = ki;
        axis->torque_controller[i].kd = kd;
        axis->torque_controller[i].kff = kff;
        axis->torque_controller[i].integrator_limit = integrator_limit;
        esp_foc_pid_reset(&axis->torque_controller[i]);
    }
}

esp_foc_err_t esp_foc_axis_set_current_loop_gains_q16(
    esp_foc_axis_t *axis,
    q16_t kp,
    q16_t ki,
    q16_t kd,
    q16_t kff,
    q16_t integrator_limit)
{
    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (kp < 0 || ki < 0 || integrator_limit < 0) {
        return ESP_FOC_ERR_INVALID_ARG;
    }

    esp_foc_critical_enter();
    apply_loop_gains_locked(axis, kp, ki, kd, kff, integrator_limit);
    esp_foc_critical_leave();
    return ESP_FOC_OK;
}

void esp_foc_axis_get_current_loop_gains_q16(
    const esp_foc_axis_t *axis,
    q16_t *kp,
    q16_t *ki,
    q16_t *kd,
    q16_t *kff,
    q16_t *integrator_limit)
{
    if (axis == NULL) {
        return;
    }
    if (kp != NULL) {
        *kp = axis->torque_controller[0].kp;
    }
    if (ki != NULL) {
        *ki = axis->torque_controller[0].ki;
    }
    if (kd != NULL) {
        *kd = axis->torque_controller[0].kd;
    }
    if (kff != NULL) {
        *kff = axis->torque_controller[0].kff;
    }
    if (integrator_limit != NULL) {
        *integrator_limit = axis->torque_controller[0].integrator_limit;
    }
}
