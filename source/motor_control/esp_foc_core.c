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
#include "espFoC/esp_foc.h"
#include "espFoC/esp_foc_calibration.h"
#if defined(CONFIG_ESP_FOC_ISR_HOT_PATH)
#include "espFoC/utils/angle_predictor_q16.h"
#endif

#if defined(ESP_FOC_AUTOGEN_GAINS_AVAILABLE)
#include "esp_foc_autotuned_gains.h"
#endif

static const char *tag = "ESP_FOC_CORE";

static void do_foc_outer_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    axis->regulator_ev = esp_foc_get_event_handle();

    while (1) {
        esp_foc_wait_notifier();
        if (axis->regulator_cb == NULL) {
            continue;
        }
#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
        if (axis->tuner_override.active) {
            /* Drain the user callback into shadow refs so its side effects
             * (logging, sensor reads, state machines) keep firing — but the
             * actual axis targets are owned by the tuner. Bumpless transfer
             * back to user control happens automatically when the override
             * flag clears. */
            esp_foc_d_current_q16_t shadow_id;
            esp_foc_q_current_q16_t shadow_iq;
            esp_foc_d_voltage_q16_t shadow_ud;
            esp_foc_q_voltage_q16_t shadow_uq;
            axis->regulator_cb(axis, &shadow_id, &shadow_iq,
                                     &shadow_ud, &shadow_uq);
            axis->target_i_d.raw = axis->tuner_override.target_id;
            axis->target_i_q.raw = axis->tuner_override.target_iq;
            axis->target_u_d.raw = axis->tuner_override.target_ud;
            axis->target_u_q.raw = axis->tuner_override.target_uq;
            continue;
        }
#endif
        axis->regulator_cb(axis, &axis->target_i_d, &axis->target_i_q,
                                 &axis->target_u_d, &axis->target_u_q);
    }
}

/* Apply autotuned gains to one of the torque controllers. The build-time
 * autogen header is the only source of truth in 3.0 — the legacy
 * continuous-time formula on motor R/L is gone. */
static void apply_autogen_gains(esp_foc_pid_controller_t *pid, q16_t out_max)
{
#if defined(ESP_FOC_AUTOGEN_GAINS_AVAILABLE)
    pid->kp = ESP_FOC_AUTOGEN_CURRENT_KP_Q16;
    pid->ki = ESP_FOC_AUTOGEN_CURRENT_KI_Q16;
    pid->integrator_limit = ESP_FOC_AUTOGEN_CURRENT_INT_LIM_Q16;
#else
    /* Without autogen, leave gains zeroed so the user MUST set them via
     * the runtime tuner before commanding any current. Better a silent
     * motor than a runaway with garbage gains. */
    pid->kp = 0;
    pid->ki = 0;
    pid->integrator_limit = 0;
#endif
    pid->kd = 0;
    pid->max_output_value = out_max;
    pid->min_output_value = q16_from_float(-q16_to_float(out_max));
    esp_foc_pid_reset(pid);
}

esp_foc_err_t esp_foc_initialize_axis(esp_foc_axis_t *axis,
                                      esp_foc_inverter_t *inverter,
                                      esp_foc_rotor_sensor_t *rotor,
                                      esp_foc_isensor_t *isensor,
                                      esp_foc_motor_control_settings_t settings)
{
    float vbus_pu;
    float pwm_rate_hz_f;
    float dt_f;

    if (axis == NULL || inverter == NULL || rotor == NULL) {
        ESP_LOGE(tag, "invalid args for axis initialization");
        return ESP_FOC_ERR_INVALID_ARG;
    }

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
    axis->magic = ESP_FOC_AXIS_MAGIC;
    axis->tuner_override.active = false;
    axis->tuner_override.target_id = 0;
    axis->tuner_override.target_iq = 0;
    axis->tuner_override.target_ud = 0;
    axis->tuner_override.target_uq = 0;
#endif

    axis->rotor_aligned = ESP_FOC_ERR_AXIS_INVALID_STATE;
    axis->inverter_driver = inverter;
    axis->rotor_sensor_driver = rotor;
    axis->isensor_driver = isensor;

    axis->dc_link_voltage = axis->inverter_driver->get_dc_link_voltage(axis->inverter_driver);
    vbus_pu = q16_to_float(axis->dc_link_voltage);
    axis->dc_link_to_normalized = q16_from_float((vbus_pu > 1e-9f) ? (1.0f / vbus_pu) : 0.0f);

#ifdef CONFIG_ESP_FOC_USE_SINE_PWM
    axis->max_voltage = q16_from_float(vbus_pu / 2.0f);
#else
    axis->max_voltage = q16_from_float(vbus_pu / 1.7320508075688772f);
#endif

    axis->inverter_driver->set_voltages(axis->inverter_driver, 0, 0, 0);

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
    axis->target_u_d.raw = 0;
    axis->target_u_q.raw = 0;
    axis->skip_torque_control = 0;

    if (isensor != NULL) {
        axis->isensor_driver->calibrate_isensors(axis->isensor_driver, ESP_FOC_ISENSOR_CALIBRATION_ROUNDS);
    }

    /* Both torque controllers (Q-axis and D-axis) share the same dt and
     * the same starting gains; the runtime tuner can rewrite either one
     * later with esp_foc_axis_set_current_pi_gains_q16(). */
    float loop_dt_s = dt_f * (float)ESP_FOC_LOW_SPEED_DOWNSAMPLING;
    float loop_fs_hz = (loop_dt_s > 1e-9f) ? (1.0f / loop_dt_s) : 0.0f;
    q16_t loop_dt = q16_from_float(loop_dt_s);
    q16_t loop_inv_dt = q16_from_float(loop_fs_hz);
    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].dt = loop_dt;
        axis->torque_controller[i].inv_dt = loop_inv_dt;
        apply_autogen_gains(&axis->torque_controller[i], axis->max_voltage);
    }

    /* Resolve the current-sensor low-pass cutoff. Precedence at boot:
     *   1. NVS calibration overlay (when its current_filter_fc != 0).
     *   2. CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ (default 300 Hz).
     * The runtime tuner can override later via the protocol. */
    float current_filter_fc_hz = (float)CONFIG_ESP_FOC_CURRENT_FILTER_CUTOFF_HZ;

    /* If a tuned calibration exists for this axis AND it was made for
     * the same motor profile, override the autogen seed gains. NVS
     * load is a cold path; failure (no entry, profile mismatch, NVS
     * empty) just skips the overlay silently. */
    {
        esp_foc_calibration_data_t cal;
        if (esp_foc_calibration_load((uint8_t)settings.motor_unit, &cal)
            == ESP_FOC_OK) {
            for (int i = 0; i < 2; ++i) {
                axis->torque_controller[i].kp = cal.kp;
                axis->torque_controller[i].ki = cal.ki;
                axis->torque_controller[i].integrator_limit =
                    cal.integrator_limit;
            }
            if (cal.current_filter_fc_hz > 0) {
                current_filter_fc_hz = q16_to_float(cal.current_filter_fc_hz);
            }
            ESP_LOGI(tag,
                     "axis %d: NVS calibration overlay applied "
                     "(Kp=%.4f Ki=%.2f fc=%.1fHz)",
                     settings.motor_unit,
                     (double)q16_to_float(cal.kp),
                     (double)q16_to_float(cal.ki),
                     (double)current_filter_fc_hz);
        }
    }

    if (isensor != NULL && axis->isensor_driver->set_filter_cutoff != NULL) {
        axis->isensor_driver->set_filter_cutoff(axis->isensor_driver,
                                                current_filter_fc_hz,
                                                loop_fs_hz);
    }
    axis->current_filter_fc_hz_q16 = q16_from_float(current_filter_fc_hz);
    axis->current_filter_fs_hz_q16 = q16_from_float(loop_fs_hz);

#if defined(CONFIG_ESP_FOC_ISR_HOT_PATH)
    axis->latest_i_alpha = 0;
    axis->latest_i_beta = 0;
    /* Predictor starts uninitialised; the first outer-loop tick after
     * alignment will seed it from the live encoder reading. Gains
     * come from Kconfig (stored x1000 to dodge float). */
    float pred_alpha = (float)CONFIG_ESP_FOC_PREDICTOR_ALPHA_X1000 / 1000.0f;
    float pred_beta = (float)CONFIG_ESP_FOC_PREDICTOR_BETA_X1000 / 1000.0f;
    esp_foc_angle_predictor_init_q16(&axis->angle_predictor,
                                     pred_alpha, pred_beta,
                                     esp_foc_now_useconds());
    /* Wire the ADC ISR's Clarke-publish path straight into the axis
     * fields the PWM ISR will read. NULL targets disable publishing,
     * so this is also the opt-in point for the new pipeline. */
    if (isensor != NULL && axis->isensor_driver->set_publish_targets != NULL) {
        axis->isensor_driver->set_publish_targets(
            axis->isensor_driver,
            (q16_t *)&axis->latest_i_alpha,
            (q16_t *)&axis->latest_i_beta);
    }
#endif

    axis->motor_pole_pairs = settings.motor_pole_pairs;

    esp_foc_biquad_butterworth_lpf_design_q16(
        &axis->velocity_filter,
        (float)CONFIG_ESP_FOC_VELOCITY_FILTER_CUTOFF_HZ,
        loop_fs_hz);

    /* settings.natural_direction is a hint; esp_foc_align_axis() probes
     * the rotor and overrides this once the motor is energised. */
    axis->natural_direction = (settings.natural_direction == ESP_FOC_MOTOR_NATURAL_DIRECTION_CW)
                              ? Q16_ONE : Q16_MINUS_ONE;

    axis->high_speed_loop_cb = do_current_mode_sensored_high_speed_loop;
    axis->low_speed_loop_cb = do_current_mode_sensored_low_speed_loop;
    axis->outer_loop_cb = do_foc_outer_loop;

    esp_foc_sleep_ms(250);
    axis->rotor_aligned = ESP_FOC_ERR_NOT_ALIGNED;
    return ESP_FOC_OK;
}

/* Minimum encoder deflection considered conclusive when probing for the
 * natural direction. Expressed in raw encoder counts; AS5600 has 4096
 * counts/rev so 50 counts is ~4.4 degrees — well above noise but small
 * enough that a stuck rotor will obviously fail to clear the bar. */
#define ESP_FOC_DIR_PROBE_MIN_COUNTS 50

esp_foc_err_t esp_foc_align_axis(esp_foc_axis_t *axis)
{
    q16_t e_sin, e_cos;
    q16_t a, b, u, v, w;

    if (axis == NULL) {
        return ESP_FOC_ERR_INVALID_ARG;
    }
    if (axis->rotor_aligned != ESP_FOC_ERR_NOT_ALIGNED) {
        return ESP_FOC_ERR_AXIS_INVALID_STATE;
    }

    axis->rotor_aligned = ESP_FOC_ERR_ALIGNMENT_IN_PROGRESS;
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0, 0, 0);

    e_sin = q16_sin(0);
    e_cos = q16_cos(0);

    axis->inverter_driver->enable(axis->inverter_driver);
    esp_foc_sleep_ms(500);

    /* Step 1 — park the rotor at electrical angle 0 by driving Vd. */
    esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                Q16_ONE, 0,
                                &a, &b, &u, &v, &w,
                                axis->max_voltage,
                                axis->dc_link_to_normalized);
    axis->inverter_driver->set_voltages(axis->inverter_driver, u, v, w);
    esp_foc_sleep_ms(500);

    /* Step 2 — zero the encoder. The current physical position is now
     * the firmware's electrical zero. */
    axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);
    int64_t ticks_zero = axis->rotor_sensor_driver->read_accumulated_counts_i64(
                            axis->rotor_sensor_driver);

    /* Step 3 — probe natural direction by applying a positive Vq pulse.
     * In a properly aligned PMSM, +Vq produces forward torque. We watch
     * raw encoder counts to decide whether the encoder counts up or
     * down on physical forward motion, and set natural_direction so
     * the computed axis->rotor_position increases in the forward
     * direction. The settings hint is kept only when the rotor refuses
     * to move enough to be conclusive (load-stuck or open phase). */
    q16_t probe_vq = q16_from_float(q16_to_float(axis->max_voltage) * 0.30f);
    esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                0, probe_vq,
                                &a, &b, &u, &v, &w,
                                axis->max_voltage,
                                axis->dc_link_to_normalized);
    axis->inverter_driver->set_voltages(axis->inverter_driver, u, v, w);
    esp_foc_sleep_ms(300);

    int64_t ticks_after = axis->rotor_sensor_driver->read_accumulated_counts_i64(
                             axis->rotor_sensor_driver);
    int64_t delta = ticks_after - ticks_zero;

    if (delta >= ESP_FOC_DIR_PROBE_MIN_COUNTS) {
        axis->natural_direction = Q16_ONE;        /* CW */
        ESP_LOGI(tag, "alignment: natural direction = CW (delta=%lld)",
                 (long long)delta);
    } else if (delta <= -ESP_FOC_DIR_PROBE_MIN_COUNTS) {
        axis->natural_direction = Q16_MINUS_ONE;  /* CCW */
        ESP_LOGI(tag, "alignment: natural direction = CCW (delta=%lld)",
                 (long long)delta);
    } else {
        ESP_LOGW(tag, "alignment: rotor moved only %lld counts (need >= %d) — "
                      "keeping the natural_direction hint from settings",
                 (long long)delta, ESP_FOC_DIR_PROBE_MIN_COUNTS);
    }

    /* Step 4 — re-park the rotor at electrical 0 and re-zero the
     * encoder so steady state matches what the control loop expects. */
    esp_foc_modulate_dq_voltage(e_sin, e_cos,
                                Q16_ONE, 0,
                                &a, &b, &u, &v, &w,
                                axis->max_voltage,
                                axis->dc_link_to_normalized);
    axis->inverter_driver->set_voltages(axis->inverter_driver, u, v, w);
    esp_foc_sleep_ms(500);
    axis->rotor_sensor_driver->set_to_zero(axis->rotor_sensor_driver);

    axis->rotor_aligned = ESP_FOC_OK;
    axis->inverter_driver->set_voltages(axis->inverter_driver, 0, 0, 0);
    esp_foc_sleep_ms(500);
    return ESP_FOC_OK;
}

esp_foc_err_t esp_foc_run(esp_foc_axis_t *axis)
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
