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

#include <stdbool.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/foc_math_q16.h"

static const char * tag = "ESP_FOC_CONTROL";

#if defined(CONFIG_ESP_FOC_ISR_HOT_PATH)

/* ============================================================
 *  Plan #2: full FOC pipeline runs inside the PWM ISR.
 *
 *  The ADC ISR has already published latest_i_alpha / latest_i_beta
 *  on the axis. The angle predictor lives on the axis too; the outer
 *  task feeds it with fresh encoder readings and we extrapolate
 *  every PWM cycle.
 *
 *  set_voltages() down to mcpwm_comparator_set_compare_value() is
 *  IRAM-attributed in IDF v5.5; the rest of the helpers used here
 *  (q16_park, q16_sin/cos LUT, esp_foc_modulate_dq_voltage,
 *  esp_foc_pid_update) are pure Q16 / IRAM-friendly arithmetic.
 *  Everything in this ISR has to either live in IRAM or be cache-
 *  warm; the IRAM_ATTR on the function itself plus the LUT pinning
 *  in esp_foc_iq31.c covers that.
 * ============================================================ */

IRAM_ATTR static void foc_hot_isr(void *data)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)data;

    /* 1) Snapshot the latest Clarke-published currents. Two volatile
     *    q16 reads — single-instruction loads, no torn read. */
    q16_t i_alpha = axis->latest_i_alpha;
    q16_t i_beta  = axis->latest_i_beta;

    /* 2) Predict electrical angle for "now". This call only touches
     *    the predictor's read-only fields; the outer task wraps
     *    update() in a critical section the ISR also honours, so the
     *    snapshot here is consistent. */
    uint64_t now_us = esp_foc_now_useconds();
    q16_t theta_e = esp_foc_angle_predictor_predict_q16(
                        &axis->angle_predictor, now_us);

    /* 3) sin / cos. Backed by the IQ31 LUT in iram, no flash hit. */
    q16_t e_sin = q16_sin(theta_e);
    q16_t e_cos = q16_cos(theta_e);

    /* 4) Park transform. */
    q16_t i_d, i_q;
    q16_park(e_sin, e_cos, i_alpha, i_beta, &i_d, &i_q);

    /* 5) Publish for scope / status. */
    axis->i_alpha.raw = i_alpha;
    axis->i_beta.raw  = i_beta;
    axis->i_d.raw = i_d;
    axis->i_q.raw = i_q;
    axis->rotor_elec_angle = theta_e;

    /* 6) Current PI (or override / open-loop voltage). */
    q16_t u_d, u_q;

#if defined(CONFIG_ESP_FOC_TUNER_ENABLE)
    if (axis->tuner_override.active) {
        /* Tuner has the steering wheel — bypass the PI completely. */
        u_d = axis->tuner_override.target_ud;
        u_q = axis->tuner_override.target_uq;
    } else
#endif
    if (axis->skip_torque_control) {
        /* Open-loop voltage mode: take the operator's u_d / u_q
         * straight through, no integration. */
        u_d = axis->target_u_d.raw;
        u_q = axis->target_u_q.raw;
    } else {
#if defined(CONFIG_ESP_FOC_INJECTION_ENABLE)
        q16_t iq_ref = esp_foc_injection_apply_q16(&axis->injection,
                                                   axis->target_i_q.raw);
#else
        q16_t iq_ref = axis->target_i_q.raw;
#endif
        u_q = esp_foc_pid_update(&axis->torque_controller[0],
                                 iq_ref, i_q);
        u_d = esp_foc_pid_update(&axis->torque_controller[1],
                                 axis->target_i_d.raw, i_d);
        /* User-provided feed-forward gets stacked on top of the PI
         * output (matches the legacy semantics 1:1). */
        u_q = q16_add(u_q, axis->target_u_q.raw);
        u_d = q16_add(u_d, axis->target_u_d.raw);
    }
    axis->u_q.raw = u_q;
    axis->u_d.raw = u_d;

    /* 7) Inverse Park + SVPWM in one shot. */
    q16_t u_alpha, u_beta, u_u, u_v, u_w;
    esp_foc_modulate_dq_voltage(e_sin, e_cos, u_d, u_q,
                                &u_alpha, &u_beta,
                                &u_u, &u_v, &u_w,
                                axis->max_voltage,
                                axis->dc_link_to_normalized);
    axis->u_alpha.raw = u_alpha;
    axis->u_beta.raw  = u_beta;
    axis->u_u.raw = u_u;
    axis->u_v.raw = u_v;
    axis->u_w.raw = u_w;

    /* 8) Push duties to the inverter. mcpwm_comparator_set_compare_value
     *    is IRAM-safe in IDF v5.5. */
    axis->inverter_driver->set_voltages(axis->inverter_driver, u_u, u_v, u_w);

    /* 9) Optionally snapshot every wired channel into the scope ring.
     *    The from_isr variant uses xTaskNotifyGiveFromISR on rollover. */
#if defined(CONFIG_ESP_FOC_SCOPE)
    esp_foc_scope_data_push_from_isr();
#endif

    /* 10) Downsample counter. When it expires, wake the outer task
     *     so it reads the encoder, updates the predictor and dispatches
     *     the user regulation callback. */
    if (axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if (!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            if (axis->low_speed_ev != NULL) {
                esp_foc_send_notification_from_isr(axis->low_speed_ev);
            }
        }
    }
}

/* The ADC publish path now lives inside the ADC driver itself (Clarke
 * + atomic write to axis->latest_i_alpha/beta). No "high-speed callback"
 * is needed any more, but the field stays wired to a no-op so existing
 * isensor implementations that always invoke it stay happy. */
void do_current_mode_sensored_high_speed_loop(void *arg)
{
    (void)arg;
}

void do_current_mode_sensored_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;

    ESP_LOGI(tag, "Starting sensored FOC outer loop (ISR hot path)");

    /* Cold-start: take one encoder reading before the ISR starts so
     * the predictor seeds with a real angle (otherwise the first
     * 25 us of PWM would extrapolate from theta_est = 0). */
    q16_t ticks0 = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_shaft_ticks = ticks0;
    axis->rotor_position = q16_mul(ticks0, axis->natural_direction);
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0;
    {
        int pp = axis->motor_pole_pairs;
        int64_t et = (int64_t)axis->rotor_position * (int64_t)pp;
        q16_t elec_frac = (q16_t)(et % (int64_t)Q16_ONE);
        if (elec_frac < 0) {
            elec_frac = (q16_t)((int64_t)elec_frac + (int64_t)Q16_ONE);
        }
        q16_t theta_meas = q16_mul(elec_frac, Q16_TWO_PI);
        esp_foc_critical_enter();
        esp_foc_angle_predictor_update_q16(&axis->angle_predictor,
                                            theta_meas,
                                            esp_foc_now_useconds());
        esp_foc_critical_leave();
    }

    /* Hand the ISR over to the inverter driver only AFTER the
     * predictor has a real seed — otherwise the first PWM tick would
     * fire set_voltages() with garbage angles. */
    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
                                                 foc_hot_isr,
                                                 axis);

    while (1) {
        esp_foc_wait_notifier();

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_set();
#endif

        /* 1) Read the slow encoder. May block on I2C / SPI for ~125 us. */
        q16_t ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
        axis->rotor_shaft_ticks = ticks;
        q16_t pos = q16_mul(ticks, axis->natural_direction);
        axis->rotor_position = pos;

        /* 2) Mechanical angle * pp -> normalised electrical angle. */
        int pp = axis->motor_pole_pairs;
        int64_t et = (int64_t)pos * (int64_t)pp;
        q16_t elec_frac = (q16_t)(et % (int64_t)Q16_ONE);
        if (elec_frac < 0) {
            elec_frac = (q16_t)((int64_t)elec_frac + (int64_t)Q16_ONE);
        }
        q16_t theta_meas = q16_mul(elec_frac, Q16_TWO_PI);

        /* 3) Fold into the alpha-beta tracker under critical section
         *    so the PWM ISR cannot land mid-update and read a torn
         *    {theta_est, omega_est, t_last} triple. */
        uint64_t now_us = esp_foc_now_useconds();
        esp_foc_critical_enter();
        esp_foc_angle_predictor_update_q16(&axis->angle_predictor,
                                            theta_meas, now_us);
        esp_foc_critical_leave();

        /* 4) Velocity smoothing: the outer loop is the rate at which
         *    rotor_position actually changes, so the biquad sees a
         *    consistent dt. */
        q16_t dtheta = q16_sub(pos, axis->rotor_position_prev);
        q16_t raw_speed = q16_mul(dtheta,
                                   axis->torque_controller[0].inv_dt);
        axis->current_speed = esp_foc_biquad_q16_update(
                                  &axis->velocity_filter, raw_speed);
        axis->rotor_position_prev = pos;

        /* 5) Wake the regulator task so the user callback can refresh
         *    target_i_d / target_i_q / target_u_d / target_u_q before
         *    the next batch of PWM cycles. */
        esp_foc_send_notification(axis->regulator_ev);

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_clear();
#endif
    }
}

#else /* CONFIG_ESP_FOC_ISR_HOT_PATH not set — legacy 2.x task path */

static void inverter_isr(void *data)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)data;

    if(axis->downsampling_low_speed) {
        axis->downsampling_low_speed--;
        if(!axis->downsampling_low_speed) {
            axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
            esp_foc_send_notification_from_isr(axis->low_speed_ev);
        }
    }
}

void do_current_mode_sensored_high_speed_loop(void *arg)
{
    /* timestamp the current readings */
    uint64_t * current_timestamp = (uint64_t *)arg;
    *current_timestamp = esp_foc_now_useconds();
}

void do_current_mode_sensored_low_speed_loop(void *arg)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)arg;
    isensor_values_t ival_q16;
    q16_t iu_q16;
    q16_t iv_q16;
    q16_t iw_q16;
    q16_t current_speed_q16;
    uint64_t current_timestamp;
    bool no_isensor = (axis->isensor_driver == NULL) ? true : false;

    axis->low_speed_ev = esp_foc_get_event_handle();
    axis->downsampling_low_speed = ESP_FOC_LOW_SPEED_DOWNSAMPLING;
    current_timestamp = esp_foc_now_useconds();

    ESP_LOGI(tag,"Starting sensored FOC loop");

    q16_t ticks_q16 = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_shaft_ticks = ticks_q16;
    axis->rotor_position = q16_mul(ticks_q16, axis->natural_direction);
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0;

    axis->inverter_driver->set_inverter_callback(axis->inverter_driver,
        inverter_isr,
        axis);

    if(!no_isensor) {
        axis->isensor_driver->set_isensor_callback(axis->isensor_driver,
            axis->high_speed_loop_cb,
            &current_timestamp);
    }

    while(1) {
        esp_foc_wait_notifier();

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
       esp_foc_debug_pin_set();
#endif
        if(!no_isensor) {
            /* The ADC readings are already buffered while the angle sensor is being read */
            axis->isensor_driver->fetch_isensors(axis->isensor_driver, &ival_q16);
            iu_q16 = ival_q16.iu_axis_0;
            iv_q16 = ival_q16.iv_axis_0;
            iw_q16 = ival_q16.iw_axis_0;
            axis->i_u = iu_q16;
            axis->i_v = iv_q16;
            axis->i_w = iw_q16;

            /* Start new sample immediately; next loop uses it */
            axis->isensor_driver->sample_isensors(axis->isensor_driver);
        }

        ticks_q16 = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
        axis->rotor_shaft_ticks = ticks_q16;
        axis->rotor_position = q16_mul(ticks_q16, axis->natural_direction);

        {
            int pp = axis->motor_pole_pairs;
            int64_t et = (int64_t)axis->rotor_position * (int64_t)pp;
            q16_t elec_frac = (q16_t)(et % (int64_t)Q16_ONE);
            if (elec_frac < 0) {
                elec_frac = (q16_t)((int64_t)elec_frac + (int64_t)Q16_ONE);
            }
            q16_t elec_from_enc = q16_mul(elec_frac, Q16_TWO_PI);
            q16_t omega_e = q16_mul(axis->current_speed, q16_from_int(pp));
            uint64_t now_us = esp_foc_now_useconds();
            uint64_t us_abs = (now_us >= current_timestamp)
                                  ? (now_us - current_timestamp)
                                  : (current_timestamp - now_us);
            q16_t theta_ts = q16_from_elapsed_us_u64(us_abs);
            axis->rotor_elec_angle = q16_normalize_angle_rad(
                q16_sub(elec_from_enc, q16_mul(omega_e, theta_ts)));
        }

        q16_t elec_angle_norm_q16 = q16_normalize_angle_rad(axis->rotor_elec_angle);
        q16_t e_sin_q16 = q16_sin(elec_angle_norm_q16);
        q16_t e_cos_q16 = q16_cos(elec_angle_norm_q16);

        /* ω [rad/s] in Q16: Δθ [rad] * (1/T_low) [1/s]; inv_dt matches core init (1/(dt_hs*N)). */
        {
            q16_t dtheta = q16_sub(axis->rotor_position, axis->rotor_position_prev);
            q16_t raw_speed_q16 = q16_mul(dtheta, axis->torque_controller[0].inv_dt);
            current_speed_q16 = esp_foc_biquad_q16_update(&axis->velocity_filter, raw_speed_q16);
        }
        axis->current_speed = current_speed_q16;
        axis->rotor_position_prev = axis->rotor_position;

        if(!no_isensor) {
            q16_t i_alpha_q16, i_beta_q16, i_q_q16, i_d_q16;
            esp_foc_get_dq_currents(e_sin_q16,
                e_cos_q16,
                axis->i_u,
                axis->i_v,
                axis->i_w,
                &i_alpha_q16,
                &i_beta_q16,
                &i_q_q16,
                &i_d_q16);

            axis->i_alpha.raw = i_alpha_q16;
            axis->i_beta.raw = i_beta_q16;
            axis->i_q.raw = i_q_q16;
            axis->i_d.raw = i_d_q16;

            if (!axis->skip_torque_control) {
                /* No EMA on i_q / i_d any more — the per-phase
                 * Butterworth inside the isensor driver has already
                 * shaped the spectrum upstream of Clarke / Park. */
#if defined(CONFIG_ESP_FOC_INJECTION_ENABLE)
                q16_t iq_ref = esp_foc_injection_apply_q16(&axis->injection,
                                                           axis->target_i_q.raw);
#else
                q16_t iq_ref = axis->target_i_q.raw;
#endif
                q16_t u_q_q16 = esp_foc_pid_update(&axis->torque_controller[0],
                                                         iq_ref,
                                                         i_q_q16);
                q16_t u_d_q16 = esp_foc_pid_update(&axis->torque_controller[1],
                                                         axis->target_i_d.raw,
                                                         i_d_q16);
                u_q_q16 = q16_add(u_q_q16, axis->target_u_q.raw);
                u_d_q16 = q16_add(u_d_q16, axis->target_u_d.raw);
                axis->u_q.raw = u_q_q16;
                axis->u_d.raw = u_d_q16;
            } else {
                axis->u_q.raw = axis->target_u_q.raw;
                axis->u_d.raw = axis->target_u_d.raw;
            }
        }
        if (no_isensor) {
            axis->u_q.raw = axis->target_u_q.raw;
            axis->u_d.raw = axis->target_u_d.raw;
        }

        q16_t u_alpha_q16, u_beta_q16, u_u_q16, u_v_q16, u_w_q16;
        esp_foc_modulate_dq_voltage(e_sin_q16,
                        e_cos_q16,
                        axis->u_d.raw,
                        axis->u_q.raw,
                        &u_alpha_q16,
                        &u_beta_q16,
                        &u_u_q16,
                        &u_v_q16,
                        &u_w_q16,
                        axis->max_voltage,
                        axis->dc_link_to_normalized);

        axis->u_alpha.raw = u_alpha_q16;
        axis->u_beta.raw = u_beta_q16;
        axis->u_u.raw = u_u_q16;
        axis->u_v.raw = u_v_q16;
        axis->u_w.raw = u_w_q16;
        axis->inverter_driver->set_voltages(axis->inverter_driver, u_u_q16, u_v_q16, u_w_q16);

        esp_foc_send_notification(axis->regulator_ev);

#ifdef CONFIG_ESP_FOC_SCOPE
        esp_foc_scope_data_push();
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
       esp_foc_debug_pin_clear();
#endif
    }
}

#endif /* CONFIG_ESP_FOC_ISR_HOT_PATH */
