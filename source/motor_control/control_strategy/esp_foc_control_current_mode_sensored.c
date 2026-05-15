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
#include <stdint.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espFoC/esp_foc.h"
#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math_q16.h"

#if defined(CONFIG_ESP_FOC_SCOPE)
q16_t esp_foc_debug_scope_hot_path_dt_us_q16;
#endif

static const char * tag = "ESP_FOC_CONTROL";

/* ============================================================
 * full FOC pipeline runs inside the PWM ISR.
 *
 *  Per tick: read the previous completed conversion (n-1) from
 *  latest_i_alpha / latest_i_beta, then re-arm the shunt ADC so the
 *  next frame runs in parallel with the rest of this ISR. The outer
 *  task does not call sample_isensors.
 *
 *  set_voltages() takes phase volts; the MCPWM driver converts to duty then
 *  comparator writes — IRAM-attributed in IDF v5.5. Other helpers used here
 *  (q16_park, q16_sin/cos LUT, esp_foc_modulate_dq_voltage,
 *  esp_foc_pid_update) are pure Q16 / IRAM-friendly arithmetic.
 *  Everything in this ISR has to either live in IRAM or be cache-
 *  warm; the IRAM_ATTR on the function itself plus the LUT pinning
 *  in esp_foc_iq31.c covers that.
 * ============================================================ */

static void foc_hot_isr(void *data)
{
    esp_foc_axis_t *axis = (esp_foc_axis_t *)data;
    if (axis == NULL || axis->state != ESP_FOC_AXIS_STATE_RUNNING) {
        return;
    }

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_set();
#endif

#if defined(CONFIG_ESP_FOC_SCOPE)
    uint64_t hot_path_t0 = esp_foc_now_useconds();
#endif

    /* 1) Previous sample (n-1), published by the ADC ISR. */
    q16_t i_alpha = axis->latest_i_alpha;
    q16_t i_beta  = axis->latest_i_beta;

    /* 2) Trigger the next conversion; it completes in parallel with
     *    the control law below (continuous and one-shot both gate off
     *    in the ADC ISR until re-armed). */
    if (axis->isensor_driver != NULL) {
        axis->isensor_driver->sample_isensors(axis->isensor_driver);
    }

    q16_t e_sin = q16_sin(q16_normalize_angle(axis->rotor_elec_angle));
    q16_t e_cos = q16_cos(q16_normalize_angle(axis->rotor_elec_angle));

    /* 3) Park transform. */
    q16_t i_d, i_q;
    q16_park(e_sin, e_cos, i_alpha, i_beta, &i_d, &i_q);

    /* 6) Publish for scope / status. */
    axis->i_alpha.raw = i_alpha;
    axis->i_beta.raw  = i_beta;
    axis->i_d.raw = i_d;
    axis->i_q.raw = i_q;

    /* 7) Current PI (or open-loop voltage).
     * Tuner override (CONFIG_ESP_FOC_TUNER_ENABLE): esp_foc_core copies
     * tuner_override.target_{id,iq,ud,uq} into axis->target_* each outer-loop
     * tick — id/iq feed the PI, ud/uq add as feed-forward (same as regulation). */
    if (axis->skip_torque_control) {
        /* Open-loop voltage mode: take the operator's u_d / u_q
         * straight through, no integration. */
        axis->u_q.raw = axis->target_u_d.raw;
        axis->u_d.raw = axis->target_u_q.raw;
    } else {
        axis->u_q.raw = esp_foc_pid_update(&axis->torque_controller[0],
                                 axis->target_i_q.raw, i_q);
        axis->u_d.raw = esp_foc_pid_update(&axis->torque_controller[1],
                                 axis->target_i_d.raw, i_d);
        /* User-provided feed-forward gets stacked on top of the PI
         * output (matches the legacy semantics 1:1). */
        axis->u_q.raw = q16_add(axis->u_q.raw, axis->target_u_q.raw);
        axis->u_d.raw = q16_add(axis->u_d.raw, axis->target_u_d.raw);
    }


    /* 8) Inverse Park + SVPWM (θ at output instant). */
    esp_foc_modulate_dq_voltage(e_sin, e_cos, axis->u_d.raw, axis->u_q.raw,
                                &axis->u_alpha.raw,
                                &axis->u_beta.raw,
                                &axis->u_u.raw,
                                &axis->u_v.raw,
                                &axis->u_w.raw,
                                axis->max_voltage);

    /* 9) Phase voltages [V] Q16 → driver applies SVPWM to duty. MCPWM
     *    comparator writes are IRAM-safe in IDF v5.5. */
    axis->inverter_driver->set_voltages(axis->inverter_driver,
                                    axis->u_u.raw,
                                    axis->u_v.raw,
                                    axis->u_w.raw);

    /* 10) Downsample counter. When it expires, wake the outer task
     *     so it reads the encoder, refreshes rotor_elec_angle and dispatches
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
    /* 11) Optionally snapshot every wired channel into the scope ring.
     *    The from_isr variant uses xTaskNotifyGiveFromISR on rollover. */
#if defined(CONFIG_ESP_FOC_SCOPE)
    esp_foc_debug_scope_hot_path_dt_us_q16 = calc_time_isr_q16(hot_path_t0,  esp_foc_now_useconds());
#endif

#ifdef CONFIG_ESP_FOC_DEBUG_CORE_TIMING
        esp_foc_debug_pin_clear();
#endif
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

    uint32_t cpr = axis->rotor_sensor_driver->get_counts_per_revolution(
        axis->rotor_sensor_driver);
    if (cpr == 0u) {
        ESP_LOGE(tag, "FATAL, INVALID CPR, CHECK YOUR ROTOR SENSOR!");
        abort();
    }

    /* Cold-start: one encoder sample → rotor_elec_angle before PWM ISR runs. */
    axis->rotor_shaft_ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
    axis->rotor_position = q16_mul(axis->rotor_shaft_ticks, axis->natural_direction);
    axis->rotor_position_prev = axis->rotor_position;
    axis->current_speed = 0;
    axis->rotor_elec_angle = q16_mul(q16_mul(axis->rotor_position,
                                            axis->encoder_inv_cpr_q16),
                                            q16_from_int(axis->motor_pole_pairs));

    /* Hand the ISR over only after rotor_elec_angle is valid. */
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

        /* 1) Read the slow encoder. May block on I2C / SPI for ~125 us. */
        axis->rotor_shaft_ticks = axis->rotor_sensor_driver->read_counts(axis->rotor_sensor_driver);
        axis->rotor_position = q16_mul(axis->rotor_shaft_ticks, axis->natural_direction);

        /* 2) θe [0, 2*pi) from encoder — ISR reads rotor_elec_angle each PWM. */
        axis->rotor_elec_angle = q16_mul(q16_mul(axis->rotor_position, axis->encoder_inv_cpr_q16),
                                 q16_from_int(axis->motor_pole_pairs));

        /* 3) Velocity smoothing: the outer loop is the rate at which
         *    rotor_position actually changes, so the biquad sees a
         *    consistent dt. */
        q16_t dtheta = q16_sub(axis->rotor_position, axis->rotor_position_prev);
        q16_t raw_speed = q16_mul(dtheta,
                                   axis->inv_dt * q16_from_int(ESP_FOC_LOW_SPEED_DOWNSAMPLING));
        axis->current_speed = esp_foc_biquad_q16_update(
                                  &axis->velocity_filter, raw_speed);
        axis->rotor_position_prev = axis->rotor_position;

#if defined(CONFIG_ESP_FOC_SCOPE)
        esp_foc_scope_data_push();
#endif
        /* 4) Wake the regulator task so the user callback can refresh
         *    target_i_d / target_i_q / target_u_d / target_u_q before
         *    the next batch of PWM cycles. */
        if (axis->regulator_ev != NULL) {
            esp_foc_send_notification(axis->regulator_ev);
        }
    }
    axis->low_speed_ev = NULL;
    vTaskDelete(NULL);
}