/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "espFoC/calibration/esp_foc_calibration.h"
#include "espFoC/tuning/esp_foc_axis_tuning.h"

void esp_foc_axis_refresh_encoder_q16_scales(esp_foc_axis_t *axis);

static const char *TAG = "esp_foc_cal";

#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)

static const char *nat_dir_short(q16_t n)
{
    if (n == Q16_ONE) {
        return "CW";
    }
    if (n == Q16_MINUS_ONE) {
        return "CCW";
    }
    return "?";
}

static void log_blob_boot_d(unsigned axis_id,
                             const esp_foc_calibration_data_t *cal,
                             bool enc_offset_applied,
                             bool gset)
{
    uint8_t af;
    uint16_t enc0;
    q16_t nat_d;
    float fc_lpf_hz = (cal->current_filter_fc_hz != 0)
                          ? q16_to_float(cal->current_filter_fc_hz) : 0.0f;

    esp_foc_calibration_get_align(cal, &af, &enc0, &nat_d);
    const bool have_off = (af & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u;
    const bool have_dir = (af & ESP_FOC_CAL_ALIGN_FLAG_DIR) != 0u
                          && (nat_d == Q16_ONE || nat_d == Q16_MINUS_ONE);
#if defined(CONFIG_ESP_FOC_USE_AUTOGEN_GAINS) && CONFIG_ESP_FOC_USE_AUTOGEN_GAINS
    ESP_LOGD(
        TAG,
        "axis %u detail: profile %s v%d hash=0x%08x",
        axis_id,
        CONFIG_ESP_FOC_MOTOR_PROFILE,
        (int)CONFIG_ESP_FOC_PROFILE_VERSION,
        (unsigned)esp_foc_calibration_profile_hash());
#else
    ESP_LOGD(TAG, "axis %u detail: hash=0x%08x (autogen off)",
             axis_id, (unsigned)esp_foc_calibration_profile_hash());
#endif
    {
        int32_t ppn = esp_foc_calibration_get_pole_pairs(cal);
        ESP_LOGD(
            TAG,
            "axis %u detail: PI Kp=%.4f Ki=%.2f ILim=%.3f I-lpf=%.1fHz "
            "motor R=%.4f L=%.4e bw=%.1f pole_pairs_stored=%d",
            axis_id,
            (double)q16_to_float(cal->kp), (double)q16_to_float(cal->ki),
            (double)q16_to_float(cal->integrator_limit), (double)fc_lpf_hz,
            (double)q16_to_float(cal->motor_r_ohm),
            (double)q16_to_float(cal->motor_l_h),
            (double)q16_to_float(cal->bandwidth_hz),
            (int)ppn);
    }
    ESP_LOGD(
        TAG,
        "axis %u detail: align off=%d enc=%u dir=%d nat=%s zero_applied=%d gset=%d",
        axis_id, (int)(have_off ? 1 : 0),
        (unsigned)(have_off ? enc0 : 0u),
        (int)(have_dir ? 1 : 0),
        have_dir ? nat_dir_short(nat_d) : "—",
        (int)(enc_offset_applied ? 1 : 0), (int)(gset ? 1 : 0));
}

#endif

void esp_foc_calibration_axis_init_store(
    esp_foc_axis_t *axis,
    const esp_foc_motor_control_settings_t *settings)
{
    axis->cal.axis_id = (uint8_t)settings->motor_unit;
    if (axis->cal.axis_id >= ESP_FOC_CALIBRATION_MAX_AXES) {
        axis->cal.axis_id = 0;
    }
    axis->cal.motor_r_ohm = 0;
    axis->cal.motor_l_h = 0;
    axis->cal.bandwidth_hz = 0;
}

void esp_foc_calibration_axis_boot_apply(
    esp_foc_axis_t *axis,
    float *inout_current_filter_fc_hz,
    float pwm_loop_fs_hz)
{
    (void)pwm_loop_fs_hz;
#if !defined(CONFIG_ESP_FOC_CALIBRATION_NVS)
    (void)axis;
    (void)inout_current_filter_fc_hz;
#else
    esp_foc_calibration_data_t cal;
    esp_foc_err_t cale = esp_foc_calibration_load(axis->cal.axis_id, &cal);
    if (cale != ESP_FOC_OK) {
        ESP_LOGI(TAG, "axis %u: no valid stored calibration for this build",
                 (unsigned)axis->cal.axis_id);
        ESP_LOGD(TAG, "axis %u: load status %d", (unsigned)axis->cal.axis_id,
                 (int)cale);
        return;
    }
    const bool gset = (axis->rotor_sensor_driver->set_zero_offset_raw_12b
                      != NULL);
    bool off_applied = false;
    for (int i = 0; i < 2; ++i) {
        axis->torque_controller[i].kp = cal.kp;
        axis->torque_controller[i].ki = cal.ki;
        axis->torque_controller[i].integrator_limit = cal.integrator_limit;
    }
    if (cal.current_filter_fc_hz > 0 && inout_current_filter_fc_hz != NULL) {
        *inout_current_filter_fc_hz = q16_to_float(cal.current_filter_fc_hz);
    }
    axis->cal.motor_r_ohm = cal.motor_r_ohm;
    axis->cal.motor_l_h = cal.motor_l_h;
    axis->cal.bandwidth_hz = cal.bandwidth_hz;
    {
        uint8_t aflags;
        uint16_t enc0;
        q16_t nat_d;
        esp_foc_calibration_get_align(&cal, &aflags, &enc0, &nat_d);
        if ((aflags & ESP_FOC_CAL_ALIGN_FLAG_DIR) != 0 &&
            (nat_d == Q16_ONE || nat_d == Q16_MINUS_ONE)) {
            axis->natural_direction = nat_d;
        }
        if ((aflags & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u && gset) {
            axis->rotor_sensor_driver->set_zero_offset_raw_12b(
                axis->rotor_sensor_driver, enc0);
            off_applied = true;
        }
    }
    ESP_LOGI(TAG, "axis %u: stored calibration loaded and applied",
             (unsigned)axis->cal.axis_id);
    log_blob_boot_d((unsigned)axis->cal.axis_id, &cal, off_applied, gset);
    {
        int32_t ppn = esp_foc_calibration_get_pole_pairs(&cal);
        if (ppn >= 1 && ppn <= 64) {
            axis->motor_pole_pairs = (int)ppn;
        }
    }
#endif
}

#if defined(CONFIG_ESP_FOC_CALIBRATION_NVS)

void esp_foc_calibration_axis_align_apply_stored_hints(
    esp_foc_axis_t *axis,
    bool *skip_dir_probe)
{
    esp_foc_calibration_data_t cal;
    const bool gset = (axis->rotor_sensor_driver->set_zero_offset_raw_12b
                      != NULL);

    *skip_dir_probe = false;
    if (esp_foc_calibration_load(axis->cal.axis_id, &cal) != ESP_FOC_OK) {
        ESP_LOGD(TAG, "align: axis %u — no valid stored blob",
                 (unsigned)axis->cal.axis_id);
        return;
    }
    uint8_t aflags;
    uint16_t enc0;
    q16_t nat_d;
    esp_foc_calibration_get_align(&cal, &aflags, &enc0, &nat_d);
    if ((aflags & ESP_FOC_CAL_ALIGN_FLAG_DIR) != 0u &&
        (nat_d == Q16_ONE || nat_d == Q16_MINUS_ONE)) {
        axis->natural_direction = nat_d;
        *skip_dir_probe = true;
    }
    if ((aflags & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u && gset) {
        axis->rotor_sensor_driver->set_zero_offset_raw_12b(
            axis->rotor_sensor_driver, enc0);
    }
    ESP_LOGD(
        TAG,
        "align hints axis %u: dir=%s skip_sweep=%d encNVS=%d raw=%u gset=%d",
        (unsigned)axis->cal.axis_id,
        ((aflags & ESP_FOC_CAL_ALIGN_FLAG_DIR) != 0u
         && (nat_d == Q16_ONE || nat_d == Q16_MINUS_ONE))
        ? nat_dir_short(nat_d) : "probe",
        (int)(*skip_dir_probe ? 1 : 0),
        (int)((aflags & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u ? 1 : 0),
        (unsigned)((aflags & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u ? enc0 : 0u),
        (int)(gset ? 1 : 0));
    if ((aflags & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) != 0u && !gset) {
        ESP_LOGD(
            TAG,
            "align: enc offset in store but rotor driver has no set_zero_offset_12b");
    }
}

void esp_foc_calibration_axis_align_persist_snapshot(esp_foc_axis_t *axis)
{
    uint8_t aflags = 0;
    uint16_t z = 0;

    if (axis->rotor_sensor_driver->get_zero_offset_12b != NULL) {
        aflags |= ESP_FOC_CAL_ALIGN_FLAG_OFFSET;
        z = axis->rotor_sensor_driver->get_zero_offset_12b(
            axis->rotor_sensor_driver);
    }
    if (axis->natural_direction == Q16_ONE ||
        axis->natural_direction == Q16_MINUS_ONE) {
        aflags |= ESP_FOC_CAL_ALIGN_FLAG_DIR;
    }

    {
        esp_foc_calibration_data_t data;
        if (esp_foc_calibration_load(axis->cal.axis_id, &data) != ESP_FOC_OK) {
            memset(&data, 0, sizeof(data));
            for (int i = 0; i < 2; ++i) {
                data.kp = axis->torque_controller[i].kp;
                data.ki = axis->torque_controller[i].ki;
                data.integrator_limit = axis->torque_controller[i].integrator_limit;
            }
            data.current_filter_fc_hz = axis->current_filter_fc_hz_q16;
        }
        esp_foc_calibration_pack_align(&data, aflags, z, axis->natural_direction);
        esp_foc_calibration_pack_pole_pairs(
            &data, (int32_t)axis->motor_pole_pairs);
        if (esp_foc_calibration_save(axis->cal.axis_id, &data) == ESP_FOC_OK) {
            axis->cal.motor_r_ohm = data.motor_r_ohm;
            axis->cal.motor_l_h = data.motor_l_h;
            axis->cal.bandwidth_hz = data.bandwidth_hz;
        }
    }
}

static q16_t read_q16_le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (q16_t)u;
}

esp_foc_err_t esp_foc_calibration_axis_tuner_persist(
    esp_foc_axis_t *axis,
    const uint8_t *motor_r_l_bw_q16,
    size_t payload_len)
{
    esp_foc_calibration_data_t data;
    if (esp_foc_calibration_load(axis->cal.axis_id, &data) != ESP_FOC_OK) {
        memset(&data, 0, sizeof(data));
    }
    esp_foc_axis_get_current_pi_gains_q16(axis, &data.kp, &data.ki,
                                          &data.integrator_limit);
    data.current_filter_fc_hz = axis->current_filter_fc_hz_q16;
    if (motor_r_l_bw_q16 != NULL && payload_len >= 12) {
        data.motor_r_ohm = read_q16_le(motor_r_l_bw_q16);
        data.motor_l_h = read_q16_le(motor_r_l_bw_q16 + 4);
        data.bandwidth_hz = read_q16_le(motor_r_l_bw_q16 + 8);
    }
    esp_foc_calibration_pack_pole_pairs(
        &data, (int32_t)axis->motor_pole_pairs);
    esp_foc_err_t err = esp_foc_calibration_save(axis->cal.axis_id, &data);
    if (err == ESP_FOC_OK) {
        axis->cal.motor_r_ohm = data.motor_r_ohm;
        axis->cal.motor_l_h = data.motor_l_h;
        axis->cal.bandwidth_hz = data.bandwidth_hz;
    }
    return err;
}

esp_foc_err_t esp_foc_calibration_axis_tuner_load_apply(esp_foc_axis_t *axis)
{
    esp_foc_calibration_data_t data;
    esp_foc_err_t err = esp_foc_calibration_load(axis->cal.axis_id, &data);
    if (err != ESP_FOC_OK) {
        return err;
    }
    err = esp_foc_axis_set_current_pi_gains_q16(axis, data.kp, data.ki,
                                                data.integrator_limit);
    if (err == ESP_FOC_OK && data.current_filter_fc_hz > 0 &&
        axis->isensor_driver != NULL &&
        axis->isensor_driver->set_filter_cutoff != NULL) {
        float fc = q16_to_float(data.current_filter_fc_hz);
        float fs = q16_to_float(axis->current_filter_fs_hz_q16);
        axis->isensor_driver->set_filter_cutoff(axis->isensor_driver,
                                                fc, fs);
        axis->current_filter_fc_hz_q16 = data.current_filter_fc_hz;
    }
    if (err == ESP_FOC_OK) {
        axis->cal.motor_r_ohm = data.motor_r_ohm;
        axis->cal.motor_l_h = data.motor_l_h;
        axis->cal.bandwidth_hz = data.bandwidth_hz;
        {
            int32_t ppn = esp_foc_calibration_get_pole_pairs(&data);
            if (ppn >= 1 && ppn <= 64) {
                axis->motor_pole_pairs = (int)ppn;
                esp_foc_axis_refresh_encoder_q16_scales(axis);
            }
        }
        {
            uint8_t af;
            uint16_t eraw;
            q16_t nstore;
            esp_foc_calibration_get_align(&data, &af, &eraw, &nstore);
            const char *cw = (nstore == Q16_ONE)   ? "CW" :
                (nstore == Q16_MINUS_ONE)         ? "CCW" : "?";
            ESP_LOGD(
                TAG,
                "LOAD axis %u: pp=%d R=%.4f L=%.4e bw=%.1f align off=%d enc=%u dir=%d %s",
                (unsigned)axis->cal.axis_id,
                axis->motor_pole_pairs,
                (double)q16_to_float(data.motor_r_ohm),
                (double)q16_to_float(data.motor_l_h),
                (double)q16_to_float(data.bandwidth_hz),
                (int)((af & ESP_FOC_CAL_ALIGN_FLAG_OFFSET) ? 1 : 0),
                (unsigned)eraw,
                (int)((af & ESP_FOC_CAL_ALIGN_FLAG_DIR) ? 1 : 0),
                (af & ESP_FOC_CAL_ALIGN_FLAG_DIR) ? cw : "—");
        }
    }
    return err;
}

#else /* !CONFIG_ESP_FOC_CALIBRATION_NVS */

void esp_foc_calibration_axis_align_apply_stored_hints(
    esp_foc_axis_t *axis,
    bool *skip_dir_probe)
{
    (void)axis;
    if (skip_dir_probe != NULL) {
        *skip_dir_probe = false;
    }
}

void esp_foc_calibration_axis_align_persist_snapshot(esp_foc_axis_t *axis)
{
    (void)axis;
}

esp_foc_err_t esp_foc_calibration_axis_tuner_persist(
    esp_foc_axis_t *axis,
    const uint8_t *motor_r_l_bw_q16,
    size_t payload_len)
{
    (void)axis;
    (void)motor_r_l_bw_q16;
    (void)payload_len;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
}

esp_foc_err_t esp_foc_calibration_axis_tuner_load_apply(esp_foc_axis_t *axis)
{
    (void)axis;
    return ESP_FOC_ERR_AXIS_INVALID_STATE;
}

#endif

void esp_foc_calibration_axis_tuner_clear_cache(esp_foc_axis_t *axis)
{
    axis->cal.motor_r_ohm = 0;
    axis->cal.motor_l_h = 0;
    axis->cal.bandwidth_hz = 0;
}
