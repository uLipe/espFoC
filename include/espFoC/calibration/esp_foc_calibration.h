/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sdkconfig.h>
#include "espFoC/esp_foc_err.h"
#include "espFoC/esp_foc_axis.h"
#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t kd;
    q16_t kff;
    q16_t integrator_limit;
    q16_t current_filter_fc_hz;
    uint8_t reserved[16];
} esp_foc_calibration_data_t;

#define ESP_FOC_CALIBRATION_MAX_AXES 4u

#define ESP_FOC_CAL_ALIGN_FLAG_OFFSET 0x01u
#define ESP_FOC_CAL_ALIGN_FLAG_DIR    0x02u

void esp_foc_calibration_pack_align(esp_foc_calibration_data_t *d,
                                     uint8_t flags,
                                     uint16_t enc_zero_12b,
                                     q16_t natural_direction);
void esp_foc_calibration_get_align(const esp_foc_calibration_data_t *d,
                                  uint8_t *flags,
                                  uint16_t *enc_zero_12b,
                                  q16_t *natural_direction);

void esp_foc_calibration_pack_pole_pairs(esp_foc_calibration_data_t *d,
                                         int32_t motor_pole_pairs);
int32_t esp_foc_calibration_get_pole_pairs(
    const esp_foc_calibration_data_t *d);

uint32_t esp_foc_calibration_profile_hash(void);

esp_foc_err_t esp_foc_calibration_save(uint8_t axis_id,
                                       const esp_foc_calibration_data_t *data);
esp_foc_err_t esp_foc_calibration_load(uint8_t axis_id,
                                       esp_foc_calibration_data_t *out);
esp_foc_err_t esp_foc_calibration_erase(void);
bool esp_foc_calibration_present(uint8_t axis_id);

void esp_foc_calibration_axis_init_store(
    esp_foc_axis_t *axis,
    const esp_foc_motor_control_settings_t *settings);

void esp_foc_calibration_axis_boot_apply(esp_foc_axis_t *axis,
                                          float *inout_current_filter_fc_hz,
                                          float pwm_loop_fs_hz);

void esp_foc_calibration_axis_align_apply_stored_hints(
    esp_foc_axis_t *axis,
    bool *skip_dir_probe);
void esp_foc_calibration_axis_align_persist_snapshot(esp_foc_axis_t *axis);

esp_foc_err_t esp_foc_calibration_axis_tuner_persist(esp_foc_axis_t *axis);
esp_foc_err_t esp_foc_calibration_axis_tuner_load_apply(esp_foc_axis_t *axis);

void esp_foc_calibration_axis_tuner_clear_cache(esp_foc_axis_t *axis);

#ifdef __cplusplus
}
#endif
