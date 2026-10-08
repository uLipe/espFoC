/*
 * MIT License
 *
 * Copyright (c) 2026 Felipe Neves
 *
 * Internal software 2p2z PID entry points.
 */
#pragma once

#include "espFoC/utils/esp_foc_pid.h"

esp_err_t esp_foc_pid_soft_init(esp_foc_pid_t *p, float kp, float ki, float kd, float n_hz, float ts);
void esp_foc_pid_soft_reset(esp_foc_pid_t *p);
q16_t esp_foc_pid_soft_update(esp_foc_pid_t *p, q16_t sp, q16_t meas);
void esp_foc_pid_soft_set_applied(esp_foc_pid_t *p, q16_t u_applied);
void esp_foc_pid_soft_set_ff(esp_foc_pid_t *p, q16_t ff);
void esp_foc_pid_soft_set_bypass(esp_foc_pid_t *p, bool on);
esp_err_t esp_foc_pid_soft_set_kp(esp_foc_pid_t *p, float kp);
esp_err_t esp_foc_pid_soft_set_ki(esp_foc_pid_t *p, float ki);
esp_err_t esp_foc_pid_soft_set_kd(esp_foc_pid_t *p, float kd);
