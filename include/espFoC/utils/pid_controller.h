/*
 * MIT License
 */
/**
 * @file pid_controller.h
 * @brief PID controller: Q16.16 I/O and coefficients; int64 integrator.
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/utils/foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t kd;

    int64_t integrator;
    q16_t prev_error;
    q16_t integrator_limit;
    q16_t max_output_value;
    q16_t min_output_value;
    q16_t dt;
    q16_t inv_dt;

} esp_foc_pid_controller_t;

static inline void esp_foc_pid_reset(esp_foc_pid_controller_t *self)
{
    self->integrator = 0;
    self->prev_error = 0;
}

static inline q16_t esp_foc_pid_update(esp_foc_pid_controller_t *self,
                                       q16_t reference,
                                       q16_t measure)
{
    q16_t err = q16_sub(reference, measure);
    int64_t p = ((int64_t)self->kp * (int64_t)err) >> 16;
    int64_t derr = (int64_t)err - (int64_t)self->prev_error;
    int64_t d = ((int64_t)self->kd * derr) >> 16;
    d = (d * (int64_t)self->inv_dt) >> 16;

    int64_t mv = p + self->integrator + d;
    int64_t omin = (int64_t)self->min_output_value;
    int64_t omax = (int64_t)self->max_output_value;
    if (mv > omax) {
        mv = omax;
    }
    if (mv < omin) {
        mv = omin;
    }

    self->integrator += (((int64_t)self->ki * (int64_t)err * (int64_t)self->dt) >> 32);
    {
        int64_t lim = (int64_t)self->integrator_limit;
        if (self->integrator > lim) {
            self->integrator = lim;
        }
        if (self->integrator < -lim) {
            self->integrator = -lim;
        }
    }

    self->prev_error = err;
    return (q16_t)mv;
}

static inline float esp_foc_pid_update_float(esp_foc_pid_controller_t *self,
                                             float reference,
                                             float measure)
{
    return q16_to_float(esp_foc_pid_update(self,
                                             q16_from_float(reference),
                                             q16_from_float(measure)));
}

static inline void esp_foc_pid_init_from_float(esp_foc_pid_controller_t *self,
                                               float kp, float ki, float kd,
                                               float dt, float out_min, float out_max,
                                               float int_lim)
{
    self->kp = q16_from_float(kp);
    self->ki = q16_from_float(ki);
    self->kd = q16_from_float(kd);
    self->dt = q16_from_float(dt);
    self->inv_dt = (dt > 1e-20f) ? q16_from_float(1.0f / dt) : 0;
    self->min_output_value = q16_from_float(out_min);
    self->max_output_value = q16_from_float(out_max);
    self->integrator_limit = q16_from_float(int_lim);
    esp_foc_pid_reset(self);
}

#ifdef __cplusplus
}
#endif
