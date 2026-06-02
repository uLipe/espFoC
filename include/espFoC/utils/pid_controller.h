/*
 * MIT License
 */
/**
 * @file pid_controller.h
 * @brief Current-loop style PID + Q16 reference feedforward; int64 state.
 *
 * Output: sat( (kff*ref)>>16 + P + I + D ) with P/I/D from error
 * err = ref - (ke*measure)>>16. Integrator freezes when the unsaturated sum
 * would exceed output limits and err still pushes in that direction.
 */
#pragma once

#include "espFoC/utils/esp_foc_q16.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t kd;
    q16_t kff;

    int64_t integrator;
    q16_t prev_error;
    q16_t integrator_limit;
    q16_t max_output_value;
    q16_t min_output_value;
    q16_t ke;
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
    q16_t meas_scaled = q16_mul(measure, self->ke);
    q16_t err = q16_sub(reference, meas_scaled);

    int64_t ff = ((int64_t)self->kff * (int64_t)reference) >> 16;
    int64_t p = ((int64_t)self->kp * (int64_t)err) >> 16;
    int64_t derr = (int64_t)err - (int64_t)self->prev_error;
    int64_t d = ((int64_t)self->kd * derr) >> 16;
    d = (d * (int64_t)self->inv_dt) >> 16;

    int64_t i_term = self->integrator;
    int64_t unsat = ff + p + i_term + d;

    int64_t omin = (int64_t)self->min_output_value;
    int64_t omax = (int64_t)self->max_output_value;
    int64_t mv = unsat;
    if (mv > omax) {
        mv = omax;
    }
    if (mv < omin) {
        mv = omin;
    }

    const int freeze_up = (unsat > omax) && (err > 0);
    const int freeze_dn = (unsat < omin) && (err < 0);
    if (!freeze_up && !freeze_dn) {
        self->integrator +=
            (((int64_t)self->ki * (int64_t)err * (int64_t)self->dt) >> 32);
        {
            int64_t lim = (int64_t)self->integrator_limit;
            if (self->integrator > lim) {
                self->integrator = lim;
            }
            if (self->integrator < -lim) {
                self->integrator = -lim;
            }
        }
    }

    self->prev_error = err;
    return (q16_t)mv;
}

static inline void esp_foc_pid_init_from_float(esp_foc_pid_controller_t *self,
                                               float kp,
                                               float ki,
                                               float kd,
                                               float kff,
                                               float ke,
                                               float dt,
                                               float out_min,
                                               float out_max,
                                               float int_lim)
{
    self->kp = q16_from_float(kp);
    self->ki = q16_from_float(ki);
    self->kd = q16_from_float(kd);
    self->kff = q16_from_float(kff);
    self->ke = q16_from_float(ke);
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
