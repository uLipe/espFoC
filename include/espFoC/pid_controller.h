#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator_limit;
    float accumulated_error;
    float previous_error;
}esp_foc_pid_controller_t;

static inline void esp_foc_pid_reset(esp_foc_pid_controller_t *self)
{
    self->accumulated_error = 0.0f;
    self->previous_error = 0.0f;
}

static inline float  esp_foc_pid_update(esp_foc_pid_controller_t *self,
                                        float reference,
                                        float measure,
                                        float dt)
{
    float error = reference - measure;
    float error_diff = error - self->previous_error;
    self->accumulated_error += error;

    if(self->accumulated_error > self->integrator_limit) {
        self->accumulated_error = self->integrator_limit;
    } else if (self->accumulated_error < -self->integrator_limit) {
        self->accumulated_error = -self->integrator_limit;
    }

    float mv = self->kp * error + 
            (self->ki * dt * self->accumulated_error) + 
            ((self->kd * error_diff) / dt);

    self->previous_error = error;

    return mv;
}