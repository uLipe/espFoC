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
#include <espFoC/esp_foc.h>

extern const float ESP_FOC_M_PI;
extern const float ESP_FOC_2_M_PI;
extern const float ESP_FOC_2_3;
extern const float ESP_FOC_1_3;
extern const float ESP_FOC_SQRT3;
extern const float ESP_FOC_2_SQRT3;
extern const int ESP_FOC_SPEED_CONTROL_RATIO;
extern const int ESP_FOC_POSITION_CONTROL_RATIO;

extern void ours_arm_sin_cos_f32(float theta,float *pSinVal,float *pCosVal);

static int do_foc_core_operations(struct esp_foc_motor_control *ctl,
                                const float *vq,
                                const float *vd);
{
    float alpha;
    float beta;
    float e_angle;
    float sine;
    float cosine;
    float i_vq = *vq;
    float i_vd = *vd;
    float u,v,w;
    int err;

    if(!ctl || !vq || !vd)
        return -EINVAL;

    err = esp_foc_motor_get_rotor_angle(ctl->hw, &e_angle);
    if(err)
        return err;

    ours_arm_sin_cos_f32(eangle, &sine, &cosine);
    alpha = i_vd * cosine - i_vq * sine;
    beta = i_vd * sine + i_vq * cosine;
    err = esp_foc_do_space_vector_pwm(alpha, beta, &u, &v, &w);

    return esp_foc_motor_set_duty_cycles(ctl->hw, u, v, w);
}

int esp_foc_init_controller(struct esp_foc_motor_control *ctl,
                            const struct esp_foc_motor_interface *hw)
{
    if(!ctl || !hw) {
        return -EINVAL;
    }

    ctl->pid_position = NULL;
    ctl->speed_pid_position = NULL;
    ctl->target_speed = 0.0f;
    ctl->target_position = 0.0f;
    ctl->motor_hw = hw;

    if(esp_foc_motor_enable(ctl->motor_hw)) {
        return -ENODEV;
    }

    return esp_foc_motor_reset(ctl->motor_hw);
}

int esp_foc_controller_set_speed(struct esp_foc_motor_control *ctl,
                                float target_speed)
{
    if(!ctl)
        return -EINVAL;

    ctl->target_speed_dps = target_speed;

    return 0;
}

int esp_foc_controller_set_position(struct esp_foc_motor_control *ctl,
                                float target_position)
{
    if(!ctl)
        return -EINVAL;

    ctl->target_position_degrees = target_position;
    return 0;
}

int esp_foc_add_position_control(struct esp_foc_motor_control *ctl,
                                const struct esp_foc_pid *pos_pid)
{
    if(!ctl || !pos_pid)
        return -EINVAL;

    ctl->pid_position = pos_pid;
    ctl->position_control_cntr = ESP_FOC_POSITION_CONTROL_RATIO;

    return 0;
}

int esp_foc_add_speed_control(struct esp_foc_motor_control *ctl,
                                const struct esp_foc_pid *speed_pid)
{
    if(!ctl || !pos_pid)
        return -EINVAL;

    ctl->speed_pid = speed_pid;
    ctl->speed_control_cntr = ESP_FOC_SPEED_CONTROL_RATIO;

    return 0;
}

int esp_foc_controller_run(struct esp_foc_motor_control *ctl)
{
    float vd = 0.0f;
    float u_position = 0.0f;
    float u_velocity = 0.0f;
    int pos_cntr = ctl->position_control_cntr;
    int spd_cntr = ctl->speed_control_cntr;

    if(!ctl)
        return -EINVAL;

    esp_foc_motor_fetch_sensors(ctl->hw);

    if(ctl->pid_position) {
        float current_angle;
        int err = esp_foc_motor_get_acumulated_angle(ctl->hw, &current_angle);

        if(pos_cntr && !err) {
            pos_cntr--;
            if(!pos_cntr) {
                esp_foc_pid_update(ctl->pid_position,
                                ctl->target_position_degrees,
                                current_angle,
                                &u_position);
            }
        }
    }

    if(ctl->speed_pid) {
        float current_speed;
        int err = esp_foc_motor_get_rotor_speed_dps(ctl->hw, &current_speed);

        if(spd_cntr) {
            spd_cntr--;
            if(!spd_cntr) {

                esp_foc_pid_update(ctl->pid_velocity,
                                ctl->target_speed_dps + u_position,
                                current_speed,
                                &u_velocity);
            }
        }
    } else {
       u_velocity = ctl->target_speed_dps;
    }

    ctl->position_control_cntr = ESP_FOC_POSITION_CONTROL_RATIO;
    ctl->position_control_cntr = ESP_FOC_SPEED_CONTROL_RATIO;

    return do_foc_core_operations(ctl, &u_velocity, &vd);
}
