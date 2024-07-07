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

#include <zephyr/kernel.h>
#include <espFoC/motor_control/esp_foc.h>
#include <espFoC/ipc/esp_foc_ipc.h>
#include <zephyr/drivers/counter.h>
#include "esp_foc_hardware_if.h"

static struct esp_foc_motor_control axis;
static struct motor_state outgoing_state;
static struct esp_foc_pid pid_vel_1;
static struct esp_foc_pid pid_pos_1;
static uint32_t timestamp_us = 0;
static bool global_enable = false;

K_SEM_DEFINE(sync, 0, 1);

static void esp_foc_motor_on_cmd (const struct motor_command *cmd)
{
    float position_deg = cmd->position_mdeg * 0.001f;
    float speed_dps = cmd->speed_mdps * 0.001f;
    float pid_gains[3] = {
        (float)cmd->pid_gains[0] * 1e-6f,
        (float)cmd->pid_gains[1] * 1e-6f,
        (float)cmd->pid_gains[2] * 1e-6f,
    };
    float pwms[3] = {
        (float)cmd->pwms[0] * 1e-6f,
        (float)cmd->pwms[1] * 1e-6f,
        (float)cmd->pwms[2] * 1e-6f,
    };

    int pole_pairs = (int)cmd->pole_pairs;

    if(cmd->command_mask & MOTOR_CMD_TRIGGER) {
        if(global_enable) {
            timestamp_us += 500;
            esp_foc_controller_run(&axis);
        }
        return;
    }

    if(cmd->command_mask & MOTOR_CMD_GLOBAL_ENABLE) {
        global_enable = true;
    }

    if(cmd->command_mask & MOTOR_CMD_GLOBAL_DISABLE) {
        global_enable = false;
    }

    if(cmd->command_mask & MOTOR_CMD_SPEED_MASK) {
        esp_foc_controller_set_speed(&axis, speed_dps);
    }

    if(cmd->command_mask & MOTOR_CMD_POSITION_MASK) {
        esp_foc_controller_set_position(&axis, position_deg);
    }

    if(cmd->command_mask & MOTOR_CMD_SHUTDOWN_MASK) {
        if(cmd->power_state) {
            esp_foc_enable_axis(&axis);
        } else {
            esp_foc_disable_axis(&axis);
        }
    }

    if(cmd->command_mask & MOTOR_CMD_RESET_MASK) {
        esp_foc_motor_reset(z_esp_foc_get_interface(Z_ESP_FOC_1));
    }

    if(cmd->command_mask & MOTOR_CMD_SPD_PID_MASK) {
        if(cmd->command_mask & MOTOR_CMD_KP_MASK) {
            esp_foc_pid_set_kp(&pid_vel_1, pid_gains[0]);
        }

        if(cmd->command_mask & MOTOR_CMD_KI_MASK) {
            esp_foc_pid_set_ki(&pid_vel_1, pid_gains[1]);
        }

        if(cmd->command_mask & MOTOR_CMD_KD_MASK) {
            esp_foc_pid_set_kd(&pid_vel_1, pid_gains[2]);
        }
    }

    if(cmd->command_mask & MOTOR_CMD_POS_PID_MASK) {
        if(cmd->command_mask & MOTOR_CMD_KP_MASK) {
            esp_foc_pid_set_kp(&pid_pos_1, pid_gains[0]);
        }

        if(cmd->command_mask & MOTOR_CMD_KI_MASK) {
            esp_foc_pid_set_ki(&pid_pos_1, pid_gains[1]);
        }

        if(cmd->command_mask & MOTOR_CMD_KD_MASK) {
            esp_foc_pid_set_kd(&pid_pos_1, pid_gains[2]);
        }
    }

    if(cmd->command_mask & MOTOR_CMD_SET_POLE_PAIRS) {
        esp_foc_motor_set_pole_pairs(z_esp_foc_get_interface(Z_ESP_FOC_1), pole_pairs);
    }

    if(cmd->command_mask & MOTOR_CMD_MAITENANCE_MODE) {
        if(cmd->command_mask & MOTOR_CMD_MAITENANCE_PWMS) {
            esp_foc_motor_set_duty_cycles(z_esp_foc_get_interface(Z_ESP_FOC_1), pwms[0], pwms[1], pwms[2]);
        }
    }
    k_sem_give(&sync);
}
ESP_FOC_DEFINE_IPC_CALLBACK(cmd_callback,esp_foc_motor_on_cmd, NULL);

static int esp_foc_early_init(void)
{
    float sample_time_seconds = (float)ESP_FOC_INNER_CONTROL_US_PERIOD * 0.000001f;
    esp_foc_pid_init(&pid_vel_1,
                    sample_time_seconds * ESP_FOC_SPEED_CONTROL_RATIO,
                    10000.0f);

    esp_foc_pid_init(&pid_pos_1,
                    sample_time_seconds * ESP_FOC_POSITION_CONTROL_RATIO,
                    10000.0f);

    esp_foc_init_controller(&axis, z_esp_foc_get_interface(Z_ESP_FOC_1));
    esp_foc_add_position_control(&axis, &pid_pos_1);
    esp_foc_add_speed_control(&axis, &pid_vel_1);

    esp_foc_ipc_init(NULL);
    esp_foc_ipc_register_callback(&cmd_callback);

    return 0;
}
SYS_INIT(esp_foc_early_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

int main (void)
{
    while(1) {
        k_sem_take(&sync, K_FOREVER);
        outgoing_state.timestamp_us = timestamp_us;
        outgoing_state.position_mdeg = (int32_t)(axis.current_position_degrees * 1000.0f);
        outgoing_state.speed_mdps = (int32_t)(axis.current_speed_dps * 1000.0f);
        outgoing_state.power_state = (uint8_t)axis.is_enabled;
        outgoing_state.system_enabled = global_enable;
        outgoing_state.motor_system_uptime = k_uptime_get_32();
        esp_foc_ipc_send_state(&outgoing_state);
    }
}
