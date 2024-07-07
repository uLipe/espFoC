/*
 * MIT License
 *
 * Copyright (c) 2024 Felipe Neves
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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#define MOTOR_CMD_TORQUE_MASK       0x00
#define MOTOR_CMD_SPEED_MASK        0x02
#define MOTOR_CMD_POSITION_MASK     0x04
#define MOTOR_CMD_SHUTDOWN_MASK     0x08
#define MOTOR_CMD_RESET_MASK        0x10
#define MOTOR_CMD_KP_MASK           0x20
#define MOTOR_CMD_KI_MASK           0x40
#define MOTOR_CMD_KD_MASK           0x80
#define MOTOR_CMD_POS_PID_MASK      0x100
#define MOTOR_CMD_SPD_PID_MASK      0x200
#define MOTOR_CMD_SET_POLE_PAIRS    0x400
#define MOTOR_CMD_MAITENANCE_MODE   0x8000
#define MOTOR_CMD_MAITENANCE_PWMS   0x10000
#define MOTOR_CMD_GLOBAL_ENABLE     0x20000
#define MOTOR_CMD_GLOBAL_DISABLE    0x40000
#define MOTOR_CMD_TRIGGER           0x80000

struct motor_state {
    uint32_t timestamp_us;
    int32_t position_mdeg;
    int32_t speed_mdps;
    int32_t qvoltage_mvolts;
    int32_t dvoltage_mvolts;
    uint32_t encoder;
    uint8_t power_state;
    uint8_t last_cmd_result;
    uint8_t motor_number;
    uint8_t direction;
    uint8_t system_enabled;
    uint32_t motor_system_uptime;
};

struct motor_command {
    uint32_t command_mask;
    int32_t speed_mdps;
    int32_t position_mdeg;
    int32_t qvoltage_mvolts;
    int32_t dvoltage_mvolts;
    uint32_t pid_gains[3];
    uint32_t pwms[3];
    uint8_t power_state;
    uint8_t motor_number;
    uint8_t direction;
    uint8_t pole_pairs;
};

struct motor_driver_info {
    uint8_t motor_channels_0[4];
    uint8_t motor_channels_1[4];
    float encoder_reading_0;
    float encoder_reading_1;
};

typedef void (*esp_foc_ipc_motor_state_callback_t) (const struct motor_state *state);
typedef void (*esp_foc_ipc_motor_cmd_callback_t) (const struct motor_command *cmd);

struct motor_report_callback {
    esp_foc_ipc_motor_state_callback_t state_cb;
    esp_foc_ipc_motor_cmd_callback_t cmd_cb;
    sys_dnode_t node;
};

#define ESP_FOC_DEFINE_IPC_CALLBACK(name,cmd_callback, state_callback) \
struct motor_report_callback name = {       \
    .state_cb = state_callback,             \
    .cmd_cb = cmd_callback,                 \
}

int esp_foc_ipc_init(struct motor_driver_info *mc_info);
int esp_foc_ipc_send_command(const struct motor_command *cmd);
int esp_foc_ipc_send_state(const struct motor_state *state);
int esp_foc_ipc_register_callback(const struct motor_report_callback *cb);
int esp_foc_ipc_remove_callback(const struct motor_report_callback *cb);
int esp_foc_ipc_encoder_put(float encoder_0, float encoder_1);
struct motor_driver_info* esp_foc_ipc_get_info(void);