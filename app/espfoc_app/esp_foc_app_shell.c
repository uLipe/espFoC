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
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <espFoC/ipc/esp_foc_ipc.h>

static int cmd_set_position(const struct shell *shell, size_t argc, char **argv)
{
    struct motor_command command;

	if (argc != 2) {
		return -EINVAL;
	}

	command.position_mdeg = (int32_t)(strtof(argv[1], NULL) * 1000.0f);
	command.command_mask = MOTOR_CMD_POSITION_MASK;

	return esp_foc_ipc_send_command(&command);
}

static int cmd_set_speed(const struct shell *shell, size_t argc, char **argv)
{
	struct motor_command command;

	if (argc != 2) {
		return -EINVAL;
	}

	command.speed_mdps = (int32_t)(strtof(argv[1], NULL) * 1000.0f);
	command.command_mask = MOTOR_CMD_SPEED_MASK;

	return esp_foc_ipc_send_command(&command);
}

static int cmd_enable(const struct shell *shell, size_t argc, char **argv)
{
	struct motor_command command;

	if (argc != 1) {
		return -EINVAL;
	}

	command.command_mask = MOTOR_CMD_SHUTDOWN_MASK | MOTOR_CMD_GLOBAL_ENABLE;
	command.power_state = 1;

	return esp_foc_ipc_send_command(&command);
}

static int cmd_disable(const struct shell *shell, size_t argc, char **argv)
{
	struct motor_command command;

	if (argc != 1) {
		return -EINVAL;
	}

	command.command_mask = MOTOR_CMD_SHUTDOWN_MASK | MOTOR_CMD_GLOBAL_DISABLE;
	command.power_state = 0;

	return esp_foc_ipc_send_command(&command);
}

static int cmd_set_gains_pid_vel(const struct shell *shell, size_t argc, char **argv)
{
    struct motor_command command;

	if (argc != 4) {
		return -EINVAL;
	}

	command.command_mask = MOTOR_CMD_SPD_PID_MASK;
	command.pid_gains[0] = (int32_t)(strtof(argv[1], NULL) * 1e+6f);
	command.pid_gains[1] = (int32_t)(strtof(argv[2], NULL) * 1e+6f);
	command.pid_gains[2] = (int32_t)(strtof(argv[3], NULL) * 1e+6f);

	return esp_foc_ipc_send_command(&command);

}

static int cmd_set_gains_pid_pos(const struct shell *shell, size_t argc, char **argv)
{
    struct motor_command command;

	if (argc != 4) {
		return -EINVAL;
	}

	command.command_mask = MOTOR_CMD_POS_PID_MASK;
	command.pid_gains[0] = (int32_t)(strtof(argv[1], NULL) * 1e+6f);
	command.pid_gains[1] = (int32_t)(strtof(argv[2], NULL) * 1e+6f);
	command.pid_gains[2] = (int32_t)(strtof(argv[3], NULL) * 1e+6f);

	return esp_foc_ipc_send_command(&command);
}

static int cmd_set_maintenance_pwm(const struct shell *shell, size_t argc, char **argv)
{
    struct motor_command command;

	if (argc != 4) {
		return -EINVAL;
	}

	command.command_mask = (MOTOR_CMD_MAITENANCE_MODE | MOTOR_CMD_MAITENANCE_PWMS);
	command.pwms[0] = (int32_t)(strtof(argv[1], NULL) * 1e+6f);
	command.pwms[1] = (int32_t)(strtof(argv[2], NULL) * 1e+6f);
	command.pwms[2] = (int32_t)(strtof(argv[3], NULL) * 1e+6f);

	return esp_foc_ipc_send_command(&command);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	esp_foc,
	SHELL_CMD(set_position, NULL, "sets position in degrees", cmd_set_position),
	SHELL_CMD(set_speed, NULL, "sets speed in degrees per second", cmd_set_speed),
	SHELL_CMD(enable, NULL, "enable motor driver", cmd_enable),
	SHELL_CMD(disable, NULL, "disable motor driver", cmd_disable),
	SHELL_CMD(set_gains_pid_vel, NULL, "set pids velocity", cmd_set_gains_pid_vel),
	SHELL_CMD(set_gains_pid_pos, NULL, "set pids speed", cmd_set_gains_pid_pos),
	SHELL_CMD(set_maintenance_pwm, NULL, "set pwms maitenance mode", cmd_set_maintenance_pwm),
	SHELL_SUBCMD_SET_END
	);

SHELL_CMD_REGISTER(esp_foc, &esp_foc, "espFoC Basic shell commands", NULL);
