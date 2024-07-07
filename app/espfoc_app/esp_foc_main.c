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

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <espFoC/motor_control/esp_foc.h>
#include <espFoC/ipc/esp_foc_ipc.h>
#include <zephyr/drivers/counter.h>

static struct motor_state motor_fw_state;
K_SEM_DEFINE(sync,0,1);

static void esp_foc_motor_on_state (const struct motor_state *state)
{
    memcpy(&motor_fw_state, state, sizeof(motor_fw_state));
    k_sem_give(&sync);
}
ESP_FOC_DEFINE_IPC_CALLBACK(state_callback, NULL, esp_foc_motor_on_state);

int main (void)
{
    /**TODO use the main application to implement espfoc commander
     * Through UART, SPI, I2C and CAN, last one is the priority
     */
    printk("espFoC is running in the %s\n", CONFIG_BOARD);
    esp_foc_ipc_register_callback(&state_callback);

    while(1) {
        k_sem_take(&sync, K_FOREVER);
        printk("espFoc Motor firmware uptime: %u [ms] \n", motor_fw_state.motor_system_uptime);
        printk("espFoc Motor firmware timestep: %u [us] \n", motor_fw_state.timestamp_us);
        printk("espFoC Motor system status: %d \n", motor_fw_state.system_enabled);
    }
}

__weak void esp_foc_user_entry(void)
{
    printk("Weak user entry of the espFoC \n");
}
K_THREAD_DEFINE(foc_tid, 4096, esp_foc_user_entry, NULL, NULL, NULL, 1, 0, 0);