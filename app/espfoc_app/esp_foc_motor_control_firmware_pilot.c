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

static const struct device *timer_dev = DEVICE_DT_GET( DT_NODELABEL(timer3));
static struct counter_alarm_cfg alarm_cfg;

const struct motor_driver_info mc_info = {
    .motor_channels_0 = {
        0,1,2,0
    },
    .motor_channels_1 = {
        0,1,2,0
    },
};

static void espfoc_pilot_motor_firmware(struct k_work *w)
{
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);
    struct motor_command command;
	command.command_mask = MOTOR_CMD_TRIGGER;
	esp_foc_ipc_send_command(&command);
}
K_WORK_DEFINE(motor_cfw_work, espfoc_pilot_motor_firmware);

static void espfoc_timer_isr(const struct device *counter_dev,
                            uint8_t chan_id, uint32_t ticks,
                            void *user_data)
{
    k_work_submit(&motor_cfw_work);
}

int esp_foc_early_init(void)
{
    esp_foc_ipc_init((struct motor_driver_info *)&mc_info);
    counter_start(timer_dev);
    alarm_cfg.flags = 0;
    alarm_cfg.ticks = counter_us_to_ticks(timer_dev, ESP_FOC_INNER_CONTROL_US_PERIOD);
    alarm_cfg.callback = espfoc_timer_isr;
    alarm_cfg.user_data = &alarm_cfg;
    counter_set_channel_alarm(timer_dev, 0, &alarm_cfg);
    k_msleep(50);

    return 0;
}
SYS_INIT(esp_foc_early_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

void esp_foc_fetch_encoder_loop(void)
{
    // const struct device *angle0_dev = DEVICE_DT_GET(DT_NODELABEL(angle_sensor0));
    // const struct device *angle1_dev = DEVICE_DT_GET(DT_NODELABEL(angle_sensor1));
    // struct sensor_value raw0, raw1;

    float dummy_encoder0 = 0.0f;
    float dummy_encoder1 = 0.0f;

    k_msleep(100);
    printk("[espFoC sensor fetch task has been started!] \n");

    while(1) {
        // sensor_sample_fetch(angle0_dev);
        // sensor_sample_fetch(angle1_dev);
        // sensor_channel_get(angle0_dev, SENSOR_CHAN_ROTATION, &raw0);
        // sensor_channel_get(angle1_dev, SENSOR_CHAN_ROTATION, &raw1);
        //esp_foc_ipc_encoder_put(sensor_value_to_float(&raw0), sensor_value_to_float(&raw1));
        esp_foc_ipc_encoder_put(dummy_encoder0, dummy_encoder1);
        dummy_encoder0 += 0.125f;
        dummy_encoder1 += 0.125f;

        if(dummy_encoder0 >= 360.0f) {
            dummy_encoder0 = 0.0f;
        }

        if(dummy_encoder1 >= 360.0f) {
            dummy_encoder1 = 0.0f;
        }

        k_msleep(200);
    }
}

K_THREAD_DEFINE(foc_feedback, 4096, esp_foc_fetch_encoder_loop, NULL, NULL, NULL, -1, 0, 0);
