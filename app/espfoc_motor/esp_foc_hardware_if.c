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

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <espFoC/motor_control/esp_foc.h>
#include "esp_foc_hardware_if.h"

#define PWM_PERIOD_NSEC                 31250
#define DC_BUS_VOLTAGE                  24.0f

struct z_esp_foc_motor_if {
    float accumulated_angle_deg;
    float current_rotor_deg;
    float prev_rotor_deg;
    float current_rotor_speed_dps;
    float pole_pairs;
    struct esp_foc_motor_interface interface;
};

static int reset(struct esp_foc_motor_interface *self)
{
    int ret;

    ret = esp_foc_motor_enable(self);
    if (ret < 0) {
        return ret;
    }

    return esp_foc_motor_align_rotor(self);
}

static int enable(struct esp_foc_motor_interface *self)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    int ret = esp_foc_motor_set_duty_cycles(self, 0.0f, 0.0f, 0.0f);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int disable(struct esp_foc_motor_interface *self)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);


    return 0;
}

static int align_rotor(struct esp_foc_motor_interface *self)
{
    int ret;

    ret = esp_foc_motor_set_duty_cycles(self, 0.0f, 0.0f, 0.0f);
    if (ret < 0) {
        return ret;
    }

    k_msleep(500);

    ret = esp_foc_motor_set_duty_cycles(self, 0.2f, 0.0f, 0.0f);
    if (ret < 0) {
        return ret;
    }

    k_msleep(500);

    return esp_foc_motor_fetch_sensors(self);
}

static int set_duty_cycles(struct esp_foc_motor_interface *self, float dc_a, float dc_b, float dc_c)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    if(dc_a > 1.0f) {
		dc_a = 1.0f;
	} else if (dc_a < 0.0f) {
		dc_a = 0.0f;
	}

	if(dc_b > 1.0f) {
		dc_b = 1.0f;
	} else if (dc_b < 0.0f) {
		dc_b = 0.0f;
	}

	if(dc_c > 1.0f) {
		dc_c = 1.0f;
	} else if (dc_c < 0.0f) {
		dc_c = 0.0f;
	}

    dc_a *= PWM_PERIOD_NSEC - 1;
    dc_b *= PWM_PERIOD_NSEC - 1;
    dc_c *= PWM_PERIOD_NSEC - 1;

    return 0;
}

static int get_rotor_angle(struct esp_foc_motor_interface *self, float *angle)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    if(!angle)
        return -EINVAL;

    *angle = itf->current_rotor_deg;

    return 0;
}
static int get_electrical_angle(struct esp_foc_motor_interface *self, float *angle)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    if(!angle)
        return -EINVAL;

    *angle = itf->current_rotor_deg * itf->pole_pairs;

    return 0;
}

static int get_rotor_speed_dps(struct esp_foc_motor_interface *self, float *speed_dps)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    if(!speed_dps)
        return -EINVAL;

    *speed_dps = itf->current_rotor_speed_dps;

    return 0;
}

static int get_acumulated_angle(struct esp_foc_motor_interface *self, float *angle)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    if(!angle)
        return -EINVAL;

    *angle = itf->accumulated_angle_deg;

    return 0;
}

static int get_dc_bus_voltage(struct esp_foc_motor_interface *self, float *vdc)
{
    if(!vdc)
        return -EINVAL;

    *vdc = DC_BUS_VOLTAGE;

    return 0;
}

static int fetch_sensors(struct esp_foc_motor_interface *self)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);
    int ret;

    itf->current_rotor_speed_dps = (itf->current_rotor_deg - itf->prev_rotor_deg);
    itf->accumulated_angle_deg += itf->current_rotor_speed_dps;

    return ret;
}

static int set_pole_pairs(struct esp_foc_motor_interface *self, int32_t pole_pairs)
{
    struct z_esp_foc_motor_if *itf = CONTAINER_OF(self, struct z_esp_foc_motor_if, interface);

    itf->pole_pairs = (float)pole_pairs;

    return 0;
}

static struct z_esp_foc_motor_if z_esp_hw_if[Z_ESP_FOC_MAX] = {
    {
        .current_rotor_deg = 0.0f,
        .current_rotor_speed_dps = 0.0f,
        .accumulated_angle_deg = 0.0f,
        .pole_pairs = 2.0f,
        .interface.reset = reset,
        .interface.enable = enable,
        .interface.disable = disable,
        .interface.set_duty_cycles = set_duty_cycles,
        .interface.get_rotor_angle = get_rotor_angle,
        .interface.get_electrical_angle = get_electrical_angle,
        .interface.get_rotor_speed_dps = get_rotor_speed_dps,
        .interface.align_rotor = align_rotor,
        .interface.get_acumulated_angle = get_acumulated_angle,
        .interface.get_encoder_ppr = NULL,
        .interface.get_dc_bus_voltage = get_dc_bus_voltage,
        .interface.get_currents = NULL,
        .interface.number_of_shunts = NULL,
        .interface.fetch_sensors = fetch_sensors,
        .interface.set_pole_pairs = set_pole_pairs,
    },
    {
        .current_rotor_deg = 0.0f,
        .current_rotor_speed_dps = 0.0f,
        .accumulated_angle_deg = 0.0f,
        .pole_pairs = 2.0f,
        .interface.reset = reset,
        .interface.enable = enable,
        .interface.disable = disable,
        .interface.set_duty_cycles = set_duty_cycles,
        .interface.get_rotor_angle = get_rotor_angle,
        .interface.get_electrical_angle = get_electrical_angle,
        .interface.get_rotor_speed_dps = get_rotor_speed_dps,
        .interface.align_rotor = align_rotor,
        .interface.get_acumulated_angle = get_acumulated_angle,
        .interface.get_encoder_ppr = NULL,
        .interface.get_dc_bus_voltage = get_dc_bus_voltage,
        .interface.get_currents = NULL,
        .interface.number_of_shunts = NULL,
        .interface.fetch_sensors = fetch_sensors,
        .interface.set_pole_pairs = set_pole_pairs,
    }
};

struct esp_foc_motor_interface* z_esp_foc_get_interface(enum zephyr_esp_foc_instances instance)
{
    if(instance >= Z_ESP_FOC_MAX)
        return NULL;

    return &z_esp_hw_if[instance].interface;
}

int z_esp_foc_set_feedback(enum zephyr_esp_foc_instances instance, float value)
{
    if(instance >= Z_ESP_FOC_MAX)
        return NULL;

    z_esp_hw_if[instance].prev_rotor_deg = z_esp_hw_if[instance].current_rotor_deg;
    z_esp_hw_if[instance].current_rotor_deg = value;

    return 0;
}