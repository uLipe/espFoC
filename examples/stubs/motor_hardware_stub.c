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
#include "motor_hardware_stub.h"

static const float pole_pairs = 12.0f;
static float erotor_angle = 0.0f;
static const float angle_step_per_ms = 0.36f;

static int normalize_angle(float *angle)
{
    if(!angle)
        return -EINVAL;

    float result =  fmod(*angle, (2.0 * 3.141569f));
    if(result > 3.141569f) {
        result -= 3.141569f;
    }

    *angle = result;

    return 0;
}

static int reset(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    return 0;
}

static int enable(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    return 0;
}

static int set_duty_cycles(struct esp_foc_motor_interface *self, float a, float b, float c)
{
    if(!self)
        return -EINVAL;

    (void)a;
    (void)b;
    (void)c;

    return 0;
}

static int get_rotor_angle(struct esp_foc_motor_interface *self, float *angle)
{
    if(!self || !angle)
        return -EINVAL;

    *angle = erotor_angle * pole_pairs;

    return normalize_angle(angle);
}

static int get_rotor_speed_dps(struct esp_foc_motor_interface *self, float *speed_dps)
{
    if(!self || speed_dps)
        return -EINVAL;


    //360 dps -> 1 RPS -> 1 RPM;
    *speed_dps = 360.0f;

    return 0;
}

static int align_rotor(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    return 0;
}

static int fetch_sensors(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    erotor_angle += angle_step_per_ms;
    if(e_angle > 360.0f) {
        erotor_angle -= 360.0f;
    }

    return 0;
}

int motor_hardware_stub_init(struct motor_hardware_stub *stub,
                            struct esp_foc_motor_interface **itf)
{
    if(!stub)
        return -EINVAL;

    stub->self->reset = reset;
    stub->self->enable = enable;
    stub->self->set_duty_cycles = set_duty_cycles;
    stub->self->get_rotor_angle = get_rotor_angle;
    stub->self->get_rotor_speed_dps = get_rotor_speed_dps;
    stub->self->align_rotor = align_rotor;
    stub->self->fetch_sensors = fetch_sensors;

    itf = stub->self;
    return 0;
}