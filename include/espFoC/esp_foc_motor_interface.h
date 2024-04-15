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

#pragma once

struct esp_foc_motor_interface {
    int (*reset)(struct esp_foc_motor_interface *self);
    int (*enable)(struct esp_foc_motor_interface *self);
    int (*set_duty_cycles)(struct esp_foc_motor_interface *self, float a, float b, float c);
    int (*get_rotor_angle)(struct esp_foc_motor_interface *self, float *angle);
    int (*get_rotor_speed_dps)(struct esp_foc_motor_interface *self, float *speed_dps);
    int (*align_rotor)(struct esp_foc_motor_interface *self);
    int (*get_acumulated_angle)(struct esp_foc_motor_interface *self, float *angle);
    int (*get_encoder_ppr)(struct esp_foc_motor_interface *self, float *ppr);
    int (*get_dc_bus_voltage)(struct esp_foc_motor_interface *self, float *vdc);
    int (*get_currents)(struct esp_foc_motor_interface *self, float *ia, float *ib, float *ic);
    int (*number_of_shunts)(struct esp_foc_motor_interface *self, int *shunts);
    int (*fetch_sensors)(struct esp_foc_motor_interface *self);
};

int esp_foc_motor_reset(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    if(!self->reset)
        return -ENOTSUP;

    return self->reset(self);

}

int esp_foc_motor_enable(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    if(!self->enable)
        return -ENOTSUP;

    return self->enable(self);
}

int esp_foc_motor_set_duty_cycles(struct esp_foc_motor_interface *self, float a, float b, float c)
{
    if(!self)
        return -EINVAL;

    if(!self->set_duty_cycles)
        return -ENOTSUP;

    return self->set_duty_cycles(Self, a, b, c);
}

int esp_foc_motor_get_rotor_angle(struct esp_foc_motor_interface *self, float *angle)
{
    if(!self)
        return -EINVAL;

    if(!self->get_rotor_angle)
        return -ENOTSUP;

    return self->get_rotor_angle(self, angle);
}

int esp_foc_motor_get_rotor_speed_dps(struct esp_foc_motor_interface *self, float *speed_dps)
{
    if(!self)
        return -EINVAL;

    if(!self->get_rotor_speed_dps)
        return -ENOTSUP;

    return self->get_rotor_speed_dps(self, speed_dps);
}

int esp_foc_motor_align_rotor(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    if(!self->align_rotor)
        return -ENOTSUP;

    return self->align_rotor(self);
}

int esp_foc_motor_get_acumulated_angle(struct esp_foc_motor_interface *self, float *angle)
{
    if(!self)
        return -EINVAL;

    if(!self->get_acumulated_encoder)
        return -ENOTSUP;

    return self->get_acumulated_angle(self, angle);
}

int esp_foc_motor_get_encoder_ppr(struct esp_foc_motor_interface *self, float *ppr)
{
    if(!self)
        return -EINVAL;

    if(!self->get_encoder_ppr)
        return -ENOTSUP;

    return self->get_encoder_ppr(self, ppr);
}

int esp_foc_motor_get_dc_bus_voltage(struct esp_foc_motor_interface *self, float *vdc)
{
    if(!self)
        return -EINVAL;

    if(!self->get_dc_bus_voltage)
        return -ENOTSUP;

    return self->get_dc_bus_voltage(self, vdc);
}

int esp_foc_motor_get_currents(struct esp_foc_motor_interface *self, float *ia, float *ib, float *ic)
{
    if(!self)
        return -EINVAL;

    if(!self->get_currents)
        return -ENOTSUP;

    return self->get_currents(self, ia, ib, ic);
}

int esp_foc_motor_number_of_shunts(struct esp_foc_motor_interface *self, int *shunts)
{
    if(!self)
        return -EINVAL;

    if(!self->number_of_shunts)
        return -ENOTSUP;

    return self->number_of_shunts(self, shunts);
}

int esp_foc_motor_fetch_sensors(struct esp_foc_motor_interface *self)
{
    if(!self)
        return -EINVAL;

    if(!self->fetch_sensors)
        return -ENOTSUP;

    return self->fetch_sensors(self);
}