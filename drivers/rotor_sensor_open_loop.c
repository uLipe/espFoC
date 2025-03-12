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
#include "espFoC/rotor_sensor_dummy.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_log.h"

#define SIMUL_PULSES_PER_REVOLUTION 4096.0f
#define SIMUL_FLUX_LINKAGE 0.015f // Lambda_m (Weber)
#define SIMUL_INERTIA 0.00024f // J (kg⋅m²)
#define SIMUL_FRICTION 0.0001f // B (Viscous friction)
#define DRIFT_COMP_CYCLE 200000
#define SIMUL_MAX_IQ 5.0f
#define SIMUL_MINIMUM_DIQ 1e-6f

const char *tag = "ROTOR_SENSOR_SIMUL";

typedef struct {
    int comp_cycle;
    float *uq_wire;
    float *dt_wire;
    float motor_resistance;
    float motor_inductance;
    float accumulated;
    float raw;
    float angle;
    float previous;
    float omega;
    float estim_iq;
    bool first_read;
    esp_foc_rotor_sensor_t interface;
}esp_foc_rotor_sensor_simul_t;

DRAM_ATTR static esp_foc_rotor_sensor_simul_t rotor_sensors[CONFIG_NOOF_AXIS];

IRAM_ATTR float read_accumulated_counts (esp_foc_rotor_sensor_t *self)
{
    return 0.0f;
}

IRAM_ATTR  static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_sensor_simul_t *obj =
        __containerof(self,esp_foc_rotor_sensor_simul_t, interface);

    obj->raw = 0;
    obj->angle = 0.0f;
}

IRAM_ATTR static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return 4096.0f;
}

IRAM_ATTR static float read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_rotor_sensor_simul_t *obj =
        __containerof(self,esp_foc_rotor_sensor_simul_t, interface);

    float angle_now = (obj->angle) * (SIMUL_PULSES_PER_REVOLUTION / (2.0f * M_PI));
    if(angle_now > SIMUL_PULSES_PER_REVOLUTION) {
        angle_now = SIMUL_PULSES_PER_REVOLUTION;
        obj->angle = 0.0f;
    } else if (angle_now < 0) {
        angle_now = 0.0f;
        obj->angle = 0.0f;
    }

    float d_iq = ((*obj->uq_wire - obj->motor_resistance * obj->estim_iq - SIMUL_FLUX_LINKAGE * obj->omega) / obj->motor_inductance);

    if(isnan(d_iq)) {
        d_iq = 0.0f;
    }

    if(fabsf(d_iq) < SIMUL_MINIMUM_DIQ) {
        d_iq = 0.0f;
    }

    if(d_iq > SIMUL_MAX_IQ) {
        d_iq = SIMUL_MAX_IQ;
    } else if (d_iq < -SIMUL_MAX_IQ) {
        d_iq = -SIMUL_MAX_IQ;
    }

    //Compurte next angle upon calling this function;
    obj->estim_iq +=  d_iq * (*obj->dt_wire);

    if(isnan(obj->estim_iq)) {
        obj->estim_iq = 0.0f;
    }

    if(obj->estim_iq > SIMUL_MAX_IQ) {
        obj->estim_iq = SIMUL_MAX_IQ;
    } else if (obj->estim_iq < -SIMUL_MAX_IQ) {
        obj->estim_iq = -SIMUL_MAX_IQ;
    }

    // Estimate rotor acceleration and update speed
    float torque = SIMUL_FLUX_LINKAGE * obj->estim_iq;
    float acceleration = (torque - SIMUL_FRICTION * obj->omega) / SIMUL_INERTIA;
    obj->omega += acceleration * (*obj->dt_wire);

    if(isnan(obj->omega)) {
        obj->omega = 0.0f;
    }

    // Update electrical rotor angle
    obj->angle += obj->omega * (*obj->dt_wire);

    if(isnan(obj->angle)) {
        obj->angle = 0.0f;
    }

    if (obj->angle > (2.0f * M_PI)) {
        obj->angle -= (2.0f * M_PI);
    } else if (obj->angle < 0.0f) {
        obj->angle += (2.0f * M_PI);
    }

    return(angle_now);
}

esp_foc_rotor_sensor_t *rotor_sensor_open_loop_new(float motor_resistance,
                                                float motor_inductance,
                                                float *uq_wire,
                                                float *dt_wire)
{

    if(uq_wire == NULL) {
        return NULL;
    }

    rotor_sensors[0].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[0].interface.read_counts = read_counts;
    rotor_sensors[0].interface.set_to_zero = set_to_zero;
    rotor_sensors[0].interface.read_accumulated_counts = read_accumulated_counts;
    rotor_sensors[0].raw = 0;
    rotor_sensors[0].accumulated = 0;
    rotor_sensors[0].uq_wire = uq_wire;
    rotor_sensors[0].dt_wire = dt_wire;
    rotor_sensors[0].motor_inductance = motor_inductance;
    rotor_sensors[0].motor_resistance = motor_resistance;
    rotor_sensors[0].omega = 0.0f;
    rotor_sensors[0].angle = 0.0f;
    rotor_sensors[0].estim_iq = 0.0f;
    rotor_sensors[0].comp_cycle = 0;
    rotor_sensors[0].first_read = false;

    return &rotor_sensors[0].interface;
}