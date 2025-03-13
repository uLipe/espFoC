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
#include "espFoC/rotor_sensor_open_loop.h"
#include "espFoC/esp_foc_simu_observer.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_log.h"

const char *tag = "ROTOR_SENSOR_SIMUL";

typedef struct {
    float *uq_wire;
    float *dt_wire;
    esp_foc_observer_t *ol_observer;
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

    obj->ol_observer->reset(obj->ol_observer);
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

    esp_foc_observer_inputs_t in = {
        .u_dq[1] = *obj->uq_wire,
    };

    /* Convert the angle to shaft ticks, assume 12bit sensor */
    float angle_now = (obj->ol_observer->get_angle(obj->ol_observer)) *
                    (4096.0f / (2.0f * M_PI));

    if(angle_now > 4096.0f) {
        angle_now = 4096.0f;
        obj->ol_observer->reset(obj->ol_observer);
    } else if (angle_now < 0) {
        angle_now = 0.0f;
        obj->ol_observer->reset(obj->ol_observer);
    }

    obj->ol_observer->update(obj->ol_observer, &in);

    return(angle_now);
}

esp_foc_rotor_sensor_t *rotor_sensor_open_loop_new(float motor_resistance,
                                                float motor_inductance,
                                                float *uq_wire,
                                                float *dt_wire)
{

    esp_foc_simu_observer_settings_t settings = {
        .phase_resistance = motor_resistance,
        .phase_inductance = motor_inductance,
        .dt = *dt_wire };

    if(uq_wire == NULL) {
        ESP_LOGE(tag, "Invalid pointer to the VQ signal!");
        return NULL;
    }

    rotor_sensors[0].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[0].interface.read_counts = read_counts;
    rotor_sensors[0].interface.set_to_zero = set_to_zero;
    rotor_sensors[0].interface.read_accumulated_counts = read_accumulated_counts;
    rotor_sensors[0].uq_wire = uq_wire;
    rotor_sensors[0].dt_wire = dt_wire;
    rotor_sensors[0].ol_observer = simu_observer_new(0, settings);

    if(rotor_sensors[0].ol_observer == NULL) {
        ESP_LOGE(tag, "Failed to create the observer for this rotor sensor!");
        return NULL;
    }

    return &rotor_sensors[0].interface;
}