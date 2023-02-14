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

#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0E
#define AS5600_PULSES_PER_REVOLUTION 4096.0f
#define AS5600_READING_MASK 0xFFF 

typedef struct {
    float accumulated;
    float raw;
    float previous;
    esp_foc_rotor_sensor_t interface;
}esp_foc_dummy_t;

DRAM_ATTR static esp_foc_dummy_t rotor_sensors[CONFIG_NOOF_AXIS];

IRAM_ATTR static float read_accumulated_counts (esp_foc_rotor_sensor_t *self)
{
    esp_foc_dummy_t *obj =
        __containerof(self,esp_foc_dummy_t, interface);

    return obj->accumulated + obj->previous;
}

IRAM_ATTR  static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_dummy_t *obj =
        __containerof(self,esp_foc_dummy_t, interface);

    obj->raw = 0;
}

IRAM_ATTR static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return 4096.0f;
}

IRAM_ATTR static void fetch_sensor(esp_foc_rotor_sensor_t *self)
{
    esp_foc_wait_notifier();
}

IRAM_ATTR static float read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_dummy_t *obj =
        __containerof(self,esp_foc_dummy_t, interface);

 
    esp_foc_critical_enter();

    obj->raw += 40.96f;

    if(obj->raw > 4096.0f) {
        obj->raw -= 4096.0f;
    } else if (obj->raw < 0.0f) {
        obj->raw += 4096.0f;
    }

    float delta = (float)obj->raw - obj->previous;

    if(fabs(delta) >= 3600.0f) {
        obj->accumulated = (delta < 0.0f) ? 
            obj->accumulated + 4096.0f :
                obj->accumulated - 4096.0f;
    }

    obj->previous = (float)obj->raw;

    esp_foc_critical_leave();

    return(obj->raw);
}

esp_foc_rotor_sensor_t *rotor_sensor_dummy_new(int port)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.fetch_sensor = fetch_sensor;
    rotor_sensors[port].interface.read_accumulated_counts = read_accumulated_counts;
    rotor_sensors[port].raw = 0;
    rotor_sensors[port].accumulated = 0;

    return &rotor_sensors[port].interface;
}