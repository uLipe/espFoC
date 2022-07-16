#include <math.h>
#include "espFoC/rotor_sensor_dummy.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_log.h"

#define AS5600_PULSES_PER_REVOLUTION (2.0f * M_PI)

const char *tag = "ROTOR_SENSOR_AS5600";

typedef struct {
    esp_foc_seconds sample_rate;
    float radians_to_increment;
    float *uq_wire;
    float accumulated;
    float raw;
    float previous;
    esp_foc_rotor_sensor_t interface;
}esp_foc_dummy_t;

DRAM_ATTR static esp_foc_dummy_t rotor_sensors[CONFIG_NOOF_AXIS];

IRAM_ATTR float read_accumulated_counts (esp_foc_rotor_sensor_t *self)
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

esp_foc_rotor_sensor_t *rotor_sensor_open_loop_new(float motor_kv, float *uq_wire, esp_foc_seconds sample_rate){

    if(uq == NULL) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.read_accumulated_counts = read_accumulated_counts;
    rotor_sensors[port].raw = 0;
    rotor_sensors[port].accumulated = 0;

    return &rotor_sensors[port].interface;
}