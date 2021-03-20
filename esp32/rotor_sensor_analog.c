#include <unistd.h>
#include <math.h>
#include <sys/cdefs.h>
#include "esp_heap_caps.h"
#include "espFoC/rotor_sensor_analog.h"
#include "esp_attr.h"


typedef struct {
    float current_count;
    float previous_read;
    float counts_per_revolution;
    int adc_channel;
    float limit_high;
    float limit_low;

    esp_foc_rotor_sensor_t interface;
}esp_foc_analog_rotor_sensor_t;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;

IRAM_ATTR static float read_adc(int adc_channel) 
{
    return (float)(adc1_get_raw((adc1_channel_t)adc_channel));
}

IRAM_ATTR  static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_analog_rotor_sensor_t *obj =
        __containerof(self,esp_foc_analog_rotor_sensor_t, interface);

    obj->current_count = 0.0f;
    obj->previous_read = read_adc(obj->adc_channel);

}

IRAM_ATTR static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    esp_foc_analog_rotor_sensor_t *obj =
        __containerof(self,esp_foc_analog_rotor_sensor_t, interface);

    return obj->counts_per_revolution;
}

IRAM_ATTR static float read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_analog_rotor_sensor_t *obj =
        __containerof(self,esp_foc_analog_rotor_sensor_t, interface);
    float raw = read_adc(obj->adc_channel);
    float delta = (raw - obj->previous_read);

    if(fabs(delta) > obj->counts_per_revolution) {
        obj->current_count = (delta < 0.0f) ? 
            obj->current_count + obj->counts_per_revolution :
                obj->current_count + obj->counts_per_revolution;
    
        if(obj->current_count > obj->limit_high) {
            obj->current_count -= obj->limit_high;
        }else if (obj->current_count < obj->limit_low) {
            obj->current_count -= obj->limit_low;
        }
    }

    obj->previous_read = raw;
    return obj->current_count + raw;
}

IRAM_ATTR static void  delay_ms(esp_foc_rotor_sensor_t *self, int ms)
{
    (void)self;
    usleep(ms * 1000);    
}

esp_err_t rotor_sensor_analog_init()
{
    adc1_config_width(width);
    return ESP_OK;
}

esp_foc_rotor_sensor_t *rotor_sensor_analog_new(int adc_channel, 
                                                int min_sensor_count,           
                                               int max_sensor_count)
{
    esp_foc_analog_rotor_sensor_t *obj = 
        heap_caps_malloc(
            sizeof(esp_foc_analog_rotor_sensor_t), MALLOC_CAP_INTERNAL
        );

    if(!obj) {
        return NULL;
    }

    obj->interface.get_counts_per_revolution = get_counts_per_revolution;
    obj->interface.read_counts = read_counts;
    obj->interface.set_to_zero = set_to_zero;
    obj->interface.delay_ms = delay_ms;

    obj->adc_channel = adc_channel;
    if(max_sensor_count < min_sensor_count) {
        obj->counts_per_revolution = min_sensor_count - max_sensor_count;
    } else {
        obj->counts_per_revolution = max_sensor_count - min_sensor_count;
    }

    obj->limit_high = 100.0f * obj->counts_per_revolution;
    obj->limit_low = -obj->limit_high;

    adc1_config_channel_atten(obj->adc_channel, atten);
    obj->previous_read = read_adc(obj->adc_channel);

    return &obj->interface;
}