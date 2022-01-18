#include <math.h>
#include "espFoC/rotor_sensor_as5600.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_attr.h"

#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0C
#define AS5600_PULSES_PER_REVOLUTION 4096.0f

typedef struct {
    float zero_offset;
    int i2c_port;
    esp_foc_rotor_sensor_t interface;
}esp_foc_as5600_t;

DRAM_ATTR static esp_foc_as5600_t rotor_sensors[CONFIG_NOOF_AXIS];

IRAM_ATTR static float read_angle_sensor(int i2c_port) 
{
    uint8_t write_buffer = AS5600_ANGLE_REGISTER_H;
    uint8_t read_buffer[2];
    uint16_t raw;

    i2c_master_write_read_device(i2c_port,
                            AS5600_SLAVE_ADDR,
                            &write_buffer,
                            1,
                            read_buffer,
                            2,
                            portMAX_DELAY);

    raw = read_buffer[0];
    raw <<= 8;
    raw |= read_buffer[1];

    return (float)(raw);
}

IRAM_ATTR  static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_t *obj =
        __containerof(self,esp_foc_as5600_t, interface);
    obj->zero_offset = read_angle_sensor(obj->i2c_port);
}

IRAM_ATTR static float get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return AS5600_PULSES_PER_REVOLUTION;
}

IRAM_ATTR static float read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_t *obj =
        __containerof(self,esp_foc_as5600_t, interface);

    return(read_angle_sensor(obj->i2c_port) - obj->zero_offset);
}

esp_foc_rotor_sensor_t *rotor_sensor_as5600_new(int pin_sda,
                                                int pin_scl,
                                                int port)
{
    if(port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].i2c_port = I2C_NUM_0;
    rotor_sensors[port].zero_offset = 0.0f;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    return &rotor_sensors[port].interface;
}