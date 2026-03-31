/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include "espFoC/utils/esp_foc_q16.h"
#include "espFoC/driver_iq31_local.h"
#include "espFoC/rotor_sensor_as5600.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0E
#define AS5600_PULSES_PER_REVOLUTION 4096.0f
#define AS5600_READING_MASK 0xFFF
#define AS5600_CPR_UINT 4096u
#define AS5600_WRAP_INT 3891

static const char *tag = "ROTOR_SENSOR_AS5600";

typedef struct {
    uint16_t zero_offset;
    int i2c_port;
    int32_t prev_raw_iq;
    int64_t accum_i64;
    esp_foc_rotor_sensor_t interface;
} esp_foc_as5600_t;

static bool i2c_bus_configured = false;
static esp_foc_as5600_t rotor_sensors[CONFIG_NOOF_AXIS];

static uint16_t read_angle_sensor(int i2c_port)
{
    uint8_t write_buffer = AS5600_ANGLE_REGISTER_H;
    uint8_t read_buffer[2] = {0, 0};
    uint16_t raw;
    esp_err_t err;

    do {
        err = i2c_master_write_read_device(i2c_port,
                                           AS5600_SLAVE_ADDR,
                                           &write_buffer,
                                           1,
                                           read_buffer,
                                           2,
                                           portMAX_DELAY);
    } while (err != ESP_OK);

    raw = read_buffer[0];
    raw <<= 8;
    raw |= read_buffer[1];

    if (raw > AS5600_PULSES_PER_REVOLUTION - 1) {
        raw = AS5600_PULSES_PER_REVOLUTION - 1;
    }

    return raw;
}

static void set_to_zero(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_t *obj = __containerof(self, esp_foc_as5600_t, interface);
    uint16_t raw = read_angle_sensor(obj->i2c_port) & AS5600_READING_MASK;
    obj->zero_offset = raw;
    ESP_LOGI(tag, "Setting %d [ticks] as offset.", obj->zero_offset);
}

static uint32_t get_counts_per_revolution(esp_foc_rotor_sensor_t *self)
{
    (void)self;
    return AS5600_CPR_UINT;
}

static q16_t read_counts(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_t *obj = __containerof(self, esp_foc_as5600_t, interface);

    uint16_t raw = read_angle_sensor(obj->i2c_port) & AS5600_READING_MASK;
    int32_t delta = (int32_t)raw - obj->prev_raw_iq;

    if (delta >= AS5600_WRAP_INT || delta <= -AS5600_WRAP_INT) {
        esp_foc_critical_enter();
        if (delta < 0) {
            obj->accum_i64 += (int64_t)AS5600_CPR_UINT;
        } else {
            obj->accum_i64 -= (int64_t)AS5600_CPR_UINT;
        }
        esp_foc_critical_leave();
    }

    esp_foc_critical_enter();
    obj->prev_raw_iq = (int32_t)raw;
    esp_foc_critical_leave();

    uint32_t cm = (uint32_t)((raw - obj->zero_offset) & AS5600_READING_MASK);
    return esp_foc_q16_from_counts_mod(cm, AS5600_CPR_UINT);
}

static int64_t read_accumulated_counts_i64(esp_foc_rotor_sensor_t *self)
{
    esp_foc_as5600_t *obj = __containerof(self, esp_foc_as5600_t, interface);
    return obj->accum_i64 + (int64_t)obj->prev_raw_iq;
}

static void set_simulation_count(esp_foc_rotor_sensor_t *self, q16_t increment_normalized)
{
    esp_foc_as5600_t *obj = __containerof(self, esp_foc_as5600_t, interface);
    int64_t dt = ((int64_t)increment_normalized * (int64_t)AS5600_CPR_UINT) >> 31;
    obj->accum_i64 += dt;
}

esp_foc_rotor_sensor_t *rotor_sensor_as5600_new(int pin_sda,
                                                int pin_scl,
                                                int port)
{
    if (port > CONFIG_NOOF_AXIS - 1) {
        return NULL;
    }

    rotor_sensors[port].interface.get_counts_per_revolution = get_counts_per_revolution;
    rotor_sensors[port].interface.read_counts = read_counts;
    rotor_sensors[port].interface.set_to_zero = set_to_zero;
    rotor_sensors[port].interface.read_accumulated_counts_i64 = read_accumulated_counts_i64;
    rotor_sensors[port].interface.set_simulation_count = set_simulation_count;
    rotor_sensors[port].i2c_port = I2C_NUM_0;
    rotor_sensors[port].zero_offset = 0;
    rotor_sensors[port].prev_raw_iq = 0;
    rotor_sensors[port].accum_i64 = 0;

    if (!i2c_bus_configured) {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = pin_sda,
            .scl_io_num = pin_scl,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 1000000,
        };

        i2c_param_config(I2C_NUM_0, &conf);
        i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
        i2c_filter_enable(I2C_NUM_0, 7);

        i2c_bus_configured = true;
    }

    ESP_LOGI(tag, "Bus-check: %d", read_angle_sensor(rotor_sensors[port].i2c_port));

    return &rotor_sensors[port].interface;
}
