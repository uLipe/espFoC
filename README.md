# espFoC: Vector FoC controller for PMSM motors for ESP32 SoCs

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![alt text](doc/images/pos_ctl.gif)

espFoC is a simple implementation of voltage mode, vector controller intended to be used with permanent-magnet synchronous motors (PMSM), and general brushless motors. This component was developed to be used with the ESP-IDF
espressif framework.

## Features:
* Voltage mode control, control a PMSM like a DC motor!;
* Position and Speed closed-loop control;
* Single-precision Floating point implementation;
* Sample inverter driver based on esp32 LEDC PWM (easy to wire!);
* Sample rotor position driver based on as5600 encoder (very popular!);
* FoC engine runs sychronized at inverter PWM rate;
* Scope function for debugging using Better Serial Plot
* **NOTICE**: Requises Espressif ESP-IDF v5.0 or above

## Limitations:
* Support for esp32 and esp32s3 only;
* Requires and rotor position sensor, for example, incremental encoder.

## Typical wiring:
* espFoC is intended to run on ESP32 board plus a motor driver;
* The current driver supports 3-PWM output suited to: L6230, DRV83xx and others;
* The suggested wiring for quick get started is shown below:
![Wiring](/doc/images/wiring.png)

## Board Reference:
The examples are configurable by nature to user select the best board,
however the testbench used for its development is based on:

* MakerBase FoC ESP32: https://github.com/makerbase-motor/MKS-ESP32FOC
* Nanotec DB42S03 PMSM Motor: https://www.renesas.cn/zh/document/sch/nanotec-motor-db24s03-specification
* MakerBase AS5600 encoder: https://pt.aliexpress.com/item/1005002515162635.html?gatewayAdapt=glo2bra

## Example code:
Setting espFoC to run your motor is simple, after wiring everything you can just put the following
on your `app_main.c` file to play with the open loop mode, the simplest one, but don't get it
wrong, it can pilot the motor at fast speeds if you have the motor parameters:

```
#include "esp_log.h"
#include "esp_err.h"

#include "espFoC/inverter_3pwm_mcpwm.h"
#include "espFoC/current_sensor_adc.h"
#include "espFoC/esp_foc.h"

static const char *TAG = "esp-foc-example";

static esp_foc_inverter_t *inverter;
static esp_foc_isensor_t  *shunts;
static esp_foc_axis_t axis;
static esp_foc_motor_control_settings_t settings = {
    .motor_pole_pairs = 4,
    .velocity_control_settings.kp = 1.0f,
    .velocity_control_settings.ki = 0.0f,
    .velocity_control_settings.kd = 0.0f,
    .motor_inductance = 0.012f,
    .motor_resistance = 1.5f,
    .velocity_control_settings.integrator_limit = 100.0f,
    .velocity_control_settings.max_output_value = 4.0f, //conservative setpoint to the current controller
    .torque_control_settings[0].max_output_value = 6.0f, //Uses the max driver voltage allowed as limit
    .natural_direction = ESP_FOC_MOTOR_NATURAL_DIRECTION_CW,
};

static void initialize_foc_drivers(void)
{
    inverter = inverter_3pwm_mpcwm_new(
        CONFIG_FOC_PWM_U_PIN,
        CONFIG_FOC_PWM_V_PIN,
        CONFIG_FOC_PWM_W_PIN,
        CONFIG_FOC_PWM_EN_PIN,
        24.0f,
        0
    );

    if(inverter == NULL) {
        ESP_LOGE(TAG, "failed to create the inverter driver, aborting!");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        abort();
    }
}

void app_main(void)
{
    esp_foc_control_data_t control_data;
    esp_foc_q_voltage uq = {.raw = 0.0f};

    ESP_LOGI(TAG, "Initializing the esp foc motor controller!");

    initialize_foc_drivers();

    esp_foc_initialize_axis(
        &axis,
        inverter,
        NULL,
        NULL,
        settings
    );

    /* Align rotor, once per startup */
    esp_foc_align_axis(&axis);

    /* esp_foc core engine runs independent after starting it */
    esp_foc_run(&axis);

    /* Set desired voltage */
    uq.raw = 12.0f;
    esp_foc_set_target_voltage(&axis, uq, (esp_foc_d_voltage){.raw = 0.0});
}
```

## Getting started:
* Just clone this project on most convenient folder;
* Inside of your IDF project CMakeLists.txt set or add the path of this component to EXTRA_COMPONENT_DIRS for example: `set(EXTRA_COMPONENT_DIRS "path/to/this/component/")`
* For batteries included getting started, refer the examples folder.
* Inside of any of examples just build: `$ idf.py build flash`

## Debug with Better Serial Plot:
* Install Better Serial Plot from here: https://hackaday.io/project/181686-better-serial-plotter
* In menuconfing enable the option: `CONFIG_ESP_FOC_SCOPE`
* Download the firmware for your target board;
* Open the Better Serial Port and select the port and baud rate of your board;
* The data should arrive automatically.

## Support:
If you find some trouble, open an issue, and if you are enjoying the project
give it a star or submir a PR. Also, you can try reaching me at:
ryukokki.felipe@gmail.com
