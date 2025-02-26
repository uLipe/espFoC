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

## Getting started:
* Just clone this project on most convenient folder;
* Inside of your IDF project CMakeLists.txt set or add the path of this component to EXTRA_COMPONENT_DIRS for example: `set(EXTRA_COMPONENT_DIRS "path/to/this/component/")`
* For batteries included getting started, refer the examples folder.
* Inside of any of examples just build: `$ idf.py build flash`

## Typical wiring:
* espFoC is intended to run on ESP32 board plus a motor driver;
* The current driver supports 3-PWM output suited to: L6230, DRV83xx and others;
* The suggested wiring for quick get started is shown below:
![Wiring](/doc/images/wiring.png)


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
