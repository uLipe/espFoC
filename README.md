# espFoC: Vector FoC controller for PMSM motors for ESP32 SoCs

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

espFoC is a simple implementation of voltage mode, vector controller intended to be used with permanent-magnet synchronous motors (PMSM), and general brushless motors. This component was developed to be used with Zephyr RTOS for example and is aimed to transform
your esp32 chip into a dual axis PMSM motor controller chip

## Features:

* Voltage mode control, control a PMSM like a DC motor!;
* Position and Speed closed-loop control;
* Single-precision Floating point implementation;
* Sample inverter driver based on esp32 LEDC PWM (easy to wire!);
* Sample rotor position driver based on as5600 encoder (very popular!);
* Uses openAMP to offload motor control tasks to one of the core, and leave the other for communication;
* support UART and CAN communication;

## Limitations:

* Support for esp32 and esp32s3 only;
* Requires and rotor position sensor, for example, incremental encoder.

## Getting started:

* Just clone this project on most convenient folder;

## Typical wiring:

* espFoC is intended to run on ESP32 board plus a motor driver;
* The current driver supports 3-PWM output suited to: L6230, DRV83xx and others;
* The suggested wiring for quick get started is shown below:
![Wiring](/doc/images/wiring.png)

## Support:

If you find some trouble, open an issue, and if you are enjoying the project
give it a star or submir a PR. Also, you can try reaching me at:
ryukokki.felipe@gmail.com