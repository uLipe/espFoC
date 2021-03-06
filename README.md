# [WIP] espFoC: Vector controller for PMSM motors for ESP-IDF

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

espFoC is a simple implementation of voltage mode, vector controller intended to be used with permanent-magnet synchronous motors (PMSM), and general brushless motors. This component was developed to be used with the ESP-IDF 
espressif framework.

## Getting started:
* Just clone this project on most convenient folder;
* Inside of your IDF project CMakeLists.txt set or add the path of this component to EXTRA_COMPONENT_DIRS for example: `set(EXTRA_COMPONENT_DIRS "path/to/this/component/")`
* For batteries included getting started, refer the examples folder.

## Features:
* Voltage mode control;
* Optional closed-loop velocity control;
* Single-precision Floating point implementation;

## Limitations:
* Initial support for esp32 only;
* No Sensorless control support;
* Requires and rotor position sensor, for example, incremental encoder.
* No torque controller, but planned.

## Support:
If you find some trouble, open an issue, and if you are enjoying the project
give it a star. Also, you can try reaching me at ryukokki.felipe@gmail.com