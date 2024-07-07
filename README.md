# espFoC: Vector FoC controller for PMSM motors for ESP32 SoCs

![Build](https://github.com/uLipe/espFoC/workflows/Build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

espFoC is a simple implementation of voltage mode, vector controller intended to be used with permanent-magnet synchronous motors (PMSM), and general brushless motors for ESP32S3 Espressif SoC, other SoC from espressif might be supported but the primary requirement is to be a dual
core variant. espFoC started as a library for motor control, but it suffered a goal change, and now it is intended to create a programmable
BLDC (FoC based) motor control-IC, the dual core capabillity splits the controller into application side and motor control core side
the former is intended for user to write it own application or exchange packets with external controller via CAN-Bus, the later is
100% focused on perform motor control algorithm achieving better control bandwidth. Up two axis are supported in this new variant, refer the
simplified architecture below:
![espFoC Simplified Architecture](/doc/images/arch.png)

## Features:

* Voltage mode control, control a PMSM like a DC motor!;
* Position and Speed closed-loop control;
* Single-precision Floating point implementation;
* Easy to wire motor using common drivers and I2C encoders out there!
* Uses ESP32 AMP solution for Zephyr RTOS to split execution;
* App CPU runs the motor control firmware;
* Pro CPU runs communication and exposes a thread for user application;
* Interaction between the firmwares are doing by a custom IPC protocol;
* Planned support for UART and CAN communication;

## Limitations:

* Support only for espressif SoC that are dual-core and have FPU;
* Once sensored support, requires an I2C magnetic encoder sensor.

## Getting started:

* Just clone this project on most convenient folder then:

```
 $ west build  -palways -besp32s3_devkitm/esp32s3/procpu --sysbuild /path/to/espFoC/app
```

* To flash:
```
 $ west flash
```

## Typical wiring:

* espFoC is intended to run on ESP32 board plus a motor driver;
* The current driver supports 3-PWM output suited to: L6230, DRV83xx and others;
* The suggested wiring for quick get started is shown below:
![Wiring](/doc/images/wiring.png)

## Support:

If you find some trouble, open an issue, and if you are enjoying the project
give it a star or submir a PR. Also, you can try reaching me at: