# espFoC simple demo:
This example demonstrates how to setup a PMSM motor using the espFoC in
its simplest configuration, that is it 3 PWM output plus analog encoder
sensor.

## Getting started:
* You must wire your ESP32 board to an driver breakout board which supports
 3 PWM driving such as: L6234, L6230 or DRV8301;

* Install the ESP-IDF and export it;
* Configure this project if needed by:
```
$ idf.py menuconfig
```
* On terminal, inside of this folder type:
```
$ idf.py build flash monitor
```
* The motor should start to moving ramping up and down the speed.