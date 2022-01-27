# espFoC simple motor tester:
This example demonstrates how to use the motor tester to verify 
the phase sequence. Use it to validate your connections before
to proceed the usage of the FoC controller.

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
* The motor should start to moving with constant speed in forward and backward 
* In case this is not occur this means wrong connections.
* In case of wrong connections turn off, switch phase connections and re-do the test