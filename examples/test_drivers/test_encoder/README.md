# espFoC current loop control demo:
This example demonstrates how to setup a PMSM motor using the espFoC in
using the simulated rotor sensor, it simulates the electrical plus the
mechanical behavior of the rotor to deliver a simulated encoder to feed
the FoC Engine. Different from the regular open loop the currents measured
feed the torque controller

No encoder is required to run this sample, you need however the motor phase
resistance in Ohms and the Phase inductance in Henry

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
* The motor should start to moving.