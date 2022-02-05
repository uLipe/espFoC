# espFoC microROS simple demo:
This example demonstrates how to setup a PMSM motor using the espFoC in
its simplest configuration, that is it 3 PWM output plus analog encoder
sensor plus commanding motor using microROS and the Robot Operating System,
the ROS2.

## Dependencies
---
This example needs `colcon` and other Python 3 packages inside the IDF virtual environment in order to build micro-ROS packages:

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
```

## Getting started:
---
* You must wire your ESP32 board to an driver breakout board which supports
 3 PWM driving such as: L6234, L6230 or DRV8301;

* Install the ESP-IDF and export it;
* Configure this project for example SSID and PASSWORD for microROS network
* Also set the microROS Agent IP and port (in general 8888)

```
$ idf.py menuconfig
```
* On terminal, inside of this folder type:
```
$ idf.py build flash monitor
```
* The motor should start to moving with constant speed.

## ROS2 and MicroROS side:
---
After flashing the ESP32 board, the device will connect to the 
WiFI network and will wait to be discovered by a ROS2 instance
that runs the microROS Agent.

Refer this link on how to setup ROS2: https://docs.ros.org/en/foxy/Installation.html
Refer this link on how to setup uROS: https://github.com/micro-ROS/micro_ros_espidf_component#example

After complete the setup you may able to publish data to control the motor by
using ROS2 CLI:

```
ros2 topic pub /esp32/bldc/vq std_msgs/Float32 "{data:<Voltage to Apply>}"
```


