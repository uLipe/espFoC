# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(espfoc_motor)

set(MOTOR_CONTROL_SRC ${APPLICATION_SOURCE_DIR}/../../common)
set(MOTOR_CONTROL_INC ${APPLICATION_SOURCE_DIR}/../../common/include)

FILE(GLOB ipc ${MOTOR_CONTROL_SRC}/ipc/*.c)
FILE(GLOB motor_control ${MOTOR_CONTROL_SRC}/motor_control/*.c)

target_sources(app PRIVATE  esp_foc_motor.c
                            esp_foc_hardware_if.c
                            ${ipc}
                            ${motor_control})

target_include_directories(app PRIVATE  ${MOTOR_CONTROL_INC})
