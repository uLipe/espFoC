# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(REMOTE_ZEPHYR_DIR ${CMAKE_CURRENT_BINARY_DIR}/../espfoc_motor_remote/zephyr)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(espfoc_app)

set(MOTOR_CONTROL_SRC ${APPLICATION_SOURCE_DIR}/../common)
set(MOTOR_CONTROL_INC ${APPLICATION_SOURCE_DIR}/../common/include)

FILE(GLOB ipc ${MOTOR_CONTROL_SRC}/ipc/*.c)
target_sources(app PRIVATE espfoc_app/esp_foc_app_shell.c
                           espfoc_app/esp_foc_main.c
                           espfoc_app/esp_foc_user.c
                           espfoc_app/esp_foc_motor_control_firmware_pilot.c
                           ${ipc})

target_include_directories(app PRIVATE  ${MOTOR_CONTROL_INC})

