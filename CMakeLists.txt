set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c"
            "source/motor_control/esp_foc_biquad_q16.c"
            "source/calibration/esp_foc_calibration_format.c"
            "source/calibration/esp_foc_calibration_nvs.c"
            "source/calibration/esp_foc_calibration_axis.c"
            "source/gui_link/esp_foc_link.c"
            "source/gui_link/esp_foc_scope.c"
            "source/motor_control/esp_foc_iq31.c"
            "source/motor_control/esp_foc_q16.c"
            )

set(includes "include" "source/motor_control")

if(ESP_PLATFORM)

idf_build_get_property(target IDF_TARGET)

set(requires esp_driver_uart esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm esp_adc driver freertos esp_system esp_timer nvs_flash)
# esp_tinyusb is fetched conditionally for USB-capable targets via
# idf_component.yml; declare the priv require eagerly here so the bridge
# source file finds tinyusb.h on those targets even when the user has not
# enabled CONFIG_ESP_FOC_BRIDGE_USBCDC at component-graph resolution time.
if(target STREQUAL "esp32s2" OR target STREQUAL "esp32s3" OR target STREQUAL "esp32p4")
    list(APPEND requires espressif__esp_tinyusb)
endif()
list(APPEND includes "source/drivers")
list(APPEND srcs    "source/drivers/inverter_3pwm_mcpwm.c"
                    "source/drivers/inverter_6pwm_mcpwm.c"
                    "source/drivers/rotor_sensor_as5600.c"
                    "source/drivers/rotor_sensor_as5048.c"
                    "source/drivers/rotor_sensor_pcnt.c"
                    "source/drivers/current_sensor_adc.c"
                    "source/drivers/current_sensor_adc_one_shot.c"
                    "source/osal/os_interface_idf.c")

if(CONFIG_ESP_FOC_TUNER_ENABLE)
    list(APPEND srcs "source/gui_link/esp_foc_tuner.c")
endif()
if(CONFIG_ESP_FOC_BRIDGE_UART)
    list(APPEND srcs "source/drivers/gui_link/esp_foc_bridge_uart.c")
endif()
if(CONFIG_ESP_FOC_BRIDGE_USBCDC)
    list(APPEND srcs "source/drivers/gui_link/esp_foc_bridge_usbcdc.c")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}"
                       LDFRAGMENTS linker.lf)

endif()
