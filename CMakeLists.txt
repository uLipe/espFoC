set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/esp_foc_axis_bench.c"
            "source/motor_control/esp_foc_estimator_q16.c"
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

set(requires esp_driver_uart esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm driver freertos esp_system esp_timer nvs_flash esp_hw_support efuse)
# esp_tinyusb is fetched conditionally for USB-capable targets via
# idf_component.yml; declare the priv require eagerly here so the bridge
# source file finds tinyusb.h on those targets even when the user has not
# enabled CONFIG_ESP_FOC_BRIDGE_USBCDC at component-graph resolution time.
if(target STREQUAL "esp32s2" OR target STREQUAL "esp32s3" OR target STREQUAL "esp32p4")
    list(APPEND requires espressif__esp_tinyusb)
endif()
list(APPEND includes "source/drivers" "source/drivers/cali")
list(APPEND srcs    "source/drivers/inverter_3pwm_mcpwm.c"
                    "source/drivers/inverter_6pwm_mcpwm.c"
                    "source/drivers/rotor_sensor_as5600.c"
                    "source/drivers/rotor_sensor_as5048.c"
                    "source/drivers/rotor_sensor_pcnt.c"
                    "source/drivers/rotor_sensor_simu.c"
                    "source/drivers/current_sensor_adc.c"
                    "source/drivers/esp_foc_adc_cali_lut.c"
                    "source/drivers/cali/esp_foc_adc_cali_curve.c"
                    "source/drivers/cali/esp_foc_adc_range_extend.c"
                    "source/osal/os_interface_idf.c")

if(target STREQUAL "esp32")
    list(APPEND srcs "source/drivers/cali/esp_foc_adc_cali_line_esp32.c")
elseif(target STREQUAL "esp32s2")
    list(APPEND srcs "source/drivers/cali/esp_foc_adc_cali_line_esp32s2.c")
elseif(target STREQUAL "esp32c2")
    list(APPEND srcs "source/drivers/cali/esp_foc_adc_cali_line_esp32c2.c")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/source/drivers/cali/coeff/esp_foc_curve_coeff_${target}.c")
    list(APPEND srcs "source/drivers/cali/coeff/esp_foc_curve_coeff_${target}.c")
endif()

if(CONFIG_SOC_ETM_SUPPORTED)
    list(APPEND srcs "source/drivers/isensor_adc_etm.c")
endif()

if(target STREQUAL "esp32")
    list(APPEND srcs "source/drivers/isensor_adc_dma_esp32.c")
    list(APPEND requires esp_driver_i2s)
elseif(target STREQUAL "esp32s2")
    list(APPEND srcs "source/drivers/isensor_adc_dma_esp32s2.c")
    list(APPEND requires esp_driver_spi)
else()
    list(APPEND srcs "source/drivers/isensor_adc_dma_gdma.c")
    list(APPEND requires esp_mm)
endif()

if(CONFIG_ESP_FOC_TUNER_ENABLE)
    list(APPEND srcs "source/gui_link/esp_foc_tuner.c"
                     "source/gui_link/esp_foc_link_session.c")
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
