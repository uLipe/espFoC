set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/esp_foc_axis_bench.c"
            "source/motor_control/esp_foc_estimator_q16.c"
            "source/motor_control/esp_foc_biquad_q16.c"
            "source/calibration/esp_foc_calibration_format.c"
            "source/calibration/esp_foc_calibration_nvs.c"
            "source/calibration/esp_foc_calibration_axis.c"
            "source/motor_control/esp_foc_iq31.c"
            "source/motor_control/esp_foc_q16.c"
            )

set(includes "include" "source/motor_control")

if(ESP_PLATFORM)

idf_build_get_property(target IDF_TARGET)

set(requires driver freertos esp_system esp_timer nvs_flash esp_hw_support efuse)

if(CONFIG_ESP_FOC_FITL)
    list(APPEND srcs "source/motor_control/foc_in_the_loop/esp_foc_itl_plant.c"
                     "source/motor_control/foc_in_the_loop/esp_foc_in_the_loop.c")
    list(APPEND includes "source/motor_control/foc_in_the_loop"
                         "source/drivers/foc_in_the_loop")
    list(APPEND srcs "source/osal/os_interface_idf.c")

    if(CONFIG_FOC_ITL_TICK_MCPWM)
        list(APPEND srcs "source/drivers/foc_in_the_loop/esp_foc_itl_tick_mcpwm.c")
        list(APPEND requires esp_driver_mcpwm)
    elseif(CONFIG_FOC_ITL_TICK_GPTIMER)
        list(APPEND srcs "source/drivers/foc_in_the_loop/esp_foc_itl_tick_gptimer.c")
        list(APPEND requires esp_driver_gptimer)
    endif()
else()
    list(APPEND requires esp_driver_uart esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm)
    list(APPEND includes "source/drivers" "source/drivers/cali")
    list(APPEND srcs    "source/drivers/inverter_3pwm_mcpwm.c"
                        "source/drivers/inverter_6pwm_mcpwm.c"
                        "source/drivers/esp_foc_inverter_mcpwm.c"
                        "source/drivers/esp_foc_encoder_as5600.c"
                        "source/drivers/esp_foc_encoder_as5048.c"
                        "source/drivers/esp_foc_encoder_pcnt.c"
                        "source/drivers/esp_foc_encoder_simu.c"
                        "source/drivers/current_sensor_adc.c"
                        "source/drivers/esp_foc_adc_cali_lut.c"
                        "source/drivers/cali/esp_foc_adc_cali_curve.c"
                        "source/drivers/cali/esp_foc_adc_range_extend.c"
                        "source/osal/os_interface_idf.c")

    if(target STREQUAL "esp32")
        list(APPEND srcs "source/drivers/cali/esp_foc_adc_cali_line_esp32.c")
    elseif(target STREQUAL "esp32c2")
        list(APPEND srcs "source/drivers/cali/esp_foc_adc_cali_line_esp32c2.c")
    elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/source/drivers/cali/coeff/esp_foc_curve_coeff_${target}.c")
        list(APPEND srcs "source/drivers/cali/coeff/esp_foc_curve_coeff_${target}.c")
    endif()

    if(CONFIG_SOC_ETM_SUPPORTED)
        list(APPEND srcs "source/drivers/isensor_adc_etm.c"
                         "source/drivers/inverter_mcpwm_etm.c")
    endif()

    if(target STREQUAL "esp32")
        list(APPEND srcs "source/drivers/isensor_adc_dma_esp32.c")
        list(APPEND requires esp_driver_i2s)
    else()
        list(APPEND srcs "source/drivers/isensor_adc_dma_gdma.c")
        list(APPEND requires esp_mm)
    endif()
endif()

if(target STREQUAL "esp32s3" OR target STREQUAL "esp32p4")
    list(APPEND requires espressif__esp_tinyusb)
endif()

if(CONFIG_ESPFOC_SHELL)
    list(APPEND srcs "source/shell/espfoc_shell_parse.c"
                     "source/shell/espfoc_shell.c")
else()
    list(APPEND srcs "source/shell/espfoc_shell_parse.c")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}"
                       LDFRAGMENTS linker.lf)

endif()
