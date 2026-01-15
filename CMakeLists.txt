set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c"
            "source/motor_control/control_strategy/esp_foc_control_voltage_mode_sensored.c"
            "source/motor_control/control_strategy/esp_foc_control_current_mode_sensorless.c"
            "source/motor_control/esp_foc_scope.c"
            "source/motor_control/esp_foc_consts.c"
            "source/motor_control/esp_foc_sine_cosine.c"
            "source/motor_control/observers/esp_foc_pll_observer.c"
            "source/motor_control/observers/esp_foc_simu_observer.c")

set(includes "include")

if(ESP_PLATFORM)

idf_build_get_property(target IDF_TARGET)

set(requires esp_driver_uart esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm esp_driver_ledc esp_adc driver freertos esp_system esp_timer)
list(APPEND includes "source/drivers")
list(APPEND srcs    "source/drivers/inverter_3pwm_ledc.c"
                    "source/drivers/inverter_3pwm_mcpwm.c"
                    "source/drivers/inverter_6pwm_mcpwm.c"
                    "source/drivers/rotor_sensor_as5600.c"
                    "source/drivers/rotor_sensor_as5048.c"
                    "source/drivers/rotor_sensor_pcnt.c"
                    "source/drivers/current_sensor_adc.c"
                    "source/drivers/current_sensor_adc_one_shot.c"
                    "source/rust/esp_foc_ffi.c"
                    "source/osal/os_interface_idf.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}"
                       LDFRAGMENTS linker.lf)
endif()