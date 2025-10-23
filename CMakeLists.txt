idf_build_get_property(target IDF_TARGET)

set(requires driver esp_adc freertos esp_system esp_timer)
set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/esp_foc_control_current_mode_sensored.c"
            "source/motor_control/esp_foc_control_voltage_mode_sensored.c"
            "source/motor_control/esp_foc_control_current_mode_sensorless.c"
            "source/motor_control/esp_foc_control_current_mode_sensored_slow.c"
            "source/motor_control/esp_foc_control_voltage_mode_sensored_slow.c"
            "source/motor_control/esp_foc_scope.c"
            "source/motor_control/esp_foc_consts.c"
            "source/motor_control/esp_foc_sine_cosine.c"
            "source/motor_control/esp_foc_sqrtf.c"
            "source/motor_control/esp_foc_pll_observer.c"
            "source/motor_control/esp_foc_simu_observer.c")

set(includes "include" "source/drivers")

list(APPEND srcs    "source/drivers/inverter_3pwm_ledc.c"
                    "source/drivers/inverter_3pwm_mcpwm.c"
                    "source/drivers/rotor_sensor_as5600.c"
                    "source/drivers/rotor_sensor_pcnt.c"
                    "source/drivers/current_sensor_adc.c"
                    "source/drivers/os_interface.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}"
                       LDFRAGMENTS linker.lf)