idf_build_get_property(target IDF_TARGET)

set(requires driver esp_adc freertos esp_system esp_timer)
set(srcs    "esp_foc_core.c"
            "esp_foc_control_current_mode_sensored.c"
            "esp_foc_control_voltage_mode_sensored.c"
            "esp_foc_control_current_mode_sensorless.c"
            "esp_foc_scope.c"
            "esp_foc_consts.c"
            "esp_foc_sine_cosine.c"
            "esp_foc_sqrtf.c"
            "esp_foc_pll_observer.c"
            "esp_foc_simu_observer.c")

set(includes "include" "drivers")

list(APPEND srcs    "drivers/inverter_3pwm_ledc.c"
                    "drivers/inverter_3pwm_mcpwm.c"
                    "drivers/rotor_sensor_as5600.c"
                    "drivers/rotor_sensor_pcnt.c"
                    "drivers/current_sensor_adc.c"
                    "drivers/os_interface.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}")