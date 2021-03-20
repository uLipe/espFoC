idf_build_get_property(target IDF_TARGET)

set(srcs "esp_foc.c" "esp_foc_consts.c")
set(includes "include")


if(${target} STREQUAL "esp32")
    list(APPEND includes "esp32")
    list(APPEND srcs     "esp32/inverter_3pwm_ledc.c"
                         "esp32/rotor_sensor_analog.c")   
endif()

idf_component_register(SRCS "esp_foc_consts.c" "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES driver freertos esp_system)