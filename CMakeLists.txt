set(srcs    "source/motor_control/esp_foc_core.c"
            "source/motor_control/control_strategy/esp_foc_control_current_mode_sensored.c"
            "source/motor_control/esp_foc_angle_predictor.c"
            "source/motor_control/esp_foc_biquad_q16.c"
            "source/motor_control/esp_foc_calibration.c"
            "source/motor_control/esp_foc_design_mpz.c"
            "source/motor_control/esp_foc_link.c"
            "source/motor_control/esp_foc_axis_tuning.c"
            "source/motor_control/esp_foc_scope.c"
            "source/motor_control/esp_foc_iq31.c"
            "source/motor_control/esp_foc_q16.c"
            "source/motor_control/observers/esp_foc_pll_observer.c"
            "source/motor_control/observers/esp_foc_pmsm_model_observer.c"
            "source/motor_control/observers/esp_foc_kf_observer.c"
            "source/motor_control/observers/esp_foc_simu_observer.c"
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
    list(APPEND srcs "source/motor_control/esp_foc_tuner.c")
endif()
if(CONFIG_ESP_FOC_BRIDGE_UART)
    list(APPEND srcs "source/drivers/esp_foc_bridge_uart.c")
endif()
if(CONFIG_ESP_FOC_BRIDGE_USBCDC)
    list(APPEND srcs "source/drivers/esp_foc_bridge_usbcdc.c")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       PRIV_REQUIRES "${requires}"
                       LDFRAGMENTS linker.lf)

# --- Build-time PI autotuning -------------------------------------------
# Generates esp_foc_autotuned_gains.h into the build dir (never the source
# tree) so it stays out of git automatically. The component code consumes
# the header behind CONFIG_ESP_FOC_USE_AUTOGEN_GAINS.
if(CONFIG_ESP_FOC_USE_AUTOGEN_GAINS)
    idf_build_get_property(python PYTHON)
    set(autogen_dir "${CMAKE_CURRENT_BINARY_DIR}/gen")
    set(autogen_header "${autogen_dir}/esp_foc_autotuned_gains.h")
    set(autogen_report "${autogen_dir}/esp_foc_autotuned_report.txt")
    string(REPLACE "\"" "" motor_profile "${CONFIG_ESP_FOC_MOTOR_PROFILE}")
    set(motor_profile_path
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/motors/${motor_profile}.json")
    if(NOT EXISTS "${motor_profile_path}")
        message(FATAL_ERROR
            "espFoC autotuner: motor profile not found: ${motor_profile_path}\n"
            "Add a JSON under scripts/motors/ or change CONFIG_ESP_FOC_MOTOR_PROFILE.")
    endif()
    add_custom_command(
        OUTPUT ${autogen_header}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${autogen_dir}
        COMMAND ${python} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_pi_gains.py
                --motor       ${motor_profile_path}
                --output      ${autogen_header}
                --report      ${autogen_report}
                --pwm-rate-hz ${CONFIG_ESP_FOC_AUTOGEN_PWM_RATE_HZ}
                --decimation  ${CONFIG_ESP_FOC_AUTOGEN_DECIMATION}
        DEPENDS ${motor_profile_path}
                ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_pi_gains.py
        COMMENT "espFoC: autotuning PI gains for profile '${motor_profile}'"
        VERBATIM)
    add_custom_target(esp_foc_autogen_gains DEPENDS ${autogen_header})
    add_dependencies(${COMPONENT_LIB} esp_foc_autogen_gains)
    target_include_directories(${COMPONENT_LIB} PRIVATE ${autogen_dir})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_FOC_AUTOGEN_GAINS_AVAILABLE=1)
endif()

endif()
