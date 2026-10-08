# espFoC — new generation

set(includes "include")

if(IDF_TARGET STREQUAL "linux")
    # Host build: the portable core only. Drivers, OSAL and the stacks need
    # the SoC or the RTOS and are covered on silicon.
    set(srcs
        "source/debug/esp_foc_trace.c"
        "source/utils/esp_foc_trig_cordic.c"
        "source/utils/esp_foc_trig.c"
        "source/utils/esp_foc_trig_hw.c"
        "source/utils/esp_foc_iir_soft.c"
        "source/utils/esp_foc_iir.c"
        "source/utils/esp_foc_iir_hw.c"
        "source/utils/esp_foc_pid_soft.c"
        "source/utils/esp_foc_pid.c"
        "source/utils/esp_foc_pid_hw.c"
        "source/utils/esp_foc_clarke_soft.c"
        "source/utils/esp_foc_clarke.c"
        "source/utils/esp_foc_clarke_hw.c"
        "source/utils/esp_foc_park_soft.c"
        "source/utils/esp_foc_park.c"
        "source/utils/esp_foc_park_hw.c"
        "source/utils/esp_foc_vlim_soft.c"
        "source/utils/esp_foc_vlim.c"
        "source/utils/esp_foc_vlim_hw.c"
        "source/utils/esp_foc_svm_soft.c"
        "source/utils/esp_foc_svm.c"
        "source/utils/esp_foc_svm_hw.c"
        "source/motor_control/esp_foc_observer_bemf.c"
        "source/motor_control/esp_foc_observer_flux.c"
        "source/motor_control/esp_foc_rotor_est.c"
        "source/motor_control/esp_foc_rotor_pll.c"
        "source/motor_control/esp_foc_if.c"
    )
    if(CONFIG_ESP_FOC_ENABLE_MOTOR_ID)
        list(APPEND srcs
            "source/motor_control/esp_foc_ident.c"
            "source/motor_control/esp_foc_motor_id_seq.c"
            "source/motor_control/esp_foc_motor_id_probe.c"
            "source/motor_control/esp_foc_mech_id.c"
        )
    endif()
    idf_component_register(SRCS "${srcs}"
                           INCLUDE_DIRS "${includes}"
                           PRIV_INCLUDE_DIRS "source/utils" "source/motor_control")
    return()
endif()

set(srcs
    "source/osal/esp_foc_osal_idf.c"
    "source/debug/esp_foc_trace.c"
    "source/utils/esp_foc_trig_cordic.c"
    "source/utils/esp_foc_trig.c"
    "source/utils/esp_foc_trig_hw.c"
    "source/utils/esp_foc_iir_soft.c"
    "source/utils/esp_foc_iir.c"
    "source/utils/esp_foc_iir_hw.c"
    "source/utils/esp_foc_pid_soft.c"
    "source/utils/esp_foc_pid.c"
    "source/utils/esp_foc_pid_hw.c"
    "source/utils/esp_foc_clarke_soft.c"
    "source/utils/esp_foc_clarke.c"
    "source/utils/esp_foc_clarke_hw.c"
    "source/utils/esp_foc_park_soft.c"
    "source/utils/esp_foc_park.c"
    "source/utils/esp_foc_park_hw.c"
    "source/utils/esp_foc_vlim_soft.c"
    "source/utils/esp_foc_vlim.c"
    "source/utils/esp_foc_vlim_hw.c"
    "source/utils/esp_foc_svm_soft.c"
    "source/utils/esp_foc_svm.c"
    "source/utils/esp_foc_svm_hw.c"
    "source/motor_control/esp_foc_observer_bemf.c"
    "source/motor_control/esp_foc_observer_flux.c"
    "source/motor_control/esp_foc_rotor_est.c"
    "source/motor_control/esp_foc_rotor_pll.c"
    "source/motor_control/esp_foc_if.c"
    "source/drivers/rotor/esp_foc_rotor_sensorless.c"
    "source/drivers/inverter/mcpwm_bridge.c"
    "source/drivers/inverter/adc_dma_sense.c"
    "source/drivers/inverter/etm_link.c"
    "source/drivers/inverter/esp_foc_inverter_mcpwm.c"
    "source/drivers/rotor/i2c_hal_bus.c"
    "source/drivers/rotor/as5600.c"
)

if(CONFIG_ESP_FOC_ROTOR_HALL)
    list(APPEND srcs
        "source/drivers/rotor/esp_foc_rotor_hall.c"
        "source/drivers/rotor/hall_timestamp.c"
        "source/drivers/rotor/hall_timestamp_etm.c"
        "source/drivers/rotor/hall_timestamp_irq.c"
    )
endif()

if(CONFIG_ESP_FOC_PHASE_DISCOVER)
    list(APPEND srcs "source/motor_control/esp_foc_phase_discover.c")
endif()

if(CONFIG_ESP_FOC_STACK_SENSORED)
    list(APPEND srcs "source/motor_control/esp_foc_sensored.c")
endif()

if(CONFIG_ESP_FOC_STACK_SENSORLESS)
    list(APPEND srcs "source/motor_control/esp_foc_sensorless.c")
endif()

if(CONFIG_ESP_FOC_ENABLE_MOTOR_ID)
    list(APPEND srcs
        "source/motor_control/esp_foc_ident.c"
        "source/motor_control/esp_foc_motor_id.c"
        "source/motor_control/esp_foc_motor_id_seq.c"
        "source/motor_control/esp_foc_motor_id_probe.c"
        "source/motor_control/esp_foc_mech_id.c"
    )
endif()

if(ESP_PLATFORM)
    idf_component_register(SRCS "${srcs}"
                           INCLUDE_DIRS "${includes}"
                           PRIV_INCLUDE_DIRS
                               "source/utils"
                               "source/motor_control"
                               "source/drivers/inverter"
                               "source/drivers/rotor"
                           PRIV_REQUIRES freertos hal soc esp_hw_support esp_rom esp_timer
                           LDFRAGMENTS linker.lf)
    # The TEZ path only fits its PWM period at -O2, whatever level the
    # project builds the rest of the firmware at.
    target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
endif()
