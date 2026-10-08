if(IDF_TARGET STREQUAL "linux")
    # Unit tests only: no inverter, rotor driver or RTOS behind them. The
    # mock-inverter suites run the stacks' tasks and stay on silicon.
    idf_component_register(SRCS
                               "test_esp_foc_angle.c"
                               "test_esp_foc_clarke.c"
                               "test_esp_foc_ident.c"
                               "test_esp_foc_if.c"
                               "test_esp_foc_iir.c"
                               "test_esp_foc_mech_id.c"
                               "test_esp_foc_motor_id.c"
                               "test_esp_foc_observer.c"
                               "test_esp_foc_observer_flux.c"
                               "test_esp_foc_park.c"
                               "test_esp_foc_pid.c"
                               "test_esp_foc_rotor_est.c"
                               "test_esp_foc_rotor_pll.c"
                               "test_esp_foc_svm.c"
                               "test_esp_foc_trig.c"
                               "test_esp_foc_vlim.c"
                           INCLUDE_DIRS "."
                           PRIV_INCLUDE_DIRS "../source/motor_control"
                           REQUIRES unity espFoC)
    return()
endif()

idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../source/motor_control"
                       REQUIRES unity espFoC)
