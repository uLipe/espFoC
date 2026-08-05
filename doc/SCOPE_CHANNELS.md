# Scope channel map (`axis_shell`, 17ch)

Fixed order on the wire (`CONFIG_ESP_FOC_SCOPE_NUM_CHANNELS=17`). Host catalog: `tools/espfoc_tool/catalog/axis_shell_17ch.py`.

| Ch | Firmware signal | Display name | Unit |
|----|-----------------|--------------|------|
| 0 | `target_i_d` | Target D-Current | A |
| 1 | `i_d` | Measured D-Current | A |
| 2 | `target_i_q` | Target Q-Current | A |
| 3 | `i_q` | Measured Q-Current | A |
| 4 | `u_d` | Applied D-Voltage | pu |
| 5 | `u_q` | Applied Q-Voltage | pu |
| 6 | `rotor_estimator.theta_meas_mech` | Rotor mechanical position | turn |
| 7 | `rotor_elec_angle` | Rotor electrical position | turn |
| 8 | `rotor_estimator.omega_est_mech` | Rotor estimated speed | turn/s |
| 9 | `u_u` (PWM duty) | Phase U duty | pu |
| 10 | `u_v` | Phase V duty | pu |
| 11 | `u_w` | Phase W duty | pu |
| 12 | `i_u` | Measured phase U current | A |
| 13 | `i_v` | Measured phase V current | A |
| 14 | `i_alpha` | Alpha current | A |
| 15 | `i_beta` | Beta current | A |
| 16 | `esp_foc_debug_scope_hot_path_dt_us_q16` | ISR hot-path execution | µs |

Phase duties are the same Q16 values passed to `set_duties()` (0…1). D/Q “voltages” are mod-index-limited PI outputs in per-unit.

Other examples (`test_encoder`, `test_isensor_characterize`) use different `n_ch` and need their own host catalogs if viewed with espFoC Tool.
