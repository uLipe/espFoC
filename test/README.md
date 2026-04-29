# espFoC unit tests

Unity-based unit tests for the espFoC component. They cover logic that does not depend on hardware and exercise code paths with mock drivers.

- **test_pid_q16.c**: Canonical PID (`pid_controller.h`) — Q16 `esp_foc_pid_update`; per-unit ref/measure in [-1,1]; compared to `esp_foc_pid_update_float` in parallel
- **test_biquad_q16.c**: 2nd-order Butterworth low-pass + DF-II Transposed biquad (`biquad_q16.h`) — bypass, designer guards, DC unity, Nyquist rejection, -3 dB at fc, passband flat, stopband 40 dB/decade, long-run stability, reset semantics
- **test_modulator_q16.c**: Q16 `esp_foc_svm_set`, `esp_foc_modulate_dq_voltage` (phase volts out), `esp_foc_get_dq_currents` — duty bounds via svm, limit+inverse-Park vs phase volts, Clarke/Park for currents
- **test_foc_pipeline_q16.c**: Manual FOC pipeline (LPF → PID → modulator): bounded duty cycles, bitwise repeatability across two identical runs, SVM saturation smoke
- **mock_drivers.c/h**: Inverter, rotor, and current mocks (Q16; records calls and values)
- **test_driver_mocks.c**: Mock API tests (`set_voltages` Q16, `get_dc_link_voltage`, `get_inverter_pwm_rate`, `read_counts`, `fetch_isensors`, etc.)
- **test_driver_q16_api.c**: Additional mock tests (overlaps part of **test_driver_mocks.c**)
- **test_axis_flow.c**: Axis flow with mocks — init, align, `set_regulation_callback`, `run`; checks that the regulation callback runs and `set_voltages` receives FOC output
- **test_observers_q16.c**: Observer smoke tests (simu, PLL, KF, PMSM model) — `update` with Q16 inputs, getters, and `reset`
- **test_q16_numerical_stability.c**: Numerical stability of the Q16.16 pipeline — arithmetic boundary saturation (overflow/underflow), sin/cos identity and wrap-around, Clarke/Park roundtrips at small and large signals, limit_voltage edge cases, PID sustained zero-error / windup / alternating-sign, EMA convergence and oscillation bounds, SVPWM extreme-input duty clamping, full 10000-step pipeline stability, Q16↔IQ31 bridge consistency, angle normalization wraps. Tags: `[espFoC][q16][stability]`

## Building and running

From **`examples/unit_test_runner`** in the espFoC tree:

```bash
cd components/espFoC/examples/unit_test_runner
. $IDF_PATH/export.sh
idf.py -D TEST_COMPONENTS=espFoC build
idf.py flash monitor
```

In the monitor, press Enter to open the test menu, then type `*` to run all tests or filter by tag (e.g. `[espFoC]`).

## Build from another project

Any project that has espFoC in `EXTRA_COMPONENT_DIRS` (or as a dependency) can include these tests by building with:

```bash
idf.py -D TEST_COMPONENTS=espFoC build
```

The project must provide a main that calls `unity_run_menu()` for the test menu to appear.
