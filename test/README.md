# espFoC unit tests

Unity-based unit tests for the espFoC component. They cover logic that does not depend on hardware and exercise code paths with mock drivers.

- **test_iq31.c**: Fixed-point IQ31 primitives — conversion (overflow/underflow/invalid), mul/add/sub, clamp, abs, rsqrt, sin/cos LUT, waveform generation, SVPWM-style duties. Uses `foc_math_iq31.h` for Clarke/Park and normalize in transform tests.
- **test_foc_math_iq31.c**: FOC math in IQ31 — Clarke, inverse Clarke, Park, inverse Park (trivial + edge + roundtrip), normalize_angle (wrap, 2π), limit_voltage (within limit, exceeds limit, zero vector, convergence vs float), apply_bias.
- **test_pid_iq31.c**: Canonical PID (`pid_controller.h`) — IQ31 `esp_foc_pid_update`; per-unit ref/measure in [-1,1]; compared to `esp_foc_pid_update_float` in parallel
- **test_biquad_q16.c**: 2nd-order Butterworth low-pass + DF-II Transposed biquad (`biquad_q16.h`) — bypass, designer guards, DC unity, Nyquist rejection, -3 dB at fc, passband flat, stopband 40 dB/decade, long-run stability, reset semantics
- **test_modulator_iq31.c**: Q16 `esp_foc_svm_set`, `esp_foc_modulate_dq_voltage`, `esp_foc_get_dq_currents` — duty bounds, limit+inverse-Park consistency vs reference q16 path, Clarke/Park consistency for currents; typical and edge cases
- **test_axis_iq31_api.c**: Public axis API (`esp_foc_initialize_axis` / `align` / `run` / `set_regulation_callback`) with mocks; valid/invalid arguments and boundary coverage
- **test_foc_pipeline_iq31.c**: IQ31-only manual FOC pipeline (LPF → PID → modulator): bounded duty cycles, bitwise repeatability across two identical runs, SVM saturation smoke
- **mock_drivers.c/h**: Inverter, rotor, and current mocks (IQ31-only; records calls and values)
- **test_driver_mocks.c**: Mock API tests (`set_voltages` IQ31, `get_dc_link_voltage`, `get_inverter_pwm_rate`, `read_counts`, `fetch_isensors`, etc.)
- **test_driver_iq31_api.c**: Additional mock tests with IQ31-scale assertions (overlaps part of **test_driver_mocks.c**)
- **test_axis_flow.c**: Axis flow with mocks — init, align, `set_regulation_callback`, `run`; checks that the regulation callback runs and `set_voltages` receives FOC output
- **test_observers_iq31.c**: Observer smoke tests (simu, PLL, KF, PMSM model) — `update` with IQ31 inputs, getters, and `reset`
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
