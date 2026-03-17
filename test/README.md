# espFoC unit tests

Unity-based unit tests for the espFoC component. They cover logic that does not depend on hardware and flow with mock drivers.

- **test_foc_math.c**: Clarke/Park transforms, normalize angle, clamp, limit voltage, rsqrt, sine/cosine
- **test_iq31.c**: Fixed-point IQ31 helpers — conversion (overflow/underflow/invalid), mul/add/sub, clamp, abs, rsqrt, sin/cos LUT, waveform generation, Clarke/Park transforms, SVPWM-style duties. Run with espFoC built (optionally with `CONFIG_ESP_FOC_USE_FIXED_POINT` for future FP path).
- **test_pid.c**: PID controller (proportional, clamping, reset, integral)
- **test_lp_filter.c**: Low-pass EMA filter (init, alpha clamp, step response, cutoff)
- **mock_drivers.c/h**: Mock implementations of inverter, rotor sensor, and current sensor interfaces (record calls, configurable return values)
- **test_driver_mocks.c**: Driver API tests using mocks (set_voltages/get_dc_link/enable/disable, read_counts/set_to_zero, fetch/calibrate)
- **test_axis_flow.c**: Axis flow with mocks — initialize_axis, align_axis, set_regulation_callback, run; verifies regulator callback runs and set_voltages receives FOC output

## Building and running

From the **unit_tests** project in this workspace:

```bash
cd esp_foc_ws/unit_tests
. /path/to/esp-idf/export.sh
idf.py -D TEST_COMPONENTS=espFoC build
idf.py flash monitor
```

In the monitor, press Enter to open the test menu, then type `*` to run all tests or `[espFoC]` to run only espFoC tests.

## Build from another project

Any project that has espFoC in `EXTRA_COMPONENT_DIRS` (or as a dependency) can include these tests by building with:

```bash
idf.py -D TEST_COMPONENTS=espFoC build
```

The project must provide a main that calls `unity_run_menu()` for the test menu to appear (e.g. the **unit_tests** project in this workspace).
