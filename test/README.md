# espFoC unit tests

Unity-based unit tests for the espFoC component. They cover logic that does not depend on hardware:

- **test_foc_math.c**: Clarke/Park transforms, normalize angle, clamp, limit voltage, rsqrt, sine/cosine
- **test_pid.c**: PID controller (proportional, clamping, reset, integral)
- **test_lp_filter.c**: Low-pass EMA filter (init, alpha clamp, step response, cutoff)

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
