# espFoC unit test runner

Minimal app that runs the Unity test menu for espFoC unit tests. Use this app to run tests on target or in CI.

## Build

From this directory:

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py -D TEST_COMPONENTS=espFoC build
```

For other targets (e.g. esp32s3, esp32p4), run `idf.py set-target <target>` first.

## Run on device

```bash
idf.py flash monitor
```

In the monitor, press **Enter** to show the test menu, then:

- `*` — run all tests
- `[espFoC]` — run only espFoC tests
- A number — run a single test by index

## CI

In CI, build and (optionally) run tests:

```yaml
# Example GitHub Actions step (run from repo root, where espFoC is the component root)
- name: Build unit tests
  run: |
    . $IDF_PATH/export.sh
    cd examples/unit_test_runner
    idf.py set-target esp32
    idf.py -D TEST_COMPONENTS=espFoC build
```

To run tests in CI you need a runner with a connected device and serial port; use [pytest-embedded](https://docs.espressif.com/projects/pytest-embedded/en/latest/) or similar to flash, run the test menu with `*`, and parse the output.
