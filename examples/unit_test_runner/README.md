# espFoC unit test runner

Minimal app that runs the Unity test menu for espFoC unit tests. Default target for automated runs is **QEMU** (no hardware); you can also run on a real device.

## Build

From this directory (default target for tests is esp32 for QEMU):

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py -D TEST_COMPONENTS=espFoC build
```

For other targets (e.g. esp32s3, esp32p4), run `idf.py set-target <target>` first.

## Run tests (default: QEMU)

From repo root, run unit tests on QEMU (CI and local, no device needed). Exit code 0 = all passed.

```bash
. $IDF_PATH/export.sh
# Install Espressif QEMU once if needed: python $IDF_PATH/tools/idf_tools.py install qemu-xtensa
# Install pexpect for the runner script: pip install pexpect
cd examples/unit_test_runner
python run_unit_tests_qemu.py
```

## Run on device

Flash and open the serial monitor, then use the Unity menu:

```bash
idf.py flash monitor
```

Press **Enter** to show the test menu, then:

- `*` — run all tests
- `[espFoC]` — run only espFoC tests
- A number — run a single test by index

To run the pytest suite against a connected board (ESPPORT set):

```bash
pytest pytest_espfoc_unittest.py -m generic -v
```

## CI

The workflow job `unit_tests` builds for esp32, installs Espressif QEMU (`idf_tools.py install qemu-xtensa`), and runs `python run_unit_tests_qemu.py`. Pass/fail is taken from the script exit code (0 = all tests passed). An optional step runs tests on a real device via `pytest -m generic` when using a self-hosted runner with `ESPPORT` set (`continue-on-error: true` when no device is present).
