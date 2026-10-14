# espFoC unit test runner

Runs the Unity suites from `test/` on any supported chip: the portable math
blocks, the OSAL and the stacks against a mock motor. Replace `<target>` with
the chip (`esp32c6`, `esp32s31`, ...) and the port with your board's.

```bash
cd examples/unit_test_runner
idf.py -D TEST_COMPONENTS=espFoC set-target <target>
idf.py -D TEST_COMPONENTS=espFoC -p /dev/ttyUSB0 flash monitor
```

On boot the app auto-runs all registered tests. Expect:

```text
N Tests 0 Failures 0 Ignored
OK
PASS unit_tests
```
