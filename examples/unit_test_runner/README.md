# espFoC unit test runner (greenfield)

Runs Unity tests from `test/` on ESP32-C6 (Q16 CORDIC trig, IIR biquad, 2p2z PID).

```bash
cd examples/unit_test_runner
idf.py -D TEST_COMPONENTS=espFoC set-target esp32c6
idf.py -D TEST_COMPONENTS=espFoC -p /dev/ttyUSB0 flash monitor
```

On boot the app auto-runs all registered tests. Expect:

```text
31 Tests 0 Failures 0 Ignored
OK
PASS unit_tests
```
