# tuner_studio_target

Service-mode firmware for live tuning with **espFoC TunerStudio**.
Boots, attaches a single axis to the runtime tuner, parks the motor
at zero current and waits for the host to drive everything else
(alignment, motion commands, persistence).

## Build & flash

```bash
cd examples/tuner_studio_target
idf.py set-target esp32s3        # or esp32 / esp32p4
idf.py menuconfig                # pins + shunts under "TunerStudio target —
                                 # pin map"; PWM rate under espFoC Settings →
                                 # Core (ESP_FOC_PWM_RATE_HZ)
idf.py build flash monitor
```

The current sensor is mandatory: the Id/Iq PI loop has nothing to
close on without measured currents, and the whole point of this
firmware is to tune that loop. Defaults match the SimpleFOCShield
(INA240A2 amp, 10 mΩ shunt, ADC1 channels 1 & 5) — adjust for
your board before flashing.

USB-CDC is the default transport (S2/S3/P4 only). On the original
ESP32, switch to UART under `Component config` →
`espFoC Settings` → `Tuner transport bridge`.

## Workflow

1. Flash this firmware on your bring-up board.
2. Launch TunerStudio: `PYTHONPATH=tools python3 -m espfoc_studio.gui --port /dev/ttyACM0`
3. Click **Align axis** in the Tuning panel — the firmware runs
   the alignment routine including the natural-direction probe.
4. Engage **Override active** and dial `iq_ref` in to spin the
   motor under tuner control. Watch the SVM hexagon and scope.
5. Tune Kp/Ki via the MPZ recompute button or by hand.
6. Click **Save to NVS** when satisfied.

The **Generate App** tab (template codegen + NVS export helpers) appears only
when you run the GUI with **`--demo`** (simulated firmware). For a production
IDF tree from your pin map and gains, use `espfoc_studio.codegen.generate_sensored_app`
or copy from `tools/espfoc_studio/templates/sensored_app` — see `doc/TUNING.md`.
