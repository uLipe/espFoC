# tuner_studio_target

Service-mode firmware for live tuning with **espFoC TunerStudio**.
Boots, attaches a single axis to the runtime tuner, parks the motor
at zero current and waits for the host to drive everything else
(alignment, motion commands, persistence, app generation).

## Build & flash

```bash
cd examples/tuner_studio_target
idf.py set-target esp32s3        # or esp32 / esp32p4
idf.py menuconfig                # adjust the pin map AND the ADC
                                 # shunt config under
                                 # "TunerStudio target — pin map"
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
3. The GUI detects the `TSGX` firmware-type and enables the
   **Generate App** tab.
4. Click **Align axis** in the Tuning panel — the firmware runs
   the alignment routine including the natural-direction probe.
5. Engage **Override active** and dial `iq_ref` in to spin the
   motor under tuner control. Watch the SVM hexagon and scope.
6. Tune Kp/Ki via the MPZ recompute button or by hand.
7. Click **Save to NVS** when satisfied.
8. Open the **Generate App** tab, fill in the pin map and a name,
   then click **Generate** — TunerStudio writes a ready-to-build
   IDF application to disk with the live gains baked in.
9. `cd <output_dir> && idf.py build flash` — production firmware
   boots already tuned, no host required.
