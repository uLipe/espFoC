"""Host-side scope sample period (matches firmware low-speed scope push).

Firmware calls ``esp_foc_scope_data_push()`` once per outer-loop iteration.
Inner-loop rate is ``read_loop_fs_hz()`` (PWM rate or legacy decimated rate).
"""

from __future__ import annotations

# Must match ``ESP_FOC_LOW_SPEED_DOWNSAMPLING`` in include/espFoC/esp_foc_controls.h
LOW_SPEED_DOWNSAMPLING = 10

# demo_firmware.DemoFirmware: SUB_STEPS * SCOPE_DECIMATION outer iterations per scope frame
_DEMO_SUB_STEPS = 40
_DEMO_SCOPE_DECIMATION = 4


def scope_uniform_dt_s(loop_fs_hz: float, *, demo: bool) -> float:
    """Seconds between consecutive scope frames for uniform X-axis plotting."""
    if loop_fs_hz <= 1.0:
        return LOW_SPEED_DOWNSAMPLING / 20000.0
    if demo:
        return (_DEMO_SUB_STEPS * _DEMO_SCOPE_DECIMATION) / loop_fs_hz
    return LOW_SPEED_DOWNSAMPLING / loop_fs_hz
