from __future__ import annotations

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from espfoc_studio.codegen.tuner_studio_sdkconfig import (  # noqa: E402
    apply_hardware_to_sdkconfig,
)
from espfoc_studio.gui.hardware_panel import (  # noqa: E402
    HardwareConfig,
)


def test_apply_replaces_tuner_keys():
    fd, path = tempfile.mkstemp(suffix="sdkconfig", text=True)
    os.close(fd)
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write("CONFIG_FOO=1\n")
            f.write("CONFIG_TUNER_TARGET_PWM_U_HI=99\n")
            f.write("CONFIG_FOO2=2\n")
        h = HardwareConfig(
            target="esp32s3", pin_u_hi=5, pin_v_hi=6, pin_w_hi=7,
            pin_u_lo=8, pin_v_lo=9, pin_w_lo=10,
        )
        apply_hardware_to_sdkconfig(path, h)
        with open(path, "r", encoding="utf-8") as f:
            body = f.read()
        # old line for U_HI removed, new block contains 5
        assert "CONFIG_TUNER_TARGET_PWM_U_HI=99" not in body
        assert "CONFIG_TUNER_TARGET_PWM_U_HI=5" in body
        assert "CONFIG_FOO=1" in body
        assert "espfoc_tuner_studio" in body
    finally:
        os.remove(path)
