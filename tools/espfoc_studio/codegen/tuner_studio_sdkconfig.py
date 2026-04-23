"""Write Hardware panel options into a :file:`sdkconfig` for
:file:`examples/tuner_studio_target`.

`idf.py set-target` creates :file:`sdkconfig` from :file:`sdkconfig.defaults` and
:file:`sdkconfig.defaults.<chip>`.  A pre-existing :file:`sdkconfig` in the
example (from a previous :command:`idf.py build` / menuconfig) can pin options so
TunerStudio panel changes appear to be ignored.  The GUI build path (see
:mod:`BuildWorker <espfoc_studio.gui.build_worker>`) removes a stale
:file:`sdkconfig` and :file:`build/`, then set-target, then this module applies
*CONFIG_TUNER_TARGET_** and related *CONFIG_ESP_FOC_** lines so the compile
matches the Hardware form.
"""

from __future__ import annotations

import os
import re
from typing import List, Optional, Set, Tuple

from ..gui.hardware_panel import HardwareConfig


def _kconfig_key_from_line(ln: str) -> Optional[str]:
    s = ln.rstrip("\n")
    t = s.strip()
    m = re.match(r"^# CONFIG_([A-Za-z0-9_]+) is not set$", t)
    if m:
        return f"CONFIG_{m.group(1)}"
    m2 = re.match(r"^(CONFIG_[A-Za-z0-9_]+)=", t)
    if m2:
        return m2.group(1)
    return None


def _build_assignments(cfg: HardwareConfig) -> List[Tuple[str, str]]:
    g = min(100_000, max(1, int(round(float(cfg.amp_gain) * 100.0))))
    sh_mohm = min(10_000, max(1, int(round(float(cfg.shunt_ohm) * 1000.0))))
    dcv = int(round(float(cfg.dc_link_v)))
    pr = (cfg.motor_profile or "default").replace(
        "\\", "\\\\").replace('"', '\\"')
    sda = int(cfg.sensor_cfg.get("sda", 4))
    scl = int(cfg.sensor_cfg.get("scl", 5))
    return [
        ("CONFIG_TUNER_TARGET_PWM_U_HI", str(int(cfg.pin_u_hi))),
        ("CONFIG_TUNER_TARGET_PWM_V_HI", str(int(cfg.pin_v_hi))),
        ("CONFIG_TUNER_TARGET_PWM_W_HI", str(int(cfg.pin_w_hi))),
        ("CONFIG_TUNER_TARGET_PWM_U_LO", str(int(cfg.pin_u_lo))),
        ("CONFIG_TUNER_TARGET_PWM_V_LO", str(int(cfg.pin_v_lo))),
        ("CONFIG_TUNER_TARGET_PWM_W_LO", str(int(cfg.pin_w_lo))),
        ("CONFIG_TUNER_TARGET_PWM_EN_PIN", str(int(cfg.inverter_en_pin))),
        ("CONFIG_TUNER_TARGET_PWM_FREQ_HZ", str(int(cfg.pwm_freq_hz))),
        ("CONFIG_TUNER_TARGET_DC_LINK_V", str(dcv)),
        ("CONFIG_TUNER_TARGET_ENC_SDA", str(sda)),
        ("CONFIG_TUNER_TARGET_ENC_SCL", str(scl)),
        ("CONFIG_TUNER_TARGET_POLE_PAIRS", str(int(cfg.pole_pairs))),
        ("CONFIG_TUNER_TARGET_ISENSE_ADC_UNIT", str(int(cfg.adc_unit))),
        ("CONFIG_TUNER_TARGET_ISENSE_CH_U", str(int(cfg.adc_ch_u))),
        ("CONFIG_TUNER_TARGET_ISENSE_CH_V", str(int(cfg.adc_ch_v))),
        ("CONFIG_TUNER_TARGET_ISENSE_AMP_GAIN_X100", str(int(g))),
        ("CONFIG_TUNER_TARGET_ISENSE_SHUNT_MOHM", str(int(sh_mohm))),
        ("CONFIG_ESP_FOC_MOTOR_PROFILE", f'"{pr}"'),
        ("CONFIG_ESP_FOC_AUTOGEN_PWM_RATE_HZ", str(int(cfg.pwm_freq_hz))),
    ]


def _tuner_en_act_low_sdkconfig_line(cfg: HardwareConfig) -> str:
    if cfg.inverter_en_pin < 0:
        return "# CONFIG_TUNER_TARGET_PWM_EN_ACT_LOW is not set\n"
    if cfg.inverter_en_inverted:
        return "CONFIG_TUNER_TARGET_PWM_EN_ACT_LOW=y\n"
    return "# CONFIG_TUNER_TARGET_PWM_EN_ACT_LOW is not set\n"


def _remove_keys(lines: List[str], keys: Set[str]) -> List[str]:
    out: List[str] = []
    for ln in lines:
        k = _kconfig_key_from_line(ln)
        if k in keys:
            continue
        out.append(ln)
    return out


def apply_hardware_to_sdkconfig(path: str, cfg: HardwareConfig) -> None:
    """Re-write *path* in place: drop any prior *CONFIG* lines we own, then
    append a ``# espfoc_tuner_studio`` block with the panel values.  We
    remove old lines for those keys so there is a single unambiguous
    assignment in the file (IDF can resolve duplicates inconsistently
    if both appear)."""
    if not os.path.isfile(path):
        raise OSError(f"sdkconfig not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    lines: List[str] = text.splitlines(keepends=True) if text else []

    asg = _build_assignments(cfg)
    keys: Set[str] = {a[0] for a in asg}
    keys.add("CONFIG_TUNER_TARGET_PWM_EN_ACT_LOW")
    if not keys:
        return
    filtered = _remove_keys(list(lines), keys)
    if filtered and not (filtered[-1] == "" or filtered[-1].endswith("\n")):
        filtered.append("\n")
    block: List[str] = [
        "\n",
        (
            "\n# --- espfoc_tuner_studio: Hardware form (TunerStudio "
            "Generate App) ---\n"
        ),
    ]
    for k, v in asg:
        block.append(f"{k}={v}\n")
    block.append(_tuner_en_act_low_sdkconfig_line(cfg))
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(filtered)
        f.writelines(block)
