"""Render a sensored current-mode application from a HardwareConfig
plus a tuning snapshot.

The generator is purely a string-templating affair: every file under
templates/sensored_app/<path>.tmpl is read, fed through str.format()
with the substitution dictionary built below, and written under
<output>/<path>. Doubled braces in the templates protect literal C
braces from str.format.

NVS partition image generation is best-effort: when
$IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py
is on disk we invoke it and ship the resulting nvs_calibration.bin
alongside the project; when the tool is not found we still write
the source files but record the miss in GenerationResult so the GUI
can surface a hint instead of failing the whole flow.
"""

from __future__ import annotations

import datetime as _dt
import os
import struct
import subprocess
from dataclasses import dataclass, field
from typing import List, Optional

from ..gui.hardware_panel import HardwareConfig


# Mirrors the on-flash layout in source/motor_control/esp_foc_calibration.c.
_BLOB_MAGIC = 0xE5F0CC11
_BLOB_VERSION = 1


def _q16(value: float) -> int:
    return int(round(value * 65536.0))


def _crc32_ieee(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return (~crc) & 0xFFFFFFFF


def _pack_calibration_blob(kp_q16: int, ki_q16: int, ilim_q16: int,
                           r_q16: int, l_q16: int, bw_q16: int,
                           fc_q16: int,
                           profile_hash: int) -> bytes:
    """Reproduces espfoc_cal_blob_t exactly so the generated image
    is bit-for-bit equivalent to one written at runtime by
    esp_foc_calibration_save()."""
    payload = struct.pack("<7i12s",
                          kp_q16, ki_q16, ilim_q16,
                          r_q16, l_q16, bw_q16,
                          fc_q16,
                          b"\x00" * 12)
    crc = _crc32_ieee(payload)
    header = struct.pack("<IB3sIIHH",
                         _BLOB_MAGIC,
                         _BLOB_VERSION,
                         b"\x00\x00\x00",
                         profile_hash,
                         crc,
                         len(payload),
                         0)
    return header + payload


def _fnv1a_32(s: str) -> int:
    h = 0x811c9dc5
    for ch in s.encode("ascii"):
        h ^= ch
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


@dataclass
class GenerationResult:
    output_dir: str
    files_written: List[str] = field(default_factory=list)
    nvs_image: Optional[str] = None
    nvs_skip_reason: Optional[str] = None


def _templates_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(
        os.path.join(here, "..", "templates", "sensored_app"))


def _espfoc_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", ".."))


def _walk_templates(root: str) -> List[str]:
    out: List[str] = []
    for dirpath, _, files in os.walk(root):
        for f in files:
            if f.endswith(".tmpl"):
                out.append(os.path.relpath(os.path.join(dirpath, f), root))
    return out


def _try_make_nvs_image(output_dir: str, blob: bytes,
                        partition_size: int = 0x6000
                        ) -> tuple[Optional[str], Optional[str]]:
    """Returns (image_path, skip_reason). When IDF's nvs_partition_gen.py
    cannot be found we leave the blob alone and let the GUI surface the
    miss as a soft warning."""
    idf_path = os.environ.get("IDF_PATH")
    if not idf_path:
        return None, "IDF_PATH not set"
    nvs_gen = os.path.join(idf_path, "components", "nvs_flash",
                           "nvs_partition_generator", "nvs_partition_gen.py")
    if not os.path.isfile(nvs_gen):
        return None, f"nvs_partition_gen.py not found at {nvs_gen}"

    raw_path = os.path.join(output_dir, "nvs_calibration.blob")
    csv_path = os.path.join(output_dir, "nvs_calibration.csv")
    bin_path = os.path.join(output_dir, "nvs_calibration.bin")
    with open(raw_path, "wb") as f:
        f.write(blob)
    with open(csv_path, "w", encoding="ascii") as f:
        f.write("key,type,encoding,value\n")
        f.write("espfoc_cal,namespace,,\n")
        f.write(f"axis_0,file,binary,{os.path.basename(raw_path)}\n")
    try:
        subprocess.run(
            ["python3", nvs_gen, "generate",
             csv_path, bin_path, hex(partition_size)],
            check=True, capture_output=True, text=True,
            cwd=output_dir)
    except subprocess.CalledProcessError as e:
        return None, f"nvs_partition_gen failed: {e.stderr.strip() or e}"
    return bin_path, None


def generate_sensored_app(cfg: HardwareConfig,
                          *,
                          app_name: str,
                          output_dir: str,
                          kp: float, ki: float, ilim: float,
                          r_ohm: float = 0.0,
                          l_h: float = 0.0,
                          bw_hz: float = 0.0,
                          current_filter_fc_hz: float = 300.0
                          ) -> GenerationResult:
    """Render the templates and return a summary. Raises ValueError if
    the output directory already contains files (we never overwrite)."""

    if os.path.exists(output_dir) and os.listdir(output_dir):
        raise ValueError(f"output directory not empty: {output_dir}")
    os.makedirs(output_dir, exist_ok=True)

    profile_hash = _fnv1a_32(f"{cfg.motor_profile}:1")
    sda = cfg.sensor_cfg.get("sda", 4)
    scl = cfg.sensor_cfg.get("scl", 5)
    if cfg.inverter_en_pin < 0:
        inv_en_gpio = -1
    else:
        inv_en_gpio = (
            -cfg.inverter_en_pin
            if cfg.inverter_en_inverted else cfg.inverter_en_pin)

    subs = {
        "app_name": app_name,
        "ESPFOC_PATH": _espfoc_root(),
        "target": cfg.target,
        "pwm_type": cfg.pwm_type,
        "pwm_freq_hz": cfg.pwm_freq_hz,
        "dc_link_v": f"{cfg.dc_link_v}",
        "pin_u_hi": cfg.pin_u_hi,
        "pin_v_hi": cfg.pin_v_hi,
        "pin_w_hi": cfg.pin_w_hi,
        "pin_u_lo": cfg.pin_u_lo,
        "pin_v_lo": cfg.pin_v_lo,
        "pin_w_lo": cfg.pin_w_lo,
        "sensor_type": cfg.sensor_type,
        "sda": sda,
        "scl": scl,
        "pole_pairs": cfg.pole_pairs,
        "motor_profile": cfg.motor_profile,
        "profile_hash": profile_hash,
        "kp": kp,
        "ki": ki,
        "ilim": ilim,
        "kp_q16": _q16(kp),
        "ki_q16": _q16(ki),
        "ilim_q16": _q16(ilim),
        "r_ohm": r_ohm,
        "l_h": l_h,
        "bw_hz": bw_hz,
        "current_filter_fc_hz": current_filter_fc_hz,
        "timestamp_iso": _dt.datetime.now().replace(microsecond=0).isoformat(),
        "inverter_en_gpio": inv_en_gpio,
    }

    written: List[str] = []
    root = _templates_root()
    for rel in _walk_templates(root):
        with open(os.path.join(root, rel), "r", encoding="utf-8") as f:
            text = f.read()
        rendered = text.format(**subs)
        out_rel = rel[:-5]  # strip .tmpl
        dest = os.path.join(output_dir, out_rel)
        os.makedirs(os.path.dirname(dest) or output_dir, exist_ok=True)
        with open(dest, "w", encoding="utf-8") as f:
            f.write(rendered)
        written.append(out_rel)

    blob = _pack_calibration_blob(
        _q16(kp), _q16(ki), _q16(ilim),
        _q16(r_ohm), _q16(l_h), _q16(bw_hz),
        _q16(current_filter_fc_hz),
        profile_hash)
    nvs_image, nvs_skip = _try_make_nvs_image(output_dir, blob)

    return GenerationResult(
        output_dir=output_dir,
        files_written=written,
        nvs_image=nvs_image,
        nvs_skip_reason=nvs_skip,
    )
