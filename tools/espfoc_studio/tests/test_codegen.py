"""Smoke + bit-exactness tests for the application generator."""

from __future__ import annotations

import os
import struct
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

# Need a QApplication on PySide6 even for the dataclass that the
# Hardware panel exports — the panel module imports Qt at top level.
from PySide6.QtWidgets import QApplication  # noqa: E402
_app = QApplication.instance() or QApplication([])

from espfoc_studio.codegen.sensored_app import (  # noqa: E402
    _crc32_ieee,
    _fnv1a_32,
    _pack_calibration_blob,
    generate_sensored_app,
)
from espfoc_studio.gui.hardware_panel import HardwareConfig  # noqa: E402


_TESTS = []


def _test(fn):
    _TESTS.append(fn)
    return fn


@_test
def test_crc32_ieee_known_vector():
    # CRC-32 (IEEE) of "123456789" = 0xCBF43926
    assert _crc32_ieee(b"123456789") == 0xCBF43926


@_test
def test_fnv1a_32_known_vector():
    # FNV-1a 32 of "" -> 0x811c9dc5; "default:1" -> deterministic value
    assert _fnv1a_32("") == 0x811c9dc5
    a = _fnv1a_32("default:1")
    b = _fnv1a_32("default:1")
    assert a == b
    assert _fnv1a_32("default:2") != a


@_test
def test_pack_calibration_blob_roundtrip():
    blob = _pack_calibration_blob(
        0x12345678, 0x77777777, 0x00010000,
        0x10000, 0x20000, 0x30000,
        0xDEADBEEF)
    # Header (20) + payload (6 q16 + 16-byte reserved = 40) = 60 bytes.
    assert len(blob) == 60
    magic, version, _pad, profile, crc, plen, _pad2 = struct.unpack_from(
        "<IB3sIIHH", blob, 0)
    assert magic == 0xE5F0CC11
    assert version == 1
    assert profile == 0xDEADBEEF
    assert plen == 40
    payload = blob[20:]
    assert _crc32_ieee(payload) == crc


@_test
def test_generate_sensored_app_writes_expected_files():
    cfg = HardwareConfig(target="esp32s3", motor_profile="default")
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "fresh_app")
        res = generate_sensored_app(
            cfg,
            app_name="my_app",
            output_dir=out,
            kp=1.234, ki=567.89, ilim=12.0,
            r_ohm=1.08, l_h=0.0018, bw_hz=150.0)

        for expected in ("CMakeLists.txt", "main/CMakeLists.txt",
                         "main/main.c", "sdkconfig.defaults", "README.md"):
            assert expected in res.files_written, expected
            assert os.path.isfile(os.path.join(out, expected)), expected

        with open(os.path.join(out, "main/main.c"), "r") as f:
            main_c = f.read()
        # Check the literal q16 substitution lands intact and the pin map
        # leaks through.
        assert str(int(round(1.234 * 65536))) in main_c
        assert "esp_foc_axis_set_current_pi_gains_q16" in main_c
        assert str(cfg.pin_u_hi) in main_c
        # Profile hash leaks into both the README and the sdkconfig.
        with open(os.path.join(out, "sdkconfig.defaults"), "r") as f:
            sdk = f.read()
        assert "default" in sdk


@_test
def test_generate_refuses_nonempty_directory():
    cfg = HardwareConfig()
    with tempfile.TemporaryDirectory() as td:
        # touch a file
        with open(os.path.join(td, "preexisting"), "w") as f:
            f.write("x")
        try:
            generate_sensored_app(
                cfg, app_name="foo", output_dir=td,
                kp=1.0, ki=1.0, ilim=1.0)
        except ValueError:
            return
        raise AssertionError("expected ValueError on non-empty dir")


def main() -> int:
    fails = 0
    for fn in _TESTS:
        try:
            fn()
            print(f"OK    {fn.__name__}")
        except AssertionError as e:
            print(f"FAIL  {fn.__name__}: {e}")
            fails += 1
        except Exception as e:
            print(f"ERR   {fn.__name__}: {e!r}")
            fails += 1
    print(f"\n{'All ' + str(len(_TESTS)) + ' tests passed' if fails == 0 else f'{fails}/{len(_TESTS)} FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
