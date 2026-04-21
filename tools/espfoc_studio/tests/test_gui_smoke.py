#!/usr/bin/env python3
"""Headless smoke test of the GUI demo pipeline.

Exercises the same path as `python -m espfoc_studio.gui --demo` but
closes the window after a couple of timer ticks so the test is cheap
and hermetic. QT_QPA_PLATFORM=offscreen makes this run on CI boxes
with no display.

Run:
    QT_QPA_PLATFORM=offscreen PYTHONPATH=tools \
        python3 tools/espfoc_studio/tests/test_gui_smoke.py
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from espfoc_studio.gui.demo_firmware import DemoFirmware
from espfoc_studio.gui.main_window import MainWindow
from espfoc_studio.link import LoopbackTransport
from espfoc_studio.protocol import TunerClient


def test_mainwindow_demo_boots_and_polls():
    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    try:
        client = TunerClient(host_t)
        app = QApplication.instance() or QApplication(sys.argv)
        w = MainWindow(client, scope_source=fw.snapshot_plant,
                       title="espFoC smoke")
        w.show()
        # Sanity: the initial gain readouts should materialize within a
        # couple of timer ticks (200 ms). We drive the event loop manually
        # so the test runs in ~1 s and exits.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            app.processEvents()
            # Stop as soon as the Kp label has been populated.
            if w._tuning._kp_label.text().strip() not in ("-", ""):
                break
            time.sleep(0.05)
        assert w._tuning._kp_label.text().strip() not in ("-", ""), (
            "tuning panel never received a gain update")
        w.close()
    finally:
        fw.stop()


def main() -> int:
    tests = [test_mainwindow_demo_boots_and_polls]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"OK    {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    if failed:
        print(f"\n{failed} test(s) failed", file=sys.stderr)
        return 1
    print(f"\nAll {len(tests)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
