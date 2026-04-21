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
from espfoc_studio.gui.theme import apply_dark_theme
from espfoc_studio.link import LinkReader, LoopbackTransport
from espfoc_studio.protocol import TunerClient


def test_mainwindow_demo_boots_polls_and_streams_scope():
    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    reader = LinkReader(host_t)
    reader.start()
    try:
        client = TunerClient(reader)
        app = QApplication.instance() or QApplication(sys.argv)
        apply_dark_theme(app)
        w = MainWindow(client, title="espFoC smoke")
        w.show()
        # Engage the override and push a non-zero iq so the demo plant
        # actually modulates — otherwise the SVM hexagon just sees zero
        # vectors and looks idle.
        client.override_on()
        client.write_target_iq(1.5)
        # 1) the tuning panel must get a gain readout within a few ticks;
        # 2) the scope panel must see the CSV stream the demo firmware
        #    pushes (6 channels today), so its channel count grows
        #    beyond zero within a few scope frames.
        # 3) the SVM panel must collect enough alpha/beta samples to
        #    draw at least a short trail.
        deadline = time.monotonic() + 4.0
        tuning_seen = False
        scope_seen = False
        svm_seen = False
        while time.monotonic() < deadline:
            app.processEvents()
            if w._tuning._kp_label.text().strip() not in ("-", ""):
                tuning_seen = True
            if w._scope._n_channels > 0:
                scope_seen = True
            if len(w._svm._alpha_buf) >= 5:
                svm_seen = True
            if tuning_seen and scope_seen and svm_seen:
                break
            time.sleep(0.05)
        assert tuning_seen, "tuning panel never received a gain update"
        assert scope_seen, (
            "scope panel never received any SCOPE-channel frame"
        )
        assert svm_seen, (
            "SVM panel never received enough samples to draw a trail"
        )
        w.close()
    finally:
        fw.stop()
        reader.stop()


def main() -> int:
    tests = [test_mainwindow_demo_boots_polls_and_streams_scope]
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
