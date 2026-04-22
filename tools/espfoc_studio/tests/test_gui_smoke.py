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


def test_scope_panel_wallclock_and_autoset():
    """Time axis tracks wall clock (not sample count) and Autoset
    drops history + rebases the time origin."""
    from espfoc_studio.gui.scope_panel import ScopePanel

    app = QApplication.instance() or QApplication(sys.argv)
    panel = ScopePanel()
    # Inject 200 frames at 1 ms wall-clock spacing.
    base = time.monotonic()
    for i in range(200):
        # Patch the inbox directly so we control the timestamps.
        with panel._inbox_lock:
            panel._inbox.append((base + i * 1e-3, b"1.0,2.0,3.0\n"))
    panel._render_tick()
    assert panel._n_channels == 3
    assert len(panel._time_buf) == 200
    span = panel._time_buf[-1] - panel._time_buf[0]
    # Span must be ~199 ms (199 deltas * 1 ms), regardless of any
    # internal sample_dt assumption — that is the whole point.
    assert 0.190 < span < 0.210, f"span={span!r} not wall-clock locked"

    # Autoset clears state and rebases.
    panel.autoset()
    assert len(panel._time_buf) == 0
    for buf in panel._channel_bufs:
        assert len(buf) == 0
    # Subsequent injection lands at near-zero relative time.
    with panel._inbox_lock:
        panel._inbox.append((time.monotonic(), b"4.0,5.0,6.0\n"))
    panel._render_tick()
    assert len(panel._time_buf) == 1
    assert panel._time_buf[0] < 0.05  # fresh origin


def test_scope_panel_roll_mode_x_axis_stays_bounded():
    """Live cursor must always sit at x = 0 (right edge) even after
    a long run — the displayed X values are 'seconds before now',
    not absolute monotonic time, so they can never run off-screen."""
    from espfoc_studio.gui.scope_panel import ScopePanel

    app = QApplication.instance() or QApplication(sys.argv)
    panel = ScopePanel()
    # Pretend the panel has been running for an hour and a frame
    # arrives one tick before "now".
    panel._t0 = time.monotonic() - 3600.0
    with panel._inbox_lock:
        panel._inbox.append((time.monotonic() - 0.010, b"1.0,2.0\n"))
        panel._inbox.append((time.monotonic() - 0.005, b"1.5,2.5\n"))
        panel._inbox.append((time.monotonic(),         b"2.0,3.0\n"))
    panel._render_tick()
    assert panel._n_channels == 2
    # Pull the rendered X data from the first curve and check that
    # the latest sample sits at ~0 and nothing is at +3600 seconds.
    x_data, _ = panel._curves[0].getData()
    assert x_data is not None and len(x_data) == 3
    assert x_data[-1] > -0.05, f"latest sample at x={x_data[-1]!r}, expected ~0"
    assert x_data[0] >= -panel.WINDOW_S, (
        f"oldest sample at x={x_data[0]!r}, outside the window")
    assert max(abs(v) for v in x_data) < panel.WINDOW_S + 0.1, (
        "X values escaped the rolling window")


def main() -> int:
    tests = [test_mainwindow_demo_boots_polls_and_streams_scope,
             test_scope_panel_wallclock_and_autoset,
             test_scope_panel_roll_mode_x_axis_stays_bounded]
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
