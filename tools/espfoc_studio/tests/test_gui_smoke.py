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
from espfoc_studio.link.scope_sample import pack_scope_i32_to_payload
from espfoc_studio.protocol import TunerClient


def _scope_bin_32(*vals: float) -> bytes:
    q = [int(round(f * 65536.0)) for f in vals]
    return pack_scope_i32_to_payload(q)


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
        w = MainWindow(
            client, title="espFoC smoke", link_mode="demo",
            link_descr="test loopback", serial_config=None)
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
        sensors_seen = False
        while time.monotonic() < deadline:
            app.processEvents()
            if w._tuning._kp_label.text().strip() not in ("-", ""):
                tuning_seen = True
            if w._scope._n_channels > 0:
                scope_seen = True
            if len(w._svm._alpha_buf) >= 5:
                svm_seen = True
            if len(w._sensors._time_buf) >= 3:
                sensors_seen = True
            if tuning_seen and scope_seen and svm_seen and sensors_seen:
                break
            time.sleep(0.05)
        assert tuning_seen, "tuning panel never received a gain update"
        assert scope_seen, (
            "scope panel never received any SCOPE-channel frame"
        )
        assert svm_seen, (
            "SVM panel never received enough samples to draw a trail"
        )
        assert sensors_seen, (
            "sensors panel never merged enough SCOPE samples (ch6+)"
        )
        w.close()
    finally:
        fw.stop()
        reader.stop()


def test_scope_panel_uniform_time_and_autoset():
    """X-axis advances by uniform_dt per frame (low-speed scope), even when
    USB delivers a burst with identical wall timestamps."""
    from espfoc_studio.gui.scope_panel import ScopePanel

    app = QApplication.instance() or QApplication(sys.argv)
    panel = ScopePanel(async_decode=False)
    panel.set_uniform_sample_period_s(1e-3)
    panel.set_display_lag_s(0.0)
    n_inj = min(40, ScopePanel.MAX_MERGE_SAMPLES_PER_TICK - 1)
    base = time.monotonic()
    for _i in range(n_inj):
        with panel._inbox_lock:
            panel._inbox.append((base, _scope_bin_32(1.0, 2.0, 3.0)))
    panel._render_tick()
    assert panel._n_channels == 3
    with panel._history_lock:
        hist = list(panel._history)
    assert len(hist) == n_inj
    times = [t for t, _ in hist]
    span = times[-1] - times[0]
    exp = (n_inj - 1) * 1e-3
    assert abs(span - exp) < 1e-9, f"span={span!r} expected {exp!r}"
    for i in range(1, len(times)):
        assert abs(times[i] - times[i - 1] - 1e-3) < 1e-12

    panel.autoset()
    with panel._history_lock:
        assert len(panel._history) == 0
    with panel._inbox_lock:
        panel._inbox.append((time.monotonic(), _scope_bin_32(4.0, 5.0, 6.0)))
    panel._render_tick()
    with panel._history_lock:
        assert len(panel._history) == 1
        assert panel._history[0][0] == 0.0


def test_generate_app_embeds_hardware_section():
    """The standalone Hardware tab is gone in favour of an embedded
    section inside GenerateAppPanel — guard against a regression that
    re-introduces the orphaned tab or drops the embedded panel."""
    from espfoc_studio.gui.generate_app_panel import GenerateAppPanel
    from espfoc_studio.gui.hardware_panel import HardwarePanel

    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    reader = LinkReader(host_t)
    reader.start()
    try:
        client = TunerClient(reader)
        app = QApplication.instance() or QApplication(sys.argv)
        panel = GenerateAppPanel(
            client,
            get_motor_params=lambda: (1.08, 0.0018, 150.0))
        # The embedded HardwarePanel must be a child widget so the
        # "Hardware tab moved into Generate App" semantics hold.
        assert hasattr(panel, "_hw"), "GenerateAppPanel lost _hw attribute"
        assert isinstance(panel._hw, HardwarePanel), (
            f"_hw is {type(panel._hw).__name__}, not HardwarePanel")
        cfg = panel._hw.get_config()
        assert cfg.target in ("esp32", "esp32s3", "esp32p4"), (
            f"unexpected default target {cfg.target!r}")
    finally:
        fw.stop()
        reader.stop()


def test_build_worker_idf_path_helper(tmp_dir=None):
    """is_valid_idf_path is a pure helper; verify it accepts a fake
    minimal layout and rejects an empty / partial one."""
    import tempfile
    from espfoc_studio.gui.build_worker import is_valid_idf_path

    assert not is_valid_idf_path("")
    assert not is_valid_idf_path("/no/such/path/at/all")
    with tempfile.TemporaryDirectory() as td:
        # Empty directory: missing tools/idf.py + components/.
        assert not is_valid_idf_path(td)
        os.makedirs(os.path.join(td, "tools"))
        os.makedirs(os.path.join(td, "components"))
        # Components dir present but tools/idf.py still missing.
        assert not is_valid_idf_path(td)
        with open(os.path.join(td, "tools", "idf.py"), "w") as f:
            f.write("# pretend\n")
        assert is_valid_idf_path(td), "minimal IDF layout should pass"


def test_generate_app_build_mode_toggle():
    """Flipping the firmware-only toggle relabels the action button
    and disables Output. Locks down the dispatch behaviour without
    actually invoking idf.py (subprocess test would be flaky in CI)."""
    from espfoc_studio.gui.generate_app_panel import GenerateAppPanel

    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    reader = LinkReader(host_t)
    reader.start()
    try:
        client = TunerClient(reader)
        app = QApplication.instance() or QApplication(sys.argv)
        panel = GenerateAppPanel(client,
                                 get_motor_params=lambda: (1.08, 0.0018, 150.0))
        # Default mode = generate; button labelled accordingly.
        assert panel._generate_btn.text() == "Generate"
        assert panel._app_name.isEnabled()
        # Flip to firmware-only.
        panel._build_fw_box.setChecked(True)
        assert panel._generate_btn.text() == "Build firmware"
        assert not panel._app_name.isEnabled()
        # Flip back.
        panel._build_fw_box.setChecked(False)
        assert panel._generate_btn.text() == "Generate"
        assert panel._app_name.isEnabled()
    finally:
        fw.stop()
        reader.stop()


def test_scope_stream_timing_hw_and_demo():
    from espfoc_studio.gui.scope_stream_timing import (
        LOW_SPEED_DOWNSAMPLING,
        scope_uniform_dt_s,
    )

    dt_hw = scope_uniform_dt_s(20000.0, demo=False)
    assert abs(dt_hw - LOW_SPEED_DOWNSAMPLING / 20000.0) < 1e-15
    dt_demo = scope_uniform_dt_s(40000.0, demo=True)
    assert abs(dt_demo - (40.0 * 4.0) / 40000.0) < 1e-15


def test_scope_panel_roll_mode_x_axis_stays_bounded():
    """X is seconds within the visible window (0 = oldest on screen);
    values stay in [0, WINDOW_S] even if the panel has been running
    for a long time (t0 is not on the plot axis)."""
    from espfoc_studio.gui.scope_panel import ScopePanel

    app = QApplication.instance() or QApplication(sys.argv)
    panel = ScopePanel(async_decode=False)
    # Pretend the panel has been running for an hour and a frame
    # arrives one tick before "now".
    panel._t0 = time.monotonic() - 3600.0
    with panel._inbox_lock:
        panel._inbox.append((time.monotonic() - 0.010, _scope_bin_32(1.0, 2.0)))
        panel._inbox.append((
            time.monotonic() - 0.005, _scope_bin_32(1.5, 2.5)))
        panel._inbox.append((time.monotonic(), _scope_bin_32(2.0, 3.0)))
    panel._render_tick()
    assert panel._n_channels == 2
    panel._checkboxes[0].setChecked(True)
    panel._render_tick()
    # Pull the rendered X data from the first curve and check that
    # the latest sample sits at ~0 and nothing is at +3600 seconds.
    x_data, _ = panel._curves[0].getData()
    assert x_data is not None and len(x_data) == 3
    assert x_data[0] < 0.02, f"oldest sample at x={x_data[0]!r}, expected ~0"
    assert x_data[-1] > x_data[0]
    assert x_data[-1] < panel.WINDOW_S + 0.1, (
        f"newest x={x_data[-1]!r} past window")
    assert min(x_data) >= -0.01, "X should be non-negative (time from t_oldest)"


def main() -> int:
    tests = [test_mainwindow_demo_boots_polls_and_streams_scope,
             test_scope_panel_uniform_time_and_autoset,
             test_scope_stream_timing_hw_and_demo,
             test_scope_panel_roll_mode_x_axis_stays_bounded,
             test_generate_app_embeds_hardware_section,
             test_build_worker_idf_path_helper,
             test_generate_app_build_mode_toggle]
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
