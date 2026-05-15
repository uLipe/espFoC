#!/usr/bin/env python3
"""Headless smoke tests for TunerStudio GUI pieces.

QT_QPA_PLATFORM=offscreen keeps CI headless.

Run:
    QT_QPA_PLATFORM=offscreen PYTHONPATH=tools \\
        python3 tools/espfoc_studio/tests/test_gui_smoke.py
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from PySide6.QtWidgets import QApplication

from espfoc_studio.link.scope_sample import pack_scope_i32_to_payload


def _scope_bin_32(*vals: float) -> bytes:
    q = [int(round(f * 65536.0)) for f in vals]
    return pack_scope_i32_to_payload(q)


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


def test_scope_stream_timing_hw():
    from espfoc_studio.gui.scope_stream_timing import (
        LOW_SPEED_DOWNSAMPLING,
        scope_uniform_dt_s,
    )

    dt_hw = scope_uniform_dt_s(20000.0)
    assert abs(dt_hw - LOW_SPEED_DOWNSAMPLING / 20000.0) < 1e-15


def test_scope_panel_roll_mode_x_axis_stays_bounded():
    """X is seconds within the visible window (0 = oldest on screen);
    values stay in [0, WINDOW_S] even if the panel has been running
    for a long time (t0 is not on the plot axis)."""
    from espfoc_studio.gui.scope_panel import ScopePanel

    app = QApplication.instance() or QApplication(sys.argv)
    panel = ScopePanel(async_decode=False)
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
    x_data, _ = panel._curves[0].getData()
    assert x_data is not None and len(x_data) == 3
    assert x_data[0] < 0.02, f"oldest sample at x={x_data[0]!r}, expected ~0"
    assert x_data[-1] > x_data[0]
    assert x_data[-1] < panel.WINDOW_S + 0.1, (
        f"newest x={x_data[-1]!r} past window")
    assert min(x_data) >= -0.01, "X should be non-negative (time from t_oldest)"


def main() -> int:
    tests = [
        test_scope_panel_uniform_time_and_autoset,
        test_scope_stream_timing_hw,
        test_scope_panel_roll_mode_x_axis_stays_bounded,
    ]
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
