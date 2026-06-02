#!/usr/bin/env python3
"""Headless smoke tests for espFoC Tool GUI."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["ESPFOC_TOOL_NO_GL"] = "1"

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from PySide6.QtWidgets import QApplication

from espfoc_tool.gui.app import create_application
from espfoc_tool.gui.connection_manager import ConnectionManager
from espfoc_tool.gui.main_window import MainWindow
from espfoc_tool.gui.scope_stream_timing import (
    LOW_SPEED_DOWNSAMPLING,
    scope_uniform_dt_s,
)


def test_main_window_offline_smoke():
    app = create_application()
    conn = ConnectionManager(fixed_port=None)
    w = MainWindow(conn, title="espFoC Tool test")
    assert w._stack.count() == 2
    from PySide6.QtWidgets import QLabel
    titles = [lb.text() for lb in w.findChildren(QLabel)
              if lb.objectName() == "PageTitle"]
    assert "Tune" in titles
    assert "Dashboard" in titles
    w.close()
    conn.stop()


def test_scope_stream_timing_hw():
    dt_hw = scope_uniform_dt_s(20000.0)
    assert abs(dt_hw - LOW_SPEED_DOWNSAMPLING / 20000.0) < 1e-15


def main() -> int:
    tests = [
        test_main_window_offline_smoke,
        test_scope_stream_timing_hw,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"OK    {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
