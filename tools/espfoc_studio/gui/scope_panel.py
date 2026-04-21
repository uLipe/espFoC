"""Scope panel: live rolling time-series of the reference vs measurement.

In --demo mode we sample the DemoFirmware's plant state directly so the
GUI has something to show even without real scope streaming wired up.
Real-mode will eventually plug into the scope channel of the link codec."""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Deque, Optional

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class ScopePanel(QWidget):
    WINDOW_S = 2.0  # rolling window width (seconds)

    def __init__(self,
                 snapshot_source: Optional[Callable[[],
                                                    tuple[float, float, float]]]
                 = None) -> None:
        super().__init__()
        self._snapshot = snapshot_source
        self._t_origin: Optional[float] = None
        cap = 2000
        self._t: Deque[float] = deque(maxlen=cap)
        self._ref: Deque[float] = deque(maxlen=cap)
        self._meas: Deque[float] = deque(maxlen=cap)

        root = QVBoxLayout(self)

        if snapshot_source is None:
            banner = QLabel(
                "Scope is only wired in --demo mode right now. The real "
                "scope stream will land in a follow-up PR.")
            banner.setWordWrap(True)
            banner.setAlignment(Qt.AlignLeft)
            root.addWidget(banner)

        self._plot = pg.PlotWidget(title="Scope (demo plant)")
        self._plot.setLabel('left', "current", units='A')
        self._plot.setLabel('bottom', "time", units='s')
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.addLegend()
        self._ref_curve = self._plot.plot(pen=pg.mkPen('#888', width=2,
                                                       style=Qt.DashLine),
                                          name="target_iq")
        self._meas_curve = self._plot.plot(pen=pg.mkPen('#2196f3', width=2),
                                           name="i_q (plant)")
        root.addWidget(self._plot, 1)

    def poll(self) -> None:
        if self._snapshot is None:
            return
        ref, meas, now = self._snapshot()
        if self._t_origin is None:
            self._t_origin = now
        self._t.append(now - self._t_origin)
        self._ref.append(ref)
        self._meas.append(meas)
        # Drop samples older than WINDOW_S seconds.
        while self._t and (self._t[-1] - self._t[0]) > self.WINDOW_S:
            self._t.popleft()
            self._ref.popleft()
            self._meas.popleft()
        self._ref_curve.setData(list(self._t), list(self._ref))
        self._meas_curve.setData(list(self._t), list(self._meas))
