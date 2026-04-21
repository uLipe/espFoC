"""Space-vector PWM hexagon view.

Reads the first three channels emitted by the firmware scope (u_u,
u_v, u_w) and renders them in the alpha/beta plane next to the six
SVPWM base vectors. Each frame adds a new dot to a fading trail so
you can literally watch the voltage vector rotate inside the
hexagon during modulation.

The view never touches the tuner protocol and subscribes to the
shared LinkReader via register_scope_callback, so it coexists with
the ScopePanel on the same scope stream.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from ..link import LinkReader


_HEX_COLOR = "#6e7681"
_CIRCLE_COLOR = "#3a3b3f"
_VERTEX_COLOR = "#ffcc66"
_TRAIL_COLOR = "#4fc3f7"
_ARROW_COLOR = "#ff7043"
_LABEL_COLOR = "#9aa0a6"


def _clarke(u_u: float, u_v: float, u_w: float) -> tuple[float, float]:
    """Amplitude-invariant Clarke transform.

    alpha = (2*u_u - u_v - u_w) / 3
    beta  = (u_v - u_w) / sqrt(3)
    """
    inv_sqrt3 = 0.5773502691896258
    alpha = (2.0 * u_u - u_v - u_w) / 3.0
    beta = (u_v - u_w) * inv_sqrt3
    return alpha, beta


class SvmPanel(QWidget):
    """Hexagon + live Vα/Vβ vector + fading trail."""

    TRAIL_CAP = 600          # fading-trail length, samples
    AUTOSCALE_ALPHA = 0.04   # EWMA weight for the hexagon radius

    _newFrame = Signal(object)  # bytes (payload)

    def __init__(self, reader: Optional[LinkReader] = None) -> None:
        super().__init__()
        self._reader = reader
        self._lock = threading.Lock()
        self._alpha_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._beta_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._hex_radius = 1.0   # dynamically tracked
        self._last_ab: tuple[float, float] = (0.0, 0.0)

        root = QVBoxLayout(self)
        info = QLabel(
            "Space-vector PWM hexagon — reads channels 0/1/2 as "
            "u_u / u_v / u_w. The active vector (orange) and its fading "
            "trail (cyan) are computed on the fly from those three "
            "phases; the hexagon auto-scales to the observed modulation.")
        info.setWordWrap(True)
        info.setAlignment(Qt.AlignLeft)
        root.addWidget(info)

        plot_row = QHBoxLayout()
        root.addLayout(plot_row, 1)

        self._plot = pg.PlotWidget(title="SVPWM voltage vector")
        self._plot.setAspectLocked(True)
        self._plot.setLabel('left', "β", units='V')
        self._plot.setLabel('bottom', "α", units='V')
        self._plot.showGrid(x=True, y=True, alpha=0.15)

        # Static layer: hexagon outline + inscribed circle + base vectors.
        self._hex_curve = self._plot.plot(
            pen=pg.mkPen(QColor(_HEX_COLOR), width=2))
        self._circle_curve = self._plot.plot(
            pen=pg.mkPen(QColor(_CIRCLE_COLOR), width=1,
                         style=Qt.DashLine))
        self._vertex_scatter = pg.ScatterPlotItem(
            size=10, symbol='o',
            pen=pg.mkPen(QColor(_VERTEX_COLOR)),
            brush=pg.mkBrush(QColor(_VERTEX_COLOR)))
        self._plot.addItem(self._vertex_scatter)

        # Dynamic layer: fading trail + current vector arrow.
        self._trail_curve = self._plot.plot(
            pen=pg.mkPen(QColor(_TRAIL_COLOR), width=1))
        self._arrow_curve = self._plot.plot(
            pen=pg.mkPen(QColor(_ARROW_COLOR), width=3))
        self._head_scatter = pg.ScatterPlotItem(
            size=14, symbol='o',
            pen=pg.mkPen(QColor(_ARROW_COLOR), width=2),
            brush=pg.mkBrush(QColor(_ARROW_COLOR)))
        self._plot.addItem(self._head_scatter)

        self._redraw_hexagon()
        plot_row.addWidget(self._plot, 1)

        # Live numeric readout on the side.
        side = QVBoxLayout()
        side.setContentsMargins(6, 6, 6, 6)
        side.setSpacing(4)
        self._alpha_label = QLabel("α = 0.000 V")
        self._beta_label = QLabel("β = 0.000 V")
        self._mag_label = QLabel("|V| = 0.000 V")
        self._sector_label = QLabel("sector: -")
        for lbl in (self._alpha_label, self._beta_label,
                    self._mag_label, self._sector_label):
            lbl.setStyleSheet("font-family: monospace; color: %s;"
                              % _LABEL_COLOR)
            side.addWidget(lbl)
        side.addStretch(1)
        plot_row.addLayout(side, 0)

        self._newFrame.connect(self._on_frame_main_thread)
        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)

    # --- Subscription helpers --------------------------------------------

    def attach_reader(self, reader: LinkReader) -> None:
        self._reader = reader
        reader.register_scope_callback(self._on_frame_reader_thread)

    def detach(self) -> None:
        if self._reader is not None:
            self._reader.unregister_scope_callback(
                self._on_frame_reader_thread)
            self._reader = None

    # --- Frame path -------------------------------------------------------

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        self._newFrame.emit(payload)

    def _on_frame_main_thread(self, payload: bytes) -> None:
        try:
            tokens = payload.decode("ascii", errors="ignore").strip().split(",")
            vals = [float(t) for t in tokens[:3] if t]
        except ValueError:
            return
        if len(vals) < 3:
            return
        u_u, u_v, u_w = vals[0], vals[1], vals[2]
        a, b = _clarke(u_u, u_v, u_w)
        self._alpha_buf.append(a)
        self._beta_buf.append(b)
        self._last_ab = (a, b)

        mag = (a * a + b * b) ** 0.5
        # EWMA on hexagon radius to avoid jittery rescaling. Add 20 %
        # headroom so vertices stay outside the trail.
        target = max(mag * 1.2, 1e-6)
        self._hex_radius = ((1.0 - self.AUTOSCALE_ALPHA) * self._hex_radius
                            + self.AUTOSCALE_ALPHA * target)
        # If the trail already exceeds the current hexagon, snap to fit.
        if target > self._hex_radius:
            self._hex_radius = target

        self._redraw_hexagon()
        self._redraw_trail_and_arrow(a, b, mag)

    # --- Static geometry --------------------------------------------------

    def _redraw_hexagon(self) -> None:
        R = self._hex_radius
        angles = np.arange(7) * (np.pi / 3.0)  # close the polygon
        hx = R * np.cos(angles)
        hy = R * np.sin(angles)
        self._hex_curve.setData(hx, hy)
        theta = np.linspace(0, 2.0 * np.pi, 181)
        rc = R * (np.sqrt(3.0) / 2.0)
        self._circle_curve.setData(rc * np.cos(theta), rc * np.sin(theta))
        vx = R * np.cos(angles[:-1])
        vy = R * np.sin(angles[:-1])
        self._vertex_scatter.setData(pos=np.column_stack((vx, vy)))

    def _redraw_trail_and_arrow(self, a: float, b: float, mag: float) -> None:
        t_a = np.fromiter(self._alpha_buf, dtype=float,
                          count=len(self._alpha_buf))
        t_b = np.fromiter(self._beta_buf, dtype=float,
                          count=len(self._beta_buf))
        self._trail_curve.setData(t_a, t_b)
        self._arrow_curve.setData([0.0, a], [0.0, b])
        self._head_scatter.setData(pos=np.array([[a, b]]))
        self._alpha_label.setText(f"α = {a:+8.3f} V")
        self._beta_label.setText(f"β = {b:+8.3f} V")
        self._mag_label.setText(f"|V| = {mag:8.3f} V")
        # Sector 1..6 based on angle (0..2π).
        if mag < 1e-6:
            sector_text = "sector: -"
        else:
            import math
            angle = math.atan2(b, a)
            if angle < 0:
                angle += 2.0 * math.pi
            sector = int(angle // (math.pi / 3.0)) + 1
            if sector > 6:
                sector = 6
            sector_text = f"sector: {sector}"
        self._sector_label.setText(sector_text)
