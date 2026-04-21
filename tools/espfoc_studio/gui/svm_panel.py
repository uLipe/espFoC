"""Space-vector PWM hexagon view.

Reads the first three channels emitted by the firmware scope
(u_u / u_v / u_w) and draws:

* the active voltage vector (Vα, Vβ) as it moves inside the SVPWM
  hexagon, with a fading trail of recent positions;
* the three-phase time-series below the hexagon so you can correlate
  the waveform shape with the rotating vector.

Like ScopePanel, this view buffers frames in an inbox on the reader
thread and renders them on a QTimer so the GUI stays responsive even
when the firmware emits a burst of samples after a current step. The
SVM redraw is intentionally slower than the scope's (smooth enough
for the eye, light on CPU when the motor is running).
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
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
# Keep these in sync with ScopePanel's first three entries so a phase
# colour in the SVM view matches its counterpart in the scope tab.
_PHASE_COLORS = ("#4fc3f7", "#ffb74d", "#81c784")


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
    """Hexagon + live Vα/Vβ vector + fading trail + three-phase plot."""

    TRAIL_CAP = 600           # hexagon trail length, samples
    WAVEFORM_CAP = 1024       # waveform history, samples
    WAVEFORM_WINDOW_S = 0.5   # visible span in the three-phase plot
    AUTOSCALE_ALPHA = 0.04    # EWMA weight for hexagon radius
    RENDER_INTERVAL_MS = 50   # 20 FPS — smooth but lighter than the scope
    INBOX_CAP = 512           # bound the producer/consumer queue

    def __init__(self, reader: Optional[LinkReader] = None,
                 sample_period_s: float = 1e-3 * 4) -> None:
        super().__init__()
        self._reader = reader
        self._sample_dt = sample_period_s
        self._inbox_lock = threading.Lock()
        self._inbox: Deque[bytes] = deque(maxlen=self.INBOX_CAP)

        self._alpha_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._beta_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._time_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uu_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uv_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uw_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._t = 0.0

        self._hex_radius = 1.0
        self._last_ab: tuple[float, float] = (0.0, 0.0)

        root = QVBoxLayout(self)
        info = QLabel(
            "Space-vector PWM hexagon — reads channels 0/1/2 as "
            "u_u / u_v / u_w. The active vector (orange) and its fading "
            "trail (cyan) are computed on the fly from those three "
            "phases; the time-series below shows the waveform feeding "
            "the hexagon. Refresh rate is deliberately lower than the "
            "scope so the Qt event loop stays snappy during bursts.")
        info.setWordWrap(True)
        info.setAlignment(Qt.AlignLeft)
        root.addWidget(info)

        # --- Top half: hexagon + readout column --------------------------
        top_row = QHBoxLayout()
        root.addLayout(top_row, 2)

        self._plot = pg.PlotWidget(title="SVPWM voltage vector")
        self._plot.setAspectLocked(True)
        self._plot.setLabel('left', "β", units='V')
        self._plot.setLabel('bottom', "α", units='V')
        self._plot.showGrid(x=True, y=True, alpha=0.15)

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
        top_row.addWidget(self._plot, 1)

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
        top_row.addLayout(side, 0)

        # --- Bottom half: three-phase waveform ---------------------------
        self._wave_plot = pg.PlotWidget(title="Three-phase output")
        self._wave_plot.setLabel('left', "voltage", units='V')
        self._wave_plot.setLabel('bottom', "time", units='s')
        self._wave_plot.showGrid(x=True, y=True, alpha=0.2)
        self._wave_plot.addLegend()
        self._uu_curve = self._wave_plot.plot(
            pen=pg.mkPen(QColor(_PHASE_COLORS[0]), width=2), name="u_u (ch0)")
        self._uv_curve = self._wave_plot.plot(
            pen=pg.mkPen(QColor(_PHASE_COLORS[1]), width=2), name="u_v (ch1)")
        self._uw_curve = self._wave_plot.plot(
            pen=pg.mkPen(QColor(_PHASE_COLORS[2]), width=2), name="u_w (ch2)")
        root.addWidget(self._wave_plot, 1)

        # --- Render timer ------------------------------------------------
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(self.RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start()

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
        with self._inbox_lock:
            self._inbox.append(payload)

    def _render_tick(self) -> None:
        with self._inbox_lock:
            if not self._inbox:
                return
            pending = list(self._inbox)
            self._inbox.clear()

        for payload in pending:
            try:
                tokens = payload.decode("ascii",
                                       errors="ignore").strip().split(",")
                vals = [float(t) for t in tokens[:3] if t]
            except ValueError:
                continue
            if len(vals) < 3:
                continue
            u_u, u_v, u_w = vals[0], vals[1], vals[2]
            a, b = _clarke(u_u, u_v, u_w)
            self._alpha_buf.append(a)
            self._beta_buf.append(b)
            self._t += self._sample_dt
            self._time_buf.append(self._t)
            self._uu_buf.append(u_u)
            self._uv_buf.append(u_v)
            self._uw_buf.append(u_w)
            self._last_ab = (a, b)

        # Trim waveform window.
        while self._time_buf and (self._time_buf[-1] - self._time_buf[0]
                                  > self.WAVEFORM_WINDOW_S):
            self._time_buf.popleft()
            self._uu_buf.popleft()
            self._uv_buf.popleft()
            self._uw_buf.popleft()

        # Auto-scale the hexagon against the most recent magnitude.
        a, b = self._last_ab
        mag = (a * a + b * b) ** 0.5
        target = max(mag * 1.2, 1e-6)
        self._hex_radius = ((1.0 - self.AUTOSCALE_ALPHA) * self._hex_radius
                            + self.AUTOSCALE_ALPHA * target)
        if target > self._hex_radius:
            self._hex_radius = target
        self._redraw_hexagon()

        # Trail + arrow in the hexagon.
        t_a = np.fromiter(self._alpha_buf, dtype=float,
                          count=len(self._alpha_buf))
        t_b = np.fromiter(self._beta_buf, dtype=float,
                          count=len(self._beta_buf))
        self._trail_curve.setData(t_a, t_b)
        self._arrow_curve.setData([0.0, a], [0.0, b])
        self._head_scatter.setData(pos=np.array([[a, b]]))

        # Three-phase waveform.
        t_arr = np.fromiter(self._time_buf, dtype=float,
                            count=len(self._time_buf))
        self._uu_curve.setData(t_arr,
                               np.fromiter(self._uu_buf, dtype=float,
                                           count=len(self._uu_buf)))
        self._uv_curve.setData(t_arr,
                               np.fromiter(self._uv_buf, dtype=float,
                                           count=len(self._uv_buf)))
        self._uw_curve.setData(t_arr,
                               np.fromiter(self._uw_buf, dtype=float,
                                           count=len(self._uw_buf)))

        # Readout labels.
        self._alpha_label.setText(f"α = {a:+8.3f} V")
        self._beta_label.setText(f"β = {b:+8.3f} V")
        self._mag_label.setText(f"|V| = {mag:8.3f} V")
        if mag < 1e-6:
            self._sector_label.setText("sector: -")
        else:
            import math
            ang = math.atan2(b, a)
            if ang < 0:
                ang += 2.0 * math.pi
            sec = int(ang // (math.pi / 3.0)) + 1
            if sec > 6:
                sec = 6
            self._sector_label.setText(f"sector: {sec}")

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
