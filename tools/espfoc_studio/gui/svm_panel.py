"""Space-vector PWM hexagon view.

Reads channels 0/1/2 (u_u, u_v, u_w) from the firmware scope stream
and shows:

* three fixed phase axes (A/B/C at 0°/120°/240°) with a colored
  "phase projection" vector each — their length is the instantaneous
  u_u / u_v / u_w, so you see the three components animate along
  their axes as the electrical angle sweeps;
* the resultant V_αβ vector (Clarke sum of the three phases) with a
  fading trail so the rotation on the hexagon is visible at a glance;
* the three-phase time-series underneath, colored identically to the
  scope tab's channels 0/1/2.

Rendering is buffered on the reader side and flushed at 20 FPS so
scope bursts after a current step do not stall the Qt loop.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..link import LinkReader
from .crosshair import attach_crosshair


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
        """sample_period_s is kept for backwards compatibility but no
        longer drives the time axis — see ScopePanel for the same
        wall-clock-locking rationale."""
        super().__init__()
        self._reader = reader
        self._sample_dt = sample_period_s  # backwards-compat only
        self._inbox_lock = threading.Lock()
        # Same (t_mono, payload) pattern as ScopePanel so the waveform
        # frequency tracks real time even when frames arrive bursty.
        self._inbox: Deque[Tuple[float, bytes]] = deque(maxlen=self.INBOX_CAP)

        self._alpha_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._beta_buf: Deque[float] = deque(maxlen=self.TRAIL_CAP)
        self._time_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uu_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uv_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._uw_buf: Deque[float] = deque(maxlen=self.WAVEFORM_CAP)
        self._t0 = time.monotonic()

        self._hex_radius = 1.0
        self._last_ab: tuple[float, float] = (0.0, 0.0)

        root = QVBoxLayout(self)

        # --- Top half: hexagon + readout column --------------------------
        top_row = QHBoxLayout()
        root.addLayout(top_row, 2)

        self._plot = pg.PlotWidget(title="SVPWM voltage vector")
        self._plot.setAspectLocked(True)
        self._plot.setLabel('left', "β", units='V')
        self._plot.setLabel('bottom', "α", units='V')
        self._plot.showGrid(x=True, y=True, alpha=0.15)
        self._hex_crosshair = attach_crosshair(
            self._plot,
            fmt=lambda x, y: f"α = {x:+.3f}\nβ = {y:+.3f}")

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

        # Three static phase axes (A at 0°, B at 120°, C at 240°) — the
        # "rails" along which the instantaneous phase values project.
        # Drawn in a neutral dim colour so they guide the eye without
        # competing with the bright phase-projection vectors that sit
        # on top of them.
        self._phase_axis_curves = tuple(
            self._plot.plot(pen=pg.mkPen(QColor("#555759"), width=1,
                                          style=Qt.DashLine))
            for _ in _PHASE_COLORS)
        # Dynamic phase projection vectors: length = u_u / u_v / u_w,
        # direction = phase-axis unit vector. These carry the per-phase
        # colour so you can tell ch0/ch1/ch2 apart at a glance.
        self._phase_vec_curves = tuple(
            self._plot.plot(pen=pg.mkPen(QColor(c), width=3))
            for c in _PHASE_COLORS)
        self._phase_vec_tips = pg.ScatterPlotItem(
            size=9, symbol='o', pen=None)
        # Prime with three points at origin so pyqtgraph accepts the
        # per-point brush list (3 items) — positions are updated every
        # render tick.
        self._phase_vec_tips.setData(
            pos=np.zeros((3, 2)),
            brush=[pg.mkBrush(QColor(c)) for c in _PHASE_COLORS])
        self._plot.addItem(self._phase_vec_tips)

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
        autoset_btn = QPushButton("Autoset")
        autoset_btn.setToolTip(
            "Re-center the hexagon and the waveform: clear trail, "
            "rebase time to now and re-enable axis autorange.")
        autoset_btn.clicked.connect(self.autoset)
        side.addWidget(autoset_btn)
        side.addStretch(1)
        top_row.addLayout(side, 0)

        # --- Bottom half: three-phase waveform ---------------------------
        self._wave_plot = pg.PlotWidget(title="Three-phase output")
        self._wave_plot.setLabel('left', "voltage", units='V')
        self._wave_plot.setLabel('bottom', "time", units='s')
        self._wave_plot.showGrid(x=True, y=True, alpha=0.2)
        self._wave_plot.addLegend()
        self._wave_crosshair = attach_crosshair(
            self._wave_plot,
            fmt=lambda x, y: f"t = {x:.4f} s\nu = {y:+.3f} V")
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

    def autoset(self) -> None:
        """Drop trail / waveform history, rebase time to 'now' and
        re-enable autorange on both plots. Hex radius reverts to its
        initial guess so the auto-scale EWMA can re-converge cleanly."""
        with self._inbox_lock:
            self._inbox.clear()
        self._t0 = time.monotonic()
        for buf in (self._alpha_buf, self._beta_buf,
                    self._time_buf, self._uu_buf, self._uv_buf, self._uw_buf):
            buf.clear()
        self._hex_radius = 1.0
        self._last_ab = (0.0, 0.0)
        self._trail_curve.setData([], [])
        self._arrow_curve.setData([0.0, 0.0], [0.0, 0.0])
        self._head_scatter.setData(pos=np.array([[0.0, 0.0]]))
        for curve in (self._uu_curve, self._uv_curve, self._uw_curve):
            curve.setData([], [])
        self._wave_plot.enableAutoRange(axis='x', enable=True)
        self._wave_plot.enableAutoRange(axis='y', enable=True)
        self._plot.enableAutoRange(axis='x', enable=True)
        self._plot.enableAutoRange(axis='y', enable=True)

    # --- Frame path -------------------------------------------------------

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        t_mono = time.monotonic()
        with self._inbox_lock:
            self._inbox.append((t_mono, payload))

    def _render_tick(self) -> None:
        with self._inbox_lock:
            if not self._inbox:
                return
            pending = list(self._inbox)
            self._inbox.clear()

        for t_mono, payload in pending:
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
            self._time_buf.append(t_mono - self._t0)
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

        # Phase projection arrows (u_u along A axis, u_v along B, u_w
        # along C). The three arrow tips sum (scaled by 2/3) to the
        # resultant V — a live Clarke transform in picture form.
        if self._uu_buf and self._uv_buf and self._uw_buf:
            phases = (self._uu_buf[-1],
                      self._uv_buf[-1],
                      self._uw_buf[-1])
            tip_positions = []
            for curve, (dx, dy), mag in zip(self._phase_vec_curves,
                                            self._PHASE_DIRS, phases):
                tip = (mag * dx, mag * dy)
                curve.setData([0.0, tip[0]], [0.0, tip[1]])
                tip_positions.append(tip)
            self._phase_vec_tips.setData(pos=np.array(tip_positions))

        # Trail + resultant arrow (drawn on top).
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

    # Phase-axis unit vectors (A=0°, B=120°, C=240°). Dashed rails in the
    # static layer; phase projection arrows in the dynamic layer sit on
    # these directions with length = instantaneous u_u / u_v / u_w.
    _PHASE_DIRS = (
        (1.0, 0.0),
        (-0.5, 0.86602540378),       # cos(120°), sin(120°)
        (-0.5, -0.86602540378),      # cos(240°), sin(240°)
    )

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
        # Phase axes span -R .. +R along each direction; the negative
        # half shows projection when a phase goes below zero.
        for curve, (dx, dy) in zip(self._phase_axis_curves, self._PHASE_DIRS):
            curve.setData([-R * dx, R * dx], [-R * dy, R * dy])
