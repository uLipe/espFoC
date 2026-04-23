"""Sensors: Q16 raw readout, engineering value, and rolling subplots for
SCOPE channels 6..11 (tuner_studio_target wire convention). X-axis uses the
same 2 s rolling window as the main Scope view."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..link import LinkReader
from ..link.scope_sample import decode_scope_payload_to_floats_csv_first
from .crosshair import attach_crosshair
from .scope_panel import ScopePanel

SENSOR_CH_FIRST = 6
SENSOR_N = 6
WINDOW_S = ScopePanel.WINDOW_S
BUFFER_CAP = ScopePanel.BUFFER_CAP
INBOX_CAP = ScopePanel.INBOX_CAP
RENDER_INTERVAL_MS = ScopePanel.RENDER_INTERVAL_MS
# Tall strip chart; scroll the page to see numeric readouts below.
PLOT_MIN_HEIGHT = 220


def _float_to_q16_int(f: float) -> int:
    v = f * 65536.0
    vi = int(round(v))
    if vi > 0x7FFFFFFF:
        return 0x7FFFFFFF
    if vi < -0x80000000:
        return -0x80000000
    return vi


def _eng_text(row: int, ch_f: float) -> str:
    if row == 0:
        return f"θ_m = {ch_f * 360.0:+.2f}°  (0–1.0 = one mech. rev, q16)"
    if row == 1:
        w_rad = ch_f * (2.0 * math.pi)
        w_rpm = ch_f * 60.0
        return (
            f"ω = {w_rad:+.4f} rad/s  |  {w_rpm:+.2f} rpm  "
            f"(rev/s as q16, shaft fraction/s)"
        )
    return f"{ch_f:+.5f} A"


def _plot_y(row: int, f: float) -> float:
    if row == 0:
        return f * 360.0
    if row == 1:
        return f * (2.0 * math.pi)
    return f


@dataclass(frozen=True)
class _RowSpec:
    title: str


def _all_specs() -> List[_RowSpec]:
    return [
        _RowSpec("Encoder (shaft, ch6)"),
        _RowSpec("Velocity from encoder (ch7)"),
        _RowSpec("Phase current I_U (ch8)"),
        _RowSpec("Phase current I_V (ch9)"),
        _RowSpec("Iα Clarke (ch10)"),
        _RowSpec("Iβ Clarke (ch11)"),
    ]


class SensorsDebugPanel(QWidget):
    def __init__(self, reader: Optional[LinkReader] = None) -> None:
        super().__init__()
        self._reader = reader
        self._inbox_lock = threading.Lock()
        self._inbox: Deque[Tuple[float, bytes]] = deque(maxlen=INBOX_CAP)
        self._t0 = time.monotonic()
        self._time_buf: Deque[float] = deque(maxlen=BUFFER_CAP)
        self._plot_bufs: List[Deque[float]] = [
            deque(maxlen=BUFFER_CAP) for _ in range(SENSOR_N)]
        self._eng_labels: List[QLabel] = []
        self._raw_fields: List[QLineEdit] = []
        self._curves: List[pg.PlotDataItem] = []
        self._plots: List[pg.PlotWidget] = []
        self._active = True
        self._row_specs = _all_specs()

        root = QVBoxLayout(self)
        intro = (
            f"Requires SCOPE stream with at least 12 fields. Traces: degrees "
            f"({SENSOR_CH_FIRST}), rad/s ({SENSOR_CH_FIRST + 1}), A ({SENSOR_CH_FIRST + 2}…). "
            f"Time window: {WINDOW_S:.0f} s (same as Scope tab)."
        )
        l0 = QLabel(intro)
        l0.setWordWrap(True)
        l0.setStyleSheet("color: #9aa0a6; font-size: 11px;")
        root.addWidget(l0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        col = QVBoxLayout(body)
        for r in range(SENSOR_N):
            col.addWidget(self._build_row(self._row_specs[r]))
        col.addStretch(0)
        scroll.setWidget(body)
        root.addWidget(scroll, 1)

        self._render_timer = QTimer(self)
        self._render_timer.setInterval(RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start()
        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)

    def _build_row(self, sp: _RowSpec) -> QWidget:
        fr = QFrame()
        v_l = QVBoxLayout(fr)
        title = QLabel(sp.title)
        title.setStyleSheet("font-weight: 600;")
        v_l.addWidget(title)
        plot = pg.PlotWidget()
        y_lab = "deg" if "Encoder" in sp.title else (
            "rad/s" if "Velocity" in sp.title else "A")
        plot.setLabel("left", y_lab)
        plot.setLabel("bottom", "time", units="s")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setMinimumHeight(PLOT_MIN_HEIGHT)
        plot.setXRange(-WINDOW_S, 0.0, padding=0)
        plot.enableAutoRange(axis="x", enable=False)
        plot.enableAutoRange(axis="y", enable=True)
        vb = plot.getViewBox()
        vb.setMouseEnabled(x=False, y=True)
        c = plot.plot(pen=pg.mkPen(QColor("#4fc3f7"), width=1.5))
        _ = attach_crosshair(
            plot, fmt=lambda x, y: f"t = {x:+.3f} s\ny = {y:+.5g}")
        self._plots.append(plot)
        self._curves.append(c)
        v_l.addWidget(plot, 0)
        form = QFormLayout()
        r = QLineEdit()
        r.setReadOnly(True)
        r.setFixedWidth(220)
        form.addRow("Raw (Q16 int32)", r)
        e = QLabel("—")
        e.setTextInteractionFlags(
            Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        form.addRow("Value", e)
        v_l.addLayout(form)
        self._raw_fields.append(r)
        self._eng_labels.append(e)
        return fr

    def set_interactive(self, active: bool) -> None:
        self._active = active
        self.setEnabled(active)

    def attach_reader(self, reader: LinkReader) -> None:
        if self._reader is not None and self._reader is not reader:
            try:
                self._reader.unregister_scope_callback(
                    self._on_frame_reader_thread)
            except ValueError:
                pass
        self._reader = reader
        reader.register_scope_callback(self._on_frame_reader_thread)

    def _on_frame_reader_thread(
            self, _c: int, _s: int, payload: bytes) -> None:
        t_mono = time.monotonic()
        with self._inbox_lock:
            self._inbox.append((t_mono, payload))

    def _render_tick(self) -> None:
        with self._inbox_lock:
            if not self._inbox:
                return
            batch = list(self._inbox)
            self._inbox.clear()
        need = SENSOR_CH_FIRST + SENSOR_N
        last_vals: Optional[List[float]] = None
        for t_mono, payload in batch:
            try:
                vals = decode_scope_payload_to_floats_csv_first(payload)
            except ValueError:
                continue
            if len(vals) < need or not self._active:
                continue
            last_vals = vals
            t_rel = t_mono - self._t0
            self._time_buf.append(t_rel)
            for r in range(SENSOR_N):
                f = vals[SENSOR_CH_FIRST + r]
                self._plot_bufs[r].append(_plot_y(r, f))
        if last_vals is not None and self._active:
            for r in range(SENSOR_N):
                f = last_vals[SENSOR_CH_FIRST + r]
                self._raw_fields[r].setText(str(_float_to_q16_int(f)))
                self._eng_labels[r].setText(_eng_text(r, f))
        while (self._time_buf
               and (self._time_buf[-1] - self._time_buf[0] > WINDOW_S)):
            self._time_buf.popleft()
            for pb in self._plot_bufs:
                if pb:
                    pb.popleft()
        if not self._time_buf or not self._active:
            for p in self._plots:
                p.setXRange(-WINDOW_S, 0.0, padding=0)
            return
        t_now_rel = time.monotonic() - self._t0
        t_arr = (np.fromiter(
            self._time_buf, dtype=float, count=len(self._time_buf)
        ) - t_now_rel)
        n = min(len(t_arr), min(len(b) for b in self._plot_bufs))
        if n <= 0:
            for c in self._curves:
                c.setData([], [])
            for p in self._plots:
                p.setXRange(-WINDOW_S, 0.0, padding=0)
            return
        t_arr = t_arr[-n:]
        for p in self._plots:
            p.setXRange(-WINDOW_S, 0.0, padding=0)
        for r, c in enumerate(self._curves):
            y = np.fromiter(
                list(self._plot_bufs[r])[-n:], dtype=float, count=n)
            c.setData(t_arr, y)

    def poll(self) -> None:
        pass
