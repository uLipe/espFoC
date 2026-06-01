"""States view: named scope channels (axis_tuning wire map, ch 0–13)."""

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
    QCheckBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..link import LinkReader
from ..link.scope_sample import decode_scope_payload_to_floats_csv_first
from .crosshair import attach_crosshair
from .plot_display import (
    configure_dynamic_curve,
    configure_rolling_time_xaxis,
    decimation_indices_peak_union,
    rolling_plot_x_upper,
)
from . import scope_constants as sc
from .scope_stream_timing import scope_uniform_dt_s

STATES_CH_FIRST = 0
STATES_N = 14
HOT_PATH_CH = 13
WINDOW_S = sc.WINDOW_S
BUFFER_CAP = sc.BUFFER_CAP
INBOX_CAP = sc.INBOX_CAP
MAX_PENDING_DECODED = sc.MAX_PENDING_DECODED
MAX_MERGE_SAMPLES_PER_TICK = sc.MAX_MERGE_SAMPLES_PER_TICK
MAX_RAW_FRAMES_DECODE_BATCH = sc.MAX_RAW_FRAMES_DECODE_BATCH
RENDER_INTERVAL_MS = 55
STRIP_DISPLAY_MAX_POINTS = 800
PLOT_MIN_HEIGHT = 160

AXIS_TUNING_LABELS = (
    "id target",
    "id meas",
    "iq target",
    "iq meas",
    "ud",
    "uq",
    "θ_meas mech",
    "θ_est mech",
    "ω_est mech",
    "PLL error",
    "iu",
    "iv",
    "iα",
    "FOC hot-path µs",
)


@dataclass(frozen=True)
class _RowSpec:
    title: str


def _all_specs() -> List[_RowSpec]:
    return [_RowSpec(f"ch{i}: {AXIS_TUNING_LABELS[i]}") for i in range(STATES_N)]


def _eng_text(row: int, ch_f: float, cpr: int) -> str:
    if row == 6:
        cnt = ch_f % float(max(cpr, 1))
        deg = (cnt / float(max(cpr, 1))) * 360.0
        return f"θ_meas = {deg:+.2f}° ({cnt:.1f} counts)"
    if row == 8:
        return f"ω_est = {ch_f:+.5f} (per-unit turns/s)"
    if row in (10, 11, 12):
        return f"{ch_f:+.5f} A"
    if row == HOT_PATH_CH:
        return f"{ch_f * 65536.0:+.1f} µs (q16→µs)"
    return f"{ch_f:+.5f}"


class StatesPanel(QWidget):
    def __init__(
            self,
            reader: Optional[LinkReader] = None,
            async_decode: bool = True) -> None:
        super().__init__()
        self._reader = reader
        self._async_decode = async_decode
        self._worker_stop = threading.Event()
        self._inbox_lock = threading.Lock()
        self._inbox: Deque[Tuple[float, bytes]] = deque(maxlen=INBOX_CAP)
        self._pending_lock = threading.Lock()
        self._pending_decoded: List[Tuple[float, Tuple[float, ...]]] = []
        self._decode_thread: Optional[threading.Thread] = None
        self._uniform_dt_s = scope_uniform_dt_s(20000.0)
        self._scope_synth_t = 0.0
        self._time_buf: Deque[float] = deque(maxlen=BUFFER_CAP)
        self._plot_bufs: List[Deque[float]] = [
            deque(maxlen=BUFFER_CAP) for _ in range(STATES_N)]
        self._eng_labels: List[QLabel] = []
        self._raw_fields: List[QLineEdit] = []
        self._trace_cbs: List[QCheckBox] = []
        self._curves: List[pg.PlotDataItem] = []
        self._plots: List[pg.PlotWidget] = []
        self._active = True
        self._row_specs = _all_specs()

        root = QVBoxLayout(self)
        intro = QLabel(
            "SCOPE channels per axis_tuning firmware map. "
            f"Window {WINDOW_S:.0f} s. Connect and start scope from firmware "
            "(auto on connect in a later build)."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet("color: #9aa0a6; font-size: 11px;")
        root.addWidget(intro)

        cpr_row = QHBoxLayout()
        cpr_row.addWidget(QLabel("Encoder CPR"))
        self._cpr_spin = QSpinBox()
        self._cpr_spin.setRange(1, 65536)
        self._cpr_spin.setValue(4096)
        cpr_row.addWidget(self._cpr_spin)
        cpr_row.addStretch(1)
        root.addLayout(cpr_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        body = QWidget()
        scroll.setWidget(body)
        body_lay = QVBoxLayout(body)
        for r, spec in enumerate(self._row_specs):
            body_lay.addWidget(self._make_strip(r, spec.title))
        body_lay.addStretch(1)
        root.addWidget(scroll, 1)

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start(RENDER_INTERVAL_MS)
        if reader is not None:
            self.attach_reader(reader)

    def _make_strip(self, row: int, title: str) -> QWidget:
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 4, 0, 8)
        head = QLabel(title)
        head.setStyleSheet("font-weight: 600; color: #e6e6e6;")
        lay.addWidget(head)
        plot = pg.PlotWidget()
        plot.setMinimumHeight(PLOT_MIN_HEIGHT)
        plot.showGrid(x=True, y=True, alpha=0.25)
        configure_rolling_time_xaxis(plot)
        curve = plot.plot(pen=pg.mkPen("#4fc3f7", width=1.5))
        attach_crosshair(plot)
        configure_dynamic_curve(curve)
        self._plots.append(plot)
        self._curves.append(curve)
        lay.addWidget(plot)
        form = QFormLayout()
        raw = QLineEdit()
        raw.setReadOnly(True)
        raw.setStyleSheet("font-family: monospace;")
        eng = QLabel("-")
        eng.setStyleSheet("color: #9aa0a6; font-size: 11px;")
        cb = QCheckBox("plot")
        cb.setChecked(row < 4)
        self._raw_fields.append(raw)
        self._eng_labels.append(eng)
        self._trace_cbs.append(cb)
        form.addRow("Q16", raw)
        form.addRow("value", eng)
        form.addRow(cb)
        lay.addLayout(form)
        return box

    def attach_reader(self, reader: Optional[LinkReader]) -> None:
        if self._reader is not None:
            self._reader.unregister_scope_callback(
                self._on_frame_reader_thread)
        self._reader = reader
        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)

    def set_interactive(self, on: bool) -> None:
        self._active = on

    def set_uniform_sample_period_s(self, dt_s: float) -> None:
        if dt_s > 0.0:
            self._uniform_dt_s = dt_s

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        if not self._active:
            return
        _ = channel, seq
        with self._inbox_lock:
            self._inbox.append((time.monotonic(), payload))

    def _render_tick(self) -> None:
        batch: List[Tuple[float, bytes]] = []
        with self._inbox_lock:
            while self._inbox:
                batch.append(self._inbox.popleft())
        if not batch:
            return
        cpr = self._cpr_spin.value()
        for _wall, payload in batch[-MAX_MERGE_SAMPLES_PER_TICK:]:
            vals = decode_scope_payload_to_floats_csv_first(payload)
            if len(vals) < STATES_CH_FIRST + STATES_N:
                continue
            self._scope_synth_t += self._uniform_dt_s
            self._time_buf.append(self._scope_synth_t)
            for r in range(STATES_N):
                f = vals[STATES_CH_FIRST + r]
                self._plot_bufs[r].append(f)
                q16 = int(round(f * 65536.0))
                self._raw_fields[r].setText(str(q16))
                self._eng_labels[r].setText(_eng_text(r, f, cpr))
        n = len(self._time_buf)
        if n < 2:
            return
        t_arr = np.fromiter(list(self._time_buf)[-n:], dtype=float, count=n)
        t_lo = max(0.0, self._scope_synth_t - WINDOW_S)
        for r, c in enumerate(self._curves):
            if not self._trace_cbs[r].isChecked():
                c.setData([], [])
                continue
            y = np.fromiter(list(self._plot_bufs[r])[-n:], dtype=float, count=n)
            mp = min(STRIP_DISPLAY_MAX_POINTS, len(t_arr))
            idx = decimation_indices_peak_union(y, mp)
            t_d = t_arr[idx]
            y_d = y[idx]
            c.setData(t_d, y_d)
            self._plots[r].setXRange(0.0, rolling_plot_x_upper(t_d, WINDOW_S),
                                     padding=0)
