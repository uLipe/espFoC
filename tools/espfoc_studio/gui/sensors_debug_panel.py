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
    QCheckBox,
    QFormLayout,
    QFrame,
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
from .scope_panel import ScopePanel

SENSOR_CH_FIRST = 6
SENSOR_N = 6
HOT_PATH_US_CH = 12
WINDOW_S = ScopePanel.WINDOW_S
BUFFER_CAP = ScopePanel.BUFFER_CAP
INBOX_CAP = ScopePanel.INBOX_CAP
MAX_PENDING_DECODED = ScopePanel.MAX_PENDING_DECODED
MAX_MERGE_SAMPLES_PER_TICK = ScopePanel.MAX_MERGE_SAMPLES_PER_TICK
MAX_RAW_FRAMES_DECODE_BATCH = ScopePanel.MAX_RAW_FRAMES_DECODE_BATCH
# Six stacked PlotWidgets: slightly lower FPS than Scope to keep repaints smooth.
SENSORS_RENDER_INTERVAL_MS = 33
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


def _wrap_counts(ch_f: float, cpr: int) -> float:
    """Scope float for ch6 == engineering encoder counts (same as q16/65536)."""
    if cpr <= 0:
        cpr = 1
    cf = float(cpr)
    x = ch_f % cf
    if x < 0.0:
        x += cf
    return x


def _eng_text(row: int, ch_f: float, cpr: int) -> str:
    if cpr <= 0:
        cpr = 1
    if row == 0:
        cnt = _wrap_counts(ch_f, cpr)
        deg = (cnt / float(cpr)) * 360.0
        return (
            f"θ_m = {deg:+.2f}°  ({cnt:.1f} counts / {cpr}, "
            f"q16 engineering counts)"
        )
    if row == 1:
        cps = ch_f
        rev_s = cps / float(cpr)
        rpm_m = rev_s * 60.0
        w_mech = rev_s * (2.0 * math.pi)
        return (
            f"dθ/dt = {cps:+.2f} counts/s  |  {rev_s:+.6f} rev/s mech  |  "
            f"{rpm_m:+.2f} rpm mech  |  ω_mech = {w_mech:+.4f} rad/s"
        )
    return f"{ch_f:+.5f} A"


def _plot_y(row: int, f: float, cpr: int) -> float:
    if cpr <= 0:
        cpr = 1
    if row == 0:
        cnt = _wrap_counts(f, cpr)
        return (cnt / float(cpr)) * 360.0
    if row == 1:
        return f
    return f


@dataclass(frozen=True)
class _RowSpec:
    title: str


def _all_specs() -> List[_RowSpec]:
    return [
        _RowSpec("Encoder counts (ch6 → rotor_shaft_ticks)"),
        _RowSpec("Velocity counts/s (ch7 → current_speed)"),
        _RowSpec("Phase current I_U (ch8)"),
        _RowSpec("Phase current I_V (ch9)"),
        _RowSpec("Iα Clarke (ch10)"),
        _RowSpec("Iβ Clarke (ch11)"),
    ]


class SensorsDebugPanel(QWidget):
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
        self._t0 = time.monotonic()
        self._time_buf: Deque[float] = deque(maxlen=BUFFER_CAP)
        self._plot_bufs: List[Deque[float]] = [
            deque(maxlen=BUFFER_CAP) for _ in range(SENSOR_N)]
        self._eng_labels: List[QLabel] = []
        self._raw_fields: List[QLineEdit] = []
        self._trace_cbs: List[QCheckBox] = []
        self._curves: List[pg.PlotDataItem] = []
        self._plots: List[pg.PlotWidget] = []
        self._active = True
        self._row_specs = _all_specs()

        root = QVBoxLayout(self)
        intro = (
            f"Requires SCOPE stream with at least 12 fields (13 for hot-path µs). "
            f"Ch{SENSOR_CH_FIRST}: encoder position as Q16 engineering counts "
            f"(e.g. AS5600 0…4095); ch{SENSOR_CH_FIRST + 1}: Δcounts×sample rate [counts/s]. "
            f"Set CPR below to convert to degrees / mech rpm. "
            f"Ch{SENSOR_CH_FIRST + 2}…: currents; ch{HOT_PATH_US_CH}: FOC hot-path [µs]. "
            f"Window: {WINDOW_S:.0f} s."
        )
        l0 = QLabel(intro)
        l0.setWordWrap(True)
        l0.setStyleSheet("color: #9aa0a6; font-size: 11px;")
        root.addWidget(l0)

        cpr_row = QHBoxLayout()
        cpr_row.addWidget(QLabel("Encoder CPR (counts/rev)"))
        self._cpr_spin = QSpinBox()
        self._cpr_spin.setRange(1, 65536)
        self._cpr_spin.setValue(4096)
        self._cpr_spin.setToolTip(
            "Must match firmware rotor sensor (AS5600: 4096, AS5048: 16384, …)."
        )
        cpr_row.addWidget(self._cpr_spin)
        cpr_row.addStretch(1)
        root.addLayout(cpr_row)

        hp = QHBoxLayout()
        hp.addWidget(QLabel("FOC hot path"))
        self._hot_path_us = QLineEdit()
        self._hot_path_us.setReadOnly(True)
        self._hot_path_us.setPlaceholderText("—")
        self._hot_path_us.setToolTip(
            f"Last channel (ch {HOT_PATH_US_CH}): execution time of the FOC hot "
            "path on the target (µs), from esp_foc_now_useconds().")
        hp.addWidget(self._hot_path_us, 1)
        root.addLayout(hp)

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
        self._render_timer.setInterval(SENSORS_RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start()
        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)
        if self._async_decode:
            self._decode_thread = threading.Thread(
                target=self._decode_worker_loop,
                daemon=True,
                name="espfoc-sensors-decode",
            )
            self._decode_thread.start()
        else:
            self._decode_thread = None

    def closeEvent(self, event) -> None:
        self._worker_stop.set()
        if self._decode_thread is not None:
            self._decode_thread.join(timeout=1.5)
        super().closeEvent(event)

    def _build_row(self, sp: _RowSpec) -> QWidget:
        fr = QFrame()
        v_l = QVBoxLayout(fr)
        head = QHBoxLayout()
        title = QLabel(sp.title)
        title.setStyleSheet("font-weight: 600;")
        head.addWidget(title)
        trace_cb = QCheckBox("Trace")
        trace_cb.setChecked(True)
        trace_cb.setToolTip(
            "Rolling strip for this channel. Uncheck to save CPU.")
        head.addWidget(trace_cb)
        head.addStretch(1)
        v_l.addLayout(head)
        self._trace_cbs.append(trace_cb)
        plot = pg.PlotWidget()
        if "Encoder" in sp.title:
            y_lab = "deg (from counts)"
        elif "Velocity" in sp.title:
            y_lab = "counts/s"
        else:
            y_lab = "A"
        plot.setLabel("left", y_lab)
        plot.setLabel("bottom", "time", units="s")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setMinimumHeight(PLOT_MIN_HEIGHT)
        plot.setXRange(0.0, WINDOW_S, padding=0)
        plot.enableAutoRange(axis="x", enable=False)
        plot.enableAutoRange(axis="y", enable=True)
        vb = plot.getViewBox()
        vb.setMouseEnabled(x=False, y=True)
        c = plot.plot(pen=pg.mkPen(QColor("#4fc3f7"), width=1.5))
        c.setClipToView(True)
        _ = attach_crosshair(
            plot, fmt=lambda x, y: f"t = {x:.3f} s\ny = {y:+.5g}")
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

    def set_counts_per_rev(self, cpr: int) -> None:
        """Optional: sync CPR from Generate App / hardware profile (clamped 1…65536)."""
        if cpr < 1:
            cpr = 1
        if cpr > 65536:
            cpr = 65536
        self._cpr_spin.setValue(cpr)

    def attach_reader(self, reader: LinkReader) -> None:
        if self._reader is not None and self._reader is not reader:
            try:
                self._reader.unregister_scope_callback(
                    self._on_frame_reader_thread)
            except ValueError:
                pass
            with self._inbox_lock:
                self._inbox.clear()
            with self._pending_lock:
                self._pending_decoded.clear()
        self._reader = reader
        reader.register_scope_callback(self._on_frame_reader_thread)

    def _decode_raw_batch(self, batch: List[Tuple[float, bytes]]) -> None:
        out: List[Tuple[float, Tuple[float, ...]]] = []
        for t_mono, payload in batch:
            try:
                values = decode_scope_payload_to_floats_csv_first(payload)
            except ValueError:
                continue
            if not values:
                continue
            out.append((t_mono, tuple(values)))
        if not out:
            return
        with self._pending_lock:
            self._pending_decoded.extend(out)
            while len(self._pending_decoded) > MAX_PENDING_DECODED:
                del self._pending_decoded[: len(self._pending_decoded) // 2]

    def _decode_worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            batch: List[Tuple[float, bytes]] = []
            with self._inbox_lock:
                n = len(self._inbox)
                if n > 0:
                    take = min(n, MAX_RAW_FRAMES_DECODE_BATCH)
                    batch = [self._inbox.popleft() for _ in range(take)]
            if not batch:
                if self._worker_stop.wait(0.003):
                    break
                continue
            self._decode_raw_batch(batch)

    def _on_frame_reader_thread(
            self, _c: int, _s: int, payload: bytes) -> None:
        t_mono = time.monotonic()
        with self._inbox_lock:
            self._inbox.append((t_mono, payload))

    def _render_tick(self) -> None:
        if not self._async_decode:
            batch: List[Tuple[float, bytes]] = []
            with self._inbox_lock:
                n = len(self._inbox)
                if n > 0:
                    take = min(n, MAX_RAW_FRAMES_DECODE_BATCH)
                    batch = [self._inbox.popleft() for _ in range(take)]
            if batch:
                self._decode_raw_batch(batch)

        chunk: List[Tuple[float, Tuple[float, ...]]]
        with self._pending_lock:
            chunk = self._pending_decoded
            self._pending_decoded = []

        cap = MAX_MERGE_SAMPLES_PER_TICK
        if len(chunk) > cap:
            chunk = chunk[-cap:]

        need = SENSOR_CH_FIRST + SENSOR_N
        last_vals: Optional[Tuple[float, ...]] = None
        for t_mono, vals in chunk:
            if len(vals) < need or not self._active:
                continue
            last_vals = vals
            t_rel = t_mono - self._t0
            self._time_buf.append(t_rel)
            cpr = self._cpr_spin.value()
            for r in range(SENSOR_N):
                f = vals[SENSOR_CH_FIRST + r]
                self._plot_bufs[r].append(_plot_y(r, f, cpr))

        if last_vals is not None and self._active:
            cpr = self._cpr_spin.value()
            for r in range(SENSOR_N):
                f = last_vals[SENSOR_CH_FIRST + r]
                self._raw_fields[r].setText(str(_float_to_q16_int(f)))
                self._eng_labels[r].setText(_eng_text(r, f, cpr))
            if len(last_vals) > HOT_PATH_US_CH:
                self._hot_path_us.setText(
                    f"{last_vals[HOT_PATH_US_CH]:.2f} µs")
            else:
                self._hot_path_us.clear()
                self._hot_path_us.setPlaceholderText("—")

        while (self._time_buf
               and (self._time_buf[-1] - self._time_buf[0] > WINDOW_S)):
            self._time_buf.popleft()
            for pb in self._plot_bufs:
                if pb:
                    pb.popleft()

        if not self._active:
            for p in self._plots:
                p.setXRange(0.0, WINDOW_S, padding=0)
            return

        if not self._time_buf:
            for c in self._curves:
                c.setData([], [])
            for p in self._plots:
                p.setXRange(0.0, WINDOW_S, padding=0)
            return

        t_rels = np.fromiter(
            self._time_buf, dtype=float, count=len(self._time_buf))
        t_arr = t_rels - float(t_rels[0])
        n = min(len(t_arr), min(len(b) for b in self._plot_bufs))
        if n <= 0:
            for c in self._curves:
                c.setData([], [])
            for p in self._plots:
                p.setXRange(0.0, WINDOW_S, padding=0)
            return
        t_arr = t_arr[-n:]
        for p in self._plots:
            p.setXRange(0.0, WINDOW_S, padding=0)
        for r, c in enumerate(self._curves):
            if self._trace_cbs[r].isChecked():
                y = np.fromiter(
                    list(self._plot_bufs[r])[-n:], dtype=float, count=n)
                c.setData(t_arr, y)
            else:
                c.setData([], [])

    def poll(self) -> None:
        pass
