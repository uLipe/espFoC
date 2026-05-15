"""Scope panel: rolling time-series of every channel emitted by the
firmware's esp_foc_scope.

Decoded samples go into a **long ring buffer**; the plot shows a window
that lags the newest sample by :attr:`ScopePanel.DISPLAY_LAG_S` so USB
bursts are absorbed instead of discarded. Eviction drops only history
older than ``window + lag + margin``. Optional :meth:`set_live_priority`
shortens the lag while the GUI runs blocking tuner traffic.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
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
from .scope_stream_timing import scope_uniform_dt_s


_CHANNEL_COLORS = (
    "#4fc3f7",
    "#ffb74d",
    "#81c784",
    "#e57373",
    "#ba68c8",
    "#f06292",
    "#aed581",
    "#fff176",
)


class ScopePanel(QWidget):
    WINDOW_S = 2.0
    RENDER_INTERVAL_MS = 55
    MAX_DISPLAY_POINTS_PER_CURVE = 1600
    INBOX_CAP = 8192
    MAX_RAW_FRAMES_DECODE_BATCH = 2048
    MAX_FRAMES_PER_UI_TICK = 2048
    # Legacy names for SensorsDebugPanel (strip charts still use pending merge).
    BUFFER_CAP = 4096
    MAX_PENDING_DECODED = 8192
    MAX_MERGE_SAMPLES_PER_TICK = 8192

    RING_MAX_SAMPLES = 200_000
    DISPLAY_LAG_S = 0.35
    LIVE_PRIORITY_LAG_S = 0.05
    EVICT_MARGIN_S = 1.5

    def __init__(self, reader: Optional[LinkReader] = None,
                 sample_period_s: float = 1e-3 * 4,
                 async_decode: bool = True) -> None:
        super().__init__()
        self._async_decode = async_decode
        self._reader = reader
        self._sample_dt = sample_period_s
        self._uniform_dt_s = scope_uniform_dt_s(20000.0)
        self._scope_synth_t = 0.0
        self._display_lag_s = float(self.DISPLAY_LAG_S)
        self._live_priority = False

        self._history_lock = threading.Lock()
        self._history: Deque[Tuple[float, Tuple[float, ...]]] = deque(
            maxlen=self.RING_MAX_SAMPLES)

        self._inbox_lock = threading.Lock()
        self._inbox: Deque[Tuple[float, bytes]] = deque(maxlen=self.INBOX_CAP)
        self._worker_stop = threading.Event()
        self._decode_thread: Optional[threading.Thread] = None
        self._t0 = time.monotonic()
        self._curves: List[pg.PlotDataItem] = []
        self._checkboxes: List[QCheckBox] = []
        self._n_channels = 0

        self._x_buf_a: Optional[np.ndarray] = None
        self._x_buf_b: Optional[np.ndarray] = None
        self._y_bufs_a: List[Optional[np.ndarray]] = []
        self._y_bufs_b: List[Optional[np.ndarray]] = []
        self._ping_pong = False

        root = QHBoxLayout(self)

        gutter = QFrame()
        gutter.setFrameShape(QFrame.NoFrame)
        gutter.setMinimumWidth(140)
        self._gutter_layout = QVBoxLayout(gutter)
        self._gutter_layout.setContentsMargins(4, 4, 4, 4)
        autoset_btn = QPushButton("Autoset")
        autoset_btn.setToolTip(
            "Clear ring buffer and synthetic time; re-enable Y autorange.")
        autoset_btn.clicked.connect(self.autoset)
        self._gutter_layout.addWidget(autoset_btn)
        self._gutter_layout.addWidget(QLabel("Channels"))
        self._gutter_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(160)
        scroll.setMaximumWidth(220)
        scroll.setWidget(gutter)
        root.addWidget(scroll)

        self._plot = pg.PlotWidget(title="Scope — firmware CSV stream")
        self._plot.setLabel('left', "amplitude")
        self._plot.setLabel('bottom', "time", units='s')
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setMinimumHeight(380)
        configure_rolling_time_xaxis(self._plot)
        self._plot.setXRange(0.0, self.WINDOW_S, padding=0)
        self._plot.enableAutoRange(axis='x', enable=False)
        self._crosshair = attach_crosshair(
            self._plot,
            fmt=lambda x, y: f"t = {x:.3f} s\ny = {y:+.4g}")
        root.addWidget(self._plot, 1)

        self._render_timer = QTimer(self)
        self._render_timer.setInterval(self.RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start()

        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)

        if self._async_decode:
            self._decode_thread = threading.Thread(
                target=self._decode_worker_loop,
                daemon=True,
                name="espfoc-scope-decode",
            )
            self._decode_thread.start()
        else:
            self._decode_thread = None

    def closeEvent(self, event) -> None:
        self._worker_stop.set()
        if self._decode_thread is not None:
            self._decode_thread.join(timeout=1.5)
        super().closeEvent(event)

    def _effective_lag_s(self) -> float:
        if self._live_priority:
            return min(self._display_lag_s, self.LIVE_PRIORITY_LAG_S)
        return self._display_lag_s

    def _playback_head_s(self, t_newest: float, t_oldest: float) -> float:
        """Replay head = newest minus lag; if lag exceeds buffered span, show up to newest."""
        h = t_newest - self._effective_lag_s()
        if h < t_oldest:
            return t_newest
        return h

    def set_sample_period(self, dt_s: float) -> None:
        if dt_s > 0:
            self.set_uniform_sample_period_s(dt_s)

    def set_uniform_sample_period_s(self, dt_s: float) -> None:
        if dt_s > 1e-9:
            self._uniform_dt_s = float(dt_s)
            self._sample_dt = self._uniform_dt_s

    def set_display_lag_s(self, lag_s: float) -> None:
        """Plot trails newest sample by *lag_s* (absorbs transport bursts)."""
        if lag_s >= 0.0:
            self._display_lag_s = float(lag_s)

    def set_live_priority(self, active: bool) -> None:
        """Shorten lag during blocking GUI→target traffic (e.g. NVS save)."""
        self._live_priority = bool(active)

    def attach_reader(self, reader: LinkReader) -> None:
        if self._reader is not None and self._reader is not reader:
            try:
                self._reader.unregister_scope_callback(
                    self._on_frame_reader_thread)
            except ValueError:
                pass
            with self._inbox_lock:
                self._inbox.clear()
            with self._history_lock:
                self._history.clear()
                self._scope_synth_t = 0.0
        self._reader = reader
        reader.register_scope_callback(self._on_frame_reader_thread)

    def autoset(self) -> None:
        with self._inbox_lock:
            self._inbox.clear()
        with self._history_lock:
            self._history.clear()
            self._scope_synth_t = 0.0
        self._t0 = time.monotonic()
        self._x_buf_a = self._x_buf_b = None
        self._y_bufs_a = []
        self._y_bufs_b = []
        self._ping_pong = False
        for curve in self._curves:
            curve.setData([], [])
        self._plot.setXRange(0.0, self.WINDOW_S, padding=0)
        self._plot.enableAutoRange(axis='y', enable=True)

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        t_mono = time.monotonic()
        with self._inbox_lock:
            self._inbox.append((t_mono, payload))

    def _decode_batch_values(self, batch: List[Tuple[float, bytes]]
                             ) -> List[Tuple[float, ...]]:
        out: List[Tuple[float, ...]] = []
        for _t_mono, payload in batch:
            try:
                values = decode_scope_payload_to_floats_csv_first(payload)
            except ValueError:
                continue
            if not values:
                continue
            out.append(tuple(values))
        return out

    def _flush_rows_to_ring(self, rows: List[Tuple[float, ...]]) -> None:
        if not rows:
            return
        dt = self._uniform_dt_s
        with self._history_lock:
            for vals in rows:
                self._history.append((self._scope_synth_t, vals))
                self._scope_synth_t += dt
            self._evict_old_locked()

    def _evict_old_locked(self) -> None:
        if not self._history:
            return
        t_newest = self._history[-1][0]
        t_oldest = self._history[0][0]
        t_play = self._playback_head_s(t_newest, t_oldest)
        t_cut = t_play - self.WINDOW_S - self.EVICT_MARGIN_S
        while self._history and self._history[0][0] < t_cut:
            self._history.popleft()

    def _decode_worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            batch: List[Tuple[float, bytes]] = []
            with self._inbox_lock:
                n = len(self._inbox)
                if n > 0:
                    take = min(n, self.MAX_RAW_FRAMES_DECODE_BATCH)
                    batch = [self._inbox.popleft() for _ in range(take)]
            if not batch:
                if self._worker_stop.wait(0.003):
                    break
                continue
            rows = self._decode_batch_values(batch)
            self._flush_rows_to_ring(rows)

    def _render_tick(self) -> None:
        if not self._async_decode:
            batch: List[Tuple[float, bytes]] = []
            with self._inbox_lock:
                n = len(self._inbox)
                if n > 0:
                    take = min(n, self.MAX_RAW_FRAMES_DECODE_BATCH)
                    batch = [self._inbox.popleft() for _ in range(take)]
            if batch:
                self._flush_rows_to_ring(self._decode_batch_values(batch))

        with self._history_lock:
            if not self._history:
                return
            t_newest = self._history[-1][0]
            t_oldest = self._history[0][0]
            t_play = self._playback_head_s(t_newest, t_oldest)
            t_lo = t_play - self.WINDOW_S
            xs: List[float] = []
            ycols: Optional[List[List[float]]] = None
            for t_s, vals in self._history:
                if t_s < t_lo:
                    continue
                if t_s > t_play:
                    break
                xs.append(t_s)
                if ycols is None:
                    ycols = [[] for _ in range(len(vals))]
                nc = len(ycols)
                for i in range(nc):
                    v = float(vals[i]) if i < len(vals) else 0.0
                    ycols[i].append(v)

        if not xs or ycols is None:
            return
        max_ch = len(ycols)
        self._ensure_channels(max_ch)
        t_arr0 = np.asarray(xs, dtype=np.float64)
        t_arr = t_arr0 - float(t_arr0[0])

        use_a = not self._ping_pong
        self._ping_pong = not self._ping_pong
        if use_a:
            self._x_buf_a = t_arr
            x_plot = self._x_buf_a
        else:
            self._x_buf_b = t_arr
            x_plot = self._x_buf_b

        n = int(x_plot.shape[0])
        mp = self.MAX_DISPLAY_POINTS_PER_CURVE
        y_for_peak: List[np.ndarray] = []
        for i in range(min(len(ycols), len(self._checkboxes))):
            if self._checkboxes[i].isChecked():
                y_for_peak.append(np.asarray(ycols[i], dtype=np.float64))
        if n <= mp or not y_for_peak:
            dec_idx: Optional[np.ndarray] = None
            x_dec = x_plot
        else:
            dec_idx = decimation_indices_peak_union(y_for_peak, mp)
            x_dec = x_plot[dec_idx]

        for i, curve in enumerate(self._curves):
            if i >= len(ycols) or not self._checkboxes[i].isChecked():
                curve.setData([], [])
                continue
            y_arr = np.asarray(ycols[i], dtype=np.float64)
            y_dec = y_arr if dec_idx is None else y_arr[dec_idx]
            if use_a:
                while len(self._y_bufs_a) <= i:
                    self._y_bufs_a.append(None)
                self._y_bufs_a[i] = y_dec
            else:
                while len(self._y_bufs_b) <= i:
                    self._y_bufs_b.append(None)
                self._y_bufs_b[i] = y_dec
            curve.setData(x_dec, y_dec)

        x_up = rolling_plot_x_upper(x_dec, self.WINDOW_S)
        self._plot.setXRange(0.0, x_up, padding=0)

    def _ensure_channels(self, n: int) -> None:
        if n <= self._n_channels:
            return
        spacer = self._gutter_layout.takeAt(self._gutter_layout.count() - 1)
        for idx in range(self._n_channels, n):
            color = _CHANNEL_COLORS[idx % len(_CHANNEL_COLORS)]
            cb = QCheckBox(f"ch{idx}")
            cb.setChecked(False)
            cb.setStyleSheet(
                f"QCheckBox {{ color: {color}; font-family: monospace; "
                f"font-weight: bold; }}")
            self._gutter_layout.addWidget(cb)
            self._checkboxes.append(cb)
            curve = self._plot.plot(
                pen=pg.mkPen(color=QColor(color), width=2))
            configure_dynamic_curve(curve)
            self._curves.append(curve)
        if spacer is not None and spacer.spacerItem() is not None:
            self._gutter_layout.addItem(spacer)
        else:
            self._gutter_layout.addStretch(1)
        self._n_channels = n

    def poll(self) -> None:
        pass
