"""Scope panel: rolling time-series of every channel emitted by the
firmware's esp_foc_scope.

The firmware encodes each sample as a single CSV line on the SCOPE
link channel (``"%f,%f,...\\n"``). This panel:

* discovers the channel count from the first line (so any number of
  scope_add_channel() calls work without UI changes);
* draws one curve per channel with a deterministic palette;
* provides a checkbox per channel so the user can hide / show signals
  on the fly — useful when overlaying six SVPWM phases at once.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, List, Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..link import LinkReader
from .crosshair import attach_crosshair


# 8-entry palette tuned for legibility on a dark pyqtgraph background.
_CHANNEL_COLORS = (
    "#4fc3f7",  # cyan
    "#ffb74d",  # amber
    "#81c784",  # green
    "#e57373",  # red
    "#ba68c8",  # purple
    "#f06292",  # pink
    "#aed581",  # lime
    "#fff176",  # yellow
)


class ScopePanel(QWidget):
    """Receives SCOPE frames from a LinkReader and plots them.

    Frames are pushed into a bounded inbox by the reader thread; a
    single timer on the Qt thread drains the inbox, parses all queued
    frames in one shot and redraws the curves once. Decoupling the
    firmware's scope rate (hundreds of Hz) from the GUI redraw rate
    keeps the UI snappy when a current step fires a burst of samples."""

    WINDOW_S = 2.0         # rolling window length
    BUFFER_CAP = 4096      # max samples retained per channel
    RENDER_INTERVAL_MS = 20  # 50 FPS render cap
    INBOX_CAP = 2048       # drop oldest when firmware outruns the GUI

    def __init__(self, reader: Optional[LinkReader] = None,
                 sample_period_s: float = 1e-3 * 4) -> None:
        """sample_period_s defaults to the DemoFirmware decimation
        (4 ms). Real firmware can override this by calling
        set_sample_period() once its scope rate is known."""
        super().__init__()
        self._reader = reader
        self._sample_dt = sample_period_s
        self._inbox_lock = threading.Lock()
        self._inbox: Deque[bytes] = deque(maxlen=self.INBOX_CAP)
        self._t = 0.0  # virtual clock (sample index * dt)
        self._time_buf: Deque[float] = deque(maxlen=self.BUFFER_CAP)
        self._channel_bufs: List[Deque[float]] = []
        self._curves: List[pg.PlotDataItem] = []
        self._checkboxes: List[QCheckBox] = []
        self._n_channels = 0

        root = QHBoxLayout(self)

        # Left gutter with the channel toggles.
        gutter = QFrame()
        gutter.setFrameShape(QFrame.NoFrame)
        gutter.setFixedWidth(140)
        self._gutter_layout = QVBoxLayout(gutter)
        self._gutter_layout.setContentsMargins(4, 4, 4, 4)
        self._gutter_layout.addWidget(QLabel("Channels"))
        self._gutter_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(150)
        scroll.setWidget(gutter)
        root.addWidget(scroll)

        # Plot.
        self._plot = pg.PlotWidget(title="Scope — firmware CSV stream")
        self._plot.setLabel('left', "amplitude")
        self._plot.setLabel('bottom', "time", units='s')
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._crosshair = attach_crosshair(
            self._plot,
            fmt=lambda x, y: f"t = {x:.3f} s\ny = {y:+.4g}")
        root.addWidget(self._plot, 1)

        # Render timer keeps the Qt thread independent from the reader
        # thread's frame rate. It drains the inbox and updates curves.
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(self.RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_tick)
        self._render_timer.start()

        if reader is not None:
            reader.register_scope_callback(self._on_frame_reader_thread)

    # --- Public helpers ---------------------------------------------------

    def set_sample_period(self, dt_s: float) -> None:
        """Override the estimated scope Ts (used for the x axis only)."""
        if dt_s > 0:
            self._sample_dt = dt_s

    def attach_reader(self, reader: LinkReader) -> None:
        self._reader = reader
        reader.register_scope_callback(self._on_frame_reader_thread)

    # --- Frame path -------------------------------------------------------

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        # Cheap hand-off to the Qt thread; no Qt call here.
        with self._inbox_lock:
            self._inbox.append(payload)

    def _render_tick(self) -> None:
        # Drain the inbox under the lock, then release so the reader
        # thread never waits on rendering.
        with self._inbox_lock:
            if not self._inbox:
                return
            pending = list(self._inbox)
            self._inbox.clear()

        # Ingest every pending frame into the per-channel buffers.
        for payload in pending:
            try:
                line = payload.decode("ascii", errors="ignore").strip()
                values = [float(tok) for tok in line.split(",") if tok]
            except ValueError:
                continue
            if not values:
                continue
            self._ensure_channels(len(values))
            self._t += self._sample_dt
            self._time_buf.append(self._t)
            for i, v in enumerate(values):
                self._channel_bufs[i].append(v)

        # Drop samples older than WINDOW_S in one sweep.
        while self._time_buf and (self._time_buf[-1] - self._time_buf[0]
                                  > self.WINDOW_S):
            self._time_buf.popleft()
            for buf in self._channel_bufs:
                buf.popleft()

        # Single redraw per timer tick.
        t_arr = np.fromiter(self._time_buf, dtype=float,
                            count=len(self._time_buf))
        for i, curve in enumerate(self._curves):
            if self._checkboxes[i].isChecked():
                y_arr = np.fromiter(self._channel_bufs[i], dtype=float,
                                    count=len(self._channel_bufs[i]))
                curve.setData(t_arr, y_arr)
            else:
                curve.setData([], [])

    # --- Channel lazy-init ------------------------------------------------

    def _ensure_channels(self, n: int) -> None:
        if n <= self._n_channels:
            return
        # Remove the spacer we added on construction so new widgets line
        # up cleanly.
        spacer = self._gutter_layout.takeAt(self._gutter_layout.count() - 1)
        for idx in range(self._n_channels, n):
            color = _CHANNEL_COLORS[idx % len(_CHANNEL_COLORS)]
            cb = QCheckBox(f"ch{idx}")
            cb.setChecked(True)
            cb.setStyleSheet(
                f"QCheckBox {{ color: {color}; font-family: monospace; "
                f"font-weight: bold; }}")
            self._gutter_layout.addWidget(cb)
            self._checkboxes.append(cb)
            curve = self._plot.plot(
                pen=pg.mkPen(color=QColor(color), width=2))
            self._curves.append(curve)
            self._channel_bufs.append(deque(maxlen=self.BUFFER_CAP))
        # Re-add the stretch at the bottom.
        if spacer is not None and spacer.spacerItem() is not None:
            self._gutter_layout.addItem(spacer)
        else:
            self._gutter_layout.addStretch(1)
        self._n_channels = n

    # --- Backwards-compatible polling hook --------------------------------

    def poll(self) -> None:
        """Older MainWindow called this from a timer. Now that scope
        frames are pushed directly from the reader, this is a no-op
        kept to keep the timer wiring happy."""
