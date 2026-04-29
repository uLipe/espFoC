"""Scope panel: rolling time-series of every channel emitted by the
firmware's esp_foc_scope.

The firmware sends SCOPE v1 binary (Q16.16 as int32 LE) or, for older
builds, CSV floats on the SCOPE link channel. This panel:

* discovers the channel count from the first line (so any number of
  scope_add_channel() calls work without UI changes);
* draws one curve per channel with a deterministic palette;
* provides a checkbox per channel so the user can hide / show signals
  on the fly — useful when overlaying six SVPWM phases at once.
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

    Raw payloads go to a bounded inbox; a dedicated decode thread turns
    them into float tuples so the Qt timer only merges samples and
    redraws. Decoding off the GUI thread keeps the UI responsive when a
    burst of scope frames arrives."""

    WINDOW_S = 2.0         # rolling window length
    BUFFER_CAP = 4096      # max samples retained per channel
    RENDER_INTERVAL_MS = 20  # timer cadence; actual plot rate is bounded below
    INBOX_CAP = 512        # drop aggressively — prefer losing samples to GUI stalls
    MAX_PENDING_DECODED = 384  # decoded backlog cap before UI drops half (small = low latency)
    # Hard cap on GUI-thread merge work per tick. Prefer dropping samples over
    # blocking the Qt event loop (serial + scope together amplify jank).
    MAX_MERGE_SAMPLES_PER_TICK = 48
    # Decode worker takes at most this many raw frames per wake so one backlog
    # burst cannot monopolize CPU before the UI runs.
    MAX_RAW_FRAMES_DECODE_BATCH = 256
    MAX_FRAMES_PER_UI_TICK = 16  # SVM / Sensors: frames drained per tick on GUI thread

    def __init__(self, reader: Optional[LinkReader] = None,
                 sample_period_s: float = 1e-3 * 4,
                 async_decode: bool = True) -> None:
        """sample_period_s is kept for backwards compatibility but no
        longer drives the time axis — the panel now timestamps each
        frame on arrival with time.monotonic(), so the visible
        frequency is immune to inbox drops, render-tick batching, or
        the firmware shipping samples at a slightly different rate
        than advertised."""
        super().__init__()
        self._async_decode = async_decode
        self._reader = reader
        self._sample_dt = sample_period_s  # kept for backwards-compat only
        self._inbox_lock = threading.Lock()
        # Inbox holds (t_mono, payload) so the time axis is wall-clock
        # locked from the moment the reader thread sees the frame, not
        # an accumulating "samples seen so far" counter that lags or
        # leads the actual data rate.
        self._inbox: Deque[Tuple[float, bytes]] = deque(maxlen=self.INBOX_CAP)
        self._pending_lock = threading.Lock()
        self._pending_decoded: List[Tuple[float, Tuple[float, ...]]] = []
        self._worker_stop = threading.Event()
        self._decode_thread: Optional[threading.Thread] = None
        self._t0 = time.monotonic()
        self._time_buf: Deque[float] = deque(maxlen=self.BUFFER_CAP)
        self._channel_bufs: List[Deque[float]] = []
        self._curves: List[pg.PlotDataItem] = []
        self._checkboxes: List[QCheckBox] = []
        self._n_channels = 0

        root = QHBoxLayout(self)

        # Left gutter with the channel toggles + Autoset button.
        # Sized as min/max instead of fixed so the plot grabs all the
        # horizontal slack first when the window grows, but the gutter
        # never collapses below something readable.
        gutter = QFrame()
        gutter.setFrameShape(QFrame.NoFrame)
        gutter.setMinimumWidth(140)
        self._gutter_layout = QVBoxLayout(gutter)
        self._gutter_layout.setContentsMargins(4, 4, 4, 4)
        # "Autoset" lives at the top so it is always reachable even
        # when the channels list scrolls — clicking it re-enables
        # autorange on both axes, drops the buffered history and
        # rebases the time axis to "now". Use this whenever a manual
        # zoom / pan parked the viewport away from the live cursor.
        autoset_btn = QPushButton("Autoset")
        autoset_btn.setToolTip(
            "Re-center the scope: clear history, reset time origin "
            "to now and re-enable axis autorange.")
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

        # Plot: x is seconds within the rolling window, 0 = oldest sample
        # on screen, WINDOW_S ≈ newest (right).
        self._plot = pg.PlotWidget(title="Scope — firmware CSV stream")
        self._plot.setLabel('left', "amplitude")
        self._plot.setLabel('bottom', "time", units='s')
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setMinimumHeight(380)
        self._plot.setXRange(0.0, self.WINDOW_S, padding=0)
        self._plot.enableAutoRange(axis='x', enable=False)
        self._crosshair = attach_crosshair(
            self._plot,
            fmt=lambda x, y: f"t = {x:.3f} s\ny = {y:+.4g}")
        root.addWidget(self._plot, 1)

        # Render timer merges decoded samples and updates curves.
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

    # --- Public helpers ---------------------------------------------------

    def set_sample_period(self, dt_s: float) -> None:
        """No-op kept for backwards compatibility. The time axis is
        wall-clock-locked now; the firmware's actual scope rate does
        not need to be communicated to the panel any more."""
        if dt_s > 0:
            self._sample_dt = dt_s

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

    def autoset(self) -> None:
        """Drop history, rebase time to 'now' and snap the viewport
        back to the rolling window. Recovers from manual zoom / pan
        that parked the viewport off the live cursor."""
        with self._inbox_lock:
            self._inbox.clear()
        with self._pending_lock:
            self._pending_decoded.clear()
        self._t0 = time.monotonic()
        self._time_buf.clear()
        for buf in self._channel_bufs:
            buf.clear()
        for curve in self._curves:
            curve.setData([], [])
        self._plot.setXRange(0.0, self.WINDOW_S, padding=0)
        self._plot.enableAutoRange(axis='y', enable=True)

    # --- Frame path -------------------------------------------------------

    def _on_frame_reader_thread(self, channel: int, seq: int,
                                payload: bytes) -> None:
        t_mono = time.monotonic()
        with self._inbox_lock:
            self._inbox.append((t_mono, payload))

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
            while len(self._pending_decoded) > self.MAX_PENDING_DECODED:
                del self._pending_decoded[: len(self._pending_decoded) // 2]

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
            self._decode_raw_batch(batch)

    def _render_tick(self) -> None:
        if not self._async_decode:
            batch: List[Tuple[float, bytes]] = []
            with self._inbox_lock:
                n = len(self._inbox)
                if n > 0:
                    take = min(n, self.MAX_RAW_FRAMES_DECODE_BATCH)
                    batch = [self._inbox.popleft() for _ in range(take)]
            if batch:
                self._decode_raw_batch(batch)

        chunk: List[Tuple[float, Tuple[float, ...]]]
        with self._pending_lock:
            chunk = self._pending_decoded
            self._pending_decoded = []

        cap = self.MAX_MERGE_SAMPLES_PER_TICK
        if len(chunk) > cap:
            chunk = chunk[-cap:]

        for t_mono, values in chunk:
            self._ensure_channels(len(values))
            self._time_buf.append(t_mono - self._t0)
            for i, v in enumerate(values):
                self._channel_bufs[i].append(v)

        while self._time_buf and (self._time_buf[-1] - self._time_buf[0]
                                  > self.WINDOW_S):
            self._time_buf.popleft()
            for buf in self._channel_bufs:
                buf.popleft()

        if not self._time_buf:
            return
        t_rels = np.fromiter(self._time_buf, dtype=float,
                             count=len(self._time_buf))
        t_arr = t_rels - float(t_rels[0])
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
            cb.setChecked(False)
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
