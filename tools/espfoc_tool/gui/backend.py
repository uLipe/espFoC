"""QML backend: serial stream + plot models."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot, QUrl

from ..catalog.axis_shell_17ch import BY_INDEX, CHANNELS, EXPECTED_N_CH
from ..stream.frame import ScopeFrame
from ..stream.reader import StreamReader, list_serial_ports
from ..stream.ring import ChannelStore
from .plot_model import PlotListModel
from .plot_util import (
    PLOT_POINTS_ABSOLUTE_MAX,
    clamp_plot_settings,
    decimate_minmax_xy,
    fixed_time_axis,
    plot_window_seconds,
    target_display_points,
)

_COLOR_PALETTE = (
    "#4fc3f7", "#ff7043", "#66bb6a", "#ab47bc",
    "#ffca28", "#26c6da", "#ec407a", "#8d6e63",
)


class ScopeBackend(QObject):
    navigateTo = Signal(str)
    requestPortDialog = Signal()
    requestAddChannelDialog = Signal()
    requestPlotSettingsDialog = Signal()

    statusTextChanged = Signal()
    splashMessageChanged = Signal()
    portListChanged = Signal()
    channelCatalogChanged = Signal()
    plotSettingsChanged = Signal()
    plotAxisChanged = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._status_text = ""
        self._splash_message = "starting"
        self._logo_url = self._resolve_logo_url()
        self._port_list: List[str] = []
        self._default_color = _COLOR_PALETTE[0]
        self._plot_model = PlotListModel(self)
        self._store = ChannelStore()
        self._reader: Optional[StreamReader] = None
        self._t0 = 0.0
        self._next_color = 0
        self._pending_port: Optional[str] = None
        self._pending_baud = 921600
        self._cli_port: Optional[str] = None
        self._cli_baud = 921600
        self._plot_sample_rate_hz = 100
        self._plot_max_points = 480
        self._plot_time_min = 0.0
        self._plot_time_max = plot_window_seconds(
            self._plot_sample_rate_hz, self._plot_max_points)

    def set_cli_port(self, port: Optional[str], baud: int) -> None:
        self._cli_port = port
        self._cli_baud = baud

    @Property(str, constant=True)
    def logoUrl(self) -> str:
        return self._logo_url

    @Property(str, notify=statusTextChanged)
    def statusText(self) -> str:
        return self._status_text

    @Property(str, notify=splashMessageChanged)
    def splashMessage(self) -> str:
        return self._splash_message

    @Property("QVariantList", notify=portListChanged)
    def portList(self) -> List[str]:
        return self._port_list

    @Property(QObject, constant=True)
    def plotModel(self) -> PlotListModel:
        return self._plot_model

    @Property("QVariantList", constant=True)
    def colorPalette(self) -> List[str]:
        return list(_COLOR_PALETTE)

    @Property(str, constant=True)
    def defaultPlotColor(self) -> str:
        return self._default_color

    @Property(int, notify=plotSettingsChanged)
    def plotSampleRateHz(self) -> int:
        return self._plot_sample_rate_hz

    @Property(int, notify=plotSettingsChanged)
    def plotMaxPoints(self) -> int:
        return self._plot_max_points

    @Property(int, constant=True)
    def plotPointsAbsoluteMax(self) -> int:
        return PLOT_POINTS_ABSOLUTE_MAX

    @Property(float, notify=plotSettingsChanged)
    def plotTimeWindowS(self) -> float:
        return plot_window_seconds(self._plot_sample_rate_hz, self._plot_max_points)

    @Property(float, notify=plotAxisChanged)
    def plotTimeMin(self) -> float:
        return self._plot_time_min

    @Property(float, notify=plotAxisChanged)
    def plotTimeMax(self) -> float:
        return self._plot_time_max

    @Slot(int, result=str)
    def channelPlotLabel(self, channel_index: int) -> str:
        desc = BY_INDEX.get(channel_index)
        if desc is None:
            return f"Channel {channel_index}"
        return desc.name

    @Slot(int, result=str)
    def channelUnit(self, channel_index: int) -> str:
        desc = BY_INDEX.get(channel_index)
        return desc.unit if desc is not None else ""

    @Property("QVariantList", notify=channelCatalogChanged)
    def channelCatalog(self) -> List[Dict]:
        in_use = set(self._plot_model.active_channel_indices())
        out = []
        for ch in CHANNELS:
            out.append({
                "index": ch.index,
                "name": ch.name,
                "unit": ch.unit,
                "inUse": ch.index in in_use,
            })
        return out

    @staticmethod
    def _resolve_logo_url() -> str:
        base = Path(__file__).resolve().parents[3] / "doc" / "images"
        for name in ("espfoc_tool_logo.svg", "espfoc_tool_logo.png"):
            p = base / name
            if p.is_file():
                return QUrl.fromLocalFile(str(p)).toString()
        return ""

    def _set_status(self, text: str) -> None:
        frames, gaps = self._store.stats()
        window_s = plot_window_seconds(self._plot_sample_rate_hz, self._plot_max_points)
        self._status_text = (
            f"{text}  |  window={window_s:.2f}s  "
            f"{self._plot_sample_rate_hz}Hz  pts≤{self._plot_max_points}  "
            f"frames={frames}  seq_gaps={gaps}")
        self.statusTextChanged.emit()

    def _set_splash(self, msg: str) -> None:
        self._splash_message = msg
        self.splashMessageChanged.emit()

    def _update_time_axis(self, t_max: float) -> None:
        window_s = plot_window_seconds(self._plot_sample_rate_hz, self._plot_max_points)
        t_min, t_hi = fixed_time_axis(t_max, window_s)
        if t_min != self._plot_time_min or t_hi != self._plot_time_max:
            self._plot_time_min = t_min
            self._plot_time_max = t_hi
            self.plotAxisChanged.emit()

    def _current_t_max(self) -> float:
        t_max = 0.0
        for ch_idx in self._plot_model.active_channel_indices():
            t, _ = self._store.snapshot(ch_idx)
            if t:
                t_max = max(t_max, t[-1])
        if self._reader is not None:
            t_max = max(t_max, time.monotonic() - self._t0)
        return t_max

    @Slot()
    def startup(self) -> None:
        self.navigateTo.emit("splash")
        self._set_splash("starting")
        self.refreshPorts()
        QTimer.singleShot(3000, self, self._continue_after_splash)

    @Slot()
    def _continue_after_splash(self) -> None:
        if self._cli_port:
            self.connectPort(self._cli_port, self._cli_baud)
        else:
            self.requestPortDialog.emit()

    @Slot()
    def refreshPorts(self) -> None:
        self._port_list = list_serial_ports()
        self.portListChanged.emit()

    @Slot(str, int)
    def connectPort(self, port: str, baud: int) -> None:
        port = (port or "").strip()
        if not port:
            return
        self._reader = StreamReader(
            port, baud, expected_n_ch=EXPECTED_N_CH, on_frames=self._on_frames)
        self._t0 = time.monotonic()
        self._reader.start()
        self._set_status(f"RX {port} @ {baud}")
        self.navigateTo.emit("main")
        if self._plot_model.rowCount() == 0:
            self.addPlot(2, _COLOR_PALETTE[0])
            self.addPlot(3, _COLOR_PALETTE[1])

    @Slot()
    def openAddChannelDialog(self) -> None:
        self.channelCatalogChanged.emit()
        self.requestAddChannelDialog.emit()

    @Slot()
    def openPlotSettingsDialog(self) -> None:
        self.requestPlotSettingsDialog.emit()

    @Slot(int, int)
    def applyPlotSettings(self, sample_rate_hz: int, max_points: int) -> None:
        sr, mp = clamp_plot_settings(sample_rate_hz, max_points)
        if sr == self._plot_sample_rate_hz and mp == self._plot_max_points:
            return
        self._plot_sample_rate_hz = sr
        self._plot_max_points = mp
        self.plotSettingsChanged.emit()
        self.refresh_plots()

    @Slot(int, str)
    def addPlot(self, channel_index: int, color: str) -> None:
        desc = BY_INDEX.get(channel_index)
        if desc is None:
            return
        self._plot_model.add_plot(
            channel_index, desc.name, desc.unit, color or self._default_color)
        self.channelCatalogChanged.emit()

    @Slot(int)
    def removePlot(self, plot_id: int) -> None:
        self._plot_model.remove_plot(plot_id)
        self.channelCatalogChanged.emit()

    @Slot(int)
    def removePlotForChannel(self, channel_index: int) -> None:
        self._plot_model.remove_plot_for_channel(channel_index)
        self.channelCatalogChanged.emit()

    @Slot(int, float, float)
    def setCursorForPlot(self, plot_id: int, x_val: float, y_val: float) -> None:
        entry = self._plot_model.find_by_plot_id(plot_id)
        if entry is None:
            return
        self._plot_model.set_cursor_label(
            plot_id, f"t={x_val:.4f} s  y={y_val:.4f} {entry.unit}")

    @Slot(int)
    def clearCursorForPlot(self, plot_id: int) -> None:
        self._plot_model.set_cursor_label(plot_id, "")

    @Slot()
    def autoset(self) -> None:
        self._store.clear()
        self._t0 = time.monotonic()
        window_s = plot_window_seconds(self._plot_sample_rate_hz, self._plot_max_points)
        self._plot_time_min = 0.0
        self._plot_time_max = window_s
        self.plotAxisChanged.emit()
        for row in range(self._plot_model.rowCount()):
            idx = self._plot_model.index(row)
            plot_id = self._plot_model.data(idx, PlotListModel.PlotIdRole)
            self._plot_model.update_points(int(plot_id), [])
            self._plot_model.set_cursor_label(int(plot_id), "")
        self.refresh_plots()

    def _on_frames(self, frames: list) -> None:
        now = time.monotonic() - self._t0
        indices = self._plot_model.active_channel_indices()
        if not indices:
            return
        for fr in frames:
            if not isinstance(fr, ScopeFrame):
                continue
            self._store.note_frame(fr.seq, now, fr.samples_q16, indices)

    def refresh_plots(self) -> None:
        if self._reader and self._reader.error:
            self._set_status(f"error: {self._reader.error}")
            return

        window_s = plot_window_seconds(self._plot_sample_rate_hz, self._plot_max_points)
        t_max = self._current_t_max()
        axis_min, axis_max = fixed_time_axis(t_max, window_s)
        self._update_time_axis(t_max)

        max_pts = target_display_points(
            window_s, self._plot_sample_rate_hz, self._plot_max_points)

        for row in range(self._plot_model.rowCount()):
            idx = self._plot_model.index(row)
            plot_id = self._plot_model.data(idx, PlotListModel.PlotIdRole)
            ch_idx = self._plot_model.data(idx, PlotListModel.ChannelIndexRole)
            desc = BY_INDEX.get(int(ch_idx))
            if desc is None:
                continue
            t, raw = self._store.snapshot(int(ch_idx))
            if not t:
                self._plot_model.update_points(int(plot_id), [])
                continue
            y = np.array([desc.to_display(int(v)) for v in raw], dtype=np.float64)
            x = np.array(t, dtype=np.float64)
            mask = (x >= axis_min) & (x <= axis_max)
            x = x[mask]
            y = y[mask]
            if x.size > max_pts:
                x, y = decimate_minmax_xy(x, y, max_pts)
            points = [{"x": float(xi), "y": float(yi)} for xi, yi in zip(x, y)]
            self._plot_model.update_points(int(plot_id), points)
        self._set_status("live")

    def stop(self) -> None:
        if self._reader is not None:
            self._reader.stop()
            self._reader = None
