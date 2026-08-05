"""QML list model for active scope plots."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import QAbstractListModel, QModelIndex, Qt, Slot


class PlotEntry:
    __slots__ = ("plot_id", "channel_index", "title", "unit", "color", "points",
                 "cursor_label")

    def __init__(
            self,
            plot_id: int,
            channel_index: int,
            title: str,
            unit: str,
            color: str,
    ) -> None:
        self.plot_id = plot_id
        self.channel_index = channel_index
        self.title = title
        self.unit = unit
        self.color = color
        self.points: List[Dict[str, float]] = []
        self.cursor_label = ""


class PlotListModel(QAbstractListModel):
    PlotIdRole = Qt.UserRole + 1
    ChannelIndexRole = Qt.UserRole + 2
    TitleRole = Qt.UserRole + 3
    UnitRole = Qt.UserRole + 4
    ColorRole = Qt.UserRole + 5
    PointsRole = Qt.UserRole + 6
    CursorLabelRole = Qt.UserRole + 7

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._plots: List[PlotEntry] = []

    def roleNames(self) -> Dict[int, bytes]:
        return {
            self.PlotIdRole: b"plotId",
            self.ChannelIndexRole: b"channelIndex",
            self.TitleRole: b"plotTitle",
            self.UnitRole: b"unit",
            self.ColorRole: b"lineColor",
            self.PointsRole: b"points",
            self.CursorLabelRole: b"cursorLabel",
        }

    def rowCount(self, parent=QModelIndex()) -> int:
        if parent.isValid():
            return 0
        return len(self._plots)

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole) -> Any:
        if not index.isValid() or index.row() >= len(self._plots):
            return None
        p = self._plots[index.row()]
        if role == self.PlotIdRole:
            return p.plot_id
        if role == self.ChannelIndexRole:
            return p.channel_index
        if role == Qt.DisplayRole:
            return p.title
        if role == self.TitleRole:
            return p.title
        if role == self.UnitRole:
            return p.unit
        if role == self.ColorRole:
            return p.color
        if role == self.PointsRole:
            return p.points
        if role == self.CursorLabelRole:
            return p.cursor_label
        return None

    def active_channel_indices(self) -> List[int]:
        return [p.channel_index for p in self._plots]

    def find_by_plot_id(self, plot_id: int) -> Optional[PlotEntry]:
        for p in self._plots:
            if p.plot_id == plot_id:
                return p
        return None

    @Slot(int, str, str, str, result=int)
    def add_plot(self, channel_index: int, title: str, unit: str, color: str) -> int:
        plot_id = max((p.plot_id for p in self._plots), default=0) + 1
        row = len(self._plots)
        self.beginInsertRows(QModelIndex(), row, row)
        self._plots.append(PlotEntry(plot_id, channel_index, title, unit, color))
        self.endInsertRows()
        return plot_id

    @Slot(int)
    def remove_plot(self, plot_id: int) -> None:
        for i, p in enumerate(self._plots):
            if p.plot_id == plot_id:
                self.beginRemoveRows(QModelIndex(), i, i)
                del self._plots[i]
                self.endRemoveRows()
                return

    @Slot(int)
    def remove_plot_for_channel(self, channel_index: int) -> None:
        for i, p in enumerate(self._plots):
            if p.channel_index == channel_index:
                self.beginRemoveRows(QModelIndex(), i, i)
                del self._plots[i]
                self.endRemoveRows()
                return

    def update_points(self, plot_id: int, points: List[Dict[str, float]]) -> None:
        p = self.find_by_plot_id(plot_id)
        if p is None:
            return
        p.points = points
        for i, entry in enumerate(self._plots):
            if entry.plot_id == plot_id:
                idx = self.index(i)
                self.dataChanged.emit(idx, idx, [self.PointsRole])
                return

    def set_cursor_label(self, plot_id: int, label: str) -> None:
        p = self.find_by_plot_id(plot_id)
        if p is None or p.cursor_label == label:
            return
        p.cursor_label = label
        for i, entry in enumerate(self._plots):
            if entry.plot_id == plot_id:
                idx = self.index(i)
                self.dataChanged.emit(idx, idx, [self.CursorLabelRole])
                return
