"""Reusable crosshair + readout overlay for pyqtgraph PlotWidgets.

Attaches a pair of InfiniteLines that follow the mouse, plus a small
text label anchored to the upper-right corner of the plot showing the
current (x, y) value in data coordinates. Works with linear and log
scales and leaves the plot's normal interactions (pan, zoom) alone.
"""

from __future__ import annotations

from typing import Callable, Optional

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor


def _default_fmt(x: float, y: float) -> str:
    return f"x = {x:+.4g}\ny = {y:+.4g}"


class CrosshairOverlay:
    """Adds a crosshair + text readout to a single PlotWidget."""

    def __init__(self, plot: pg.PlotWidget,
                 fmt: Optional[Callable[[float, float], str]] = None,
                 color: str = "#9aa0a6") -> None:
        self._plot = plot
        self._fmt = fmt or _default_fmt
        pen = pg.mkPen(QColor(color), width=1, style=Qt.DashLine)

        self._vline = pg.InfiniteLine(angle=90, movable=False, pen=pen)
        self._hline = pg.InfiniteLine(angle=0, movable=False, pen=pen)
        for line in (self._vline, self._hline):
            line.setZValue(10)
            # Do not force the view to include the crosshair line —
            # otherwise the plot autoscales to follow the mouse.
            try:
                line.setFlag(line.GraphicsItemFlag.ItemIgnoresTransformations, False)
            except Exception:
                pass
            plot.addItem(line, ignoreBounds=True)

        self._label = pg.TextItem(
            text="", anchor=(0, 0),
            color=QColor("#e6e6e6"),
            fill=pg.mkBrush(QColor(0, 0, 0, 160)),
            border=pg.mkPen(QColor("#3a3b3f")))
        self._label.setZValue(11)
        # Positioned lazily the first time the mouse enters the plot.
        self._label_placed = False
        plot.addItem(self._label, ignoreBounds=True)

        self._hide_all()
        plot.scene().sigMouseMoved.connect(self._on_moved)

    def _hide_all(self) -> None:
        self._vline.hide()
        self._hline.hide()
        self._label.hide()

    def _show_all(self) -> None:
        self._vline.show()
        self._hline.show()
        self._label.show()

    def _place_label(self) -> None:
        """Anchor the text to the upper-right corner of the view in
        data coordinates. Called once the plot has a stable view box."""
        vb = self._plot.getPlotItem().getViewBox()
        rng = vb.viewRange()
        x_hi, y_hi = rng[0][1], rng[1][1]
        self._label.setPos(x_hi, y_hi)
        self._label.setAnchor((1.05, 0))
        self._label_placed = True

    def _on_moved(self, scene_pos) -> None:
        plot_item = self._plot.getPlotItem()
        vb = plot_item.getViewBox()
        if not vb.sceneBoundingRect().contains(scene_pos):
            self._hide_all()
            return
        pt = vb.mapSceneToView(scene_pos)
        x, y = float(pt.x()), float(pt.y())
        self._vline.setPos(x)
        self._hline.setPos(y)
        if not self._label_placed:
            self._place_label()
        # Re-anchor every tick so the label stays in the top-right
        # even as the view autoscales.
        rng = vb.viewRange()
        self._label.setPos(rng[0][1], rng[1][1])
        self._label.setText(self._fmt(x, y))
        self._show_all()

def attach_crosshair(plot: pg.PlotWidget,
                     fmt: Optional[Callable[[float, float], str]] = None,
                     color: str = "#9aa0a6") -> CrosshairOverlay:
    """Attach a CrosshairOverlay to *plot* and return it so the caller
    can keep a reference (otherwise Python would garbage-collect it
    and the connection would stop firing)."""
    return CrosshairOverlay(plot, fmt=fmt, color=color)
