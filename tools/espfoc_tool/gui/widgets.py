"""Shared layout primitives: page chrome, elevated cards, live metrics."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .theme import monospace_font


def spin_box(
    minimum: float,
    maximum: float,
    decimals: int,
    value: float,
    *,
    step: float = 0.1,
    suffix: str = "",
) -> QDoubleSpinBox:
    if minimum > maximum:
        minimum, maximum = maximum, minimum
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(suffix)
    box.setMinimumWidth(120)
    return box


def horizontal_splitter(
    *panes: QWidget,
    stretches: tuple[int, ...] | None = None,
    sizes: tuple[int, ...] | None = None,
) -> QSplitter:
    split = QSplitter(Qt.Horizontal)
    split.setChildrenCollapsible(False)
    split.setHandleWidth(6)
    for i, pane in enumerate(panes):
        split.addWidget(pane)
        if stretches is not None and i < len(stretches):
            split.setStretchFactor(i, stretches[i])
    if sizes is not None:
        split.setSizes(list(sizes))
    return split


def vertical_splitter(
    *panes: QWidget,
    stretches: tuple[int, ...] | None = None,
    sizes: tuple[int, ...] | None = None,
) -> QSplitter:
    split = QSplitter(Qt.Vertical)
    split.setChildrenCollapsible(False)
    split.setHandleWidth(6)
    for i, pane in enumerate(panes):
        split.addWidget(pane)
        if stretches is not None and i < len(stretches):
            split.setStretchFactor(i, stretches[i])
    if sizes is not None:
        split.setSizes(list(sizes))
    return split


class PageHeader(QWidget):
    def __init__(
        self,
        title: str,
        subtitle: str = "",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        t = QLabel(title)
        t.setObjectName("PageTitle")
        lay.addWidget(t)
        if subtitle.strip():
            s = QLabel(subtitle)
            s.setObjectName("PageSubtitle")
            s.setWordWrap(True)
            lay.addWidget(s)


class PageShell(QWidget):
    """Top title band + stretchable body (typical view wrapper)."""

    def __init__(
        self,
        title: str,
        subtitle: str,
        body: QWidget,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 18, 20, 16)
        root.setSpacing(14)
        root.addWidget(PageHeader(title, subtitle))
        root.addWidget(body, 1)


class SurfaceCard(QFrame):
    def __init__(
        self,
        title: str = "",
        subtitle: str = "",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("SurfaceCard")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 14, 16, 14)
        outer.setSpacing(10)
        if title:
            t = QLabel(title)
            t.setObjectName("CardTitle")
            outer.addWidget(t)
        if subtitle.strip():
            s = QLabel(subtitle)
            s.setObjectName("CardSubtitle")
            s.setWordWrap(True)
            outer.addWidget(s)
        self._body = QWidget()
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(8)
        outer.addWidget(self._body)

    @property
    def body_layout(self) -> QVBoxLayout:
        return self._body_layout


class LiveMetric(QWidget):
    def __init__(self, label: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)
        self.setObjectName("LiveMetric")
        cap = QLabel(label)
        cap.setObjectName("MetricCaption")
        self.value = QLabel("—")
        self.value.setObjectName("MetricValue")
        self.value.setFont(monospace_font(14))
        lay.addWidget(cap)
        lay.addWidget(self.value)

    def set_value(self, text: str) -> None:
        self.value.setText(text)


class LiveMetricGrid(QWidget):
    def __init__(self, labels: list[str], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        self._metrics: list[LiveMetric] = []
        cols = 2
        for i, lab in enumerate(labels):
            m = LiveMetric(lab)
            self._metrics.append(m)
            grid.addWidget(m, i // cols, i % cols)

    def metric(self, index: int) -> LiveMetric:
        return self._metrics[index]

    def set_values(self, texts: list[str]) -> None:
        for m, t in zip(self._metrics, texts):
            m.set_value(t)

