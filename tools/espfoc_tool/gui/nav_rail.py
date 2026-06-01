"""Left navigation rail (Material-style list)."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QButtonGroup, QPushButton, QVBoxLayout, QWidget

from .theme import make_nav_button_qss


class NavRail(QWidget):
    page_selected = Signal(int)

    LABELS = ("Config", "Current", "Control", "States")

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("NavRail")
        self.setFixedWidth(132)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 12, 8, 12)
        lay.setSpacing(6)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: list[QPushButton] = []
        for i, label in enumerate(self.LABELS):
            b = QPushButton(label)
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setStyleSheet(make_nav_button_qss())
            if i == 0:
                b.setChecked(True)
            b.clicked.connect(lambda _c=False, idx=i: self.page_selected.emit(idx))
            self._group.addButton(b, i)
            self._buttons.append(b)
            lay.addWidget(b)
        lay.addStretch(1)

    def set_current_index(self, index: int) -> None:
        if 0 <= index < len(self._buttons):
            self._buttons[index].setChecked(True)
