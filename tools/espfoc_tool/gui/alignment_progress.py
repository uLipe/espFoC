"""Modal “alignment in progress” dialog with indeterminate bar + elapsed time."""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QShowEvent
from PySide6.QtWidgets import (
    QDialog,
    QFrame,
    QLabel,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)


class AlignmentProgressDialog(QDialog):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowFlags(
            Qt.Dialog | Qt.FramelessWindowHint | Qt.NoDropShadowWindowHint
        )
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setObjectName("alignmentProgressRoot")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(24, 24, 24, 24)

        card = QFrame()
        card.setObjectName("alignmentCard")
        card.setStyleSheet(
            "QFrame#alignmentCard {"
            " background: #2a2b30;"
            " border: 1px solid #3a3b3f;"
            " border-radius: 10px;"
            " }"
        )
        inner = QVBoxLayout(card)
        inner.setSpacing(12)
        inner.setContentsMargins(24, 20, 24, 20)

        title = QLabel("Alignment in progress, please wait…")
        title.setStyleSheet("font-size: 14px; font-weight: 600; color: #e6e6e6;")
        title.setWordWrap(True)
        inner.addWidget(title)

        hint = QLabel("The motor is detecting electrical angle and direction.")
        hint.setStyleSheet("font-size: 12px; color: #9aa0a6;")
        hint.setWordWrap(True)
        inner.addWidget(hint)

        self._bar = QProgressBar()
        self._bar.setTextVisible(False)
        self._bar.setRange(0, 0)
        self._bar.setFixedHeight(6)
        self._bar.setStyleSheet(
            "QProgressBar {"
            " border: 0; border-radius: 3px;"
            " background: #1e1f22;"
            " }"
            "QProgressBar::chunk {"
            " background: #4fc3f7; border-radius: 3px;"
            " }"
        )
        inner.addWidget(self._bar)

        self._elapsed = QLabel("Elapsed: 0.0 s")
        self._elapsed.setStyleSheet("font-size: 11px; color: #9aa0a6;")
        self._elapsed.setAlignment(Qt.AlignHCenter)
        inner.addWidget(self._elapsed)
        outer.addWidget(card)
        self._t0 = time.monotonic()
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()
        self.adjustSize()

    def _on_tick(self) -> None:
        dt = time.monotonic() - self._t0
        self._elapsed.setText(f"Elapsed: {dt:4.1f} s")

    def showEvent(self, event: QShowEvent) -> None:  # noqa: N802
        super().showEvent(event)
        w = self.parentWidget()
        if w is not None and w.isVisible():
            g = self.frameGeometry()
            g.moveCenter(w.frameGeometry().center())
            self.move(g.topLeft())

    def closeEvent(self, event) -> None:  # noqa: N802
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.stop()
        super().closeEvent(event)
