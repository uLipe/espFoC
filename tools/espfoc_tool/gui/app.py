"""Qt application bootstrap: theme, OpenGL plots, optional GPU opt-out."""

from __future__ import annotations

import os
import sys

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QSurfaceFormat
from PySide6.QtWidgets import QApplication

from .theme import apply_dark_theme


def graphics_disabled() -> bool:
    v = os.environ.get("ESPFOC_TOOL_NO_GL", "").strip().lower()
    return v in ("1", "true", "yes", "on")


def configure_graphics() -> None:
    if graphics_disabled():
        pg.setConfigOptions(useOpenGL=False)
        return
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_UseDesktopOpenGL, True)
    fmt = QSurfaceFormat()
    fmt.setSamples(4)
    QSurfaceFormat.setDefaultFormat(fmt)
    pg.setConfigOptions(useOpenGL=True)


def create_application(argv: list[str] | None = None) -> QApplication:
    app = QApplication.instance()
    if app is None:
        app = QApplication(argv if argv is not None else sys.argv)
    configure_graphics()
    apply_dark_theme(app)
    return app
