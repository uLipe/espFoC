"""Qt application bootstrap (QML + Material)."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication, QIcon
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle


def _gui_dir() -> Path:
    return Path(__file__).resolve().parent


def _window_icon() -> QIcon | None:
    base = _gui_dir().parents[2] / "doc" / "images"
    for name in ("espfoc_tool_logo.png",):
        p = base / name
        if p.is_file():
            return QIcon(str(p))
    return None


def create_qml_engine(scope_backend) -> QQmlApplicationEngine:
    QQuickStyle.setStyle("Material")
    app = QGuiApplication.instance()
    if app is None:
        raise RuntimeError("QGuiApplication required before engine")
    icon = _window_icon()
    if icon is not None:
        app.setWindowIcon(icon)

    engine = QQmlApplicationEngine()
    gui = _gui_dir()
    qml_root = gui / "qml"
    engine.addImportPath(str(qml_root))
    engine.setBaseUrl(QUrl.fromLocalFile(str(qml_root) + "/"))
    engine.rootContext().setContextProperty("scope", scope_backend)
    engine.load(QUrl.fromLocalFile(str(gui / "qml" / "Main.qml")))
    if not engine.rootObjects():
        raise RuntimeError("Failed to load Main.qml")
    return engine


def create_application(argv: list[str] | None = None) -> QGuiApplication:
    return QGuiApplication(argv if argv is not None else sys.argv)
