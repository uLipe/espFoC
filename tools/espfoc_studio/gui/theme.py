"""Dark theme applied before any window is shown.

Uses Qt's Fusion style with a handcrafted palette so it looks the same
on every OS and does not require an extra dependency. pyqtgraph gets
its own dark background via setConfigOption; the two conventions need
to agree or the plots look washed out against the panel chrome.
"""

from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication


_BG = "#1e1f22"        # window background
_BG_ALT = "#26272b"    # group box / card
_FG = "#e6e6e6"        # primary text
_DIM = "#9aa0a6"       # secondary text
_ACCENT = "#4fc3f7"    # highlight (links, selection)
_BORDER = "#3a3b3f"
_ERROR = "#ef5350"


def apply_dark_theme(app: QApplication) -> None:
    app.setStyle("Fusion")

    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(_BG))
    pal.setColor(QPalette.WindowText, QColor(_FG))
    pal.setColor(QPalette.Base, QColor(_BG_ALT))
    pal.setColor(QPalette.AlternateBase, QColor(_BG))
    pal.setColor(QPalette.Text, QColor(_FG))
    pal.setColor(QPalette.Button, QColor(_BG_ALT))
    pal.setColor(QPalette.ButtonText, QColor(_FG))
    pal.setColor(QPalette.ToolTipBase, QColor(_BG))
    pal.setColor(QPalette.ToolTipText, QColor(_FG))
    pal.setColor(QPalette.PlaceholderText, QColor(_DIM))
    pal.setColor(QPalette.Highlight, QColor(_ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor("#0b0c0d"))
    pal.setColor(QPalette.Link, QColor(_ACCENT))
    pal.setColor(QPalette.BrightText, QColor(_ERROR))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor(_DIM))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(_DIM))
    pal.setColor(QPalette.Disabled, QPalette.WindowText, QColor(_DIM))
    app.setPalette(pal)

    # Extra polish on top of Fusion: softer group-box borders, denser
    # header typography. Kept short so the theme ships dep-free.
    app.setStyleSheet(f"""
        QMainWindow, QWidget {{
            background-color: {_BG};
            color: {_FG};
        }}
        QGroupBox {{
            border: 1px solid {_BORDER};
            border-radius: 6px;
            margin-top: 12px;
            padding: 6px;
            background-color: {_BG_ALT};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 8px;
            padding: 0 4px;
            color: {_DIM};
            font-size: 11px;
            letter-spacing: 0.5px;
            text-transform: uppercase;
        }}
        QLabel, QCheckBox, QRadioButton {{
            color: {_FG};
        }}
        QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {{
            background-color: #2b2c30;
            border: 1px solid {_BORDER};
            border-radius: 4px;
            padding: 3px 6px;
            selection-background-color: {_ACCENT};
            selection-color: #0b0c0d;
        }}
        QPushButton {{
            background-color: #303236;
            border: 1px solid {_BORDER};
            border-radius: 4px;
            padding: 6px 12px;
        }}
        QPushButton:hover {{ background-color: #3a3c41; }}
        QPushButton:pressed {{ background-color: #2a2c30; }}
        QTabWidget::pane {{ border: 1px solid {_BORDER}; border-radius: 4px; }}
        QTabBar::tab {{
            background: {_BG};
            color: {_DIM};
            padding: 6px 12px;
            border: 1px solid transparent;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }}
        QTabBar::tab:selected {{
            background: {_BG_ALT};
            color: {_FG};
            border: 1px solid {_BORDER};
            border-bottom-color: {_BG_ALT};
        }}
        QSplitter::handle {{ background: {_BORDER}; }}
        QScrollArea, QScrollArea > QWidget > QWidget {{
            background-color: {_BG};
        }}
    """)

    # Align pyqtgraph's own colours with the palette so the plot canvas
    # blends with the panel around it.
    pg.setConfigOption("background", _BG_ALT)
    pg.setConfigOption("foreground", _FG)
    pg.setConfigOption("antialias", True)
