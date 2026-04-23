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


# Axis-state badge palette. Picked the dominant flag (override > running >
# aligned > init > none) and rendered it as a single colored pill instead
# of the old "+INITIALIZED -ALIGNED -RUNNING -TUNER_OVERRIDE" text. Keep
# the foreground / background pair high-contrast on the dark theme.
BADGE_STYLES = {
    "OFFLINE":  ("OFFLINE",  "#0b0c0d", "#6c757d"),
    "INIT":     ("INIT",     "#0b0c0d", "#ffb300"),
    "ALIGNING": ("ALIGNING", "#0b0c0d", "#ff9800"),
    "ALIGNED":  ("ALIGNED",  "#0b0c0d", _ACCENT),
    "RUNNING":  ("RUNNING",  "#0b0c0d", "#66bb6a"),
    "OVERRIDE": ("OVERRIDE", "#ffffff", "#ab47bc"),
    "LINK_DEMO":   ("DEMO",      "#0b0c0d", "#ab47bc"),
    "LINK_OK":     ("CONNECTED", "#0b0c0d", "#66bb6a"),
    "LINK_WAIT":   ("CONNECTING", "#0b0c0d", "#ffb300"),
    "LINK_DOWN":   ("NO LINK",  "#ffffff", _ERROR),
}


def make_badge_qss(state_key: str) -> tuple[str, str]:
    """Returns (label, qss) for the axis state badge. Unknown keys
    fall back to the OFFLINE style. Caller applies the qss to a
    plain QLabel via setStyleSheet()."""
    label, fg, bg = BADGE_STYLES.get(state_key, BADGE_STYLES["OFFLINE"])
    qss = (
        f"QLabel {{"
        f" background-color: {bg};"
        f" color: {fg};"
        f" border-radius: 6px;"
        f" padding: 4px 12px;"
        f" font-weight: 600;"
        f" font-size: 11px;"
        f" letter-spacing: 1px;"
        f" min-width: 78px;"
        f" qproperty-alignment: 'AlignCenter';"
        f"}}"
    )
    return label, qss


def make_reset_board_button_qss() -> str:
    """Pill matching link badges, with a clear border and hover for a
    pushbutton (emergency board reset)."""
    fg, bg = "#ffffff", _ERROR
    return (
        f"QPushButton {{"
        f" background-color: {bg};"
        f" color: {fg};"
        f" border: 1px solid #ffcdd2;"
        f" border-radius: 6px;"
        f" padding: 4px 12px;"
        f" font-weight: 600;"
        f" font-size: 11px;"
        f" letter-spacing: 0.5px;"
        f" min-width: 96px;"
        f"}}"
        f"QPushButton:hover {{"
        f" background-color: #e53935;"
        f" border: 1px solid #ffebee;"
        f" color: #fff;"
        f"}}"
        f"QPushButton:pressed {{ background-color: #c62828; }}"
        f"QPushButton:disabled {{ background-color: #4e342e; color: #bcaaa4;"
        f" border-color: #5d4037; }}"
    )


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
        /* Fusion's default indicator melts into the dark background.
         * Force a visible square with a clear checked state. */
        QCheckBox::indicator {{
            width: 16px;
            height: 16px;
            border: 1px solid {_BORDER};
            border-radius: 3px;
            background-color: #2b2c30;
        }}
        QCheckBox::indicator:hover {{
            border: 1px solid {_ACCENT};
        }}
        QCheckBox::indicator:checked {{
            background-color: {_ACCENT};
            border: 1px solid {_ACCENT};
            image: none;
        }}
        QCheckBox::indicator:disabled {{
            background-color: #1a1b1d;
            border: 1px solid {_BORDER};
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
