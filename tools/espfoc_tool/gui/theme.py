"""Dark theme applied before any window is shown.

Uses Qt's Fusion style with a handcrafted palette so it looks the same
on every OS and does not require an extra dependency. pyqtgraph gets
its own dark background via setConfigOption; the two conventions need
to agree or the plots look washed out against the panel chrome.
"""

from __future__ import annotations

import sys

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication


_BG = "#1e1f22"        # window background
_BG_ALT = "#26272b"    # group box / card
_FG = "#e6e6e6"        # primary text
_DIM = "#9aa0a6"       # secondary text
_ACCENT = "#4fc3f7"    # highlight (links, selection)
_BORDER = "#3a3b3f"
_ERROR = "#ef5350"

# Raised surfaces (neutral warm gray — cards, metrics, inputs)
_SURF_TOP = "#35383f"
_SURF_BOT = "#27292e"
_SURF_BORDER = "#43474f"
_SURF_INSET_TOP = "#2f3238"
_SURF_INSET_BOT = "#25272c"

# Action buttons (cool slate — distinct from surfaces)
_BTN_TOP = "#3d5563"
_BTN_MID = "#334a57"
_BTN_BOT = "#2a3b47"
_BTN_BORDER = "#4a6a7c"
_BTN_FG = "#b8d4e0"
_BTN_HOVER_TOP = "#476372"
_BTN_HOVER_BOT = "#324957"
_BTN_PRESS = "#243038"


# Axis-state badge palette. Picked the dominant flag (override > running >
# aligned > init > none) and rendered it as a single colored pill instead
# of the old "+INITIALIZED -ALIGNED -RUNNING -TUNER_OVERRIDE" text. Keep
# the foreground / background pair high-contrast on the dark theme.
BADGE_STYLES = {
    "NO_DEVICE": ("NO DEVICE", "#0b0c0d", "#6c757d"),
    "SCANNING":  ("SCANNING",  "#0b0c0d", "#ffb300"),
    "OFFLINE":  ("OFFLINE",  "#0b0c0d", "#6c757d"),
    "INIT":     ("INIT",     "#0b0c0d", "#26a69a"),
    "ALIGNING": ("ALIGNING", "#0b0c0d", "#ff9800"),
    "ALIGNED":  ("ALIGNED",  "#0b0c0d", _ACCENT),
    "RUNNING":  ("RUNNING",  "#0b0c0d", "#66bb6a"),
    "OVERRIDE": ("OVERRIDE", "#ffffff", "#ab47bc"),
    "LINK_OK":     ("CONNECTED", "#0b0c0d", "#66bb6a"),
    "LINK_WAIT":   ("CONNECTING", "#0b0c0d", "#ffb300"),
    "LINK_DOWN":   ("NO LINK",  "#ffffff", _ERROR),
}


def make_nvs_badge_qss(stored: bool) -> str:
    if stored:
        grad = (
            "qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #2e4038, stop:1 #243530)"
        )
        fg = "#a8c8b4"
        border = "#3d5a4a"
    else:
        grad = (
            f"qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            f"stop:0 {_SURF_TOP}, stop:1 {_SURF_BOT})"
        )
        fg = _DIM
        border = _SURF_BORDER
    return (
        f"QLabel#NvsBadge {{"
        f" background: {grad}; color: {fg};"
        f" border: 1px solid {border};"
        f" border-radius: 8px; padding: 4px 12px;"
        f" font-size: 11px; font-weight: 600;"
        f"}}"
    )


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


def _btn_qss_block() -> str:
    """Single neutral action style; legacy role names map to BtnDefault."""
    disabled = (
        "background: #2e3036; color: #6c757d; border: 1px solid #3a3b3f;"
    )
    action_sel = (
        "QPushButton#BtnDefault,"
        "QPushButton#BtnSecondary,"
        "QPushButton#BtnPrimary,"
        "QPushButton#PrimaryButton,"
        "QPushButton#BtnAccent,"
        "QPushButton#BtnApply,"
        "QPushButton#BtnAlign,"
        "QPushButton#BtnDanger,"
        "QPushButton#BtnEstop,"
        "QPushButton#BtnReset,"
        "QPushButton#BtnNudge,"
        "QPushButton#BtnCompact"
    )
    return f"""
        QPushButton#BtnNav {{
            background: transparent;
            color: {_DIM};
            border: none;
            border-radius: 12px;
            padding: 10px 14px;
            font-size: 13px;
            font-weight: 500;
            text-align: left;
        }}
        QPushButton#BtnNav:hover {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 #32363d, stop:1 #2a2c31);
            color: {_FG};
        }}
        QPushButton#BtnNav:checked {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
                stop:0 #2e353d, stop:0.5 #2a3038, stop:1 #262a31);
            color: {_FG};
            border: 1px solid #4a4e56;
            font-weight: 600;
        }}
        {action_sel} {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_BTN_TOP}, stop:0.5 {_BTN_MID}, stop:1 {_BTN_BOT});
            color: {_BTN_FG};
            border: 1px solid {_BTN_BORDER};
            border-radius: 8px;
            padding: 8px 14px;
            min-height: 20px;
        }}
        {action_sel}:hover {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_BTN_HOVER_TOP}, stop:1 {_BTN_HOVER_BOT});
            border-color: {_ACCENT};
            color: #d4eaf2;
        }}
        {action_sel}:pressed {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_BTN_PRESS}, stop:1 #1e2a32);
            border-color: #3a5560;
            color: #e8f4f8;
        }}
        {action_sel}:disabled {{ {disabled} }}
        QPushButton#BtnEstop {{
            padding: 12px 16px;
            min-height: 28px;
            font-weight: 600;
        }}
        QPushButton#BtnNudge {{
            padding: 6px 8px;
            min-width: 36px;
            max-width: 48px;
        }}
        QPushButton#BtnReset {{
            padding: 5px 12px;
            min-width: 88px;
            font-size: 11px;
        }}
        QPushButton#BtnCompact {{
            padding: 7px 10px;
            min-height: 18px;
        }}
    """


def _ui_font() -> QFont:
    font = QFont()
    if sys.platform == "darwin":
        font.setFamilies([".AppleSystemUIFont", "SF Pro Text", "Helvetica Neue"])
    elif sys.platform == "win32":
        font.setFamilies(["Segoe UI Variable", "Segoe UI"])
    else:
        font.setFamilies(["Ubuntu", "Cantarell", "Noto Sans", "sans-serif"])
    font.setPointSize(10)
    font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    return font


def monospace_font(point_size: int = 10) -> QFont:
    f = QFont()
    f.setFamilies([
        "JetBrains Mono", "Cascadia Mono", "SF Mono",
        "Consolas", "Liberation Mono", "monospace",
    ])
    f.setPointSize(point_size)
    f.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    return f


def button_font(
    point_size: int = 12,
    weight: QFont.Weight = QFont.Weight.Medium,
) -> QFont:
    f = _ui_font()
    f.setPointSize(point_size)
    f.setWeight(weight)
    return f


def apply_dark_theme(app: QApplication, *, use_opengl_plots: bool = True) -> None:
    app.setStyle("Fusion")
    app.setFont(_ui_font())

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
            border: 1px solid {_SURF_BORDER};
            border-radius: 12px;
            margin-top: 14px;
            padding: 10px 8px 8px 8px;
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_TOP}, stop:1 {_SURF_BOT});
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
            font-size: 13px;
        }}
        QFormLayout QLabel {{
            color: #c8ccd2;
            font-size: 12px;
        }}
        /* Fusion's default indicator melts into the dark background.
         * Force a visible square with a clear checked state. */
        QCheckBox::indicator {{
            width: 16px;
            height: 16px;
            border: 1px solid {_SURF_BORDER};
            border-radius: 3px;
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_INSET_TOP}, stop:1 {_SURF_INSET_BOT});
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
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_INSET_TOP}, stop:1 {_SURF_INSET_BOT});
            border: 1px solid {_SURF_BORDER};
            border-radius: 6px;
            padding: 3px 6px;
            selection-background-color: {_ACCENT};
            selection-color: #0b0c0d;
        }}
        QPlainTextEdit {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_INSET_TOP}, stop:1 {_SURF_INSET_BOT});
            border: 1px solid {_SURF_BORDER};
            border-radius: 8px;
            padding: 6px;
            selection-background-color: {_ACCENT};
            selection-color: #0b0c0d;
        }}
        {_btn_qss_block()}
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
        QSplitter::handle {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 #3a3d44, stop:1 #2a2c31);
            width: 5px;
            margin: 4px 0;
            border-radius: 2px;
        }}
        QSplitter::handle:hover {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 #4a6270, stop:1 #354a57);
        }}
        #NavRail {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 #1c1e22, stop:1 #141518);
            border-right: 1px solid {_BORDER};
        }}
        #NavBrand {{
            font-size: 15px;
            font-weight: 700;
            color: {_ACCENT};
            letter-spacing: 0.5px;
            padding: 4px 6px 12px 6px;
        }}
        #NavHint {{
            color: {_DIM};
            font-size: 10px;
            padding: 0 6px 10px 6px;
        }}
        #PageTitle {{
            font-size: 24px;
            font-weight: 600;
            color: {_FG};
            letter-spacing: -0.3px;
        }}
        #SurfaceCard {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_TOP}, stop:1 {_SURF_BOT});
            border: 1px solid {_SURF_BORDER};
            border-radius: 14px;
        }}
        #CardTitle {{
            font-size: 12px;
            font-weight: 600;
            color: #c8ccd2;
            letter-spacing: 0.6px;
            text-transform: uppercase;
        }}
        #NvsBadge {{
            font-size: 11px;
            font-weight: 600;
            letter-spacing: 0.4px;
            padding: 4px 10px;
            border-radius: 8px;
        }}
        #LiveMetric {{
            background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                stop:0 {_SURF_INSET_TOP}, stop:1 {_SURF_INSET_BOT});
            border: 1px solid {_SURF_BORDER};
            border-radius: 10px;
        }}
        #MetricCaption {{
            color: {_DIM};
            font-size: 11px;
            font-weight: 500;
        }}
        #MetricValue {{
            font-size: 15px;
            font-weight: 500;
            color: {_FG};
            letter-spacing: -0.2px;
        }}
        QScrollArea, QScrollArea > QWidget > QWidget {{
            background-color: transparent;
        }}
    """)

    # Align pyqtgraph's own colours with the palette so the plot canvas
    # blends with the panel around it.
    pg.setConfigOption("background", _BG_ALT)
    pg.setConfigOption("foreground", _FG)
    pg.setConfigOption("antialias", True)
    if not use_opengl_plots:
        pg.setConfigOptions(useOpenGL=False)
