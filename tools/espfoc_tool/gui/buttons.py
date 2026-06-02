"""Styled push buttons (single neutral action palette)."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QPushButton

from .theme import button_font, monospace_font

def action_button(text: str, role: str = "BtnDefault") -> QPushButton:
    btn = QPushButton(text)
    apply_button_style(btn, role)
    return btn


def apply_button_style(btn: QPushButton, role: str) -> None:
    estop = role == "BtnEstop" or btn.text().upper() in ("E-STOP", "ESTOP")
    nudge = role == "BtnNudge"
    reset = role == "BtnReset"
    if role == "BtnNav":
        obj = "BtnNav"
    elif estop:
        obj = "BtnEstop"
    elif nudge:
        obj = "BtnNudge"
    elif reset:
        obj = "BtnReset"
    elif role == "BtnCompact":
        obj = "BtnCompact"
    else:
        obj = "BtnDefault"
    btn.setObjectName(obj)
    btn.setCursor(Qt.PointingHandCursor)
    btn.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    if obj == "BtnNav":
        btn.setFont(button_font(13, QFont.Weight.Medium))
    elif estop:
        f = button_font(13, QFont.Weight.Bold)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.5)
        btn.setFont(f)
    elif nudge:
        btn.setFont(monospace_font(10))
    elif obj == "BtnCompact":
        btn.setFont(button_font(11, QFont.Weight.Medium))
    elif reset:
        btn.setFont(button_font(11, QFont.Weight.Medium))
    else:
        btn.setFont(button_font(12, QFont.Weight.Medium))
