"""NVS / live tuning diff: editor vs device, RMW store."""

from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..protocol import TunerClient, TunerError
from . import labels as L
from .theme import make_nvs_badge_qss, monospace_font
from .buttons import action_button
from .widgets import SurfaceCard


class NvsDiffPanel(QWidget):
    """Right-hand Config column: live vs editor, write dirty fields, store NVS."""

    _EPS_KP = 0.02
    _EPS_KI = 1.0
    _EPS_LIM = 0.05
    _EPS_FC = 1.0

    def __init__(
        self,
        client: Optional[TunerClient] = None,
        kp_spin=None,
        ki_spin=None,
        lim_spin=None,
        fc_spin=None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._client = client
        self._kp_spin = kp_spin
        self._ki_spin = ki_spin
        self._lim_spin = lim_spin
        self._fc_spin = fc_spin
        self._live = {"kp": 0.0, "ki": 0.0, "lim": 0.0, "fc": 0.0}

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        card = SurfaceCard("Flash")
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        hdr_style = "color: #9aa0a6; font-size: 11px; font-weight: 600;"
        mono_font = monospace_font(12)
        for col, text in enumerate(("", "Device", "Pending")):
            h = QLabel(text)
            h.setStyleSheet(hdr_style)
            grid.addWidget(h, 0, col)

        self._rows: list[tuple[str, QLabel, QLabel]] = []
        row_labels = (
            ("kp", L.PROPORTIONAL_GAIN),
            ("ki", L.INTEGRAL_GAIN),
            ("lim", L.CURRENT_LIMIT),
            ("fc", L.CURRENT_FILTER),
        )
        for row_i, (key, label) in enumerate(row_labels, start=1):
            name = QLabel(label)
            live = QLabel("—")
            live.setFont(mono_font)
            delta = QLabel("—")
            delta.setFont(mono_font)
            grid.addWidget(name, row_i, 0)
            grid.addWidget(live, row_i, 1)
            grid.addWidget(delta, row_i, 2)
            self._rows.append((key, live, delta))

        self._live_kp = self._rows[0][1]
        self._live_ki = self._rows[1][1]
        self._live_lim = self._rows[2][1]
        self._live_fc = self._rows[3][1]
        self._delta_kp = self._rows[0][2]
        self._delta_ki = self._rows[1][2]
        self._delta_lim = self._rows[2][2]
        self._delta_fc = self._rows[3][2]

        card.body_layout.addLayout(grid)
        self._nvs_badge = QLabel("—")
        self._nvs_badge.setObjectName("NvsBadge")
        card.body_layout.addWidget(self._nvs_badge)
        root.addWidget(card)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        btn_row.setContentsMargins(0, 0, 0, 0)
        self._read_btn = action_button("Read", "BtnCompact")
        self._write_btn = action_button("Write", "BtnCompact")
        self._patch_btn = action_button("Patch", "BtnCompact")
        for btn in (self._read_btn, self._write_btn, self._patch_btn):
            btn.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            btn_row.addWidget(btn, 1)
        root.addLayout(btn_row)
        root.addStretch(1)
        self._status = QLabel("")
        self._status.setStyleSheet("color: #ef5350; font-size: 11px;")
        root.addWidget(self._status)

        self._read_btn.clicked.connect(self._on_read)
        self._write_btn.clicked.connect(self._on_write_dirty)
        self._patch_btn.clicked.connect(self._on_patch)

    def set_client(self, client: Optional[TunerClient]) -> None:
        self._client = client

    def set_actions_enabled(self, on: bool) -> None:
        self._read_btn.setEnabled(on)
        self._write_btn.setEnabled(on)
        self._patch_btn.setEnabled(on)

    def set_calibration_present(self, present: bool) -> None:
        self._nvs_badge.setText("Stored" if present else "Empty")
        self._nvs_badge.setStyleSheet(make_nvs_badge_qss(present))

    def update_live(self, kp: float, ki: float, lim: float, fc: float) -> None:
        self._live = {"kp": kp, "ki": ki, "lim": lim, "fc": fc}
        self._live_kp.setText(f"{kp:.4f}")
        self._live_ki.setText(f"{ki:.1f}")
        self._live_lim.setText(f"{lim:.2f}")
        self._live_fc.setText(f"{fc:.0f}")
        self._refresh_deltas()

    def _editor_values(self) -> dict[str, float]:
        return {
            "kp": self._kp_spin.value() if self._kp_spin else 0.0,
            "ki": self._ki_spin.value() if self._ki_spin else 0.0,
            "lim": self._lim_spin.value() if self._lim_spin else 0.0,
            "fc": self._fc_spin.value() if self._fc_spin else 0.0,
        }

    def _refresh_deltas(self) -> None:
        ed = self._editor_values()
        self._set_delta(self._delta_kp, ed["kp"] - self._live["kp"], self._EPS_KP)
        self._set_delta(self._delta_ki, ed["ki"] - self._live["ki"], self._EPS_KI)
        self._set_delta(self._delta_lim, ed["lim"] - self._live["lim"], self._EPS_LIM)
        self._set_delta(self._delta_fc, ed["fc"] - self._live["fc"], self._EPS_FC)

    @staticmethod
    def _set_delta(lbl: QLabel, d: float, eps: float) -> None:
        if abs(d) <= eps:
            lbl.setText("—")
            lbl.setStyleSheet("color: #66bb6a;")
        else:
            lbl.setText(f"{d:+.4f}")
            lbl.setStyleSheet("color: #ffb300;")

    def refresh_from_editor(self) -> None:
        self._refresh_deltas()

    def _require_client(self) -> Optional[TunerClient]:
        if self._client is None:
            self._status.setText("Connect a board first.")
            return None
        self._status.setText("")
        return self._client

    def _on_read(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        if not all((self._kp_spin, self._ki_spin, self._lim_spin, self._fc_spin)):
            return
        try:
            kp = cli.read_kp()
            ki = cli.read_ki()
            lim = cli.read_int_lim()
            fc = cli.read_current_filter_fc()
        except TunerError as e:
            self._status.setText(str(e))
            return
        for sp in (self._kp_spin, self._ki_spin, self._lim_spin, self._fc_spin):
            sp.blockSignals(True)
        try:
            self._kp_spin.setValue(kp)
            self._ki_spin.setValue(ki)
            self._lim_spin.setValue(lim)
            self._fc_spin.setValue(fc)
        finally:
            for sp in (self._kp_spin, self._ki_spin, self._lim_spin, self._fc_spin):
                sp.blockSignals(False)
        self.update_live(kp, ki, lim, fc)
        self._refresh_deltas()

    def _on_write_dirty(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        ed = self._editor_values()
        try:
            if abs(ed["kp"] - self._live["kp"]) > self._EPS_KP:
                cli.write_kp(ed["kp"])
            if abs(ed["ki"] - self._live["ki"]) > self._EPS_KI:
                cli.write_ki(ed["ki"])
            if abs(ed["lim"] - self._live["lim"]) > self._EPS_LIM:
                cli.write_int_lim(ed["lim"])
            if abs(ed["fc"] - self._live["fc"]) > self._EPS_FC:
                cli.write_current_filter_fc(ed["fc"])
        except TunerError as e:
            self._status.setText(str(e))

    def _on_patch(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            self._on_write_dirty()
            cli.store_calibration()
        except TunerError as e:
            self._status.setText(str(e))

    def capture_nvs_reference(self, *args, **kwargs) -> None:
        pass
