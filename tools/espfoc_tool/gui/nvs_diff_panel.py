"""NVS / live tuning diff: editor vs device, RMW store."""

from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..protocol import TunerClient, TunerError


class NvsDiffPanel(QWidget):
    """Right-hand Config card: live vs editor, write dirty fields, store NVS."""

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
        box = QGroupBox("Live vs editor (NVS RMW on store)")
        form = QFormLayout(box)
        mono = "font-family: monospace; font-size: 12px;"
        self._live_kp = QLabel("-")
        self._live_ki = QLabel("-")
        self._live_lim = QLabel("-")
        self._live_fc = QLabel("-")
        self._delta_kp = QLabel("-")
        self._delta_ki = QLabel("-")
        self._delta_lim = QLabel("-")
        self._delta_fc = QLabel("-")
        for w in (
            self._live_kp, self._live_ki, self._live_lim, self._live_fc,
            self._delta_kp, self._delta_ki, self._delta_lim, self._delta_fc,
        ):
            w.setStyleSheet(mono)
        form.addRow("Kp live", self._live_kp)
        form.addRow("Kp Δ edit", self._delta_kp)
        form.addRow("Ki live", self._live_ki)
        form.addRow("Ki Δ edit", self._delta_ki)
        form.addRow("ILim live", self._live_lim)
        form.addRow("ILim Δ edit", self._delta_lim)
        form.addRow("fc live", self._live_fc)
        form.addRow("fc Δ edit", self._delta_fc)
        self._nvs_hint = QLabel("NVS: —")
        self._nvs_hint.setWordWrap(True)
        self._nvs_hint.setStyleSheet("color: #9aa0a6; font-size: 11px;")
        form.addRow(self._nvs_hint)
        root.addWidget(box)

        self._write_btn = QPushButton("Write dirty fields to device")
        self._write_btn.clicked.connect(self._on_write_dirty)
        self._store_btn = QPushButton("Store to NVS (RMW)")
        self._store_btn.clicked.connect(self._on_store)
        row = QHBoxLayout()
        row.addWidget(self._write_btn)
        row.addWidget(self._store_btn)
        root.addLayout(row)
        self._erase_btn = QPushButton("Erase NVS calibration")
        self._erase_btn.clicked.connect(self._on_erase)
        root.addWidget(self._erase_btn)
        root.addStretch(1)
        self._status = QLabel("")
        self._status.setStyleSheet("color: #ef5350; font-size: 11px;")
        root.addWidget(self._status)

    def set_client(self, client: Optional[TunerClient]) -> None:
        self._client = client

    def set_actions_enabled(self, on: bool) -> None:
        self._write_btn.setEnabled(on)
        self._store_btn.setEnabled(on)
        self._erase_btn.setEnabled(on)

    def set_calibration_present(self, present: bool) -> None:
        self._nvs_hint.setText(
            "NVS: calibration present — Store merges only changed tuning fields."
            if present else "NVS: empty — Store writes a new blob.")

    def update_live(self, kp: float, ki: float, lim: float, fc: float) -> None:
        self._live = {"kp": kp, "ki": ki, "lim": lim, "fc": fc}
        self._live_kp.setText(f"{kp:9.4f}")
        self._live_ki.setText(f"{ki:9.2f}")
        self._live_lim.setText(f"{lim:9.3f}")
        self._live_fc.setText(f"{fc:9.1f}")
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
            lbl.setStyleSheet("font-family: monospace; color: #66bb6a;")
        else:
            lbl.setText(f"{d:+.4f}")
            lbl.setStyleSheet("font-family: monospace; color: #ffb300;")

    def refresh_from_editor(self) -> None:
        self._refresh_deltas()

    def _require_client(self) -> Optional[TunerClient]:
        if self._client is None:
            self._status.setText("Connect a board first.")
            return None
        self._status.setText("")
        return self._client

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

    def _on_store(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            self._on_write_dirty()
            cli.store_calibration()
        except TunerError as e:
            self._status.setText(str(e))

    def _on_erase(self) -> None:
        cli = self._require_client()
        if cli is None:
            return
        try:
            cli.erase_calibration()
        except TunerError as e:
            self._status.setText(str(e))
