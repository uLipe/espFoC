"""Generate App tab.

Renders a ready-to-build IDF application from the live tuning state
plus the values typed in on the Hardware tab. Visible only when the
firmware identifies itself as TunerStudio target (or in --demo mode
where the demo firmware reports the same magic).
"""

from __future__ import annotations

import os
from typing import Callable

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..codegen import generate_sensored_app
from ..protocol import TunerClient, TunerError
from .hardware_panel import HardwarePanel


_DEFAULT_BASE = os.path.expanduser("~/espfoc_apps")


class GenerateAppPanel(QWidget):
    """Form + log box that drives generate_sensored_app()."""

    appGenerated = Signal(str)  # emits output dir on success

    def __init__(self, client: TunerClient,
                 hardware: HardwarePanel,
                 get_motor_params: Callable[[], tuple[float, float, float]]
                 ) -> None:
        super().__init__()
        self._client = client
        self._hw = hardware
        self._get_motor_params = get_motor_params

        root = QVBoxLayout(self)

        intro = QLabel(
            "Captures the live Kp/Ki/integrator_limit, packages them with "
            "the Hardware tab snapshot and writes a self-contained IDF "
            "project that boots already tuned. The runtime tuner stays "
            "wired in so you can re-tune later without rebuilding.")
        intro.setWordWrap(True)
        root.addWidget(intro)

        form_box = QGroupBox("Output")
        form = QFormLayout(form_box)
        self._app_name = QLineEdit("my_motor_app")
        form.addRow("App name", self._app_name)

        self._override_box = QCheckBox("Override default output directory")
        form.addRow(self._override_box)

        path_row = QHBoxLayout()
        self._out_dir = QLineEdit(_DEFAULT_BASE)
        self._out_dir.setEnabled(False)
        self._browse_btn = QPushButton("Browse...")
        self._browse_btn.setEnabled(False)
        self._browse_btn.clicked.connect(self._on_browse)
        path_row.addWidget(self._out_dir)
        path_row.addWidget(self._browse_btn)
        form.addRow("Output directory", path_row)
        self._override_box.toggled.connect(self._on_override_toggled)
        root.addWidget(form_box)

        gen_btn = QPushButton("Generate")
        gen_btn.clicked.connect(self._on_generate)
        root.addWidget(gen_btn)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(200)
        root.addWidget(self._log)
        root.addStretch(0)

    def _on_override_toggled(self, on: bool) -> None:
        self._out_dir.setEnabled(on)
        self._browse_btn.setEnabled(on)

    def _on_browse(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "Choose base directory",
                                             self._out_dir.text())
        if d:
            self._out_dir.setText(d)

    def _output_path(self, app_name: str) -> str:
        base = self._out_dir.text() if self._override_box.isChecked() \
                                   else _DEFAULT_BASE
        return os.path.join(os.path.expanduser(base), app_name)

    def _on_generate(self) -> None:
        app_name = (self._app_name.text() or "").strip()
        if not app_name or not all(c.isalnum() or c == "_" for c in app_name):
            self._append("error: app_name must be alphanumeric / underscore")
            return
        try:
            kp = self._client.read_kp()
            ki = self._client.read_ki()
            ilim = self._client.read_int_lim()
            fc_hz = self._client.read_current_filter_fc()
        except TunerError as e:
            self._append(f"error: failed to read live gains: {e}")
            return

        r_ohm, l_h, bw_hz = self._get_motor_params()
        out_dir = self._output_path(app_name)
        try:
            res = generate_sensored_app(
                self._hw.get_config(),
                app_name=app_name,
                output_dir=out_dir,
                kp=kp, ki=ki, ilim=ilim,
                r_ohm=r_ohm, l_h=l_h, bw_hz=bw_hz,
                current_filter_fc_hz=fc_hz)
        except ValueError as e:
            self._append(f"error: {e}")
            return
        except Exception as e:  # template miss, file IO, etc
            self._append(f"error: generation failed: {e}")
            return

        self._append(f"wrote {len(res.files_written)} files under {res.output_dir}")
        for f in res.files_written:
            self._append(f"  + {f}")
        if res.nvs_image:
            self._append(f"NVS image  : {res.nvs_image}")
        elif res.nvs_skip_reason:
            self._append(f"NVS image  : SKIPPED ({res.nvs_skip_reason})")
            self._append("            re-run with $IDF_PATH set to also "
                         "produce nvs_calibration.bin")
        self._append("done.")
        self.appGenerated.emit(res.output_dir)

    def _append(self, line: str) -> None:
        self._log.appendPlainText(line)
