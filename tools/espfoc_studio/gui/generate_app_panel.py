"""Generate App tab.

Single-stop workflow that turns a live tuning session into a
ready-to-build IDF project. The Hardware section was a sibling tab
before; living inside this panel makes the order of operations
self-explanatory: configure the board, name the output, hit Generate.
"""

from __future__ import annotations

import os
from typing import Callable, Optional

from PySide6.QtCore import Qt, QSettings, QThread, Signal
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
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..codegen import generate_sensored_app
from ..protocol import TunerClient, TunerError
from .build_worker import BuildWorker, is_valid_idf_path
from .hardware_panel import HardwarePanel


_DEFAULT_BASE = os.path.expanduser("~/espfoc_apps")

# Where the in-tree tuner_studio_target firmware lives. Resolved
# relative to this file so the panel works from any cwd.
_TUNER_TARGET_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "examples", "tuner_studio_target"))
_TUNER_TARGET_BIN = "espfoc_tuner_studio_target.bin"

_QSETTINGS_ORG = "espfoc"
_QSETTINGS_APP = "TunerStudio"
_QSETTINGS_KEY_IDF = "idf_path"


class GenerateAppPanel(QWidget):
    """Hardware + Output sections + log box. Drives generate_sensored_app()."""

    appGenerated = Signal(str)  # emits output dir on success

    def __init__(self, client: TunerClient,
                 get_motor_params: Callable[[], tuple[float, float, float]]
                 ) -> None:
        super().__init__()
        self._client = client
        self._get_motor_params = get_motor_params
        # Worker + thread for the optional firmware build path. Both
        # stay None when no build is in flight; created on demand by
        # _start_build_worker. Kept as members so the worker is not
        # garbage-collected mid-stream.
        self._build_thread: Optional[QThread] = None
        self._build_worker: Optional[BuildWorker] = None

        # Outer scroll so the panel never clips even when the window
        # is short — the embedded Hardware section is form-heavy.
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QScrollArea.NoFrame)
        outer.addWidget(scroll)
        body = QWidget()
        scroll.setWidget(body)
        root = QVBoxLayout(body)

        # --- Hardware configuration (was a sibling tab before) ---
        hw_box = QGroupBox("Hardware configuration")
        hw_layout = QVBoxLayout(hw_box)
        hw_layout.setContentsMargins(6, 12, 6, 6)
        self._hw = HardwarePanel()
        hw_layout.addWidget(self._hw)
        root.addWidget(hw_box)

        # --- Output ---
        out_box = QGroupBox("Output")
        form = QFormLayout(out_box)
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
        root.addWidget(out_box)

        # --- Build (optional firmware-only mode) ---
        build_box = QGroupBox("Build")
        bform = QFormLayout(build_box)
        self._build_fw_box = QCheckBox(
            "Skip code generation, build tuner_studio_target firmware")
        self._build_fw_box.setToolTip(
            "When checked, runs `idf.py set-target` and `idf.py build` on "
            "examples/tuner_studio_target.  Removes prior sdkconfig/build, "
            "re-materializes from sdkconfig.defaults* for the chip, then "
            "merges the Hardware form (pin map, motor profile, shunt) into "
            "the project sdkconfig before compiling. App name / Output are "
            "ignored. The action button relabels.")
        self._build_fw_box.toggled.connect(self._on_build_mode_toggled)
        bform.addRow(self._build_fw_box)

        idf_row = QHBoxLayout()
        self._idf_path_edit = QLineEdit(os.environ.get("IDF_PATH", "")
                                        or self._load_persisted_idf())
        self._idf_path_edit.setPlaceholderText(
            "Path to ESP-IDF (defaults to $IDF_PATH if set)")
        self._idf_browse_btn = QPushButton("Browse...")
        self._idf_browse_btn.clicked.connect(self._on_browse_idf)
        idf_row.addWidget(self._idf_path_edit)
        idf_row.addWidget(self._idf_browse_btn)
        bform.addRow("IDF path", idf_row)
        root.addWidget(build_box)

        # --- Action button (label flips based on the build mode) ---
        action_row = QHBoxLayout()
        action_row.addStretch(1)
        self._generate_btn = QPushButton("Generate")
        self._generate_btn.setToolTip(
            "Capture the live Kp / Ki / integrator_limit + current-filter "
            "cutoff, package them with the Hardware section above and "
            "write a self-contained IDF project that boots already tuned.")
        self._generate_btn.clicked.connect(self._on_action_clicked)
        action_row.addWidget(self._generate_btn)
        root.addLayout(action_row)

        # --- Log ---
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(200)
        self._log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self._log.setMinimumHeight(140)
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

    def _on_build_mode_toggled(self, on: bool) -> None:
        # Output section is meaningless under firmware-only mode.
        self._app_name.setEnabled(not on)
        self._override_box.setEnabled(not on)
        self._out_dir.setEnabled(not on and self._override_box.isChecked())
        self._browse_btn.setEnabled(not on and self._override_box.isChecked())
        self._generate_btn.setText("Build firmware" if on else "Generate")

    def _on_browse_idf(self) -> None:
        start = self._idf_path_edit.text() or os.path.expanduser("~")
        d = QFileDialog.getExistingDirectory(
            self, "Locate ESP-IDF installation", start)
        if not d:
            return
        if not is_valid_idf_path(d):
            self._append(f"error: {d} does not look like an ESP-IDF "
                         f"installation (missing tools/idf.py)")
            return
        self._idf_path_edit.setText(d)
        self._persist_idf(d)

    def _output_path(self, app_name: str) -> str:
        base = self._out_dir.text() if self._override_box.isChecked() \
                                   else _DEFAULT_BASE
        return os.path.join(os.path.expanduser(base), app_name)

    # --- Action dispatch -------------------------------------------------

    def _on_action_clicked(self) -> None:
        if self._build_fw_box.isChecked():
            self._on_build_firmware()
        else:
            self._on_generate()

    # --- IDF path discovery ---------------------------------------------

    def _load_persisted_idf(self) -> str:
        s = QSettings(_QSETTINGS_ORG, _QSETTINGS_APP)
        v = s.value(_QSETTINGS_KEY_IDF, "", type=str)
        return v if isinstance(v, str) else ""

    def _persist_idf(self, path: str) -> None:
        s = QSettings(_QSETTINGS_ORG, _QSETTINGS_APP)
        s.setValue(_QSETTINGS_KEY_IDF, path)

    def _resolve_idf_path(self) -> str:
        # Order of precedence (matches the rest of the calibration
        # pipeline): live field > $IDF_PATH > QSettings cache > prompt.
        candidates = (self._idf_path_edit.text().strip(),
                      os.environ.get("IDF_PATH", ""),
                      self._load_persisted_idf())
        for c in candidates:
            if is_valid_idf_path(c):
                # Push the resolved value back into the field +
                # cache so the user sees what we picked and the
                # next session starts from there.
                self._idf_path_edit.setText(c)
                self._persist_idf(c)
                return c
        # Nothing valid yet — prompt.
        d = QFileDialog.getExistingDirectory(
            self, "Locate ESP-IDF installation",
            os.path.expanduser("~"))
        if d and is_valid_idf_path(d):
            self._idf_path_edit.setText(d)
            self._persist_idf(d)
            return d
        return ""

    # --- Firmware build mode --------------------------------------------

    def _on_build_firmware(self) -> None:
        if self._build_thread is not None:
            self._append("error: a build is already in flight, wait for it to finish")
            return
        idf_path = self._resolve_idf_path()
        if not idf_path:
            self._append("build aborted: no valid ESP-IDF installation selected")
            return
        chip = self._hw.get_config().target
        self._append(f">>> building tuner_studio_target for {chip}")
        self._append("    (clean sdkconfig+build, set-target, merge Hardware "
                      "-> sdkconfig, then build)")
        self._append(f"    IDF_PATH = {idf_path}")
        self._append(f"    target dir = {_TUNER_TARGET_DIR}")
        self._start_build_worker(idf_path, _TUNER_TARGET_DIR, chip)

    def _start_build_worker(self, idf_path: str, target_dir: str,
                            chip: str) -> None:
        self._set_inputs_enabled(False)
        self._generate_btn.setText("Building…")
        self._generate_btn.setEnabled(False)

        self._build_thread = QThread(self)
        self._build_worker = BuildWorker(
            idf_path, target_dir, chip, _TUNER_TARGET_BIN,
            hardware=self._hw.get_config())
        self._build_worker.moveToThread(self._build_thread)
        self._build_worker.line.connect(self._append)
        self._build_worker.finished.connect(self._on_build_finished)
        self._build_thread.started.connect(self._build_worker.run)
        self._build_thread.start()

    def _on_build_finished(self, rc: int, bin_path: str) -> None:
        if rc == 0 and bin_path:
            self._append(f">>> build OK: {bin_path}")
            self._append(
                f"    Flash with: idf.py -p <PORT> -C {_TUNER_TARGET_DIR} flash")
        else:
            self._append(f">>> build FAILED (exit {rc}); see log above")
        # Tear down the worker thread cleanly.
        if self._build_thread is not None:
            self._build_thread.quit()
            self._build_thread.wait()
            self._build_thread = None
            self._build_worker = None
        self._set_inputs_enabled(True)
        self._generate_btn.setEnabled(True)
        self._generate_btn.setText(
            "Build firmware" if self._build_fw_box.isChecked() else "Generate")

    def _set_inputs_enabled(self, on: bool) -> None:
        # Lock down everything that would mutate the config the worker
        # is consuming; keep the log scrollable.
        for w in (self._hw, self._app_name, self._override_box,
                  self._out_dir, self._browse_btn,
                  self._build_fw_box,
                  self._idf_path_edit, self._idf_browse_btn):
            w.setEnabled(on)
        # Output dir / browse stay disabled when the override box is off
        # — _on_override_toggled and _on_build_mode_toggled own that.
        if on:
            self._on_build_mode_toggled(self._build_fw_box.isChecked())

    # --- Generate-app mode (existing flow) ------------------------------

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
