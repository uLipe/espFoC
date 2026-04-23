"""Asynchronous idf.py build worker.

For ``tuner_studio_target`` the run sequence is: remove a stale
``sdkconfig`` / ``build/``, ``idf.py set-target <chip>`` (so
``sdkconfig.defaults`` + ``sdkconfig.defaults.<chip>`` apply), merge
the TunerStudio Hardware form into ``sdkconfig``, then ``idf.py build``.

Spawns ``bash -lc`` in a subprocess, streams its
stdout / stderr (merged) line-by-line over a Qt signal. The Qt
thread connects the signal to the GenerateAppPanel log widget so
the operator sees the build live without the GUI freezing.

Sourcing export.sh is the cleanest cross-distro way to inherit the
full IDF environment (PATH, IDF_PYTHON_ENV_PATH, xtensa toolchain,
etc.) without re-implementing the discovery dance. Linux / macOS
in scope; Windows comes as a follow-up.
"""

from __future__ import annotations

import os
import shlex
import subprocess
from typing import Optional

from PySide6.QtCore import QObject, Signal, Slot

from .hardware_panel import HardwareConfig


class BuildWorker(QObject):
    """Runs idf.py in a child process. Parent thread connects the
    signals; this object lives in a worker QThread."""

    line = Signal(str)              # stdout / stderr line, no trailing newline
    finished = Signal(int, str)     # exit_code, bin_path ("" on failure)

    def __init__(self, idf_path: str, target_dir: str, chip: str,
                 bin_basename: str,
                 hardware: Optional[HardwareConfig] = None) -> None:
        super().__init__()
        self._idf_path = idf_path
        self._target_dir = target_dir
        self._chip = chip
        self._bin_basename = bin_basename
        self._hardware = hardware
        self._proc: Optional[subprocess.Popen] = None

    @Slot()
    def run(self) -> None:
        if not self._run_bash(
            f'set -e; source "{self._idf_path}/export.sh" '
            f'&& cd "{self._target_dir}" '
            f"&& rm -f sdkconfig sdkconfig.old && rm -rf build && "
            f'idf.py set-target {shlex.quote(self._chip)}'):
            return

        sdk = os.path.join(self._target_dir, "sdkconfig")
        if not os.path.isfile(sdk):
            self.line.emit("error: sdkconfig missing after idf.py set-target")
            self.finished.emit(1, "")
            return

        if self._hardware is not None:
            try:
                from ..codegen.tuner_studio_sdkconfig import (
                    apply_hardware_to_sdkconfig,
                )
                apply_hardware_to_sdkconfig(sdk, self._hardware)
            except Exception as e:
                self.line.emit(f"error: sdkconfig hardware merge failed: {e}")
                self.finished.emit(1, "")
                return
            self.line.emit(">>> TunerStudio: applied Hardware form to "
                           "tuner_studio_target/sdkconfig")

        if not self._run_bash(
            f'set -e; source "{self._idf_path}/export.sh" '
            f'&& cd "{self._target_dir}" '
            f"&& idf.py build"):
            return

        bin_path = self._find_bin()
        self.finished.emit(0, bin_path)

    def _run_bash(self, script: str) -> bool:
        try:
            self._proc = subprocess.Popen(
                ["bash", "-lc", script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as e:
            self.line.emit(f"build error: {e}")
            self.finished.emit(127, "")
            return False
        assert self._proc.stdout is not None
        for raw in self._proc.stdout:
            self.line.emit(raw.rstrip("\n"))
        rc = self._proc.wait()
        if rc != 0:
            self.finished.emit(rc, "")
            return False
        return True

    def _find_bin(self) -> str:
        # IDF builds drop the application .bin under build/<name>.bin.
        candidate = os.path.join(self._target_dir, "build", self._bin_basename)
        return candidate if os.path.isfile(candidate) else ""


def is_valid_idf_path(path: str) -> bool:
    """Sanity check before saving an IDF path: must contain
    tools/idf.py and components/. Cheap, no subprocess."""
    if not path or not os.path.isdir(path):
        return False
    if not os.path.isfile(os.path.join(path, "tools", "idf.py")):
        return False
    if not os.path.isdir(os.path.join(path, "components")):
        return False
    return True
