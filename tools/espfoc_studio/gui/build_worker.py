"""Asynchronous idf.py build worker.

Spawns `bash -lc "source <IDF>/export.sh && cd <target> && idf.py
set-target <chip> && idf.py build"` in a subprocess, streams its
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
import subprocess
from typing import Optional

from PySide6.QtCore import QObject, Signal, Slot


class BuildWorker(QObject):
    """Runs idf.py in a child process. Parent thread connects the
    signals; this object lives in a worker QThread."""

    line = Signal(str)              # stdout / stderr line, no trailing newline
    finished = Signal(int, str)     # exit_code, bin_path ("" on failure)

    def __init__(self, idf_path: str, target_dir: str, chip: str,
                 bin_basename: str) -> None:
        super().__init__()
        self._idf_path = idf_path
        self._target_dir = target_dir
        self._chip = chip
        self._bin_basename = bin_basename
        self._proc: Optional[subprocess.Popen] = None

    @Slot()
    def run(self) -> None:
        cmd = (
            f'set -e; '
            f'source "{self._idf_path}/export.sh" '
            f'&& cd "{self._target_dir}" '
            f'&& idf.py set-target {self._chip} '
            f'&& idf.py build'
        )
        try:
            self._proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as e:
            # bash missing or PATH borked; surface and bail.
            self.line.emit(f"build error: {e}")
            self.finished.emit(127, "")
            return

        assert self._proc.stdout is not None
        for raw in self._proc.stdout:
            self.line.emit(raw.rstrip("\n"))
        rc = self._proc.wait()
        bin_path = self._find_bin() if rc == 0 else ""
        self.finished.emit(rc, bin_path)

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
