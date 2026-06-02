#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
OpenOCD (USB Serial/JTAG) + GDB in one terminal for espFoC ESP-IDF apps.

Typical workflow (esp32c6 + axis_shell):
  Terminal 1 — logs / shell on UART:
    cd examples/axis_shell
    idf.py flash monitor

  Terminal 2 — JTAG debug (this script):
    cd examples/axis_shell
    idf.py build
    python3 ../../scripts/esp_foc_debug.py

Requires: IDF environment (export.sh), project built, OpenOCD + target gdb on PATH.
Closing GDB (quit) stops OpenOCD and exits this script.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

GDB_PORT = 3333
OPENOCD_LOG = "openocd_debug.log"
SCRIPT_DIR = Path(__file__).resolve().parent
EXTRA_GDBINIT = SCRIPT_DIR / "esp_foc_gdbinit.cmd"


def _app_dir() -> Path:
    cwd = Path.cwd()
    if (cwd / "build" / "project_description.json").is_file():
        return cwd
    raise SystemExit(
        "Run from an ESP-IDF app directory with build/project_description.json "
        "(e.g. cd examples/axis_shell && idf.py build)."
    )


def _load_project_desc(app_dir: Path) -> Dict[str, Any]:
    path = app_dir / "build" / "project_description.json"
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def _which_or_die(name: str) -> str:
    from shutil import which

    path = which(name)
    if not path:
        raise SystemExit(f"{name} not found on PATH — source $IDF_PATH/export.sh")
    return path


def _openocd_script_root() -> Optional[str]:
    env = os.environ.get("OPENOCD_SCRIPTS")
    if env:
        return env
    openocd = _which_or_die("openocd")
    candidate = Path(openocd).resolve().parent.parent / "share" / "openocd" / "scripts"
    if candidate.is_dir():
        return str(candidate)
    return None


def _build_openocd_cmd(project_desc: Dict[str, Any]) -> List[str]:
    args = project_desc.get("debug_arguments_openocd", "").strip()
    if not args:
        target = project_desc.get("target", "esp32c6")
        args = f"-f board/{target}-builtin.cfg"
    cmd = [_which_or_die("openocd")] + args.split()
    script_root = _openocd_script_root()
    if script_root:
        cmd[1:1] = ["-s", script_root]
    return cmd


def _build_gdb_cmd(project_desc: Dict[str, Any]) -> List[str]:
    prefix = project_desc.get("monitor_toolprefix", "riscv32-esp-elf-")
    gdb = _which_or_die(f"{prefix}gdb")
    gdbinit_files: Dict[str, str] = project_desc.get("gdbinit_files", {})
    if not gdbinit_files:
        raise SystemExit("gdbinit_files missing in project_description.json — run idf.py build")

    cmd: List[str] = [gdb, "-q"]
    for key in sorted(gdbinit_files.keys()):
        if key == "04_connect" and EXTRA_GDBINIT.is_file():
            cmd.append(f"-x={EXTRA_GDBINIT}")
        cmd.append(f"-x={gdbinit_files[key]}")
    return cmd


def _wait_for_port(port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return True
        except OSError:
            time.sleep(0.1)
    return False


def _tail_openocd_log(log_path: Path, lines: int = 30) -> None:
    if not log_path.is_file():
        return
    try:
        text = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return
    if not text:
        return
    print(f"\n--- last {min(lines, len(text))} lines of {log_path.name} ---", file=sys.stderr)
    for line in text[-lines:]:
        print(line, file=sys.stderr)


def main() -> int:
    app_dir = _app_dir()
    build_dir = app_dir / "build"
    project_desc = _load_project_desc(app_dir)
    target = project_desc.get("target", "?")

    openocd_cmd = _build_openocd_cmd(project_desc)
    gdb_cmd = _build_gdb_cmd(project_desc)
    log_path = build_dir / OPENOCD_LOG

    print(f"esp_foc_debug: app={app_dir.name} target={target}")
    print(f"  OpenOCD: {' '.join(openocd_cmd)}")
    print(f"  log:     {log_path}")
    print(f"  GDB:     {' '.join(gdb_cmd)}")
    print("  (UART monitor can stay open in another terminal)\n")

    log_fp = log_path.open("w", encoding="utf-8")
    openocd_proc = subprocess.Popen(
        openocd_cmd,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    def _cleanup(signum: Optional[int] = None) -> None:
        if openocd_proc.poll() is None:
            try:
                os.killpg(os.getpgid(openocd_proc.pid), signal.SIGTERM)
            except (ProcessLookupError, OSError):
                openocd_proc.terminate()
            try:
                openocd_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(openocd_proc.pid), signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    openocd_proc.kill()

    def _on_signal(signum: int, _frame: Any) -> None:
        _cleanup(signum)
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    exit_code = 1
    try:
        if openocd_proc.poll() is not None:
            _tail_openocd_log(log_path)
            raise SystemExit(f"OpenOCD exited early (code {openocd_proc.returncode})")

        if not _wait_for_port(GDB_PORT, timeout_s=30.0):
            _tail_openocd_log(log_path)
            raise SystemExit(f"OpenOCD did not open GDB port :{GDB_PORT} in time")

        print(f"OpenOCD ready on :{GDB_PORT} (pid {openocd_proc.pid})\n")
        exit_code = subprocess.call(gdb_cmd)
    finally:
        log_fp.close()
        _cleanup()
        if openocd_proc.returncode not in (None, 0) and exit_code == 0:
            _tail_openocd_log(log_path)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
