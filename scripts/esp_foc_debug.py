#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
OpenOCD + GDB + UART monitor in one tmux session for espFoC ESP-IDF apps.

On ESP32-C6 (and similar) flash/monitor use the **UART** USB adapter; OpenOCD
uses the **native USB Serial/JTAG** (raw USB, not ttyACM). Do not flash via ACM
or OpenOCD will fail with "could not find or open device".

  cd examples/axis_shell
  idf.py set-target esp32c6
  python3 ../../scripts/esp_foc_debug.py -p /dev/ttyUSB0

Runs idf.py build flash, starts OpenOCD, then tmux:
  left  — GDB (Ctrl+C stops continue; quit ends session)
  right — idf.py monitor on UART
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

GDB_PORT = 3333
OPENOCD_LOG = "openocd_debug.log"
TMUX_SESSION = "espfoc-debug"
USJ_VID_PID = "303a:1001"
SCRIPT_DIR = Path(__file__).resolve().parent
EXTRA_GDBINIT = SCRIPT_DIR / "esp_foc_gdbinit.cmd"


def _app_dir() -> Path:
    cwd = Path.cwd()
    if (cwd / "CMakeLists.txt").is_file():
        return cwd
    raise SystemExit(
        "Run from an ESP-IDF app directory (e.g. cd examples/axis_shell)."
    )


def _require_set_target(app_dir: Path) -> None:
    if (app_dir / "sdkconfig").is_file():
        return
    raise SystemExit(
        f"No sdkconfig in {app_dir} — run idf.py set-target <chip> first."
    )


def _load_project_desc(app_dir: Path) -> Dict[str, Any]:
    path = app_dir / "build" / "project_description.json"
    if not path.is_file():
        raise SystemExit(
            "build/project_description.json missing — run idf.py set-target first."
        )
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def _which_or_die(name: str) -> str:
    from shutil import which

    path = which(name)
    if not path:
        raise SystemExit(f"{name} not found on PATH — source $IDF_PATH/export.sh")
    return path


def _idf_path() -> str:
    idf = os.environ.get("IDF_PATH")
    if not idf:
        raise SystemExit("IDF_PATH not set — source $IDF_PATH/export.sh")
    return idf


def _tmux(*args: str) -> subprocess.CompletedProcess[Any]:
    return subprocess.run(["tmux", *args], check=False)


def _kill_tmux_session() -> None:
    try:
        _tmux("kill-session", "-t", TMUX_SESSION)
    except OSError:
        pass


def _glob_ports(patterns: Sequence[str]) -> List[str]:
    found: List[str] = []
    for pat in patterns:
        found.extend(glob.glob(pat))
    return sorted(set(found))


def _resolve_uart_port(explicit: Optional[str], target: str) -> str:
    if explicit:
        if not Path(explicit).exists():
            raise SystemExit(f"Serial port not found: {explicit}")
        return explicit

    env_port = os.environ.get("ESPPORT")
    if env_port:
        if not Path(env_port).exists():
            raise SystemExit(f"ESPPORT={env_port} does not exist")
        return env_port

    by_id = _glob_ports(["/dev/serial/by-id/*"])
    for path in by_id:
        name = Path(path).name.lower()
        if "jtag" in name or "303a" in name:
            continue
        if "usb" in name or "uart" in name or "serial" in name or "cp210" in name or "ch340" in name or "ftdi" in name:
            return path

    tty_usb = _glob_ports(["/dev/ttyUSB*"])
    if tty_usb:
        return tty_usb[0]

    if target.startswith("esp32c6") or target.startswith("esp32c3") or target.startswith("esp32h2"):
        tty_acm = _glob_ports(["/dev/ttyACM*"])
        raise SystemExit(
            "No UART USB adapter found (ttyUSB*).\n"
            "On ESP32-C6: connect **two** USB cables — UART bridge for flash/monitor "
            "(ttyUSB*) and native USB-JTAG for OpenOCD/GDB.\n"
            "Pass the UART port explicitly:  -p /dev/ttyUSB0\n"
            + (f"Found ttyACM* ({', '.join(tty_acm)}) — that is USJ, not for idf.py flash here."
               if tty_acm else "")
        )

    tty_acm = _glob_ports(["/dev/ttyACM*"])
    if len(tty_acm) == 1:
        return tty_acm[0]

    raise SystemExit(
        "Could not detect serial port — set ESPPORT or use -p /dev/ttyUSB0"
    )


def _usb_jtag_present() -> bool:
    try:
        out = subprocess.run(
            ["lsusb"],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return USJ_VID_PID in out.stdout


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
        raise SystemExit("gdbinit_files missing — run idf.py build")

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


def _tail_openocd_log(log_path: Path, lines: int = 40) -> None:
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


def _openocd_help(target: str, uart_port: str) -> None:
    print(
        "\nOpenOCD could not open USB-JTAG. Check:\n"
        f"  • Native USB-JTAG cable plugged in (lsusb should show {USJ_VID_PID})\n"
        f"  • Flash/monitor use UART only: -p {uart_port}  (not ttyACM* on C6)\n"
        "  • No other OpenOCD / idf.py gdb holding the adapter\n"
        "  • udev rules: idf.py docs → JTAG debugging → Configure USB drivers",
        file=sys.stderr,
    )


def _run_idf(app_dir: Path, uart_port: str, *args: str) -> int:
    cmd = ["idf.py", "-p", uart_port, *args]
    print(f"  $ {' '.join(shlex.quote(c) for c in cmd)}")
    return subprocess.call(cmd, cwd=str(app_dir))


def _shell_preamble(app_dir: Path) -> str:
    export_sh = Path(_idf_path()) / "export.sh"
    return f'. {shlex.quote(str(export_sh))} && cd {shlex.quote(str(app_dir))}'


def _start_tmux_debug(app_dir: Path, gdb_cmd: List[str], uart_port: str) -> None:
    _which_or_die("tmux")
    _kill_tmux_session()

    preamble = _shell_preamble(app_dir)
    gdb_line = " ".join(shlex.quote(c) for c in gdb_cmd)
    port_q = shlex.quote(uart_port)
    left_cmd = f"{preamble} && {gdb_line}; tmux kill-session -t {TMUX_SESSION}"
    right_cmd = f"{preamble} && idf.py -p {port_q} monitor"

    _tmux("new-session", "-d", "-s", TMUX_SESSION, "-c", str(app_dir), "bash")
    _tmux("split-window", "-h", "-t", TMUX_SESSION, "-c", str(app_dir))
    _tmux("select-layout", "-t", TMUX_SESSION, "even-horizontal")
    _tmux("send-keys", "-t", f"{TMUX_SESSION}:0.0", left_cmd, "Enter")
    _tmux("send-keys", "-t", f"{TMUX_SESSION}:0.1", right_cmd, "Enter")
    _tmux("select-pane", "-t", f"{TMUX_SESSION}:0.0")

    print(f"tmux '{TMUX_SESSION}': left=GDB (USB-JTAG), right=monitor ({uart_port})")
    print("  Ctrl+C in GDB — stop continue; quit — end session\n")
    _tmux("attach", "-t", TMUX_SESSION)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="espFoC OpenOCD + GDB + UART monitor (tmux)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Dual-USB (ESP32-C6): UART adapter = flash + monitor (-p /dev/ttyUSB*).\n"
            "Native USB-JTAG = OpenOCD/GDB only (do not pass -p to OpenOCD)."
        ),
    )
    parser.add_argument(
        "-p", "--port",
        help="UART port for idf.py flash/monitor (default: ESPPORT or auto ttyUSB*)",
    )
    parser.add_argument(
        "--skip-flash",
        action="store_true",
        help="Skip idf.py build flash",
    )
    args = parser.parse_args()

    app_dir = _app_dir()
    _require_set_target(app_dir)
    _which_or_die("idf.py")

    project_desc: Optional[Dict[str, Any]] = None
    if (app_dir / "build" / "project_description.json").is_file():
        project_desc = _load_project_desc(app_dir)
    target = (project_desc or {}).get("target", "esp32c6")

    uart_port = _resolve_uart_port(args.port, target)
    print(f"esp_foc_debug: app={app_dir.name} target={target}")
    print(f"  UART (flash/monitor): {uart_port}")
    print(f"  JTAG (OpenOCD/GDB):   USB {USJ_VID_PID} (native USB, not ttyACM)")

    if not _usb_jtag_present():
        print(
            f"  warning: {USJ_VID_PID} not seen in lsusb — plug in the JTAG USB before OpenOCD starts",
            file=sys.stderr,
        )

    if not args.skip_flash:
        print("Building and flashing on UART…")
        rc = _run_idf(app_dir, uart_port, "build", "flash")
        if rc != 0:
            return rc
        time.sleep(0.5)
    elif project_desc is None:
        print("No build/ — running idf.py build…")
        rc = _run_idf(app_dir, uart_port, "build")
        if rc != 0:
            return rc

    project_desc = _load_project_desc(app_dir)
    target = project_desc.get("target", target)
    openocd_cmd = _build_openocd_cmd(project_desc)
    gdb_cmd = _build_gdb_cmd(project_desc)
    log_path = app_dir / "build" / OPENOCD_LOG

    print(f"  OpenOCD: {' '.join(openocd_cmd)}")
    print(f"  log:     {log_path}\n")

    log_fp = log_path.open("w", encoding="utf-8")
    openocd_proc = subprocess.Popen(
        openocd_cmd,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    def _cleanup_openocd() -> None:
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

    exit_code = 1
    try:
        if openocd_proc.poll() is not None:
            _tail_openocd_log(log_path)
            _openocd_help(target, uart_port)
            return 1

        if not _wait_for_port(GDB_PORT, timeout_s=30.0):
            _tail_openocd_log(log_path)
            _openocd_help(target, uart_port)
            return 1

        print(f"OpenOCD ready on :{GDB_PORT} (pid {openocd_proc.pid})\n")
        _start_tmux_debug(app_dir, gdb_cmd, uart_port)
        exit_code = 0
    except KeyboardInterrupt:
        print("\nInterrupted — stopping OpenOCD.", file=sys.stderr)
        exit_code = 130
    finally:
        log_fp.close()
        _cleanup_openocd()
        _kill_tmux_session()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
