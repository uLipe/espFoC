#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Smoke-test espfoc_shell + FITL on QEMU (esp32 + GPTimer).

  . $IDF_PATH/export.sh
  cd examples/axis_shell
  idf.py set-target esp32
  idf.py build
  python3 ../../scripts/run_shell_qemu.py
"""
from __future__ import annotations

import os
import re
import signal
import sys
import time
from pathlib import Path

try:
    import pexpect
except ImportError:
    print("run_shell_qemu.py requires pexpect (pip install pexpect)")
    sys.exit(2)

PROMPT = "espFoC>"
READY_RE = re.compile(r"ready|espfoc shell|try: list axis", re.I)
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_APP_DIR = SCRIPT_DIR.parent / "examples" / "axis_shell"


def _app_dir() -> Path:
    cwd = Path.cwd()
    if (cwd / "build" / "project_description.json").is_file():
        return cwd
    if (DEFAULT_APP_DIR / "build" / "project_description.json").is_file():
        return DEFAULT_APP_DIR
    raise SystemExit(
        "Run from an ESP-IDF app with build/ (e.g. cd examples/axis_shell && idf.py build)."
    )


def _spawn_qemu(cwd: Path) -> pexpect.spawn:
    common = dict(timeout=30, encoding="utf-8", codec_errors="replace", cwd=str(cwd))
    try:
        return pexpect.spawn("idf.py --no-hints qemu", start_new_session=True, **common)
    except (TypeError, PermissionError):
        return pexpect.spawn("idf.py --no-hints qemu", **common)


def _teardown(child: pexpect.spawn) -> None:
    if child.closed:
        return
    if child.isalive():
        try:
            child.sendcontrol("c")
            child.expect(pexpect.TIMEOUT, timeout=2)
        except (pexpect.TIMEOUT, pexpect.EOF, OSError):
            pass
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGTERM)
        except (ProcessLookupError, OSError):
            try:
                os.kill(child.pid, signal.SIGTERM)
            except (ProcessLookupError, OSError):
                pass
        deadline = time.monotonic() + 8.0
        while child.isalive() and time.monotonic() < deadline:
            time.sleep(0.1)
    child.close(force=True)


def _send_cmd(child: pexpect.spawn, cmd: str, expect: str, timeout: float = 15.0) -> None:
    child.sendline(cmd)
    child.expect(expect, timeout=timeout)


def main() -> int:
    app_dir = _app_dir()
    os.chdir(app_dir)
    child = None
    try:
        child = _spawn_qemu(app_dir)
        child.logfile = sys.stdout
        child.expect(READY_RE, timeout=120)
        child.expect(PROMPT, timeout=30)

        _send_cmd(child, "list axis", r"registered axes:")
        _send_cmd(child, "list axis 0", r"state = idle")
        _send_cmd(child, "help", r"Grammar:")
        _send_cmd(child, "align 0", r"ok", timeout=30)
        _send_cmd(child, "set iq 0.5 0", r"ok")
        _send_cmd(child, "run 0", r"ok", timeout=10)
        _send_cmd(child, "stop 0", r"ok", timeout=10)
        child.sendline("list axis 0")
        child.expect(r"state = (idle|aligned|running)", timeout=10)

        print("shell_qemu: all checks passed")
        return 0
    except (pexpect.TIMEOUT, pexpect.EOF) as exc:
        print(f"shell_qemu: FAILED — {exc}")
        return 1
    finally:
        if child is not None:
            _teardown(child)


if __name__ == "__main__":
    sys.exit(main())
