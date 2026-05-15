#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Run espFoC unit tests in QEMU and exit with 0 if all pass, 1 otherwise.

CI and local use:
  . $IDF_PATH/export.sh
  cd examples/unit_test_runner
  idf.py set-target esp32
  idf.py -D TEST_COMPONENTS=espFoC build
  python run_unit_tests_qemu.py
"""
from __future__ import annotations

import os
import re
import signal
import sys
import time

try:
    import pexpect
except ImportError:
    print("run_unit_tests_qemu.py requires pexpect. Install with: pip install pexpect")
    sys.exit(2)

RESULT_RE = re.compile(r"(\d+) Tests (\d+) Failures")
MENU_PROMPT = "Enter test for running"
BOOT_PROMPT = "Press ENTER to see the list of tests."


def _spawn_idf_qemu(cwd: str) -> pexpect.spawn:
    """Start idf.py qemu; prefer a new session so SIGTERM reaches QEMU too."""
    common = dict(
        timeout=30,
        encoding="utf-8",
        codec_errors="replace",
        cwd=cwd,
    )
    try:
        return pexpect.spawn(
            "idf.py --no-hints qemu",
            start_new_session=True,
            **common,
        )
    except (TypeError, PermissionError):
        try:
            return pexpect.spawn(
                "idf.py --no-hints qemu",
                preexec_fn=os.setsid,
                **common,
            )
        except PermissionError:
            return pexpect.spawn("idf.py --no-hints qemu", **common)


def _signal_process_tree(pid: int, sig: int) -> None:
    try:
        os.killpg(os.getpgid(pid), sig)
    except (ProcessLookupError, OSError):
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, OSError):
            pass


def _wait_dead(child: pexpect.spawn, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while child.isalive() and time.monotonic() < deadline:
        time.sleep(0.1)


def _teardown(child: pexpect.spawn) -> None:
    """Stop idf.py and the QEMU child even when Unity returns to the interactive menu."""
    if child.closed:
        return
    if child.isalive():
        try:
            child.sendcontrol("c")
            child.expect(pexpect.TIMEOUT, timeout=2)
        except (pexpect.TIMEOUT, pexpect.EOF, OSError):
            pass
        _signal_process_tree(child.pid, signal.SIGTERM)
        _wait_dead(child, 8.0)
        if child.isalive():
            _signal_process_tree(child.pid, signal.SIGKILL)
            _wait_dead(child, 3.0)
    child.close(force=True)


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    child = None
    exit_code = 1

    try:
        child = _spawn_idf_qemu(script_dir)
        child.logfile = sys.stdout
        try:
            child.expect(BOOT_PROMPT, timeout=180)
        except (pexpect.TIMEOUT, pexpect.EOF) as exc:
            print(f"Failed to reach Unity boot prompt: {exc}")
            return 1

        child.sendline("")
        try:
            child.expect(MENU_PROMPT, timeout=60)
        except (pexpect.TIMEOUT, pexpect.EOF):
            print("Unity menu prompt never appeared")
            return 1

        child.sendline("*")

        try:
            child.expect(RESULT_RE, timeout=600)
        except pexpect.TIMEOUT:
            print("Timeout waiting for Unity summary (tests still running or hung).")
            return 1
        except pexpect.EOF:
            print("QEMU/idf.py exited before Unity printed the summary.")
            return 1

        total = int(child.match.group(1))
        failures = int(child.match.group(2))
        if failures == 0:
            print(f"All unit tests passed ({total} tests).")
            exit_code = 0
        else:
            print(f"Unity reported {failures} failure(s) out of {total} tests.")
            exit_code = 1
    finally:
        if child is not None:
            _teardown(child)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
