#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Run espFoC unit tests in QEMU and exit with 0 if all pass, 1 otherwise.
Use from examples/unit_test_runner with IDF environment sourced:
  . $IDF_PATH/export.sh
  python run_unit_tests_qemu.py
"""
import os
import re
import sys

try:
    import pexpect
except ImportError:
    print("run_unit_tests_qemu.py requires pexpect. Install with: pip install pexpect")
    sys.exit(2)


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    # Run idf.py qemu; it will build first if needed
    child = pexpect.spawn("idf.py qemu", timeout=300, encoding="utf-8",
                          codec_errors="replace", cwd=script_dir)
    child.logfile = sys.stdout

    try:
        child.expect("Press ENTER to see the list of tests.", timeout=120)
    except (pexpect.TIMEOUT, pexpect.EOF) as e:
        print(f"Failed to get Unity menu: {e}")
        return 1

    # Press Enter to print the menu, then wait for Unity's input prompt
    # before sending "*". Doing them back-to-back is race-prone under CI
    # I/O buffering and Unity silently discards the "*" if it arrives
    # before the menu prompt is ready.
    child.sendline("")
    try:
        child.expect("Enter test for running", timeout=60)
    except (pexpect.TIMEOUT, pexpect.EOF):
        print("Unity menu prompt never appeared")
        return 1
    child.sendline("*")

    # Match "0 Failures" in the summary line (success). QEMU runs
    # tests serially; ~200 tests need comfortably more than two minutes
    # on a loaded CI runner, so we budget five.
    try:
        child.expect(re.compile(r"\d+ Tests 0 Failures"), timeout=300)
    except pexpect.TIMEOUT:
        print("Timeout waiting for test results or tests failed (non-zero failures).")
        return 1
    except pexpect.EOF:
        print("Process ended before test results.")
        return 1

    print("All unit tests passed.")
    child.close(force=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
