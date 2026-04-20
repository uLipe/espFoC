#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Run the tuner_demo binary in QEMU and capture the output until the
"Demo complete" sentinel appears (or 30 s timeout).
Use from examples/tuner_demo with IDF env sourced:
  . $IDF_PATH/export.sh
  python run_qemu.py
"""
import os
import sys
try:
    import pexpect
except ImportError:
    print("requires pexpect: pip install pexpect")
    sys.exit(2)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    os.chdir(here)
    child = pexpect.spawn("idf.py qemu", timeout=60,
                          encoding="utf-8", codec_errors="replace", cwd=here)
    child.logfile = sys.stdout
    try:
        child.expect("=== Demo complete ===", timeout=30)
        return 0
    except (pexpect.TIMEOUT, pexpect.EOF) as e:
        print(f"\nFailed to reach demo completion: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
