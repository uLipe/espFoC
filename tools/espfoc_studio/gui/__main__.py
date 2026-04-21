"""Entry point for `python -m espfoc_studio.gui`.

Two run modes:
  --demo              Embedded DemoFirmware. Ideal for trying the GUI on
                      a machine with no hardware attached. This is what
                      the README's one-liner uses.
  --port /dev/ttyX    Real serial transport (UART or USB-CDC bridge).

Closing the window stops the demo threads and exits cleanly.
"""

from __future__ import annotations

import argparse
import signal
import sys
from typing import Optional

from ..link import LinkReader, LoopbackTransport
from ..protocol import TunerClient
from .main_window import MainWindow
from .theme import apply_dark_theme


def _parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="python -m espfoc_studio.gui",
                                description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--demo", action="store_true",
                     help="spin an embedded DemoFirmware (no HW required)")
    src.add_argument("--port",
                     help="serial port of a real espFoC target (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=921600,
                   help="baud rate when --port is used")
    p.add_argument("--axis", type=int, default=0,
                   help="axis id the GUI should attach to (0..3)")
    return p.parse_args(argv)


def _setup_demo() -> tuple[TunerClient, Callable[[], None]]:
    from .demo_firmware import DemoFirmware
    host_t, fw_t = LoopbackTransport.pair()
    fw = DemoFirmware(fw_t)
    fw.start()
    # One reader, shared by the TunerClient and the ScopePanel (attached
    # inside MainWindow). DemoFirmware streams scope frames over the
    # same bus via the SCOPE channel.
    reader = LinkReader(host_t)
    reader.start()
    client = TunerClient(reader)

    def shutdown() -> None:
        fw.stop()
        reader.stop()

    return client, shutdown


def _setup_serial(port: str, baud: int, axis: int
                  ) -> tuple[TunerClient, Callable[[], None]]:
    from ..link.transport_serial import SerialTransport
    transport = SerialTransport(port=port, baud=baud)
    reader = LinkReader(transport)
    reader.start()
    client = TunerClient(reader, axis=axis)

    def shutdown() -> None:
        reader.stop()

    return client, shutdown


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)

    # Import Qt lazily so tests that only exercise the library layers do
    # not have to pay the GUI import cost.
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    if args.demo:
        client, shutdown = _setup_demo()
        title = "espFoC TunerStudio — DEMO (simulated firmware)"
    else:
        client, shutdown = _setup_serial(args.port, args.baud, args.axis)
        title = f"espFoC TunerStudio — {args.port} @ {args.baud}"

    app = QApplication.instance() or QApplication(sys.argv)
    apply_dark_theme(app)
    window = MainWindow(client, title=title)
    window.show()

    # Allow Ctrl-C in the terminal to close the window.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    # Qt ignores SIGINT while in exec(); a short QTimer drop-through keeps
    # Python's signal handler serviced.
    interrupt_tick = QTimer()
    interrupt_tick.start(200)
    interrupt_tick.timeout.connect(lambda: None)

    exit_code = app.exec()
    shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
