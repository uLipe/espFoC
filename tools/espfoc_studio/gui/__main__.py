"""Entry point for `python -m espfoc_studio.gui`.

Requires ``--port`` for a serial transport (UART or USB-CDC bridge).

Closing the window stops the link reader and exits cleanly.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from typing import Callable, Optional

from .main_window import MainWindow
from .theme import apply_dark_theme


def _parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="python -m espfoc_studio.gui",
                                description=__doc__)
    p.add_argument("--port", required=True,
                   help="serial port of a real espFoC target (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=921600,
                   help="baud rate")
    p.add_argument("--axis", type=int, default=0,
                   help="axis id the GUI should attach to (0..3)")
    p.add_argument(
        "--scope-csv", action="store_true",
        help="decode legacy SCOPE as CSV (match CONFIG_ESP_FOC_SCOPE_LEGACY_CSV on device)")
    return p.parse_args(argv)


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
    if getattr(args, "scope_csv", False):
        os.environ["ESP_FOC_STUDIO_SCOPE_CSV"] = "1"

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    client, shutdown = _setup_serial(args.port, args.baud, args.axis)
    title = f"espFoC TunerStudio — {args.port} @ {args.baud}"
    link_descr = f"{args.port} @ {args.baud}"
    serial_config = (args.port, args.baud, args.axis)

    app = QApplication.instance() or QApplication(sys.argv)
    apply_dark_theme(app)
    window = MainWindow(
        client, title=title, link_descr=link_descr,
        serial_config=serial_config)
    window.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    interrupt_tick = QTimer()
    interrupt_tick.start(200)
    interrupt_tick.timeout.connect(lambda: None)

    exit_code = app.exec()
    shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
