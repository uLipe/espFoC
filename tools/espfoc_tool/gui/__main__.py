"""Entry: python -m espfoc_tool.gui (QML / Material 3)."""

from __future__ import annotations

import argparse
import signal
import sys
from typing import Optional

from PySide6.QtCore import QTimer

from .app import create_application, create_qml_engine
from .backend import ScopeBackend


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="espFoC Tool — passive ESPF stream viewer")
    p.add_argument("--port", default=None, help="serial port (omit for picker)")
    p.add_argument("--baud", type=int, default=921600)
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    app = create_application()
    backend = ScopeBackend()
    if args.port:
        backend.set_cli_port(args.port, args.baud)
    engine = create_qml_engine(backend)

    ui_timer = QTimer()
    ui_timer.setInterval(33)
    ui_timer.timeout.connect(lambda: backend.refresh_plots())
    ui_timer.start()

    def _cleanup() -> None:
        backend.stop()

    app.aboutToQuit.connect(_cleanup)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    poll = QTimer()
    poll.start(200)
    poll.timeout.connect(lambda: None)

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
