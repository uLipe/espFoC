"""Entry point: ``python -m espfoc_tool.gui``."""

from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path
from typing import Optional

from PySide6.QtGui import QIcon

from .app import create_application
from .connection_manager import ConnectionManager
from .main_window import MainWindow


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="python -m espfoc_tool.gui",
        description="espFoC Tool — motor control GUI (optional --port).",
    )
    p.add_argument(
        "--port",
        default=None,
        help="serial port (e.g. /dev/ttyACM0); omit to auto-scan USB",
    )
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--axis", type=int, default=0)
    p.add_argument(
        "--no-gl",
        action="store_true",
        help="disable OpenGL plot rendering",
    )
    p.add_argument(
        "--scope-csv",
        action="store_true",
        help="legacy SCOPE CSV decode (CONFIG_ESP_FOC_SCOPE_LEGACY_CSV)",
    )
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    if args.no_gl:
        os.environ["ESPFOC_TOOL_NO_GL"] = "1"
    if args.scope_csv:
        os.environ["ESPFOC_TOOL_SCOPE_CSV"] = "1"

    from PySide6.QtCore import QTimer

    app = create_application()
    for icon_name in ("espfoc_tool_logo.svg", "espfoc_tool_logo.png"):
        icon_path = Path(__file__).resolve().parents[3] / "doc" / "images" / icon_name
        if icon_path.is_file():
            app.setWindowIcon(QIcon(str(icon_path)))
            break
    conn = ConnectionManager(
        baud=args.baud, axis=args.axis, fixed_port=args.port)
    window = MainWindow(conn, title="espFoC Tool")
    window.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    tick = QTimer()
    tick.start(200)
    tick.timeout.connect(lambda: None)

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
