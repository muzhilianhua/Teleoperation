"""`teleop-host`: start the GUI server and open the browser.

uv run teleop-host                 # connect every ESP32 found (one per hand), open the GUI
uv run teleop-host --port /dev/cu.usbmodem11101 --port /dev/cu.usbmodem11201
uv run teleop-host --sim           # two simulated hands, no hardware
uv run teleop-host --dev           # open the Vite dev server (bun run dev in host/web)
"""

from __future__ import annotations

import argparse
import threading
import webbrowser
from pathlib import Path

import uvicorn

from . import __version__
from .config_files import DEFAULT_DIR, HandBackups
from .hands import HANDS, HandManager
from .link.discovery import find_esp32s
from .plugins import builtin_plugins
from .server import create_app

STATIC_DIR = Path(__file__).resolve().parent / "static"
DEV_SERVER = "http://localhost:5173"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="teleop-host", description="Teleoperation GUI server")
    parser.add_argument("--version", action="version", version=f"teleop-host {__version__}")
    parser.add_argument(
        "--port",
        action="append",
        default=[],
        help="serial port of a hand board; repeat for both (default: every ESP32 found)",
    )
    parser.add_argument("--sim", action="store_true", help="offer two simulated hands (and connect to them)")
    parser.add_argument("--bind", default="127.0.0.1:8790", help="host:port to serve on (default 127.0.0.1:8790)")
    parser.add_argument("--backups", type=Path, default=DEFAULT_DIR, help=f"backup root (default {DEFAULT_DIR})")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    parser.add_argument("--dev", action="store_true", help=f"open the Vite dev server ({DEV_SERVER}) instead")
    args = parser.parse_args(argv)

    host, _, port = args.bind.rpartition(":")
    sim_factories = {}
    if args.sim:
        from .sim import FakeDevice

        for hand in HANDS:
            fake = FakeDevice(hand=hand)
            sim_factories[f"sim-{hand}"] = lambda fake=fake: fake
    initial = args.port or (list(sim_factories) if args.sim else find_esp32s())

    app = create_app(
        HandManager(),
        HandBackups(args.backups),
        builtin_plugins(),
        static_dir=STATIC_DIR,
        sim_factories=sim_factories,
        initial_ports=initial,
    )
    url = DEV_SERVER if args.dev else f"http://{host}:{port}"
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, args=(url,)).start()
    print(f"teleop-host {__version__}: {url}  (boards: {', '.join(initial) or 'pick them in the GUI'})")
    uvicorn.run(app, host=host or "127.0.0.1", port=int(port), log_level="warning")


if __name__ == "__main__":
    main()
