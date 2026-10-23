"""`teleop-host`: start the GUI server and open the browser.

uv run teleop-host                 # connect to the only ESP32 found, open the GUI
uv run teleop-host --port /dev/cu.usbmodem11101
uv run teleop-host --sim           # simulated device, no hardware
uv run teleop-host --dev           # open the Vite dev server (bun run dev in host/web)
"""

from __future__ import annotations

import argparse
import threading
import webbrowser
from pathlib import Path

import uvicorn

from . import __version__
from .config_files import DEFAULT_DIR, BackupStore
from .device import Device
from .plugins import builtin_plugins
from .server import SIM_PORT, create_app

STATIC_DIR = Path(__file__).resolve().parent / "static"
DEV_SERVER = "http://localhost:5173"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="teleop-host", description="Teleoperation GUI server")
    parser.add_argument("--version", action="version", version=f"teleop-host {__version__}")
    parser.add_argument("--port", help="serial port of the ESP32 (default: the only one found, else pick in the GUI)")
    parser.add_argument("--sim", action="store_true", help="offer a simulated device (and connect to it)")
    parser.add_argument("--bind", default="127.0.0.1:8790", help="host:port to serve on (default 127.0.0.1:8790)")
    parser.add_argument("--backups", type=Path, default=DEFAULT_DIR, help=f"backup directory (default {DEFAULT_DIR})")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    parser.add_argument("--dev", action="store_true", help=f"open the Vite dev server ({DEV_SERVER}) instead")
    args = parser.parse_args(argv)

    host, _, port = args.bind.rpartition(":")
    sim_factory = None
    if args.sim:
        from .sim import FakeDevice

        fake = FakeDevice()
        sim_factory = lambda: fake  # noqa: E731
    initial = SIM_PORT if args.sim and not args.port else args.port
    if initial is None:
        from .link.discovery import find_esp32

        initial = find_esp32()

    app = create_app(
        Device(),
        BackupStore(args.backups),
        builtin_plugins(),
        static_dir=STATIC_DIR,
        sim_factory=sim_factory,
        initial_port=initial,
    )
    url = DEV_SERVER if args.dev else f"http://{host}:{port}"
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, args=(url,)).start()
    print(f"teleop-host {__version__}: {url}  (device: {initial or 'pick one in the GUI'})")
    uvicorn.run(app, host=host or "127.0.0.1", port=int(port), log_level="warning")


if __name__ == "__main__":
    main()
