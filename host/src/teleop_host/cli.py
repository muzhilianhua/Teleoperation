"""`teleop-host` command line entry point."""

from __future__ import annotations

import argparse

from . import __version__


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="teleop-host", description="Teleoperation GUI server")
    parser.add_argument("--version", action="version", version=f"teleop-host {__version__}")
    parser.parse_args(argv)


if __name__ == "__main__":
    main()
