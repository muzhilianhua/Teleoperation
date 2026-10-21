# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial"]
# ///
"""Calibration backup / restore for the teleoperation firmware.

The firmware's ``dump`` prints every setting as console commands; pasting
them back and running ``save`` restores them. This tool does both over the
USB console so calibrations can live in files (and in git):

    uv run tools/calib.py backup --tag woody          # -> calib/20261021-143005_woody.txt
    uv run tools/calib.py diff calib/20261021-143005_woody.txt
    uv run tools/calib.py restore calib/20261021-143005_woody.txt
    uv run tools/calib.py ports

``restore`` refuses a file made for a different joint table (ids or names in
``config.h`` changed, e.g. the rebuilt hand) unless ``--force``. It shows what
will change, asks before applying, then applies the commands, dumps again to
verify, and only then runs ``save``. If anything is rejected it sends
``load`` so the RAM settings fall back to what NVS holds.

Close ``pio device monitor`` first: only one program can hold the port.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A
DUMP_START = "defaults"
DUMP_END = "# end of dump"
ERROR_MARKERS = ("error:", "rejected", "refused", "FAILED", "no joint with")
PROJECT_DIR = Path(__file__).resolve().parent.parent
LINE_PACING_S = 0.03  # the console handles pasted bursts, but keep its queue short


class Console:
    """Line-oriented access to the firmware's USB console."""

    def __init__(self, port: str, baud: int) -> None:
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baud
        self._serial.timeout = 0.05
        # De-asserted before open, so opening the native USB port does not
        # reset the ESP32-S3.
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()
        self._pending = b""

    @property
    def port(self) -> str:
        return self._serial.port

    def close(self) -> None:
        self._serial.close()

    def send(self, line: str) -> None:
        self._serial.write(line.encode() + b"\n")
        self._serial.flush()

    def read_lines(self, seconds: float, until: str | None = None) -> list[str]:
        """Lines received within `seconds`; stops early after a line starting with `until`."""
        lines: list[str] = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._pending += self._serial.read(4096)
            while b"\n" in self._pending:
                raw, self._pending = self._pending.split(b"\n", 1)
                line = raw.decode(errors="replace").rstrip("\r")
                lines.append(line)
                if until is not None and line.startswith(until):
                    return lines
        return lines


def extract_dump(lines: list[str]) -> list[str] | None:
    """The dump block (``defaults`` .. end marker), without interleaved output."""
    try:
        start = lines.index(DUMP_START)
    except ValueError:
        return None
    block: list[str] = []
    for line in lines[start:]:
        line = line.strip()
        # [ctl] messages and monitor rows can interleave; dump lines are only these.
        if line == DUMP_START or line.startswith(("set ", "#")):
            block.append(line)
        if line.startswith(DUMP_END):
            return block
    return None


def settings_of(dump: list[str]) -> dict[tuple[str, str], str]:
    """`set` commands as {(joint id or "global", key): value}."""
    settings: dict[tuple[str, str], str] = {}
    for line in dump:
        tokens = line.split()
        if not tokens or tokens[0] != "set":
            continue
        if len(tokens) >= 3 and tokens[1].isdigit():
            key = tokens[2]
            value = " ".join(tokens[3:])
            if key in ("on", "off"):  # `set <j> on|off`
                key, value = "enabled", key
            settings[(tokens[1], key)] = value
        elif len(tokens) >= 3:
            settings[("global", tokens[1])] = " ".join(tokens[2:])
    return settings


def joint_names(dump: list[str]) -> dict[str, str]:
    """Joint table from the ``# <id> <name>`` lines."""
    names: dict[str, str] = {}
    for line in dump:
        tokens = line.lstrip("#").split()
        if line.startswith("#") and len(tokens) == 2 and tokens[0].isdigit():
            names[tokens[0]] = tokens[1]
    return names


def sort_key(item: tuple[str, str]) -> tuple[int, str]:
    scope, key = item
    return (int(scope) if scope.isdigit() else 999, key)


def describe_changes(current: dict, wanted: dict) -> list[str]:
    changes = []
    for item in sorted(set(current) | set(wanted), key=sort_key):
        old, new = current.get(item), wanted.get(item)
        if old == new:
            continue
        scope = f"joint {item[0]}" if item[0].isdigit() else "global"
        changes.append(f"  {scope:8} {item[1]:8} {old or '-':>16}  ->  {new or '-'}")
    return changes


def read_dump(console: Console) -> list[str]:
    for attempt in range(2):
        console.read_lines(0.2)  # drop whatever was already queued
        console.send("dump")
        dump = extract_dump(console.read_lines(3.0, until=DUMP_END))
        if dump is not None:
            return dump
        if attempt == 0:
            time.sleep(2.0)  # the board may just have booted
    sys.exit("no dump from the device: is the firmware running and the port right?")


def read_file_dump(path: Path) -> list[str]:
    dump = extract_dump(path.read_text().splitlines())
    if dump is None:
        sys.exit(f"{path}: no dump block (`defaults` ... `{DUMP_END}`)")
    return dump


def errors_in(lines: list[str]) -> list[str]:
    return [line for line in lines if any(marker in line for marker in ERROR_MARKERS)]


def pick_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = [p for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
    if len(ports) == 1:
        return ports[0].device
    found = ", ".join(p.device for p in ports) or "none"
    sys.exit(f"pass --port: {len(ports)} Espressif port(s) found ({found}); `ports` lists all")


def connect(args: argparse.Namespace) -> Console:
    port = pick_port(args.port)
    try:
        return Console(port, args.baud)
    except serial.SerialException as exc:
        sys.exit(f"cannot open {port}: {exc}\n(close `pio device monitor` first)")


def cmd_backup(args: argparse.Namespace) -> None:
    console = connect(args)
    try:
        dump = read_dump(console)
    finally:
        console.close()
    stamp = datetime.now()
    if args.output:
        path = Path(args.output)
    else:
        suffix = f"_{args.tag}" if args.tag else ""
        path = PROJECT_DIR / "calib" / f"{stamp:%Y%m%d-%H%M%S}{suffix}.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# Teleoperation calibration backup",
        f"# saved {stamp:%Y-%m-%d %H:%M:%S} from {console.port}",
    ]
    if args.tag:
        header.append(f"# tag: {args.tag}")
    if args.note:
        header.append(f"# note: {args.note}")
    header.append("# restore: uv run tools/calib.py restore <this file> (or paste it, then `save`)")
    path.write_text("\n".join(header + dump) + "\n")
    names = joint_names(dump)
    enabled = [j for (j, key), value in settings_of(dump).items() if key == "enabled" and value == "on"]
    print(f"saved {path}")
    print(f"  {len(names)} joints, enabled: {', '.join(sorted(enabled, key=int)) or 'none'}")


def cmd_diff(args: argparse.Namespace) -> None:
    wanted = read_file_dump(Path(args.file))
    console = connect(args)
    try:
        current = read_dump(console)
    finally:
        console.close()
    if joint_names(current) != joint_names(wanted):
        print("joint table differs (ids / names in config.h):")
        print(f"  device {joint_names(current)}\n  file   {joint_names(wanted)}")
    changes = describe_changes(settings_of(current), settings_of(wanted))
    print("device matches the file" if not changes else "device -> file:\n" + "\n".join(changes))


def cmd_restore(args: argparse.Namespace) -> None:
    path = Path(args.file)
    wanted = read_file_dump(path)
    console = connect(args)
    try:
        current = read_dump(console)
        if joint_names(current) != joint_names(wanted) and not args.force:
            sys.exit(
                "refused: the file was made for another joint table (ids / names in config.h)\n"
                f"  device {joint_names(current)}\n  file   {joint_names(wanted)}\n"
                "use --force only if the joints really are the same"
            )
        changes = describe_changes(settings_of(current), settings_of(wanted))
        if changes:
            print(f"{path} will change:\n" + "\n".join(changes))
        else:
            print("RAM settings already match the file")
        if not args.yes and input("apply and save? [y/N] ").strip().lower() != "y":
            sys.exit("nothing changed")

        console.read_lines(0.2)
        replies: list[str] = []
        for line in wanted:
            if line.startswith("#"):
                continue
            console.send(line)
            replies += console.read_lines(LINE_PACING_S)
            if errors_in(replies):
                break  # e.g. armed: every further line would be refused too
        replies += console.read_lines(0.5)
        problems = list(dict.fromkeys(errors_in(replies)))
        if problems:
            console.send("load")
            sys.exit(
                "rejected by the firmware (RAM reverted to NVS with `load`):\n  " + "\n  ".join(problems)
            )

        mismatch = describe_changes(settings_of(read_dump(console)), settings_of(wanted))
        if mismatch:
            console.send("load")
            sys.exit("verification failed (RAM reverted with `load`):\n" + "\n".join(mismatch))
        if args.no_save:
            print("applied and verified in RAM; not saved (`save` on the console keeps it)")
            return

        console.send("save")
        result = console.read_lines(3.0, until="[ctl] configuration saved")
        if not any("configuration saved to NVS" in line for line in result):
            sys.exit("save not confirmed:\n  " + "\n  ".join(errors_in(result) or ["no reply"]))
        print("applied, verified and saved to NVS")
    finally:
        console.close()


def cmd_ports(_: argparse.Namespace) -> None:
    for port in list_ports.comports():
        mark = "  <- ESP32" if port.vid == ESPRESSIF_VID else ""
        print(f"{port.device}  {port.description}{mark}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", help="serial port (default: the only Espressif USB port)")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="command", required=True)

    backup = sub.add_parser("backup", help="save the device's settings to a file")
    backup.add_argument("-o", "--output", help="file (default: calib/<timestamp>[_tag].txt)")
    backup.add_argument("--tag", help="short label, e.g. the operator")
    backup.add_argument("--note", help="free text stored in the file header")
    backup.set_defaults(func=cmd_backup)

    diff = sub.add_parser("diff", help="compare a file with the device's RAM settings")
    diff.add_argument("file")
    diff.set_defaults(func=cmd_diff)

    restore = sub.add_parser("restore", help="apply a file, verify, then save to NVS")
    restore.add_argument("file")
    restore.add_argument("--no-save", action="store_true", help="apply and verify in RAM only")
    restore.add_argument("--force", action="store_true", help="ignore a joint-table mismatch")
    restore.add_argument("-y", "--yes", action="store_true", help="do not ask before applying")
    restore.set_defaults(func=cmd_restore)

    ports = sub.add_parser("ports", help="list serial ports")
    ports.set_defaults(func=cmd_ports)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
