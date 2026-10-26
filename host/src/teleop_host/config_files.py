"""Calibration backups: the firmware's `dump` text in files under calib/.

Same format and rules as tools/calib.py, which this replaces: a backup is the
dump block (`defaults` ... `# end of dump`) behind a header of `#` lines, so
it can also be pasted into the text console by hand.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

DUMP_START = "defaults"
DUMP_END = "# end of dump"
DEFAULT_DIR = Path(__file__).resolve().parents[3] / "calib"  # Teleoperation/calib, shared with tools/calib.py
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")


def extract_dump(lines: list[str]) -> list[str] | None:
    """The dump block without interleaved output, or None if incomplete."""
    try:
        start = lines.index(DUMP_START)
    except ValueError:
        return None
    block: list[str] = []
    for raw in lines[start:]:
        line = raw.strip()
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
        if len(tokens) < 3 or tokens[0] != "set":
            continue
        if tokens[1].isdigit():
            key, value = tokens[2], " ".join(tokens[3:])
            if key in ("on", "off"):
                key, value = "enabled", key
            settings[(tokens[1], key)] = value
        else:
            settings[("global", tokens[1])] = " ".join(tokens[2:])
    return settings


def hand_of_dump(dump: list[str]) -> str:
    """The `# hand <name>` line; dumps from before two-hand firmware are the right hand's."""
    for line in dump:
        if line.startswith("# hand "):
            return line.split()[2]
    return "right"


def joint_names(dump: list[str]) -> dict[str, str]:
    names: dict[str, str] = {}
    for line in dump:
        tokens = line.lstrip("#").split()
        if line.startswith("#") and len(tokens) == 2 and tokens[0].isdigit():
            names[tokens[0]] = tokens[1]
    return names


@dataclass(frozen=True, slots=True)
class Change:
    scope: str  # joint id or "global"
    key: str
    current: str | None
    wanted: str | None


def _order(item: tuple[str, str]) -> tuple[int, str]:
    return (int(item[0]) if item[0].isdigit() else 999, item[1])


def diff(current: list[str], wanted: list[str]) -> list[Change]:
    a, b = settings_of(current), settings_of(wanted)
    return [Change(k[0], k[1], a.get(k), b.get(k)) for k in sorted(set(a) | set(b), key=_order) if a.get(k) != b.get(k)]


def commands_of(dump: list[str]) -> list[str]:
    return [line for line in dump if not line.startswith("#")]


@dataclass(frozen=True, slots=True)
class Backup:
    name: str
    path: Path
    saved: str
    tag: str
    note: str
    dump: list[str]

    @property
    def hand(self) -> str:
        return hand_of_dump(self.dump)


class BackupStore:
    """Backups of one hand. `legacy` is a read-only directory also listed (older flat backups)."""

    def __init__(self, directory: Path = DEFAULT_DIR, legacy: Path | None = None) -> None:
        self.directory = directory
        self.legacy = legacy

    def list(self) -> list[Backup]:
        paths = [p for d in (self.directory, self.legacy) if d is not None and d.is_dir() for p in d.glob("*.txt")]
        return [b for p in sorted(paths, key=lambda p: p.name, reverse=True) if (b := self._read(p)) is not None]

    def get(self, name: str) -> Backup:
        path = self._path(name)
        if not path.is_file() and self.legacy is not None:
            path = self.legacy / path.name
        backup = self._read(path)
        if backup is None:
            raise FileNotFoundError(f"{name}: not a backup with a complete dump")
        return backup

    def save(self, dump: list[str], tag: str = "", note: str = "", source: str = "") -> Backup:
        stamp = datetime.now()
        tag = _SAFE_NAME.sub("-", tag).strip("-")
        name = f"{stamp:%Y%m%d-%H%M%S}{'_' + tag if tag else ''}.txt"
        header = ["# Teleoperation calibration backup", f"# saved {stamp:%Y-%m-%d %H:%M:%S} from {source or 'GUI'}"]
        if tag:
            header.append(f"# tag: {tag}")
        if note:
            header.append(f"# note: {' '.join(note.splitlines())}")
        header.append("# restore: GUI Config Store, uv run tools/calib.py restore <file>, or paste it and `save`")
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / name
        path.write_text("\n".join(header + dump) + "\n")
        return self.get(name)

    def _path(self, name: str) -> Path:
        if Path(name).name != name or not name.endswith(".txt"):
            raise ValueError(f"invalid backup name {name!r}")
        return self.directory / name

    @staticmethod
    def _read(path: Path) -> Backup | None:
        try:
            lines = path.read_text().splitlines()
        except OSError:
            return None
        dump = extract_dump(lines)
        if dump is None:
            return None
        meta = {"saved": "", "tag": "", "note": ""}
        for line in lines:
            for key in meta:
                prefix = f"# {key}: " if key != "saved" else "# saved "
                if line.startswith(prefix) and not meta[key]:
                    meta[key] = line[len(prefix) :]
        return Backup(path.name, path, meta["saved"], meta["tag"], meta["note"], dump)


class HandBackups:
    """calib/<hand>/*.txt; flat calib/*.txt from before the second hand count as the right hand's."""

    def __init__(self, root: Path = DEFAULT_DIR) -> None:
        self.root = root

    def for_hand(self, hand: str) -> BackupStore:
        return BackupStore(self.root / hand, legacy=self.root if hand == "right" else None)
