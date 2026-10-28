"""Recordings on disk: recordings/<YYYYmmdd-HHMMSS>_<name>/meta.json + <hand>.csv.

meta.json holds what replay and later analysis need: the hands, each hand's
firmware, board, joint table, settings dump, clock fit, which joints were
following the glove, sample and dropped-cycle counts, and the markers. Each
<hand>.csv has one row per received control cycle:

  t (s from the start, host clock), cycle, t_us, then per joint (servo id k)
  j<k>_goal, j<k>_sent, j<k>_pos, j<k>_exo_deg, j<k>_load, j<k>_current, j<k>_state

`goal` (rate-limited target before the grip limiter) is what replay plays back.
"""

from __future__ import annotations

import csv
import json
import re
import shutil
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

DEFAULT_DIR = Path(__file__).resolve().parents[4] / "recordings"  # Teleoperation/recordings (git-ignored)
JOINT_FIELDS = ("goal", "sent", "pos", "exo_deg", "load", "current", "state")
_SAFE = re.compile(r"[^A-Za-z0-9_.-]+")


@dataclass(slots=True)
class Marker:
    t: float  # seconds from the start
    label: str


@dataclass(slots=True)
class HandMeta:
    firmware: str
    board_id: str
    joints: list[dict[str, Any]]  # servo_id, name, model
    active: list[int]  # servo ids that followed the glove: the ones replay plays
    dump: list[str]
    clock: dict[str, float]  # a, b of host = a * device + b
    samples: int = 0
    dropped: int = 0


@dataclass(slots=True)
class RecordingMeta:
    id: str
    name: str
    created: str
    duration_s: float
    hands: dict[str, HandMeta]
    markers: list[Marker] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> RecordingMeta:
        hands = {h: HandMeta(**m) for h, m in data["hands"].items()}
        markers = [Marker(**m) for m in data.get("markers", [])]
        return cls(data["id"], data["name"], data["created"], data["duration_s"], hands, markers)


@dataclass(slots=True)
class HandTrack:
    """One hand's samples as columns: t, cycle, t_us and '<field>' per servo id."""

    t: list[float] = field(default_factory=list)
    cycle: list[int] = field(default_factory=list)
    t_us: list[int] = field(default_factory=list)
    joints: dict[int, dict[str, list[float]]] = field(default_factory=dict)

    def column(self, servo_id: int, name: str) -> list[float]:
        return self.joints[servo_id][name]


@dataclass(slots=True)
class Recording:
    meta: RecordingMeta
    tracks: dict[str, HandTrack]


class RecordingStore:
    def __init__(self, directory: Path = DEFAULT_DIR) -> None:
        self.directory = directory

    def new_id(self, name: str, when: datetime | None = None) -> str:
        stamp = (when or datetime.now()).strftime("%Y%m%d-%H%M%S")
        safe = _SAFE.sub("-", name).strip("-")
        base = f"{stamp}_{safe}" if safe else stamp
        rid, n = base, 2
        while (self.directory / rid).exists():
            rid, n = f"{base}-{n}", n + 1
        return rid

    def save(self, meta: RecordingMeta, tracks: dict[str, HandTrack]) -> RecordingMeta:
        folder = self._folder(meta.id)
        folder.mkdir(parents=True, exist_ok=False)
        for hand, track in tracks.items():
            ids = sorted(track.joints)
            with (folder / f"{hand}.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "cycle", "t_us"] + [f"j{k}_{name}" for k in ids for name in JOINT_FIELDS])
                for i in range(len(track.t)):
                    row: list[Any] = [f"{track.t[i]:.4f}", track.cycle[i], track.t_us[i]]
                    for k in ids:
                        row += [_num(track.joints[k][name][i]) for name in JOINT_FIELDS]
                    w.writerow(row)
        (folder / "meta.json").write_text(json.dumps(meta.to_json(), indent=2))
        return meta

    def list(self) -> list[RecordingMeta]:
        if not self.directory.is_dir():
            return []
        metas = []
        for folder in sorted(self.directory.iterdir(), reverse=True):
            meta_file = folder / "meta.json"
            if meta_file.is_file():
                try:
                    metas.append(RecordingMeta.from_json(json.loads(meta_file.read_text())))
                except (ValueError, KeyError, TypeError):
                    continue  # not a recording this version understands
        return metas

    def meta(self, rid: str) -> RecordingMeta:
        path = self._folder(rid) / "meta.json"
        if not path.is_file():
            raise FileNotFoundError(f"no recording {rid!r}")
        return RecordingMeta.from_json(json.loads(path.read_text()))

    def load(self, rid: str) -> Recording:
        meta = self.meta(rid)
        tracks = {}
        for hand in meta.hands:
            track = HandTrack()
            with (self._folder(rid) / f"{hand}.csv").open(newline="") as f:
                reader = csv.reader(f)
                header = next(reader)
                columns: list[tuple[int, str]] = []
                for name in header[3:]:
                    joint, _, fieldname = name.partition("_")
                    servo = int(joint[1:])
                    track.joints.setdefault(servo, {n: [] for n in JOINT_FIELDS})
                    columns.append((servo, fieldname))
                for row in reader:
                    track.t.append(float(row[0]))
                    track.cycle.append(int(row[1]))
                    track.t_us.append(int(row[2]))
                    for (servo, fieldname), value in zip(columns, row[3:], strict=True):
                        track.joints[servo][fieldname].append(float(value))
            tracks[hand] = track
        return Recording(meta, tracks)

    def rename(self, rid: str, name: str) -> RecordingMeta:
        meta = self.meta(rid)
        meta.name = name
        (self._folder(rid) / "meta.json").write_text(json.dumps(meta.to_json(), indent=2))
        return meta

    def delete(self, rid: str) -> None:
        folder = self._folder(rid)
        if not (folder / "meta.json").is_file():
            raise FileNotFoundError(f"no recording {rid!r}")
        shutil.rmtree(folder)

    def _folder(self, rid: str) -> Path:
        if Path(rid).name != rid or rid in ("", ".", ".."):
            raise ValueError(f"invalid recording id {rid!r}")
        return self.directory / rid


def _num(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else f"{v:.3f}"
