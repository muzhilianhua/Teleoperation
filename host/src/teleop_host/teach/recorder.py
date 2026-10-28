"""Records the hands' motion under teleoperation.

Fed every (hand, message) from the HandManager on the event loop. While
recording, each TELEMETRY sample of a recorded hand becomes one row; the
row's time is the board's own clock mapped to the host clock (clock.py), so
both hands land on one time axis. Missing cycles (telemetry dropped on the
way) are counted; replay interpolates across them.
"""

from __future__ import annotations

import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

from ..protocol.messages import DeviceInfo, Telemetry
from .clock import ClockFit, Unwrapper
from .store import JOINT_FIELDS, HandMeta, HandTrack, Marker, RecordingMeta, RecordingStore

FOLLOW = 2  # JointState::Follow


@dataclass(slots=True)
class _Hand:
    info: DeviceInfo
    dump: list[str]
    track: HandTrack = field(default_factory=HandTrack)
    clock: ClockFit = field(default_factory=ClockFit)
    unwrap: Unwrapper = field(default_factory=Unwrapper)
    device_s: list[float] = field(default_factory=list)  # per row, mapped to host time at stop
    last_cycle: int | None = None
    dropped: int = 0
    active: set[int] = field(default_factory=set)


@dataclass(slots=True)
class RecorderStatus:
    recording: bool = False
    name: str = ""
    hands: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0
    samples: dict[str, int] = field(default_factory=dict)
    dropped: dict[str, int] = field(default_factory=dict)
    markers: list[Marker] = field(default_factory=list)
    waiting_s: float = 0.0  # until a scheduled start


class Recorder:
    def __init__(
        self,
        store: RecordingStore,
        info_of: Callable[[str], DeviceInfo | None],
        dump_of: Callable[[str], Awaitable[list[str]]],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.store = store
        self._info_of = info_of
        self._dump_of = dump_of
        self._clock = clock
        self._hands: dict[str, _Hand] = {}
        self._name = ""
        self._start: float | None = None  # host monotonic
        self._markers: list[Marker] = []

    @property
    def recording(self) -> bool:
        return self._start is not None

    async def start(self, hands: list[str], name: str, at: float | None = None) -> None:
        """Starts recording `hands` now, or at epoch time `at` (time.time()) for scheduled starts."""
        if self.recording:
            raise RuntimeError("already recording")
        if not hands:
            raise ValueError("no hand to record")
        prepared: dict[str, _Hand] = {}
        for hand in hands:
            info = self._info_of(hand)
            if info is None:
                raise RuntimeError(f"{hand} hand not connected")
            prepared[hand] = _Hand(info, await self._dump_of(hand))
        self._hands = prepared
        self._name = name
        self._markers = []
        delay = 0.0 if at is None else max(0.0, at - time.time())
        self._start = self._clock() + delay

    def marker(self, label: str) -> Marker:
        if self._start is None:
            raise RuntimeError("not recording")
        mark = Marker(round(max(0.0, self._clock() - self._start), 3), label)
        self._markers.append(mark)
        return mark

    def on_message(self, hand: str | None, message: Any) -> None:
        if self._start is None or hand not in self._hands or not isinstance(message, Telemetry):
            return
        now = self._clock()
        if now < self._start:
            return
        h = self._hands[hand]
        if h.last_cycle is not None and message.cycle > h.last_cycle + 1:
            h.dropped += message.cycle - h.last_cycle - 1
        h.last_cycle = message.cycle
        device_s = h.unwrap.seconds(message.t_us)
        h.clock.add(device_s, now)
        h.device_s.append(device_s)
        track = h.track
        track.cycle.append(message.cycle)
        track.t_us.append(message.t_us)
        k = h.info.degrees_per_count
        for joint, info in zip(message.joints, h.info.joints, strict=False):
            cols = track.joints.setdefault(info.servo_id, {name: [] for name in JOINT_FIELDS})
            cols["goal"].append(round(joint.goal, 2))
            cols["sent"].append(round(joint.sent_goal, 2))
            cols["pos"].append(joint.position)
            cols["exo_deg"].append(round(joint.exo_filtered_counts * k, 3))
            cols["load"].append(joint.load_deci_pct / 10)
            cols["current"].append(joint.current_ma)
            cols["state"].append(joint.state)
            if joint.state == FOLLOW:
                h.active.add(info.servo_id)

    def status(self) -> RecorderStatus:
        if self._start is None:
            return RecorderStatus()
        now = self._clock()
        return RecorderStatus(
            True,
            self._name,
            list(self._hands),
            round(max(0.0, now - self._start), 2),
            {hand: len(h.device_s) for hand, h in self._hands.items()},
            {hand: h.dropped for hand, h in self._hands.items()},
            list(self._markers),
            round(max(0.0, self._start - now), 2),
        )

    def cancel(self) -> None:
        self._start = None
        self._hands = {}

    def stop(self) -> RecordingMeta:
        """Ends the recording and saves it; returns its metadata."""
        if self._start is None:
            raise RuntimeError("not recording")
        start = self._start
        self._start = None
        tracks: dict[str, HandTrack] = {}
        metas: dict[str, HandMeta] = {}
        duration = 0.0
        for hand, h in self._hands.items():
            a, b = h.clock.fit()
            h.track.t = [round(a * d + b - start, 4) for d in h.device_s]
            if h.track.t:
                duration = max(duration, h.track.t[-1])
            tracks[hand] = h.track
            metas[hand] = HandMeta(
                h.info.firmware,
                h.info.board_id,
                [{"servo_id": j.servo_id, "name": j.name, "model": j.model} for j in h.info.joints],
                sorted(h.active),
                h.dump,
                {"a": a, "b": b - start},
                len(h.device_s),
                h.dropped,
            )
        now = datetime.now()
        meta = RecordingMeta(
            self.store.new_id(self._name, now),
            self._name or "untitled",
            now.isoformat(timespec="seconds"),
            round(duration, 3),
            metas,
            list(self._markers),
        )
        self._hands = {}
        return self.store.save(meta, tracks)
