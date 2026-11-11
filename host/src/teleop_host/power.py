"""Servo power monitoring for both hands, from the 100 Hz telemetry.

Supply trouble shows up as `supply voltage`, `servo voltage error` or
`comm lost` when the rail sags under load. To tell which, this keeps the last
seconds of every hand's voltages and currents on the host clock (the hands
share one) and, when a fault switches joints off, records what both hands
did in the second before it.

Current estimate: an H-bridge draws about phase current x drive duty from the
supply (the load register is the duty). Only HLS3606 servos report current,
so the estimate leaves out the SCS0009 joints, and it inherits the 6.5 mA per
unit scale, which is not verified on the bench yet.
"""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from statistics import median
from typing import Any

from .protocol.messages import DeviceInfo, Event, EventCode, JointFlag, Telemetry

WINDOW_S = 3.0  # history kept per hand
LOOKBACK_S = 1.0  # what a trip record and the live peaks cover
MAX_TRIPS = 50


@dataclass(frozen=True, slots=True)
class PowerSample:
    t: float  # host monotonic seconds
    median_v: float | None  # of the servos with a fresh reading
    min_v: float | None
    min_v_joint: int  # 1-based joint index, 0 = none
    supply_a: float  # estimated supply current of the HLS servos
    phase_a: float  # sum of |phase current| of the HLS servos
    armed: int


@dataclass(slots=True)
class HandWindow:
    """One hand over a look-back window."""

    min_median_v: float | None = None
    min_v: float | None = None
    min_v_joint: int = 0
    peak_supply_a: float = 0.0
    peak_phase_a: float = 0.0
    now_median_v: float | None = None
    now_supply_a: float = 0.0
    armed: int = 0
    samples: int = 0


@dataclass(slots=True)
class TripRecord:
    wall_time: float  # epoch seconds
    hand: str
    fault: str
    joint: int  # 0 = whole hand
    text: str
    hands: dict[str, HandWindow] = field(default_factory=dict)  # both hands, LOOKBACK_S before the trip


def sample_of(t: Telemetry, now: float) -> PowerSample:
    # (volts, 1-based joint index) of every servo with a fresh reading
    volts = [(j.voltage_deci_v / 10, i + 1) for i, j in enumerate(t.joints) if j.flags & JointFlag.SERVO_FRESH]
    lowest = min(volts, default=None)
    hls = [j for j in t.joints if j.flags & JointFlag.HAS_CURRENT and j.flags & JointFlag.SERVO_FRESH]
    supply = sum(abs(j.current_ma) * abs(j.load_deci_pct) / 1000 for j in hls) / 1000
    phase = sum(abs(j.current_ma) for j in hls) / 1000
    return PowerSample(
        now,
        median(v for v, _ in volts) if volts else None,
        lowest[0] if lowest else None,
        lowest[1] if lowest else 0,
        supply,
        phase,
        t.armed_count,
    )


def summarize(samples: list[PowerSample], servo_ids: list[int]) -> HandWindow:
    w = HandWindow(samples=len(samples))
    for s in samples:
        if s.median_v is not None and (w.min_median_v is None or s.median_v < w.min_median_v):
            w.min_median_v = s.median_v
        if s.min_v is not None and (w.min_v is None or s.min_v < w.min_v):
            w.min_v = s.min_v
            w.min_v_joint = servo_ids[s.min_v_joint - 1] if 0 < s.min_v_joint <= len(servo_ids) else s.min_v_joint
        w.peak_supply_a = max(w.peak_supply_a, s.supply_a)
        w.peak_phase_a = max(w.peak_phase_a, s.phase_a)
    if samples:
        last = samples[-1]
        w.now_median_v, w.now_supply_a, w.armed = last.median_v, last.supply_a, last.armed
    return w


class PowerMonitor:
    """Feed it (hand, message) pairs; read summary() and trips."""

    def __init__(
        self, info_of: Callable[[str], DeviceInfo | None], clock: Callable[[], float] = time.monotonic
    ) -> None:
        self._info_of = info_of
        self._clock = clock
        self._history: dict[str, deque[PowerSample]] = {}
        self.trips: deque[TripRecord] = deque(maxlen=MAX_TRIPS)
        self._listeners: list[Callable[[TripRecord], None]] = []

    def on_trip(self, callback: Callable[[TripRecord], None]) -> None:
        self._listeners.append(callback)

    def on_message(self, hand: str | None, message: Any) -> None:
        if hand is None:
            return
        now = self._clock()
        if isinstance(message, Telemetry):
            history = self._history.setdefault(hand, deque())
            history.append(sample_of(message, now))
            while history and now - history[0].t > WINDOW_S:
                history.popleft()
        elif isinstance(message, Event) and message.code == EventCode.DISARMED and message.args[0] != 0:
            self._record(hand, message, now)

    def window(self, hand: str, seconds: float = LOOKBACK_S, until: float | None = None) -> HandWindow:
        end = self._clock() if until is None else until
        samples = [s for s in self._history.get(hand, ()) if end - seconds <= s.t <= end]
        info = self._info_of(hand)
        return summarize(samples, [j.servo_id for j in info.joints] if info else [])

    def summary(self) -> dict[str, HandWindow]:
        return {hand: self.window(hand) for hand in self._history}

    def _record(self, hand: str, event: Event, now: float) -> None:
        info = self._info_of(hand)
        fault = event.args[0]
        name = info.faults[fault] if info and fault < len(info.faults) else f"fault {fault}"
        record = TripRecord(
            time.time(), hand, name, event.joint, event.text, {h: self.window(h, until=now) for h in self._history}
        )
        self.trips.append(record)
        for callback in self._listeners:
            callback(record)
