"""Clock alignment, recorder and recording store."""

from __future__ import annotations

import random
import time
from pathlib import Path

import pytest

from teleop_host.protocol.messages import Telemetry, TelemetryJoint
from teleop_host.sim.model import HandModel
from teleop_host.teach.clock import ClockFit, Unwrapper
from teleop_host.teach.recorder import Recorder
from teleop_host.teach.store import RecordingStore


def test_clock_fit_follows_the_lower_envelope() -> None:
    rng = random.Random(1)
    fit = ClockFit()
    a_true, b_true = 1.00005, 1234.5  # 50 ppm drift
    for i in range(1000):  # 10 s at 100 Hz
        device = 3.0 + i / 100
        delay = 0.002 + rng.expovariate(1 / 0.004)  # >= 2 ms, mostly a few ms
        fit.add(device, a_true * device + b_true + delay)
    a, b = fit.fit()
    for device in (3.0, 8.0, 13.0):
        assert abs((a * device + b) - (a_true * device + b_true)) < 0.003


def test_unwrapper_crosses_the_32_bit_wrap() -> None:
    u = Unwrapper()
    assert u.seconds(2**32 - 1_000_000) == pytest.approx((2**32 - 1_000_000) / 1e6)
    assert u.seconds(500_000) == pytest.approx((2**32 + 500_000) / 1e6)


class Clock:
    def __init__(self) -> None:
        self.t = 50.0

    def __call__(self) -> float:
        return self.t


def telemetry(cycle: int, goal: float, state: int = 2) -> Telemetry:
    joints = [TelemetryJoint(state=state if i == 1 else 0, goal=goal if i == 1 else 0.0) for i in range(9)]
    return Telemetry(cycle=cycle, t_us=cycle * 10_000, joints=joints)


async def test_record_store_round_trip(tmp_path: Path) -> None:
    clock = Clock()
    model = HandModel(hand="left")
    store = RecordingStore(tmp_path)

    async def dump(hand: str) -> list[str]:
        return model.dump().splitlines()

    rec = Recorder(store, lambda hand: model.device_info(), dump, clock)
    await rec.start(["left"], "pick cup")
    for i in range(100):
        cycle = 100 + i + (2 if i >= 40 else 0)  # cycles 140 and 141 never arrive
        clock.t = 50.0 + (cycle - 100) / 100 + 0.003
        rec.on_message("left", telemetry(cycle, 1000 + i))
        if i == 50:
            rec.marker("grasp")
        rec.on_message("right", telemetry(cycle, 0))  # not recorded
    status = rec.status()
    assert status.recording and status.samples == {"left": 100} and status.dropped == {"left": 2}
    meta = rec.stop()
    assert not rec.recording
    assert meta.name == "pick cup" and meta.id.endswith("_pick-cup")
    assert meta.hands["left"].active == [2] and meta.hands["left"].dropped == 2
    assert meta.markers[0].label == "grasp" and meta.markers[0].t == pytest.approx(0.523, abs=0.01)
    assert "# hand left" in meta.hands["left"].dump

    loaded = store.load(meta.id)
    track = loaded.tracks["left"]
    assert len(track.t) == 100 and track.cycle[40] == 142
    assert track.column(2, "goal")[10] == 1010
    assert track.t[0] == pytest.approx(0.003, abs=0.002) and track.t[-1] == pytest.approx(1.013, abs=0.005)
    assert [m.id for m in store.list()] == [meta.id]
    store.rename(meta.id, "pick red cup")
    assert store.meta(meta.id).name == "pick red cup"
    store.delete(meta.id)
    assert store.list() == []
    with pytest.raises(ValueError):
        store.meta("../etc")


async def test_scheduled_start_and_errors(tmp_path: Path) -> None:
    clock = Clock()
    model = HandModel()

    async def dump(hand: str) -> list[str]:
        return model.dump().splitlines()

    rec = Recorder(RecordingStore(tmp_path), lambda h: model.device_info() if h == "right" else None, dump, clock)
    with pytest.raises(RuntimeError):
        await rec.start(["left"], "x")  # not connected
    await rec.start(["right"], "later", at=time.time() + 1.0)
    rec.on_message("right", telemetry(1, 5))  # before the start: ignored
    assert rec.status().samples == {"right": 0} and rec.status().waiting_s > 0.5
    clock.t += 1.5
    rec.on_message("right", telemetry(2, 5))
    assert rec.status().samples == {"right": 1}
    with pytest.raises(RuntimeError):
        await rec.start(["right"], "again")
    rec.cancel()
    assert not rec.recording
