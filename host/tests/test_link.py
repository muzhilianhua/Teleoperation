"""LinkManager against the fake device, in real time."""

from __future__ import annotations

import threading
import time

import pytest

from teleop_host.link.link import LinkManager, LinkState
from teleop_host.protocol.frames import UnframedText
from teleop_host.protocol.messages import Ack, DeviceInfo, Done, Event, EventCode, MsgType, Telemetry
from teleop_host.sim import FakeDevice


class Recorder:
    def __init__(self) -> None:
        self.messages: list = []
        self.states: list[LinkState] = []
        self.lock = threading.Lock()

    def on_message(self, m) -> None:
        with self.lock:
            self.messages.append(m)

    def on_status(self, s) -> None:
        self.states.append(s.state)

    def wait_for(self, predicate, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for m in self.messages:
                    if predicate(m):
                        return m
            time.sleep(0.01)
        raise AssertionError("timed out waiting for a message")


@pytest.fixture
def device():
    dev = FakeDevice()
    yield dev
    dev.shutdown()


def open_link(device: FakeDevice) -> tuple[LinkManager, Recorder]:
    rec = Recorder()
    link = LinkManager(lambda: device, "sim", rec.on_message, rec.on_status, reconnect=False)
    link.start()
    assert link.wait_connected(3.0)
    return link, rec


def test_connect_command_and_stop(device: FakeDevice) -> None:
    link, rec = open_link(device)
    try:
        assert isinstance(link.info, DeviceInfo) and link.info.firmware == "sim"
        assert any(isinstance(m, UnframedText) and b"[boot]" in m.data for m in rec.messages)
        rec.wait_for(lambda m: isinstance(m, Telemetry))
        seq = link.send(MsgType.COMMAND, b"ping")
        rec.wait_for(lambda m: isinstance(m, Done) and m.ack_seq == seq)
        pings = [m for m in rec.messages if isinstance(m, Event) and m.code == EventCode.PING_RESULT]
        assert len(pings) == 9 and all(p.cmd_seq == seq for p in pings)
    finally:
        link.stop()
    assert not device.host_mode  # HOST_EXIT returned it to text mode
    assert rec.states[-1] == LinkState.DISCONNECTED


def test_heartbeat_keeps_the_session_alive(device: FakeDevice) -> None:
    link, rec = open_link(device)
    try:
        time.sleep(1.6)  # longer than the device's 1 s watchdog
        assert device.host_mode and link.connected
    finally:
        link.stop()


def test_estop_jumps_the_queue(device: FakeDevice) -> None:
    link, rec = open_link(device)
    try:
        seq = link.estop()
        rec.wait_for(lambda m: isinstance(m, Ack) and m.ack_seq == seq)
        assert device.estops == 1
    finally:
        link.stop()


def test_takes_over_a_device_left_in_host_mode(device: FakeDevice) -> None:
    link, _ = open_link(device)
    link._stop.set()  # simulate a crashed host: no HOST_EXIT
    link._thread.join(2.0)  # type: ignore[union-attr]
    assert device.host_mode
    link2, rec2 = open_link(device)  # within the device's heartbeat timeout
    try:
        assert link2.connected and device.host_mode
    finally:
        link2.stop()
