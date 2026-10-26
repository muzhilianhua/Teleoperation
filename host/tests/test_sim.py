"""The fake device speaks the host link like the firmware does."""

from __future__ import annotations

from teleop_host.protocol.frames import Frame, FrameDecoder, encode_frame
from teleop_host.protocol.messages import (
    Ack,
    AckStatus,
    Bye,
    ByeReason,
    Config,
    DeviceInfo,
    Done,
    Event,
    EventCode,
    HostHello,
    MsgType,
    SlowStatus,
    Telemetry,
    decode_device_message,
)
from teleop_host.sim import FakeDevice


class Session:
    """Drives a non-realtime FakeDevice frame by frame."""

    def __init__(self) -> None:
        self.dev = FakeDevice(realtime=False)
        self.decoder = FrameDecoder()
        self.seq = 0
        self.now = 1000.0
        self.raw = b""

    def receive(self) -> list:
        out = []
        while chunk := self.dev.read(timeout=0):
            self.raw += chunk
            for item in self.decoder.feed(chunk):
                if isinstance(item, Frame):
                    out.append(decode_device_message(item.type, item.payload))
        return out

    def send(self, type_: MsgType, payload: bytes = b"") -> int:
        self.seq += 1
        self.dev.write(encode_frame(type_, self.seq, payload))
        return self.seq

    def command(self, text: str) -> list:
        seq = self.send(MsgType.COMMAND, text.encode())
        return [
            m
            for m in self.receive()
            if getattr(m, "ack_seq", None) == seq
            or getattr(m, "cmd_seq", 0) == seq
            or type(m).__name__ in ("Log", "Config")
        ]

    def open(self) -> DeviceInfo:
        self.dev.write(b"host on\n")
        hello = [m for m in self.receive() if isinstance(m, DeviceInfo)]
        assert len(hello) == 1
        self.send(MsgType.HOST_HELLO, HostHello(telemetry_divider=1, slow_divider=10).encode())
        assert isinstance(self.receive()[-1], Ack)
        return hello[0]


def done(messages: list) -> Done:
    return next(m for m in messages if isinstance(m, Done))


def test_handshake_and_streams() -> None:
    s = Session()
    info = s.open()
    assert len(info.joints) == 9 and info.models[2] == "HLS3606"
    assert info.hand == 1 and info.board_id
    for _ in range(10):
        s.dev.step(s.now)
    msgs = s.receive()
    assert sum(isinstance(m, Telemetry) for m in msgs) == 10
    assert sum(isinstance(m, SlowStatus) for m in msgs) == 1


def test_command_ack_events_done_order() -> None:
    s = Session()
    s.open()
    msgs = s.command("set 2 enc 3")
    assert isinstance(msgs[0], Ack) and msgs[0].status == AckStatus.ACCEPTED
    assert isinstance(msgs[1], Event) and msgs[1].code == EventCode.SETTING_CHANGED
    assert isinstance(msgs[-1], Done) and msgs[-1].ok
    bad = s.command("frobnicate")
    assert bad == [Ack(s.seq, AckStatus.PARSE_ERROR, "unknown command (type `help`)")]


def test_calibrate_arm_and_refuse_while_armed() -> None:
    s = Session()
    s.open()
    assert not done(s.command("arm 1")).ok  # not ready yet
    for line in ("cal 1 a", "cal 1 hand a"):
        assert done(s.command(line)).ok
    for _ in range(300):  # the simulated glove moves
        s.dev.step(s.now)
    s.dev.model.joints[0].position = 600
    for line in ("cal 1 b", "cal 1 hand b", "set 1 on", "arm 1"):
        assert done(s.command(line)).ok, line
    refused = s.command("save")
    assert not done(refused).ok
    assert any(isinstance(m, Event) and m.code == EventCode.REFUSED for m in refused)
    assert done(s.command("set 1 vmax 500")).ok  # live parameters stay allowed
    assert done(s.command("disarm")).ok


def test_dump_returns_config_frame() -> None:
    s = Session()
    s.open()
    msgs = s.command("dump")
    config = next(m for m in msgs if isinstance(m, Config))
    assert config.text.startswith("defaults\n# hand right\n# 1 sim_j1\nset 1 enc 3\n")
    assert config.text.rstrip().endswith("# end of dump: run `save` to store it")
    assert done(msgs).ok


def test_heartbeat_loss_returns_to_text_and_stops_an_armed_hand() -> None:
    s = Session()
    s.open()
    joint = s.dev.model.joints[0]
    joint.state, joint.torque = 2, True
    s.dev.step(s.dev._last_rx + 2.0)
    msgs = s.receive()
    assert Bye(ByeReason.HEARTBEAT_TIMEOUT) in msgs
    assert not s.dev.host_mode and s.dev.estops == 1 and not joint.torque
    assert b"link lost, torque off, text console" in s.raw


def test_estop_frame() -> None:
    s = Session()
    s.open()
    seq = s.send(MsgType.ESTOP)
    msgs = s.receive()
    assert any(isinstance(m, Event) and m.code == EventCode.TORQUE_OFF for m in msgs)
    assert Ack(seq, AckStatus.ACCEPTED) in msgs


def test_left_hand_simulator() -> None:
    s = Session()
    s.dev = FakeDevice(realtime=False, hand="left")
    info = s.open()
    assert info.hand == 2
    config = next(m for m in s.command("dump") if isinstance(m, Config))
    assert "# hand left" in config.text
