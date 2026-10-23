"""Host link v1 payloads, mirroring lib/HostLink/src/host_messages.{h,cpp}.

Every message has encode() and decode() so the same code serves the real
link and the fake device (teleop_host.sim). Layouts are checked against the
firmware's golden vectors (test/test_hostlink/golden_vectors.h).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

__all__ = [
    "AckStatus",
    "ByeReason",
    "ConfigFormat",
    "DeviceInfo",
    "EventCode",
    "HandFlag",
    "JointFlag",
    "JointInfo",
    "LogSource",
    "MsgType",
    "Severity",
    "Ack",
    "Bye",
    "Config",
    "Done",
    "Event",
    "HostHello",
    "Log",
    "SlowEncoder",
    "SlowJoint",
    "SlowStatus",
    "TelemConfig",
    "Telemetry",
    "TelemetryJoint",
    "decode_device_message",
    "DeviceMessage",
]


class MsgType(IntEnum):
    HOST_HELLO = 0x01
    HEARTBEAT = 0x02
    COMMAND = 0x03
    ESTOP = 0x04
    TELEM_CONFIG = 0x05
    HOST_EXIT = 0x06
    GET_INFO = 0x07
    HELLO = 0x81
    TELEMETRY = 0x82
    SLOW_STATUS = 0x83
    EVENT = 0x84
    ACK = 0x85
    DONE = 0x86
    CONFIG = 0x87
    LOG = 0x88
    BYE = 0x89


class EventCode(IntEnum):
    GENERIC = 0
    REFUSED = 1
    ARMED = 2
    DISARMED = 3
    TORQUE_OFF = 4
    FAULT = 5
    JOINT_STATE = 6
    CAL_CAPTURED = 7
    HOMED = 8
    PING_RESULT = 9
    LIMITS_ROW = 10
    SWEEP_RUN = 11
    SWEEP_DONE = 12
    CONFIG_SAVED = 13
    CONFIG_LOADED = 14
    DEFAULTS = 15
    SETTING_CHANGED = 16
    MONITOR = 17


class Severity(IntEnum):
    INFO = 0
    WARNING = 1
    ERROR = 2


class AckStatus(IntEnum):
    ACCEPTED = 0
    PARSE_ERROR = 1
    BUSY = 2
    BAD_FRAME = 3


class ConfigFormat(IntEnum):
    DUMP_TEXT = 0


class LogSource(IntEnum):
    CONSOLE = 0
    ESP_LOG = 1
    BOOT = 2


class ByeReason(IntEnum):
    HOST_EXIT = 0
    HEARTBEAT_TIMEOUT = 1


class JointFlag(IntFlag):
    ENABLED = 1 << 0
    READY = 1 << 1
    REF_CAPTURED = 1 << 2
    HOMED = 1 << 3
    SINGLE_TURN = 1 << 4
    ENC_FRESH = 1 << 5
    CONTACT = 1 << 6
    GOAL_SENT = 1 << 7
    MODEL_OK = 1 << 8
    SERVO_OK = 1 << 9
    TORQUE_ON = 1 << 10
    MOVING = 1 << 11
    HAS_CURRENT = 1 << 12
    SERVO_FRESH = 1 << 13


class HandFlag(IntFlag):
    TORQUE_OFF_CONFIRMED = 1 << 0
    MONITOR = 1 << 1
    SWEEP_ACTIVE = 1 << 2
    TELEMETRY_DROPPED = 1 << 3


class _Reader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0

    def take(self, fmt: struct.Struct) -> tuple:
        values = fmt.unpack_from(self.data, self.pos)
        self.pos += fmt.size
        return values

    def u8(self) -> int:
        if self.pos >= len(self.data):
            raise struct.error("payload truncated")
        value = self.data[self.pos]
        self.pos += 1
        return value

    def str8(self) -> str:
        n = self.u8()
        if self.pos + n > len(self.data):
            raise struct.error("payload truncated")
        text = self.data[self.pos : self.pos + n].decode("utf-8", errors="replace")
        self.pos += n
        return text

    def rest(self) -> bytes:
        data = self.data[self.pos :]
        self.pos = len(self.data)
        return data

    def done(self) -> bool:
        return self.pos == len(self.data)


def _str8(text: str) -> bytes:
    data = text.encode()[:255]
    return bytes([len(data)]) + data


def _names(names: list[str]) -> bytes:
    return bytes([len(names)]) + b"".join(_str8(n) for n in names)


# ---- host -> device ----------------------------------------------------------

HELLO_ESTOP_ON_LINK_LOSS = 0x01


@dataclass(slots=True)
class HostHello:
    protocol: int = 1
    flags: int = HELLO_ESTOP_ON_LINK_LOSS
    heartbeat_timeout_ms: int = 1000
    telemetry_divider: int = 1
    slow_divider: int = 10

    _S = struct.Struct("<BBHBB")

    def encode(self) -> bytes:
        return self._S.pack(
            self.protocol, self.flags, self.heartbeat_timeout_ms, self.telemetry_divider, self.slow_divider
        )

    @classmethod
    def decode(cls, data: bytes) -> HostHello:
        return cls(*cls._S.unpack_from(data))


@dataclass(slots=True)
class TelemConfig:
    telemetry_divider: int = 1
    slow_divider: int = 10

    def encode(self) -> bytes:
        return bytes([self.telemetry_divider, self.slow_divider])

    @classmethod
    def decode(cls, data: bytes) -> TelemConfig:
        return cls(*struct.unpack_from("<BB", data))


# ---- HELLO -------------------------------------------------------------------


@dataclass(slots=True)
class JointInfo:
    servo_id: int
    model: int
    travel_min: int
    travel_max: int
    vmax: int
    vsat: int
    amax: int
    max_speed: int
    encoder_default: int
    enabled_default: int
    step_scale: float
    degrees_per_step: float
    name: str

    _S = struct.Struct("<BBhhHHHHBBff")


@dataclass(slots=True)
class DeviceInfo:
    protocol: int
    firmware: str
    control_hz: int
    degrees_per_count: float
    encoder_count: int
    min_current_limit_ma: int
    max_current_limit_ma: int
    min_current_trip_pct: int
    max_current_trip_pct: int
    default_current_trip_pct: int
    min_exo_span_counts: int
    max_exo_span_counts: int
    joints: list[JointInfo]
    faults: list[str]
    states: list[str]
    events: list[str]
    models: list[str]

    _HEAD = struct.Struct("<HfBHHBBBii")

    def encode(self) -> bytes:
        out = bytes([self.protocol]) + _str8(self.firmware)
        out += self._HEAD.pack(
            self.control_hz,
            self.degrees_per_count,
            self.encoder_count,
            self.min_current_limit_ma,
            self.max_current_limit_ma,
            self.min_current_trip_pct,
            self.max_current_trip_pct,
            self.default_current_trip_pct,
            self.min_exo_span_counts,
            self.max_exo_span_counts,
        )
        out += bytes([len(self.joints)])
        for j in self.joints:
            out += JointInfo._S.pack(
                j.servo_id,
                j.model,
                j.travel_min,
                j.travel_max,
                j.vmax,
                j.vsat,
                j.amax,
                j.max_speed,
                j.encoder_default,
                j.enabled_default,
                j.step_scale,
                j.degrees_per_step,
            )
            out += _str8(j.name)
        return out + _names(self.faults) + _names(self.states) + _names(self.events) + _names(self.models)

    @classmethod
    def decode(cls, data: bytes) -> DeviceInfo:
        r = _Reader(data)
        protocol = r.u8()
        firmware = r.str8()
        head = r.take(cls._HEAD)
        joints = []
        for _ in range(r.u8()):
            values = r.take(JointInfo._S)
            joints.append(JointInfo(*values, name=r.str8()))
        tables = [[r.str8() for _ in range(r.u8())] for _ in range(4)]
        return cls(protocol, firmware, *head, joints, *tables)


# ---- TELEMETRY -----------------------------------------------------------------


@dataclass(slots=True)
class TelemetryJoint:
    state: int = 0
    fault: int = 0
    flags: int = 0
    enc_raw: int = 0
    exo_counts: float = 0.0
    exo_filtered_counts: float = 0.0
    target: float = 0.0
    goal: float = 0.0
    sent_goal: float = 0.0
    reference: float = 0.0
    speed_command: int = 0
    grip_lead: int = 0
    position: int = 0
    speed: int = 0
    load_deci_pct: int = 0
    current_ma: int = 0
    voltage_deci_v: int = 0
    temperature_c: int = 0
    status: int = 0
    error_byte: int = 0
    read_age_ms: int = 0

    _S = struct.Struct("<BBHHffffffHhhhhhBbBBH")

    def has(self, flag: JointFlag) -> bool:
        return bool(self.flags & flag)


@dataclass(slots=True)
class Telemetry:
    cycle: int = 0
    t_us: int = 0
    uptime_ms: int = 0
    armed_count: int = 0
    hand_fault: int = 0
    hand_flags: int = 0
    exec_us: int = 0
    joints: list[TelemetryJoint] = field(default_factory=list)

    _HAND = struct.Struct("<IIIBBBHB")
    SIZE_HAND = 18
    SIZE_JOINT = 48

    def encode(self) -> bytes:
        out = self._HAND.pack(
            self.cycle,
            self.t_us,
            self.uptime_ms,
            self.armed_count,
            self.hand_fault,
            self.hand_flags,
            self.exec_us,
            len(self.joints),
        )
        for j in self.joints:
            out += TelemetryJoint._S.pack(
                j.state,
                j.fault,
                j.flags,
                j.enc_raw,
                j.exo_counts,
                j.exo_filtered_counts,
                j.target,
                j.goal,
                j.sent_goal,
                j.reference,
                j.speed_command,
                j.grip_lead,
                j.position,
                j.speed,
                j.load_deci_pct,
                j.current_ma,
                j.voltage_deci_v,
                j.temperature_c,
                j.status,
                j.error_byte,
                j.read_age_ms,
            )
        return out

    @classmethod
    def decode(cls, data: bytes) -> Telemetry:
        *hand, count = cls._HAND.unpack_from(data)
        if len(data) != cls.SIZE_HAND + count * cls.SIZE_JOINT:
            raise struct.error(f"TELEMETRY of {len(data)} B for {count} joints")
        joints = [
            TelemetryJoint(*TelemetryJoint._S.unpack_from(data, cls.SIZE_HAND + i * cls.SIZE_JOINT))
            for i in range(count)
        ]
        return cls(*hand, joints=joints)


# ---- SLOW_STATUS ---------------------------------------------------------------


@dataclass(slots=True)
class SlowJoint:
    enabled: int = 0
    encoder_index: int = 0xFF
    exo_ref: int = 0
    exo_b: int = 0
    servo_a: int = 0
    servo_b: int = 0
    vmax: int = 0
    amax: int = 0
    vsat: int = 0
    track_steps: int = 0
    grip_steps: int = 0
    current_limit_ma: int = 0
    current_trip_pct: int = 0
    detected_model: int = 0
    effective_current_ma: int = 0
    jump_rejects: int = 0

    _S = struct.Struct("<BBHihhHHHHHHBBHI")


@dataclass(slots=True)
class SlowEncoder:
    address: int = 0
    state: int = 0
    valid: int = 0
    angle: int = 0
    agc: int = 0
    magnet_ok: int = 0
    errors: int = 0

    _S = struct.Struct("<BBBHBBI")


@dataclass(slots=True)
class SlowStatus:
    lead_ms: float = 0.0
    min_cutoff_hz: float = 0.0
    beta: float = 0.0
    jump_deg: float = 0.0
    grip_load_pct: float = 0.0
    load_trip: int = 0
    cycles: int = 0
    overruns: int = 0
    last_period_us: int = 0
    max_jitter_us: int = 0
    p99_jitter_us: int = 0
    last_exec_us: int = 0
    max_exec_us: int = 0
    average_exec_us: int = 0
    bus_sent: int = 0
    bus_received: int = 0
    bus_timeouts: int = 0
    bus_parse_errors: int = 0
    bus_echo_errors: int = 0
    bus_last_reply_us: int = 0
    bus_max_reply_us: int = 0
    bus_echo: int = 0
    encoder_average_us: int = 0
    encoder_max_us: int = 0
    encoder_max_skew_us: int = 0
    dropped_events: int = 0
    dropped_telemetry: int = 0
    rx_crc_errors: int = 0
    rx_malformed: int = 0
    monitor: int = 0
    monitor_ok_mask: int = 0
    monitor_position: list[int] = field(default_factory=list)
    joints: list[SlowJoint] = field(default_factory=list)
    encoders: list[SlowEncoder] = field(default_factory=list)

    _HEAD = struct.Struct("<fffffB8I7IB3I4IBH")

    def encode(self) -> bytes:
        out = self._HEAD.pack(
            self.lead_ms,
            self.min_cutoff_hz,
            self.beta,
            self.jump_deg,
            self.grip_load_pct,
            self.load_trip,
            self.cycles,
            self.overruns,
            self.last_period_us,
            self.max_jitter_us,
            self.p99_jitter_us,
            self.last_exec_us,
            self.max_exec_us,
            self.average_exec_us,
            self.bus_sent,
            self.bus_received,
            self.bus_timeouts,
            self.bus_parse_errors,
            self.bus_echo_errors,
            self.bus_last_reply_us,
            self.bus_max_reply_us,
            self.bus_echo,
            self.encoder_average_us,
            self.encoder_max_us,
            self.encoder_max_skew_us,
            self.dropped_events,
            self.dropped_telemetry,
            self.rx_crc_errors,
            self.rx_malformed,
            self.monitor,
            self.monitor_ok_mask,
        )
        out += bytes([len(self.joints)])
        out += struct.pack(f"<{len(self.joints)}h", *self.monitor_position)
        for j in self.joints:
            out += SlowJoint._S.pack(
                j.enabled,
                j.encoder_index,
                j.exo_ref,
                j.exo_b,
                j.servo_a,
                j.servo_b,
                j.vmax,
                j.amax,
                j.vsat,
                j.track_steps,
                j.grip_steps,
                j.current_limit_ma,
                j.current_trip_pct,
                j.detected_model,
                j.effective_current_ma,
                j.jump_rejects,
            )
        out += bytes([len(self.encoders)])
        for e in self.encoders:
            out += SlowEncoder._S.pack(e.address, e.state, e.valid, e.angle, e.agc, e.magnet_ok, e.errors)
        return out

    @classmethod
    def decode(cls, data: bytes) -> SlowStatus:
        r = _Reader(data)
        s = cls(*r.take(cls._HEAD))
        count = r.u8()
        s.monitor_position = list(r.take(struct.Struct(f"<{count}h")))
        s.joints = [SlowJoint(*r.take(SlowJoint._S)) for _ in range(count)]
        s.encoders = [SlowEncoder(*r.take(SlowEncoder._S)) for _ in range(r.u8())]
        if not r.done():
            raise struct.error("SLOW_STATUS has trailing bytes")
        return s


# ---- events and replies --------------------------------------------------------


@dataclass(slots=True)
class Event:
    code: int = EventCode.GENERIC
    severity: int = Severity.INFO
    joint: int = 0
    cmd_seq: int = 0
    args: tuple[int, int, int, int] = (0, 0, 0, 0)
    text: str = ""

    _S = struct.Struct("<BBBH4i")

    def encode(self) -> bytes:
        return self._S.pack(self.code, self.severity, self.joint, self.cmd_seq, *self.args) + self.text.encode()

    @classmethod
    def decode(cls, data: bytes) -> Event:
        code, severity, joint, cmd_seq, *args = cls._S.unpack_from(data)
        text = data[cls._S.size :].decode("utf-8", errors="replace")
        return cls(code, severity, joint, cmd_seq, tuple(args), text)  # type: ignore[arg-type]


@dataclass(slots=True)
class Ack:
    ack_seq: int
    status: int
    text: str = ""

    def encode(self) -> bytes:
        return struct.pack("<HB", self.ack_seq, self.status) + self.text.encode()

    @classmethod
    def decode(cls, data: bytes) -> Ack:
        ack_seq, status = struct.unpack_from("<HB", data)
        return cls(ack_seq, status, data[3:].decode("utf-8", errors="replace"))


@dataclass(slots=True)
class Done:
    ack_seq: int
    ok: bool

    def encode(self) -> bytes:
        return struct.pack("<HB", self.ack_seq, 0 if self.ok else 1)

    @classmethod
    def decode(cls, data: bytes) -> Done:
        ack_seq, result = struct.unpack_from("<HB", data)
        return cls(ack_seq, result == 0)


@dataclass(slots=True)
class Config:
    ack_seq: int
    format: int
    text: str

    def encode(self) -> bytes:
        return struct.pack("<HB", self.ack_seq, self.format) + self.text.encode()

    @classmethod
    def decode(cls, data: bytes) -> Config:
        ack_seq, fmt = struct.unpack_from("<HB", data)
        return cls(ack_seq, fmt, data[3:].decode("utf-8", errors="replace"))


@dataclass(slots=True)
class Log:
    source: int
    text: str

    def encode(self) -> bytes:
        return bytes([self.source]) + self.text.encode()

    @classmethod
    def decode(cls, data: bytes) -> Log:
        return cls(data[0], data[1:].decode("utf-8", errors="replace"))


@dataclass(slots=True)
class Bye:
    reason: int

    def encode(self) -> bytes:
        return bytes([self.reason])

    @classmethod
    def decode(cls, data: bytes) -> Bye:
        return cls(data[0])


DeviceMessage = DeviceInfo | Telemetry | SlowStatus | Event | Ack | Done | Config | Log | Bye

_DECODERS = {
    MsgType.HELLO: DeviceInfo.decode,
    MsgType.TELEMETRY: Telemetry.decode,
    MsgType.SLOW_STATUS: SlowStatus.decode,
    MsgType.EVENT: Event.decode,
    MsgType.ACK: Ack.decode,
    MsgType.DONE: Done.decode,
    MsgType.CONFIG: Config.decode,
    MsgType.LOG: Log.decode,
    MsgType.BYE: Bye.decode,
}


def decode_device_message(type_: int, payload: bytes) -> DeviceMessage:
    """Raises KeyError for an unknown type, struct.error / IndexError for a bad payload."""
    return _DECODERS[MsgType(type_)](payload)
