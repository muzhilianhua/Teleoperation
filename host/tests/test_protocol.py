"""Codec tests against the firmware's golden vectors (test/test_hostlink)."""

from __future__ import annotations

import re
import struct
from pathlib import Path

import pytest

from teleop_host.protocol import cobs
from teleop_host.protocol.crc import crc16
from teleop_host.protocol.frames import Frame, FrameDecoder, UnframedText, encode_frame
from teleop_host.protocol.messages import (
    Ack,
    DeviceInfo,
    Done,
    Event,
    EventCode,
    HostHello,
    JointInfo,
    MsgType,
    SlowEncoder,
    SlowJoint,
    SlowStatus,
    Telemetry,
    TelemetryJoint,
    decode_device_message,
)

GOLDEN_FILE = Path(__file__).resolve().parents[2] / "test" / "test_hostlink" / "golden_vectors.h"


def golden(name: str) -> bytes:
    match = re.search(rf'{name}\s*=\s*"([0-9a-f]+)"', GOLDEN_FILE.read_text())
    assert match, f"{name} missing from {GOLDEN_FILE}"
    return bytes.fromhex(match.group(1))


# ---- CRC / COBS / frames ------------------------------------------------------


def test_crc_check_value() -> None:
    assert crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize(
    ("raw", "wire"),
    [
        (b"", b"\x01"),
        (b"\x00", b"\x01\x01"),
        (b"\x00\x00", b"\x01\x01\x01"),
        (b"\x00\x11\x00", b"\x01\x02\x11\x01"),
        (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
        (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
        (b"\x11\x00\x00\x00", b"\x02\x11\x01\x01\x01"),
        (bytes(range(1, 255)), b"\xff" + bytes(range(1, 255))),
        (bytes(range(1, 255)) + b"\xff", b"\xff" + bytes(range(1, 255)) + b"\x02\xff"),
        (bytes(range(1, 255)) + b"\x00", b"\xff" + bytes(range(1, 255)) + b"\x01\x01"),
    ],
)
def test_cobs_reference_vectors(raw: bytes, wire: bytes) -> None:
    assert cobs.encode(raw) == wire
    assert cobs.decode(wire) == raw


def test_cobs_rejects_malformed() -> None:
    for bad in (b"\x02\x00", b"\x05\x11\x22", b"\x00"):
        with pytest.raises(cobs.CobsError):
            cobs.decode(bad)


def test_frame_golden_vector() -> None:
    wire = encode_frame(MsgType.COMMAND, 0x1234, b"arm", flags=1)
    assert wire == bytes([0x0B, 0x01, 0x03, 0x34, 0x12, 0x01]) + b"arm" + bytes([0x44, 0xCE, 0x00])


def test_decoder_round_trip_in_pieces() -> None:
    payload = bytes(i % 7 and i % 256 for i in range(600))
    wire = encode_frame(MsgType.TELEMETRY, 0x0100, payload)
    decoder = FrameDecoder()
    out = []
    for i in range(0, len(wire), 13):
        out += decoder.feed(wire[i : i + 13])
    assert out == [Frame(MsgType.TELEMETRY, 0x0100, 0, payload)]


def test_decoder_reports_text_and_resyncs() -> None:
    good = encode_frame(MsgType.LOG, 1, b"\x00hello")
    bad = bytearray(good)
    bad[3] ^= 0x40
    decoder = FrameDecoder()
    out = decoder.feed(b"[boot] encoders responding: 9 / 9\n\x00" + bytes(bad) + good)
    assert isinstance(out[0], UnframedText) and "encoders responding" in out[0].text
    assert isinstance(out[1], UnframedText)
    assert out[2] == Frame(MsgType.LOG, 1, 0, b"\x00hello")
    assert decoder.stats.frames == 1 and decoder.stats.crc_errors == 1


# ---- payloads (same sample values as test/test_hostlink/test_main.cpp) ----------


def sample_hello() -> DeviceInfo:
    joint = JointInfo(2, 2, 794, 1911, 1333, 167, 26666, 12492, 2, 0, 3.25, 0.087890625, "thumb_near")
    return DeviceInfo(
        1, "abc1234", 100, 360 / 16384, 9, 7, 13306, 50, 100, 90, 182, 49152, [joint],
        ["none", "comm lost"], ["OFF", "ALIGN"], ["generic"], ["unknown", "SCS0009", "HLS3606"], 2, "a0b1c2d3e4f5",
    )  # fmt: skip


def sample_telemetry() -> Telemetry:
    a = TelemetryJoint(
        state=2, flags=0x2F23, enc_raw=8191, exo_counts=-1234.5, exo_filtered_counts=-1230.25, target=512.5,
        goal=510.0, sent_goal=508.75, reference=505.5, speed_command=1300, grip_lead=60, position=507,
        speed=-120, load_deci_pct=-355, voltage_deci_v=60, temperature_c=35, read_age_ms=12,
    )  # fmt: skip
    b = TelemetryJoint(
        fault=10, flags=0x1200, position=2048, current_ma=-2700, voltage_deci_v=74, temperature_c=-5, status=8,
        read_age_ms=65535,
    )  # fmt: skip
    return Telemetry(123456, 0x89ABCDEF, 1234567, 2, 0, 0x01, 850, [a, b])


def sample_slow() -> SlowStatus:
    return SlowStatus(
        min_cutoff_hz=1.0, beta=0.0078125, jump_deg=25.0, grip_load_pct=60.0, cycles=1000, overruns=1,
        last_period_us=10000, max_jitter_us=12, p99_jitter_us=8, last_exec_us=900, max_exec_us=1800,
        average_exec_us=850, bus_sent=500, bus_received=498, bus_timeouts=2, bus_last_reply_us=180,
        bus_max_reply_us=950, bus_echo=1, encoder_average_us=1500, encoder_max_us=1700, encoder_max_skew_us=40,
        dropped_events=3, rx_crc_errors=1, rx_malformed=2, monitor=1, monitor_ok_mask=1, monitor_position=[512],
        joints=[SlowJoint(1, 2, 1000, -8192, 794, 1900, 1333, 26666, 167, 133, 200, 500, 90, 2, 494, 7)],
        encoders=[SlowEncoder(0x40, 0, 1, 4095, 88, 1, 3)],
    )  # fmt: skip


def test_hello_golden() -> None:
    assert sample_hello().encode() == golden("kGoldenHelloHex")
    info = decode_device_message(MsgType.HELLO, golden("kGoldenHelloHex"))
    assert info == sample_hello()
    # Firmware before the hand fields: they stay at their defaults.
    legacy = DeviceInfo.decode(golden("kGoldenHelloHex")[: -(2 + 12)])
    assert legacy.hand == 0 and legacy.board_id == ""


def test_telemetry_golden() -> None:
    assert sample_telemetry().encode() == golden("kGoldenTelemetryHex")
    t = decode_device_message(MsgType.TELEMETRY, golden("kGoldenTelemetryHex"))
    assert t == sample_telemetry()
    with pytest.raises(struct.error):
        Telemetry.decode(golden("kGoldenTelemetryHex")[:-1])


def test_slow_status_golden() -> None:
    assert sample_slow().encode() == golden("kGoldenSlowStatusHex")
    assert decode_device_message(MsgType.SLOW_STATUS, golden("kGoldenSlowStatusHex")) == sample_slow()


def test_event_golden() -> None:
    event = Event(EventCode.SWEEP_DONE, 0, 4, 77, (1333, 167, 1, 0), "sweep j4: done")
    assert event.encode() == golden("kGoldenEventHex")
    assert decode_device_message(MsgType.EVENT, golden("kGoldenEventHex")) == event


def test_replies_and_host_hello() -> None:
    assert Ack.decode(bytes([7, 0, 1]) + b"bad") == Ack(7, 1, "bad")
    assert Done.decode(bytes([2, 1, 1])) == Done(0x0102, False)
    assert HostHello(heartbeat_timeout_ms=1000, telemetry_divider=2).encode() == bytes([1, 1, 0xE8, 3, 2, 10])
