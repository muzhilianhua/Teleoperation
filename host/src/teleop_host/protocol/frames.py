"""Host link frames: COBS(header | payload | crc16) + 0x00 (docs/host-protocol.md)."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from . import cobs
from .crc import crc16

PROTOCOL_VERSION = 1
HEADER = struct.Struct("<BBHB")  # version, type, seq, flags
MAX_FRAME = 4096  # decoded header + payload + crc
MAX_PAYLOAD = MAX_FRAME - HEADER.size - 2
FLAG_REPLY_EXPECTED = 0x01


@dataclass(frozen=True, slots=True)
class Frame:
    type: int
    seq: int
    flags: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class UnframedText:
    """Bytes between delimiters that were not a valid frame, e.g. boot text."""

    data: bytes

    @property
    def text(self) -> str:
        return self.data.decode("utf-8", errors="replace")


def encode_frame(type_: int, seq: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} B exceeds {MAX_PAYLOAD} B")
    body = HEADER.pack(PROTOCOL_VERSION, type_, seq & 0xFFFF, flags) + payload
    body += struct.pack("<H", crc16(body))
    return cobs.encode(body) + b"\x00"


@dataclass(slots=True)
class DecoderStats:
    frames: int = 0
    crc_errors: int = 0
    malformed: int = 0
    overflows: int = 0


@dataclass(slots=True)
class FrameDecoder:
    """Streaming decoder: feed() any chunk of received bytes."""

    stats: DecoderStats = field(default_factory=DecoderStats)
    _buffer: bytearray = field(default_factory=bytearray)
    _discarding: bool = False

    def feed(self, data: bytes) -> list[Frame | UnframedText]:
        out: list[Frame | UnframedText] = []
        start = 0
        while True:
            end = data.find(0, start)
            if end < 0:
                self._append(data[start:])
                return out
            self._append(data[start:end])
            chunk, discarding = bytes(self._buffer), self._discarding
            self._buffer.clear()
            self._discarding = False
            if chunk and not discarding:
                out.append(self._finish(chunk))
            start = end + 1

    def _append(self, data: bytes) -> None:
        if self._discarding:
            return
        self._buffer += data
        if len(self._buffer) > MAX_FRAME + MAX_FRAME // 254 + 1:
            self.stats.overflows += 1
            self._buffer.clear()
            self._discarding = True

    def _finish(self, chunk: bytes) -> Frame | UnframedText:
        try:
            body = cobs.decode(chunk)
        except cobs.CobsError:
            self.stats.malformed += 1
            return UnframedText(chunk)
        if len(body) < HEADER.size + 2 or len(body) > MAX_FRAME:
            self.stats.malformed += 1
            return UnframedText(chunk)
        (crc,) = struct.unpack_from("<H", body, len(body) - 2)
        if crc16(body[:-2]) != crc:
            self.stats.crc_errors += 1
            return UnframedText(chunk)
        version, type_, seq, flags = HEADER.unpack_from(body)
        if version != PROTOCOL_VERSION:
            self.stats.malformed += 1
            return UnframedText(chunk)
        self.stats.frames += 1
        return Frame(type_, seq, flags, body[HEADER.size : -2])
