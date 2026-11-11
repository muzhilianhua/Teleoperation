"""CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor."""

from __future__ import annotations


def _table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_TABLE = _table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc
