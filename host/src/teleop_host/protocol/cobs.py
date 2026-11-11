"""Consistent Overhead Byte Stuffing, same block rules as lib/HostLink/src/frame.cpp
(no delimiter included)."""

from __future__ import annotations


class CobsError(ValueError):
    pass


def encode(data: bytes) -> bytes:
    out = bytearray()
    is_open = False
    code_pos = 0
    code = 1
    for byte in data:
        if not is_open:
            code_pos, code, is_open = len(out), 1, True
            out.append(0)
        if byte == 0:
            # A zero always closes the block and starts a new one.
            out[code_pos] = code
            code_pos, code = len(out), 1
            out.append(0)
            continue
        out.append(byte)
        code += 1
        if code == 0xFF:  # full block; the next byte opens a new one
            out[code_pos] = code
            is_open = False
    if not is_open and not out:  # empty input still encodes as one block
        code_pos, code, is_open = 0, 1, True
        out.append(0)
    if is_open:
        out[code_pos] = code
    return bytes(out)


def decode(data: bytes) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        i += 1
        end = i + code - 1
        if code == 0 or end > n:
            raise CobsError("malformed COBS block")
        chunk = data[i:end]
        if 0 in chunk:
            raise CobsError("zero byte inside a COBS block")
        out += chunk
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)
