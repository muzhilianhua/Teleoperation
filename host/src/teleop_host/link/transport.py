"""Byte transport under the host link: the USB serial port, or the fake device."""

from __future__ import annotations

from typing import Protocol


class Transport(Protocol):
    def write(self, data: bytes) -> None: ...

    def read(self, timeout: float = 0.05) -> bytes:
        """Returns what arrived within `timeout` (b"" if nothing). Raises OSError when the port is gone."""
        ...

    def close(self) -> None: ...
