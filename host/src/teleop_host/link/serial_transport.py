"""pyserial transport for the ESP32-S3's native USB port."""

from __future__ import annotations

import serial


class SerialTransport:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baud  # nominal: the native USB port ignores it
        self._serial.timeout = 0.05
        self._serial.write_timeout = 1.0
        # De-asserted before open, so opening the port does not reset the ESP32-S3
        # (same as tools/calib.py).
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.exclusive = True
        try:
            self._serial.open()
        except serial.SerialException as exc:
            raise OSError(f"cannot open {port}: {exc}") from exc

    @property
    def port(self) -> str:
        return str(self._serial.port)

    def write(self, data: bytes) -> None:
        try:
            self._serial.write(data)
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def read(self, timeout: float = 0.05) -> bytes:
        try:
            if self._serial.timeout != timeout:
                self._serial.timeout = timeout
            return self._serial.read(max(1, self._serial.in_waiting))
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def close(self) -> None:
        self._serial.close()
