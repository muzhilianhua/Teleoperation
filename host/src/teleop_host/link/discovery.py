"""Serial port discovery: the ESP32-S3 native USB port has Espressif's VID."""

from __future__ import annotations

from dataclasses import dataclass

from serial.tools import list_ports

ESPRESSIF_VID = 0x303A


@dataclass(frozen=True, slots=True)
class PortInfo:
    device: str
    description: str
    is_esp32: bool


def list_serial_ports() -> list[PortInfo]:
    ports = []
    for p in list_ports.comports():
        if p.vid is None:
            continue  # Bluetooth and debug pseudo-ports
        device = p.device.replace("/dev/tty.", "/dev/cu.")  # macOS: cu.* does not wait for carrier
        ports.append(PortInfo(device, p.description or "", p.vid == ESPRESSIF_VID))
    return sorted(ports, key=lambda p: (not p.is_esp32, p.device))


def find_esp32s() -> list[str]:
    """Every ESP32 port (one per hand board)."""
    return [p.device for p in list_serial_ports() if p.is_esp32]


def find_esp32() -> str | None:
    """The ESP32 port if exactly one is connected; never guesses between several."""
    candidates = [p.device for p in list_serial_ports() if p.is_esp32]
    return candidates[0] if len(candidates) == 1 else None
