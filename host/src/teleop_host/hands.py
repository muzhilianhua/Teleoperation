"""Both hands: one Device per board, bound to a hand by the hand in its HELLO.

Boards are connected by port, but which hand a board drives is only known
from its HELLO (GPIO4 strap), so swapping USB cables or ports never swaps
hands. A board without a hand field (older firmware) is taken as the right
hand. A second board claiming a hand that is already bound is disconnected
and reported: its strap is wrong.
"""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .device import Device
from .link.link import LinkState, LinkStatus
from .link.transport import Transport
from .protocol.messages import DeviceInfo, HostHello

HANDS = ("right", "left")
HAND_BY_CODE = {1: "right", 2: "left"}


def hand_of(info: DeviceInfo) -> str:
    return HAND_BY_CODE.get(info.hand, "right")


@dataclass(frozen=True, slots=True)
class HandNotice:
    """Something about the pairing of boards and hands the operator must see."""

    text: str


# (hand or None while the board is not bound yet, message)
HandSubscriber = Callable[[str | None, Any], None]


class HandManager:
    def __init__(self, hello: HostHello | None = None) -> None:
        self._hello = hello
        self.devices: dict[str, Device] = {}  # hand -> bound device
        self._unbound: dict[str, Device] = {}  # port -> device waiting for its HELLO
        self._ports: dict[int, str] = {}  # id(device) -> port
        self._subscribers: list[HandSubscriber] = []

    # ---- connections ------------------------------------------------------------

    def connect(self, factory: Callable[[], Transport], port: str, *, reconnect: bool = True) -> None:
        """Opens a board; it joins a hand once its HELLO arrives. Must run on the event loop."""
        if port in self.ports_in_use():
            raise ValueError(f"{port} is already connected")
        device = Device(self._hello)
        self._unbound[port] = device
        self._ports[id(device)] = port
        device.subscribe(lambda m, d=device: self._on_message(d, m))
        device.connect(factory, port, reconnect=reconnect)

    def disconnect(self, hand: str) -> None:
        device = self.devices.pop(hand, None)
        if device is not None:
            self._ports.pop(id(device), None)
            device.disconnect()
            self._publish(hand, LinkStatus(LinkState.DISCONNECTED))

    def disconnect_port(self, port: str) -> None:
        device = self._unbound.pop(port, None)
        if device is not None:
            self._ports.pop(id(device), None)
            device.disconnect()
        for hand, d in list(self.devices.items()):
            if self._ports.get(id(d)) == port:
                self.disconnect(hand)

    def disconnect_all(self) -> None:
        for port in list(self._unbound):
            self.disconnect_port(port)
        for hand in list(self.devices):
            self.disconnect(hand)

    def device(self, hand: str) -> Device | None:
        return self.devices.get(hand)

    def port_of(self, hand: str) -> str:
        device = self.devices.get(hand)
        return self._ports.get(id(device), "") if device else ""

    def ports_in_use(self) -> set[str]:
        return set(self._ports.values())

    async def estop_all(self) -> list[str]:
        """E-stop on every connected hand; returns the hands it reached."""
        reached = []
        for hand, device in self.devices.items():
            if await device.estop():
                reached.append(hand)
        return reached

    def subscribe(self, callback: HandSubscriber) -> Callable[[], None]:
        self._subscribers.append(callback)
        return lambda: self._subscribers.remove(callback) if callback in self._subscribers else None

    # ---- binding -----------------------------------------------------------------

    def _hand_of_device(self, device: Device) -> str | None:
        return next((h for h, d in self.devices.items() if d is device), None)

    def _on_message(self, device: Device, message: Any) -> None:
        if isinstance(message, DeviceInfo):
            self._bind(device, message)
        self._publish(self._hand_of_device(device), message)

    def _bind(self, device: Device, info: DeviceInfo) -> None:
        hand = hand_of(info)
        current = self._hand_of_device(device)
        if current == hand:
            return
        holder = self.devices.get(hand)
        port = self._ports.get(id(device), "?")
        if holder is not None and holder is not device:
            # Disconnect later: this runs inside the device's own dispatch.
            self._unbound.pop(port, None)
            self._ports.pop(id(device), None)
            asyncio.get_running_loop().call_soon(device.disconnect)
            self._publish(
                None,
                HandNotice(
                    f"board {info.board_id or '?'} on {port} also reports the {hand} hand "
                    f"(already {self.port_of(hand)}): check its GPIO4 strap; it was disconnected"
                ),
            )
            return
        if current is not None:  # the board's strap changed: move it
            self.devices.pop(current, None)
        self._unbound.pop(port, None)
        self.devices[hand] = device

    def _publish(self, hand: str | None, message: Any) -> None:
        for callback in list(self._subscribers):
            with contextlib.suppress(Exception):
                callback(hand, message)
