"""HandManager binds boards to hands by their HELLO."""

from __future__ import annotations

import asyncio

import pytest

from teleop_host.hands import HandManager, HandNotice
from teleop_host.sim import FakeDevice


@pytest.fixture
def boards():
    made: list[FakeDevice] = []

    def make(hand: str) -> FakeDevice:
        dev = FakeDevice(hand=hand)
        made.append(dev)
        return dev

    yield make
    for dev in made:
        dev.shutdown()


async def wait_for(predicate, timeout: float = 3.0) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while not predicate():
        assert asyncio.get_running_loop().time() < deadline, "timed out"
        await asyncio.sleep(0.02)


async def test_binds_by_hello_not_by_port(boards) -> None:
    left, right = boards("left"), boards("right")
    hands = HandManager()
    seen: list[tuple[str | None, object]] = []
    hands.subscribe(lambda h, m: seen.append((h, m)))
    # Deliberately misleading port names.
    hands.connect(lambda: left, "port-right")
    hands.connect(lambda: right, "port-left")
    await wait_for(lambda: set(hands.devices) == {"right", "left"})
    await wait_for(lambda: all(d.connected for d in hands.devices.values()))
    assert hands.port_of("left") == "port-right" and hands.port_of("right") == "port-left"
    assert any(h == "left" for h, _ in seen)

    result = await hands.devices["left"].command("ping")
    assert result.ok

    hands.disconnect("left")  # the other hand is unaffected
    assert "left" not in hands.devices
    await wait_for(lambda: not left.host_mode)  # HOST_EXIT: back to its text console
    assert hands.devices["right"].connected
    assert (await hands.devices["right"].command("ping")).ok
    hands.disconnect_all()


async def test_second_board_on_the_same_hand_is_rejected(boards) -> None:
    first, second = boards("right"), boards("right")
    hands = HandManager()
    notices: list[HandNotice] = []
    hands.subscribe(lambda h, m: notices.append(m) if isinstance(m, HandNotice) else None)
    hands.connect(lambda: first, "a")
    await wait_for(lambda: "right" in hands.devices and hands.devices["right"].connected)
    hands.connect(lambda: second, "b")
    await wait_for(lambda: bool(notices))
    assert "also reports the right hand" in notices[0].text and "GPIO4" in notices[0].text
    assert hands.port_of("right") == "a" and hands.ports_in_use() == {"a"}
    hands.disconnect_all()


async def test_estop_reaches_both_hands(boards) -> None:
    left, right = boards("left"), boards("right")
    hands = HandManager()
    hands.connect(lambda: left, "l")
    hands.connect(lambda: right, "r")
    await wait_for(lambda: len(hands.devices) == 2 and all(d.connected for d in hands.devices.values()))
    assert sorted(await hands.estop_all()) == ["left", "right"]
    await wait_for(lambda: left.estops == 1 and right.estops == 1)
    hands.disconnect_all()
