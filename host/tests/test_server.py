"""REST + WebSocket smoke tests with two simulated hands."""

from __future__ import annotations

import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from teleop_host.config_files import HandBackups
from teleop_host.hands import HandManager
from teleop_host.plugins import builtin_plugins
from teleop_host.server import create_app
from teleop_host.sim import FakeDevice


@pytest.fixture
def boards():
    made = {"sim-right": FakeDevice(hand="right"), "sim-left": FakeDevice(hand="left")}
    yield made
    for dev in made.values():
        dev.shutdown()


@pytest.fixture
def client(tmp_path: Path, boards):
    factories = {name: (lambda d=dev: d) for name, dev in boards.items()}
    app = create_app(
        HandManager(), HandBackups(tmp_path), builtin_plugins(), sim_factories=factories, initial_ports=list(factories)
    )
    with TestClient(app) as c:
        deadline = time.monotonic() + 4
        while not all((h["link"] or {}).get("state") == "connected" for h in c.get("/api/hands").json().values()):
            assert time.monotonic() < deadline, "simulated hands did not connect"
            time.sleep(0.05)
        yield c


def receive_until(ws, kind: str, hand: str | None = None, limit: int = 2000) -> dict:
    for _ in range(limit):
        message = ws.receive_json()
        if message["type"] == kind and (hand is None or message.get("hand") == hand):
            return message
    raise AssertionError(f"no {kind} message for {hand}")


def test_hands_rest(client: TestClient, boards) -> None:
    hands = client.get("/api/hands").json()
    assert hands["right"]["port"] == "sim-right" and hands["left"]["port"] == "sim-left"
    assert client.get("/api/left/info").json()["info"]["hand"] == 2
    result = client.post("/api/left/command", json={"text": "set 2 vmax 900"}).json()
    assert result["ok"]
    assert boards["sim-left"].model.joints[1].s.vmax == 900 and boards["sim-right"].model.joints[1].s.vmax == 1333
    assert client.get("/api/middle/info").status_code == 404
    assert set(client.post("/api/estop").json()["reached"]) == {"right", "left"}
    assert {p["device"] for p in client.get("/api/ports").json()["ports"]} >= {"sim-right", "sim-left"}
    assert {p["id"] for p in client.get("/api/plugins").json()} >= {"recording", "camera"}


def test_disconnecting_one_hand_leaves_the_other(client: TestClient) -> None:
    client.post("/api/disconnect", json={"hand": "left"})
    hands = client.get("/api/hands").json()
    assert hands["left"]["link"] is None and hands["right"]["link"]["state"] == "connected"
    assert client.post("/api/right/command", json={"text": "ping"}).json()["ok"]
    assert client.post("/api/left/command", json={"text": "ping"}).status_code == 409
    assert client.post("/api/connect", json={"port": "sim-right"}).status_code == 409  # already in use


def test_websocket_is_tagged_by_hand(client: TestClient) -> None:
    with client.websocket_connect("/ws") as ws:
        snapshot = receive_until(ws, "snapshot")
        assert snapshot["data"]["hands"]["left"]["info"]["hand"] == 2
        assert len(receive_until(ws, "tel", "left")["data"]["joints"]) == 9
        receive_until(ws, "tel", "right")
        ws.send_json({"type": "cmd", "id": 7, "hand": "right", "text": "set 3 vmax 777"})
        result = receive_until(ws, "cmd_result")
        assert result["id"] == 7 and result["data"]["ok"]
        ws.send_json({"type": "plot_subscribe", "keys": ["right.j2.present", "left.j2.present", "left.j99.x"]})
        plot = receive_until(ws, "plot")
        assert set(plot["data"]["series"]) <= {"right.j2.present", "left.j2.present"}
        assert set(plot["data"]["t_us"]) <= {"right", "left"}


def test_backups_are_per_hand_and_never_cross(client: TestClient, tmp_path: Path) -> None:
    right = client.post("/api/right/backups", json={"tag": "bench"}).json()
    left = client.post("/api/left/backups", json={"tag": "bench"}).json()
    assert right["hand"] == "right" and left["hand"] == "left"
    assert (tmp_path / "right" / right["name"]).is_file() and (tmp_path / "left" / left["name"]).is_file()
    assert [b["name"] for b in client.get("/api/left/backups").json()] == [left["name"]]

    client.post("/api/right/command", json={"text": "set 2 vmax 900"})
    changes = client.post(f"/api/right/backups/{right['name']}/diff").json()["changes"]
    assert changes == [{"scope": "2", "key": "vmax", "current": "900", "wanted": "1333"}]
    restored = client.post(f"/api/right/backups/{right['name']}/restore", json={"save": True}).json()
    assert restored["ok"], restored["message"]

    # The left backup imported into the right hand's store must not restore onto the right hand.
    text = "\n".join(client.get(f"/api/left/backups/{left['name']}").json()["dump"])
    imported = client.post("/api/right/backups/import", json={"text": text, "tag": "wrong"}).json()
    crossed = client.post(f"/api/right/backups/{imported['name']}/restore", json={"save": False}).json()
    assert not crossed["ok"] and "left hand's" in crossed["message"]
    assert client.get("/api/right/backups/nope.txt").status_code == 404


def test_flat_backups_belong_to_the_right_hand(client: TestClient, tmp_path: Path) -> None:
    legacy = "\n".join(["defaults", "# 1 sim_j1", "set 1 enc 3", "set lead 0", "# end of dump: run `save` to store it"])
    (tmp_path / "20261021-143005.txt").write_text(legacy + "\n")
    assert [b["name"] for b in client.get("/api/right/backups").json()] == ["20261021-143005.txt"]
    assert client.get("/api/left/backups").json() == []


def test_power_summary_and_stream(client: TestClient) -> None:
    deadline = time.monotonic() + 3
    hands = {}
    while not (set(hands) == {"right", "left"} and all(w["samples"] > 10 for w in hands.values())):
        assert time.monotonic() < deadline, "no power windows"
        time.sleep(0.05)
        hands = client.get("/api/power").json()["hands"]
    assert hands["right"]["min_v"] == 6.0 and hands["left"]["armed"] == 0
    with client.websocket_connect("/ws") as ws:
        assert "trips" in receive_until(ws, "snapshot")["data"]
        power = receive_until(ws, "power")
        assert set(power["data"]) == {"right", "left"}
