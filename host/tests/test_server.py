"""REST + WebSocket smoke tests with the simulated device."""

from __future__ import annotations

import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from teleop_host.config_files import BackupStore
from teleop_host.device import Device
from teleop_host.plugins import builtin_plugins
from teleop_host.server import create_app
from teleop_host.sim import FakeDevice


@pytest.fixture
def client(tmp_path: Path):
    fake = FakeDevice()
    app = create_app(Device(), BackupStore(tmp_path), builtin_plugins(), sim_factory=lambda: fake, initial_port="sim")
    with TestClient(app) as c:
        deadline = time.monotonic() + 3
        while c.get("/api/info").json()["link"]["state"] != "connected":
            assert time.monotonic() < deadline, "simulated device did not connect"
            time.sleep(0.05)
        yield c
    fake.shutdown()


def receive_until(ws, kind: str, limit: int = 500) -> dict:
    for _ in range(limit):
        message = ws.receive_json()
        if message["type"] == kind:
            return message
    raise AssertionError(f"no {kind} message")


def test_rest_command_ports_and_plugins(client: TestClient) -> None:
    info = client.get("/api/info").json()
    assert info["info"]["firmware"] == "sim" and len(info["info"]["joints"]) == 9
    result = client.post("/api/command", json={"text": "ping"}).json()
    assert result["ok"] and len(result["events"]) == 9
    assert client.get("/api/ports").json()["ports"][0]["device"] == "sim"
    assert {p["id"] for p in client.get("/api/plugins").json()} >= {"recording", "camera"}


def test_websocket_snapshot_telemetry_command_and_plot(client: TestClient) -> None:
    with client.websocket_connect("/ws") as ws:
        snapshot = receive_until(ws, "snapshot")
        assert snapshot["data"]["link"]["state"] == "connected"
        tel = receive_until(ws, "tel")
        assert len(tel["data"]["joints"]) == 9
        ws.send_json({"type": "cmd", "id": 7, "text": "set 2 vmax 900"})
        result = receive_until(ws, "cmd_result")
        assert result["id"] == 7 and result["data"]["ok"]
        ws.send_json({"type": "plot_subscribe", "keys": ["j2.exo_deg", "j2.present", "j99.bogus"]})
        plot = receive_until(ws, "plot")
        assert set(plot["data"]["series"]) == {"j2.exo_deg", "j2.present"}
        assert len(plot["data"]["t_us"]) == len(plot["data"]["series"]["j2.present"])


def test_backup_endpoints(client: TestClient) -> None:
    saved = client.post("/api/backups", json={"tag": "bench", "note": "n"}).json()
    name = saved["name"]
    assert [b["name"] for b in client.get("/api/backups").json()] == [name]
    assert client.get(f"/api/backups/{name}").json()["dump"][0] == "defaults"
    client.post("/api/command", json={"text": "set 2 vmax 900"})
    changes = client.post(f"/api/backups/{name}/diff").json()["changes"]
    assert changes == [{"scope": "2", "key": "vmax", "current": "900", "wanted": "1333"}]
    restored = client.post(f"/api/backups/{name}/restore", json={"save": True}).json()
    assert restored["ok"], restored["message"]
    assert client.post(f"/api/backups/{name}/diff").json()["changes"] == []
    assert client.get("/api/backups/nope.txt").status_code == 404
