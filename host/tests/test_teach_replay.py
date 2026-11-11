"""Replay: resampling, and record / replay through the API with two simulated hands."""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from teleop_host.config_files import HandBackups
from teleop_host.hands import HandManager
from teleop_host.plugins import builtin_plugins
from teleop_host.protocol.messages import ReplayPhase
from teleop_host.server import create_app
from teleop_host.sim import FakeDevice
from teleop_host.teach.player import resample
from teleop_host.teach.store import JOINT_FIELDS, HandMeta, HandTrack, RecordingMeta, RecordingStore


def test_resample_bridges_gaps_and_scales_speed() -> None:
    track = HandTrack(t=[0.0, 0.01, 0.04], cycle=[1, 2, 5], t_us=[0, 0, 0])
    track.joints[2] = {name: [0.0, 0.0, 0.0] for name in JOINT_FIELDS}
    track.joints[2]["goal"] = [100.0, 110.0, 140.0]
    assert resample(track, [2], 100, 1.0) == [[100], [110], [120], [130], [140]]
    assert resample(track, [2], 100, 0.5) == [[100], [105], [110], [115], [120], [125], [130], [135], [140]]
    late = HandTrack(t=[0.02, 0.03], cycle=[1, 2], t_us=[0, 0], joints={2: {n: [7.0, 9.0] for n in JOINT_FIELDS}})
    assert resample(late, [2], 100, 1.0) == [[7], [7], [7], [9]]  # holds the first value before it starts


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
        HandManager(),
        HandBackups(tmp_path / "calib"),
        builtin_plugins(),
        recordings=RecordingStore(tmp_path / "recordings"),
        sim_factories=factories,
        initial_ports=list(factories),
    )
    with TestClient(app) as c:
        deadline = time.monotonic() + 4
        while not all((h["link"] or {}).get("state") == "connected" for h in c.get("/api/hands").json().values()):
            assert time.monotonic() < deadline, "simulated hands did not connect"
            time.sleep(0.05)
        for hand in ("right", "left"):
            assert c.post(f"/api/{hand}/command", json={"text": "set 2 on"}).json()["ok"]
        yield c


def ramp_recording(store: RecordingStore, client: TestClient, boards, rise: float, seconds: float = 1.5) -> str:
    """A recording of servo 2 on both hands rising by `rise` steps from where it is now."""
    hands, tracks = {}, {}
    n = round(seconds * 100)
    for hand in ("right", "left"):
        info = client.get(f"/api/{hand}/info").json()["info"]
        start = boards[f"sim-{hand}"].model.joints[1].position
        track = HandTrack(t=[i / 100 for i in range(n)], cycle=list(range(n)), t_us=[i * 10_000 for i in range(n)])
        for j in info["joints"]:
            track.joints[j["servo_id"]] = {name: [0.0] * n for name in JOINT_FIELDS}
        track.joints[2]["goal"] = [start + rise * i / (n - 1) for i in range(n)]
        joints = [{"servo_id": j["servo_id"], "name": j["name"], "model": j["model"]} for j in info["joints"]]
        hands[hand] = HandMeta("sim", "", joints, [2], [f"# hand {hand}"], {"a": 1.0, "b": 0.0}, n, 0)
        tracks[hand] = track
    meta = RecordingMeta(store.new_id("ramp"), "ramp", "2026-10-28T12:00:00", seconds, hands)
    return store.save(meta, tracks).id


def store_of(client: TestClient) -> RecordingStore:
    return client.app.state.teach.store  # type: ignore[attr-defined]


def test_record_through_the_api(client: TestClient) -> None:
    started = client.post("/api/teach/record/start", json={"hands": ["right", "left"], "name": "pick"}).json()
    assert started["record"]["recording"]
    assert client.post("/api/teach/record/start", json={"hands": ["right"]}).status_code == 409
    time.sleep(0.4)
    assert client.post("/api/teach/record/marker", json={"label": "grasp"}).json()["label"] == "grasp"
    time.sleep(0.2)
    meta = client.post("/api/teach/record/stop").json()
    assert set(meta["hands"]) == {"right", "left"} and meta["markers"][0]["label"] == "grasp"
    assert all(h["samples"] > 30 for h in meta["hands"].values())
    assert meta["hands"]["left"]["dump"][1] == "# hand left"
    rid = meta["id"]
    assert [r["id"] for r in client.get("/api/teach/recordings").json()] == [rid]
    preview = client.get(f"/api/teach/recordings/{rid}/preview?points=20").json()
    assert len(preview["hands"]["right"]["t"]) <= 20 and "2" in preview["hands"]["right"]["joints"]
    csv = client.get(f"/api/teach/recordings/{rid}/left.csv")
    assert csv.status_code == 200 and csv.text.startswith("t,cycle,t_us,j1_goal")
    assert client.patch(f"/api/teach/recordings/{rid}", json={"name": "pick cup"}).json()["name"] == "pick cup"
    # nothing followed the glove: there is nothing to replay
    assert client.post("/api/teach/replay/prepare", json={"id": rid}).status_code == 400
    assert client.delete(f"/api/teach/recordings/{rid}").json()["ok"]
    assert client.get(f"/api/teach/recordings/{rid}").status_code == 404
    assert client.get("/api/teach/recordings/..%2Fx").status_code in (400, 404)


def test_replay_both_hands_to_the_end(client: TestClient, boards) -> None:
    rid = ramp_recording(store_of(client), client, boards, rise=120)
    ready = client.post("/api/teach/replay/prepare", json={"id": rid, "wait": True, "timeout": 10}).json()
    assert ready["state"] == "ready", ready["message"]
    assert all(b.model.replay_phase == ReplayPhase.READY for b in boards.values())
    assert client.post("/api/teach/replay/prepare", json={"id": rid}).status_code == 409  # busy
    starts = {h: b.model.joints[1].goal for h, b in boards.items()}
    done = client.post("/api/teach/replay/go", json={"at": time.time() + 0.2, "wait": True, "timeout": 10}).json()
    assert done["state"] == "done", done["message"]
    assert done["position_s"] == pytest.approx(1.49, abs=0.02)
    for name, board in boards.items():
        assert board.model.replay_phase == ReplayPhase.HOLDING
        assert board.model.joints[1].goal == pytest.approx(starts[name] + 120, abs=2)
        assert board.model.joints[0].state == 0  # only the recorded joint was armed


def test_replay_one_hand_and_estop(client: TestClient, boards) -> None:
    rid = ramp_recording(store_of(client), client, boards, rise=80, seconds=3.0)
    ready = client.post("/api/teach/replay/prepare", json={"id": rid, "hands": ["left"], "wait": True}).json()
    assert ready["state"] == "ready" and list(ready["hands"]) == ["left"]
    assert boards["sim-right"].model.replay_phase == ReplayPhase.IDLE
    client.post("/api/teach/replay/go", json={})
    time.sleep(0.5)
    assert client.get("/api/teach/status").json()["replay"]["state"] == "playing"
    client.post("/api/estop")
    deadline = time.monotonic() + 3
    while client.get("/api/teach/status").json()["replay"]["state"] == "playing":
        assert time.monotonic() < deadline, "E-stop did not end the replay"
        time.sleep(0.05)
    status = client.get("/api/teach/status").json()["replay"]
    assert status["state"] == "stopped" and "left" in status["message"]


def test_one_hand_dropping_out_leaves_the_other_playing(client: TestClient, boards) -> None:
    rid = ramp_recording(store_of(client), client, boards, rise=100, seconds=2.0)
    assert client.post("/api/teach/replay/prepare", json={"id": rid, "wait": True}).json()["state"] == "ready"
    client.post("/api/teach/replay/go", json={})
    time.sleep(0.4)
    client.post("/api/disconnect", json={"hand": "left"})
    deadline = time.monotonic() + 5
    while (status := client.get("/api/teach/status").json()["replay"])["state"] == "playing":
        assert time.monotonic() < deadline, "the replay did not end"
        time.sleep(0.05)
    assert status["state"] == "stopped" and status["message"] == "left: link lost"
    assert status["hands"]["right"]["play_index"] == status["hands"]["right"]["frames"]
    assert boards["sim-right"].model.replay_phase == ReplayPhase.HOLDING


def test_websocket_reports_teach(client: TestClient) -> None:
    with client.websocket_connect("/ws") as ws:
        snapshot = ws.receive_json()
        assert snapshot["type"] == "snapshot" and snapshot["data"]["teach"]["replay"]["state"] == "idle"
        client.post("/api/teach/record/start", json={"hands": ["left"], "name": "ws"})
        for _ in range(2000):
            message = ws.receive_json()
            if message["type"] == "teach_event":
                break
        assert message["data"]["kind"] == "record_started"
        for _ in range(2000):
            message = ws.receive_json()
            if message["type"] == "teach" and message["data"]["record"]["samples"].get("left", 0) > 0:
                break
        assert message["data"]["record"]["hands"] == ["left"]
        client.post("/api/teach/record/cancel")


def test_replay_refuses_what_does_not_fit(client: TestClient, boards) -> None:
    store = store_of(client)
    rid = ramp_recording(store, client, boards, rise=50)
    assert client.post("/api/teach/replay/prepare", json={"id": rid, "speed": 2}).status_code == 400
    assert client.post("/api/teach/replay/prepare", json={"id": "nope"}).status_code == 404
    assert client.post("/api/teach/replay/go", json={}).status_code == 409  # not ready
    meta = store.meta(rid)
    meta.hands["right"].joints[0]["name"] = "other"  # recorded with another joint table
    (store.directory / rid / "meta.json").write_text(json.dumps(meta.to_json()))
    refused = client.post("/api/teach/replay/prepare", json={"id": rid, "hands": ["right"]})
    assert refused.status_code == 400 and "joint table" in refused.json()["detail"]
    # a right-only recording cannot go onto the left hand
    meta.hands.pop("left")
    (store.directory / rid / "meta.json").write_text(json.dumps(meta.to_json()))
    assert client.post("/api/teach/replay/prepare", json={"id": rid, "hands": ["left"]}).status_code == 400
    assert client.get("/api/time").json()["time"] == pytest.approx(time.time(), abs=1)
