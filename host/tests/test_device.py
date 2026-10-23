"""Device (asyncio) over a real-time fake device."""

from __future__ import annotations

from pathlib import Path

import pytest

from teleop_host.config_files import BackupStore, diff, extract_dump, settings_of
from teleop_host.device import Device
from teleop_host.protocol.messages import EventCode
from teleop_host.sim import FakeDevice


@pytest.fixture
async def device():
    fake = FakeDevice()
    dev = Device()
    dev.connect(lambda: fake, "sim", reconnect=False)
    assert await dev.wait_connected(3.0)
    yield dev, fake
    dev.disconnect()
    fake.shutdown()


async def test_command_results(device) -> None:
    dev, _ = device
    ping = await dev.command("ping")
    assert ping.ok and sum(e.code == EventCode.PING_RESULT for e in ping.events) == 9
    bad = await dev.command("frobnicate")
    assert bad.status == "parse_error" and "unknown command" in bad.error
    refused = await dev.command("arm 1")
    assert refused.status == "failed" and "not ready" in refused.error
    status = await dev.command("status")
    assert status.ok and status.logs and status.logs[0].startswith("armed 0 joint(s)")
    assert dev.info is not None and dev.telemetry is not None


async def test_dump_backup_restore_round_trip(device, tmp_path: Path) -> None:
    dev, fake = device
    assert (await dev.command("set 2 vmax 900")).ok
    store = BackupStore(tmp_path)
    backup = store.save(await dev.dump(), tag="woody pc", note="first")
    assert backup.name.endswith("_woody-pc.txt") and backup.tag == "woody-pc" and backup.note == "first"
    assert store.list()[0].name == backup.name

    assert (await dev.command("defaults")).ok
    changes = await dev.diff(backup.dump)
    assert [(c.scope, c.key, c.current, c.wanted) for c in changes] == [("2", "vmax", "1333", "900")]

    result = await dev.restore(backup.dump)
    assert result.ok, result.message
    assert await dev.diff(backup.dump) == []
    assert fake.model.nvs is not None  # saved


async def test_restore_refuses_another_hand_and_stops_at_errors(device) -> None:
    dev, _ = device
    dump = await dev.dump()
    other = [line.replace("sim_j2", "index_near") for line in dump]
    assert not (await dev.restore(other)).ok
    broken = [line.replace("set 2 vmax 1333", "set 2 vmax 99999") for line in dump]
    result = await dev.restore(broken)
    assert not result.ok and "set 2 vmax 99999" in result.message and "load" in result.message


def test_backup_parsing_matches_calib_tool_format(tmp_path: Path) -> None:
    text = "\n".join(
        [
            "# Teleoperation calibration backup",
            "# saved 2026-10-21 14:30:05 from /dev/cu.usbmodem1101",
            "defaults",
            "# 1 thumb_far",
            "set 1 enc 3",
            "set 1 on",
            "[ctl] interleaved message",
            "set lead 0",
            "# end of dump: run `save` to store it",
        ]
    )
    (tmp_path / "20261021-143005.txt").write_text(text)
    backup = BackupStore(tmp_path).get("20261021-143005.txt")
    assert backup.saved == "2026-10-21 14:30:05 from /dev/cu.usbmodem1101"
    assert settings_of(backup.dump) == {("1", "enc"): "3", ("1", "enabled"): "on", ("global", "lead"): "0"}
    assert extract_dump(["no dump here"]) is None
    assert diff(backup.dump, backup.dump) == []
    with pytest.raises(ValueError):
        BackupStore(tmp_path).get("../etc/passwd.txt")
