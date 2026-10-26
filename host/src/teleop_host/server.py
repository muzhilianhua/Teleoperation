"""FastAPI app for both hands: REST for connections and backups, one WebSocket per browser tab.

Every device message carries the hand it came from ("right" / "left"; null
for a board whose HELLO has not arrived yet).

WebSocket messages (JSON, `type` field):

server -> browser
  snapshot   on connect: per hand link, info, latest tel / slow, recent events and logs; plugins
  link       {hand, port, data}: link status changed
  info       {hand, data}: HELLO (device info, joint table)
  tel        {hand, data}: latest telemetry, at most TEL_HZ per hand
  slow       {hand, data}: latest slow status, at most SLOW_HZ per hand
  plot       {t_us: {hand: [...]}, series: {"right.j2.exo_deg": [...]}}: every sample of subscribed channels
  event      {hand, data}: one device event
  log        {hand, data}: one printed / unframed line
  notice     {data: {text}}: pairing problems (two boards on one hand)
  power      {data: {hand: window}}: voltages and estimated supply current over the last second, POWER_HZ
  trip       {data: record}: a fault switched joints off; both hands' power in the second before
  cmd_result reply to a `cmd`

browser -> server
  cmd             {id, hand, text}: one console command on that hand
  estop           E-stop on every hand (priority frame)
  plot_subscribe  {keys: ["right.j2.exo_deg", ...]}
  telemetry_rate  {telemetry_divider, slow_divider}: both hands
"""

from __future__ import annotations

import asyncio
import contextlib
import dataclasses
import logging
import math
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import APIRouter, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .config_files import Backup, BackupStore, HandBackups, extract_dump
from .device import CommandResult, Device
from .hands import HANDS, HandManager, HandNotice
from .link.discovery import list_serial_ports
from .link.link import LinkStatus
from .link.serial_transport import SerialTransport
from .link.transport import Transport
from .plugins import Plugin
from .power import PowerMonitor
from .protocol.frames import UnframedText
from .protocol.messages import DeviceInfo, Event, Log, SlowStatus, Telemetry, TelemetryJoint

TEL_HZ = 30.0
SLOW_HZ = 5.0
PLOT_FLUSH_S = 0.05
POWER_HZ = 5.0
HISTORY_ON_CONNECT = 300

# Plot channels per joint: name -> value from (joint record, deg per encoder count).
PLOT_CHANNELS: dict[str, Callable[[TelemetryJoint, float], float]] = {
    "exo_deg": lambda j, k: j.exo_counts * k,
    "exo_filt_deg": lambda j, k: j.exo_filtered_counts * k,
    "target": lambda j, k: j.target,
    "goal": lambda j, k: j.goal,
    "sent": lambda j, k: j.sent_goal,
    "ref": lambda j, k: j.reference,
    "present": lambda j, k: j.position,
    "speed_cmd": lambda j, k: j.speed_command,
    "grip_lead": lambda j, k: j.grip_lead,
    "load_pct": lambda j, k: j.load_deci_pct / 10,
    "current_ma": lambda j, k: j.current_ma,
    "supply_est_ma": lambda j, k: abs(j.current_ma) * abs(j.load_deci_pct) / 1000,
    "temp_c": lambda j, k: j.temperature_c,
    "voltage_v": lambda j, k: j.voltage_deci_v / 10,
}


def to_json(obj: Any) -> Any:
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_json(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, dict):
        return {str(k): to_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_json(v) for v in obj]
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def result_json(r: CommandResult) -> dict[str, Any]:
    return {
        "text": r.text,
        "ok": r.ok,
        "status": r.status,
        "error": r.error,
        "events": to_json(r.events),
        "logs": r.logs,
        "config": r.config,
    }


def link_json(s: LinkStatus) -> dict[str, str]:
    return {"state": s.state.value, "port": s.port, "detail": s.detail, "firmware": s.firmware}


def backup_json(b: Backup) -> dict[str, Any]:
    return {"name": b.name, "saved": b.saved, "tag": b.tag, "note": b.note, "hand": b.hand}


class Client:
    def __init__(self, ws: WebSocket) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=4000)
        self.plot_keys: dict[str, list[tuple[int, str, str]]] = {}  # hand -> (joint index, channel, key)
        self.plot_t: dict[str, list[int]] = {}
        self.plot_series: dict[str, list[float]] = {}

    def push(self, message: dict[str, Any]) -> None:
        with contextlib.suppress(asyncio.QueueFull):  # a stalled tab loses messages, not the server
            self.queue.put_nowait(message)


class Hub:
    """Fans both hands' messages out to every connected browser."""

    def __init__(self, hands: HandManager, plugins: list[Plugin], power: PowerMonitor) -> None:
        self.hands = hands
        self.plugins = plugins
        self.power = power
        self.clients: set[Client] = set()
        self._last_tel: dict[str, float] = {}
        self._last_slow: dict[str, float] = {}
        self._tasks: list[asyncio.Task[None]] = []

    def start(self) -> None:
        self.hands.subscribe(self.on_message)
        self.power.on_trip(lambda r: self.broadcast({"type": "trip", "data": to_json(r)}))
        self._tasks.append(asyncio.create_task(self._flush_plots()))
        self._tasks.append(asyncio.create_task(self._send_power()))

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task

    def broadcast(self, message: dict[str, Any]) -> None:
        for client in self.clients:
            client.push(message)

    def on_message(self, hand: str | None, m: Any) -> None:
        self.power.on_message(hand, m)
        for plugin in self.plugins:
            with contextlib.suppress(Exception):
                plugin.on_message(hand, m)
        now = asyncio.get_running_loop().time()
        key = hand or "?"
        match m:
            case Telemetry():
                if hand:
                    self._collect_plot(hand, m)
                if now - self._last_tel.get(key, 0.0) >= 1 / TEL_HZ:
                    self._last_tel[key] = now
                    self.broadcast({"type": "tel", "hand": hand, "data": to_json(m)})
            case SlowStatus():
                if now - self._last_slow.get(key, 0.0) >= 1 / SLOW_HZ:
                    self._last_slow[key] = now
                    self.broadcast({"type": "slow", "hand": hand, "data": to_json(m)})
            case DeviceInfo():
                self.broadcast({"type": "info", "hand": hand, "data": to_json(m)})
            case Event():
                self.broadcast({"type": "event", "hand": hand, "data": to_json(m)})
            case Log():
                self.broadcast({"type": "log", "hand": hand, "data": {"source": m.source, "text": m.text}})
            case UnframedText():
                for line in m.text.replace("\x00", "").splitlines():
                    if line.strip():
                        self.broadcast({"type": "log", "hand": hand, "data": {"source": -1, "text": line}})
            case LinkStatus():
                self.broadcast({"type": "link", "hand": hand, "port": m.port, "data": link_json(m)})
            case HandNotice():
                self.broadcast({"type": "notice", "data": {"text": m.text}})

    def _collect_plot(self, hand: str, t: Telemetry) -> None:
        device = self.hands.device(hand)
        k = device.info.degrees_per_count if device and device.info else 360 / 16384
        for client in self.clients:
            keys = client.plot_keys.get(hand)
            if not keys:
                continue
            client.plot_t.setdefault(hand, []).append(t.t_us)
            for index, channel, key in keys:
                if index < len(t.joints):
                    client.plot_series.setdefault(key, []).append(PLOT_CHANNELS[channel](t.joints[index], k))

    async def _flush_plots(self) -> None:
        while True:
            await asyncio.sleep(PLOT_FLUSH_S)
            for client in self.clients:
                if client.plot_t:
                    client.push({"type": "plot", "data": {"t_us": client.plot_t, "series": client.plot_series}})
                    client.plot_t, client.plot_series = {}, {}

    async def _send_power(self) -> None:
        while True:
            await asyncio.sleep(1 / POWER_HZ)
            if self.clients:
                self.broadcast({"type": "power", "data": to_json(self.power.summary())})

    def subscribe_plot(self, client: Client, keys: list[str]) -> None:
        parsed: dict[str, list[tuple[int, str, str]]] = {}
        for key in keys[:128]:
            hand, _, rest = key.partition(".")
            joint, _, channel = rest.partition(".")
            device = self.hands.device(hand)
            info = device.info if device else None
            if info is None or channel not in PLOT_CHANNELS or not joint.startswith("j") or not joint[1:].isdigit():
                continue
            ids = [j.servo_id for j in info.joints]
            if int(joint[1:]) in ids:
                parsed.setdefault(hand, []).append((ids.index(int(joint[1:])), channel, key))
        client.plot_keys, client.plot_t, client.plot_series = parsed, {}, {}


def hand_state(hands: HandManager, hand: str) -> dict[str, Any]:
    device = hands.device(hand)
    if device is None:
        return {"link": None, "port": "", "info": None, "slow": None, "tel": None, "events": [], "logs": []}
    return {
        "link": link_json(device.status),
        "port": hands.port_of(hand),
        "info": to_json(device.info),
        "slow": to_json(device.slow),
        "tel": to_json(device.telemetry),
        "events": to_json(list(device.events)[-HISTORY_ON_CONNECT:]),
        "logs": list(device.logs)[-HISTORY_ON_CONNECT:],
    }


class ConnectRequest(BaseModel):
    port: str


class DisconnectRequest(BaseModel):
    hand: str | None = None
    port: str | None = None


class CommandRequest(BaseModel):
    text: str


class BackupRequest(BaseModel):
    tag: str = ""
    note: str = ""


class ImportRequest(BaseModel):
    text: str
    tag: str = "imported"
    note: str = ""


class RestoreRequest(BaseModel):
    save: bool = True
    force: bool = False


def create_app(
    hands: HandManager,
    backups: HandBackups,
    plugins: list[Plugin],
    *,
    static_dir: Path | None = None,
    sim_factories: dict[str, Callable[[], Transport]] | None = None,
    initial_ports: list[str] | None = None,
) -> FastAPI:
    power = PowerMonitor(lambda hand: d.info if (d := hands.device(hand)) else None)
    hub = Hub(hands, plugins, power)
    sims = sim_factories or {}

    def factory_for(port: str) -> Callable[[], Transport]:
        if port in sims:
            return sims[port]
        return lambda: SerialTransport(port)

    def connect_port(port: str) -> None:
        try:
            hands.connect(factory_for(port), port)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        hub.start()
        for plugin in plugins:
            await plugin.start(hands)
        for port in initial_ports or []:
            with contextlib.suppress(HTTPException):
                connect_port(port)
        yield
        hands.disconnect_all()
        for plugin in plugins:
            await plugin.stop()
        await hub.stop()

    app = FastAPI(title="teleop-host", lifespan=lifespan)
    api = APIRouter(prefix="/api")

    def require_hand(hand: str) -> str:
        if hand not in HANDS:
            raise HTTPException(404, f"no hand {hand!r}: use right or left")
        return hand

    def device_of(hand: str) -> Device:
        device = hands.device(require_hand(hand))
        if device is None:
            raise HTTPException(409, f"{hand} hand not connected")
        return device

    def store_of(hand: str) -> BackupStore:
        return backups.for_hand(require_hand(hand))

    # ---- connections ----------------------------------------------------------------

    @api.get("/ports")
    def ports() -> dict[str, Any]:
        found = [{"device": name, "description": "Simulated hand", "is_esp32": True} for name in sims]
        found += [dataclasses.asdict(p) for p in list_serial_ports()]
        return {"ports": found, "in_use": sorted(hands.ports_in_use())}

    @api.post("/connect")
    def connect(req: ConnectRequest) -> dict[str, Any]:
        connect_port(req.port)
        return {"port": req.port}

    @api.post("/disconnect")
    def disconnect(req: DisconnectRequest) -> dict[str, Any]:
        if req.hand:
            hands.disconnect(require_hand(req.hand))
        elif req.port:
            hands.disconnect_port(req.port)
        else:
            hands.disconnect_all()
        return {"ok": True}

    @api.get("/hands")
    def hand_list() -> dict[str, Any]:
        out = {}
        for hand in HANDS:
            device = hands.device(hand)
            out[hand] = {
                "link": link_json(device.status) if device else None,
                "port": hands.port_of(hand),
                "board_id": device.info.board_id if device and device.info else "",
            }
        return out

    @api.post("/estop")
    async def estop() -> dict[str, list[str]]:
        return {"reached": await hands.estop_all()}

    # ---- per hand ---------------------------------------------------------------------

    @api.get("/{hand}/info")
    def info(hand: str) -> dict[str, Any]:
        device = device_of(hand)
        return {"link": link_json(device.status), "info": to_json(device.info)}

    @api.post("/{hand}/command")
    async def command(hand: str, req: CommandRequest) -> dict[str, Any]:
        return result_json(await device_of(hand).command(req.text))

    @api.get("/{hand}/backups")
    def backup_list(hand: str) -> list[dict[str, Any]]:
        return [backup_json(b) for b in store_of(hand).list()]

    def load_backup(hand: str, name: str) -> Backup:
        try:
            return store_of(hand).get(name)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(404, str(exc)) from exc

    @api.get("/{hand}/backups/{name}")
    def backup(hand: str, name: str) -> dict[str, Any]:
        b = load_backup(hand, name)
        return backup_json(b) | {"dump": b.dump}

    @api.get("/{hand}/backups/{name}/download", response_class=PlainTextResponse)
    def download(hand: str, name: str) -> str:
        return load_backup(hand, name).path.read_text()

    @api.post("/{hand}/backups")
    async def save_backup(hand: str, req: BackupRequest) -> dict[str, Any]:
        device = device_of(hand)
        try:
            dump = await device.dump()
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        return backup_json(store_of(hand).save(dump, req.tag, req.note, hands.port_of(hand)))

    @api.post("/{hand}/backups/import")
    def import_backup(hand: str, req: ImportRequest) -> dict[str, Any]:
        dump = extract_dump(req.text.splitlines())
        if dump is None:
            raise HTTPException(400, "no complete dump (`defaults` ... `# end of dump`) in the text")
        return backup_json(store_of(hand).save(dump, req.tag, req.note, "pasted text"))

    @api.post("/{hand}/backups/{name}/diff")
    async def diff(hand: str, name: str) -> dict[str, Any]:
        b = load_backup(hand, name)
        try:
            changes = await device_of(hand).diff(b.dump)
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"changes": to_json(changes)}

    @api.post("/{hand}/backups/{name}/restore")
    async def restore(hand: str, name: str, req: RestoreRequest) -> dict[str, Any]:
        b = load_backup(hand, name)
        try:
            result = await device_of(hand).restore(b.dump, save=req.save, force=req.force)
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"ok": result.ok, "message": result.message, "mismatch": to_json(result.mismatch)}

    @api.get("/power")
    def power_state() -> dict[str, Any]:
        return {"hands": to_json(power.summary()), "trips": to_json(list(power.trips))}

    @api.get("/plugins")
    def plugin_list() -> list[dict[str, str]]:
        return [p.describe() for p in plugins]

    # Plugin routes first: /api/plugins/... must not be taken for a hand.
    for plugin in plugins:
        if (router := plugin.router()) is not None:
            app.include_router(router, prefix=f"/api/plugins/{plugin.id}")
    app.include_router(api)

    # ---- WebSocket ------------------------------------------------------------------

    @app.websocket("/ws")
    async def websocket(ws: WebSocket) -> None:
        await ws.accept()
        client = Client(ws)
        hub.clients.add(client)
        client.push(
            {
                "type": "snapshot",
                "data": {
                    "hands": {h: hand_state(hands, h) for h in HANDS},
                    "plugins": [p.describe() for p in plugins],
                    "power": to_json(power.summary()),
                    "trips": to_json(list(power.trips)),
                },
            }
        )
        sender = asyncio.create_task(_send_loop(ws, client))
        try:
            while True:
                await _handle_browser(await ws.receive_json(), client)
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(client)
            sender.cancel()

    async def _handle_browser(message: dict[str, Any], client: Client) -> None:
        kind = message.get("type")
        if kind == "estop":
            await hands.estop_all()
        elif kind == "cmd":

            async def run() -> None:
                device = hands.device(str(message.get("hand")))
                if device is None:
                    result = CommandResult(str(message.get("text", "")), status="not_connected")
                    result.error = f"{message.get('hand')} hand not connected"
                else:
                    result = await device.command(str(message.get("text", "")))
                client.push({"type": "cmd_result", "id": message.get("id"), "data": result_json(result)})

            asyncio.create_task(run())  # commands may take seconds; keep reading (E-stop!)
        elif kind == "plot_subscribe":
            hub.subscribe_plot(client, [str(k) for k in message.get("keys", [])])
        elif kind == "telemetry_rate":
            for device in hands.devices.values():
                device.set_telemetry(int(message.get("telemetry_divider", 1)), int(message.get("slow_divider", 10)))

    if static_dir is not None and (static_dir / "index.html").is_file():
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="web")
    else:

        @app.get("/", response_class=HTMLResponse)
        def no_ui() -> str:
            return (
                "<h1>teleop-host</h1><p>The web UI is not built. Run <code>bun install && bun run build</code> "
                "in <code>host/web</code>, or use <code>--dev</code> with the Vite dev server.</p>"
            )

    return app


async def _send_loop(ws: WebSocket, client: Client) -> None:
    while True:
        message = await client.queue.get()
        try:
            await ws.send_json(message)
        except (TypeError, ValueError):
            # One message that does not serialise must not end the tab's stream.
            logging.getLogger(__name__).exception("dropped a %s message", message.get("type"))
        except Exception:  # noqa: BLE001  (the tab went away)
            return
