"""FastAPI app: REST for connection and backups, one WebSocket per browser tab.

WebSocket messages (JSON, `type` field):

server -> browser
  snapshot   on connect: link status, device info, latest slow status, recent events and log lines
  link       link status changed
  info       HELLO (device info, joint table)
  tel        latest telemetry, at most TEL_HZ
  slow       latest slow status, at most SLOW_HZ
  plot       {t_us: [...], series: {"j2.exo_deg": [...]}}: every sample of the subscribed channels
  event      one device event
  log        one printed / unframed line
  cmd_result reply to a `cmd`

browser -> server
  cmd             {id, text}: one console command
  estop           E-stop now (priority frame)
  plot_subscribe  {keys: ["j2.exo_deg", ...]}
  telemetry_rate  {telemetry_divider, slow_divider}
"""

from __future__ import annotations

import asyncio
import contextlib
import dataclasses
import math
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import APIRouter, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .config_files import BackupStore, extract_dump
from .device import CommandResult, Device
from .link.discovery import find_esp32, list_serial_ports
from .link.link import LinkStatus
from .link.serial_transport import SerialTransport
from .link.transport import Transport
from .plugins import Plugin
from .protocol.frames import UnframedText
from .protocol.messages import DeviceInfo, Event, Log, SlowStatus, Telemetry, TelemetryJoint

TEL_HZ = 30.0
SLOW_HZ = 5.0
PLOT_FLUSH_S = 0.05
HISTORY_ON_CONNECT = 300
SIM_PORT = "sim"

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
}


def to_json(obj: Any) -> Any:
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_json(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
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


class Client:
    def __init__(self, ws: WebSocket) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=2000)
        self.plot_keys: list[tuple[int, str, str]] = []  # (joint index, channel, key)
        self.plot_t: list[int] = []
        self.plot_series: dict[str, list[float]] = {}

    def push(self, message: dict[str, Any]) -> None:
        with contextlib.suppress(asyncio.QueueFull):  # a stalled tab loses messages, not the server
            self.queue.put_nowait(message)


class Hub:
    """Fans device messages out to every connected browser."""

    def __init__(self, device: Device, plugins: list[Plugin]) -> None:
        self.device = device
        self.plugins = plugins
        self.clients: set[Client] = set()
        self._last_tel = 0.0
        self._last_slow = 0.0
        self._tasks: list[asyncio.Task[None]] = []

    def start(self) -> None:
        self.device.subscribe(self.on_message)
        self._tasks.append(asyncio.create_task(self._flush_plots()))

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task

    def broadcast(self, message: dict[str, Any]) -> None:
        for client in self.clients:
            client.push(message)

    def on_message(self, m: Any) -> None:
        for plugin in self.plugins:
            with contextlib.suppress(Exception):
                plugin.on_message(m)
        loop_time = asyncio.get_running_loop().time()
        match m:
            case Telemetry():
                self._collect_plot(m)
                if loop_time - self._last_tel >= 1 / TEL_HZ:
                    self._last_tel = loop_time
                    self.broadcast({"type": "tel", "data": to_json(m)})
            case SlowStatus():
                if loop_time - self._last_slow >= 1 / SLOW_HZ:
                    self._last_slow = loop_time
                    self.broadcast({"type": "slow", "data": to_json(m)})
            case DeviceInfo():
                self.broadcast({"type": "info", "data": to_json(m)})
            case Event():
                self.broadcast({"type": "event", "data": to_json(m)})
            case Log():
                self.broadcast({"type": "log", "data": {"source": m.source, "text": m.text}})
            case UnframedText():
                for line in m.text.replace("\x00", "").splitlines():
                    if line.strip():
                        self.broadcast({"type": "log", "data": {"source": -1, "text": line}})
            case LinkStatus():
                self.broadcast({"type": "link", "data": link_json(m)})

    def _collect_plot(self, t: Telemetry) -> None:
        info = self.device.info
        k = info.degrees_per_count if info else 360 / 16384
        for client in self.clients:
            if not client.plot_keys:
                continue
            client.plot_t.append(t.t_us)
            for index, channel, key in client.plot_keys:
                if index < len(t.joints):
                    client.plot_series.setdefault(key, []).append(PLOT_CHANNELS[channel](t.joints[index], k))

    async def _flush_plots(self) -> None:
        while True:
            await asyncio.sleep(PLOT_FLUSH_S)
            for client in self.clients:
                if client.plot_t:
                    client.push({"type": "plot", "data": {"t_us": client.plot_t, "series": client.plot_series}})
                    client.plot_t, client.plot_series = [], {}

    def subscribe_plot(self, client: Client, keys: list[str]) -> None:
        info = self.device.info
        ids = [j.servo_id for j in info.joints] if info else []
        parsed = []
        for key in keys[:64]:
            joint, _, channel = key.partition(".")
            if channel in PLOT_CHANNELS and joint.startswith("j") and joint[1:].isdigit() and int(joint[1:]) in ids:
                parsed.append((ids.index(int(joint[1:])), channel, key))
        client.plot_keys, client.plot_t, client.plot_series = parsed, [], {}


def link_json(s: LinkStatus) -> dict[str, str]:
    return {"state": s.state.value, "port": s.port, "detail": s.detail, "firmware": s.firmware}


class ConnectRequest(BaseModel):
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
    device: Device,
    store: BackupStore,
    plugins: list[Plugin],
    *,
    static_dir: Path | None = None,
    sim_factory: Callable[[], Transport] | None = None,
    initial_port: str | None = None,
) -> FastAPI:
    hub = Hub(device, plugins)

    def factory_for(port: str) -> Callable[[], Transport]:
        if port == SIM_PORT:
            if sim_factory is None:
                raise HTTPException(400, "simulator not enabled (start with --sim)")
            return sim_factory
        return lambda: SerialTransport(port)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        hub.start()
        for plugin in plugins:
            await plugin.start(device)
        if initial_port:
            device.connect(factory_for(initial_port), initial_port)
        yield
        device.disconnect()
        for plugin in plugins:
            await plugin.stop()
        await hub.stop()

    app = FastAPI(title="teleop-host", lifespan=lifespan)
    api = APIRouter(prefix="/api")

    @api.get("/ports")
    def ports() -> dict[str, Any]:
        found = [dataclasses.asdict(p) for p in list_serial_ports()]
        if sim_factory is not None:
            found.insert(0, {"device": SIM_PORT, "description": "Simulated device", "is_esp32": True})
        return {"ports": found, "suggested": SIM_PORT if sim_factory and not find_esp32() else find_esp32()}

    @api.post("/connect")
    async def connect(req: ConnectRequest) -> dict[str, Any]:
        port = req.port or find_esp32()
        if not port:
            raise HTTPException(400, "no ESP32 found: pick a port")
        device.connect(factory_for(port), port)
        return link_json(device.status)

    @api.post("/disconnect")
    def disconnect() -> dict[str, Any]:
        device.disconnect()
        return link_json(device.status)

    @api.get("/info")
    def info() -> dict[str, Any]:
        return {"link": link_json(device.status), "info": to_json(device.info)}

    @api.post("/command")
    async def command(req: CommandRequest) -> dict[str, Any]:
        return result_json(await device.command(req.text))

    @api.post("/estop")
    async def estop() -> dict[str, bool]:
        return {"sent": await device.estop()}

    @api.get("/backups")
    def backups() -> list[dict[str, Any]]:
        return [{"name": b.name, "saved": b.saved, "tag": b.tag, "note": b.note} for b in store.list()]

    def load_backup(name: str):
        try:
            return store.get(name)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(404, str(exc)) from exc

    @api.get("/backups/{name}")
    def backup(name: str) -> dict[str, Any]:
        b = load_backup(name)
        return {"name": b.name, "saved": b.saved, "tag": b.tag, "note": b.note, "dump": b.dump}

    @api.get("/backups/{name}/download", response_class=PlainTextResponse)
    def download(name: str) -> str:
        return load_backup(name).path.read_text()

    @api.post("/backups")
    async def save_backup(req: BackupRequest) -> dict[str, Any]:
        try:
            dump = await device.dump()
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        b = store.save(dump, req.tag, req.note, device.status.port)
        return {"name": b.name, "saved": b.saved, "tag": b.tag, "note": b.note}

    @api.post("/backups/import")
    def import_backup(req: ImportRequest) -> dict[str, Any]:
        dump = extract_dump(req.text.splitlines())
        if dump is None:
            raise HTTPException(400, "no complete dump (`defaults` ... `# end of dump`) in the text")
        b = store.save(dump, req.tag, req.note, "pasted text")
        return {"name": b.name, "saved": b.saved, "tag": b.tag, "note": b.note}

    @api.post("/backups/{name}/diff")
    async def diff(name: str) -> dict[str, Any]:
        b = load_backup(name)
        try:
            changes = await device.diff(b.dump)
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"changes": to_json(changes)}

    @api.post("/backups/{name}/restore")
    async def restore(name: str, req: RestoreRequest) -> dict[str, Any]:
        b = load_backup(name)
        try:
            result = await device.restore(b.dump, save=req.save, force=req.force)
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"ok": result.ok, "message": result.message, "mismatch": to_json(result.mismatch)}

    @api.get("/plugins")
    def plugin_list() -> list[dict[str, str]]:
        return [p.describe() for p in plugins]

    app.include_router(api)
    for plugin in plugins:
        if (router := plugin.router()) is not None:
            app.include_router(router, prefix=f"/api/plugins/{plugin.id}")

    @app.websocket("/ws")
    async def websocket(ws: WebSocket) -> None:
        await ws.accept()
        client = Client(ws)
        hub.clients.add(client)
        client.push(
            {
                "type": "snapshot",
                "data": {
                    "link": link_json(device.status),
                    "info": to_json(device.info),
                    "slow": to_json(device.slow),
                    "tel": to_json(device.telemetry),
                    "events": to_json(list(device.events)[-HISTORY_ON_CONNECT:]),
                    "logs": list(device.logs)[-HISTORY_ON_CONNECT:],
                    "plugins": [p.describe() for p in plugins],
                },
            }
        )
        sender = asyncio.create_task(_send_loop(ws, client))
        try:
            while True:
                message = await ws.receive_json()
                await _handle_browser(message, client)
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(client)
            sender.cancel()

    async def _handle_browser(message: dict[str, Any], client: Client) -> None:
        kind = message.get("type")
        if kind == "estop":
            await device.estop()
        elif kind == "cmd":

            async def run() -> None:
                result = await device.command(str(message.get("text", "")))
                client.push({"type": "cmd_result", "id": message.get("id"), "data": result_json(result)})

            asyncio.create_task(run())  # commands may take seconds; keep reading (E-stop!)
        elif kind == "plot_subscribe":
            hub.subscribe_plot(client, [str(k) for k in message.get("keys", [])])
        elif kind == "telemetry_rate":
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
    with contextlib.suppress(Exception):
        while True:
            await ws.send_json(await client.queue.get())
