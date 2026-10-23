"""asyncio model of the connected device on top of the LinkManager threads.

- Keeps the latest HELLO, TELEMETRY and SLOW_STATUS plus a ring of events and
  log lines, and fans every message out to subscribers (the web hub, plugins).
- `command(text)` sends one console command and resolves on its DONE with the
  events it raised (they carry its seq), the lines it printed and, for `dump`,
  the configuration text.
- dump / restore / diff build on that, replacing tools/calib.py.
"""

from __future__ import annotations

import asyncio
import contextlib
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

from .config_files import Change, commands_of, diff, extract_dump, joint_names
from .link.link import LinkError, LinkManager, LinkState, LinkStatus
from .link.transport import Transport
from .protocol.frames import UnframedText
from .protocol.messages import (
    Ack,
    AckStatus,
    Config,
    DeviceInfo,
    DeviceMessage,
    Done,
    Event,
    HostHello,
    Log,
    MsgType,
    SlowStatus,
    TelemConfig,
    Telemetry,
)

ACK_TIMEOUT_S = 1.0
DONE_TIMEOUT_S = 8.0
EVENT_HISTORY = 1000
LOG_HISTORY = 1000

Subscriber = Callable[[DeviceMessage | UnframedText | LinkStatus], None]


@dataclass(slots=True)
class CommandResult:
    text: str
    seq: int = 0
    status: str = "ok"  # ok | failed | parse_error | busy | bad_frame | timeout | not_connected
    error: str = ""
    events: list[Event] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)
    config: str | None = None

    @property
    def ok(self) -> bool:
        return self.status == "ok"


@dataclass(slots=True)
class _Pending:
    result: CommandResult
    acked: asyncio.Future[None]
    done: asyncio.Future[None]


class Device:
    def __init__(self, hello: HostHello | None = None) -> None:
        self._hello = hello or HostHello()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._link: LinkManager | None = None
        self.status = LinkStatus(LinkState.DISCONNECTED)
        self.info: DeviceInfo | None = None
        self.telemetry: Telemetry | None = None
        self.slow: SlowStatus | None = None
        self.events: deque[Event] = deque(maxlen=EVENT_HISTORY)
        self.logs: deque[str] = deque(maxlen=LOG_HISTORY)
        self._pending: dict[int, _Pending] = {}
        self._log_target: int | None = None
        self._subscribers: list[Subscriber] = []

    # ---- connection ---------------------------------------------------------------

    def connect(self, factory: Callable[[], Transport], label: str, *, reconnect: bool = True) -> None:
        self._loop = asyncio.get_running_loop()
        if self._link is not None:
            self.disconnect()
        self._link = LinkManager(
            factory, label, self._from_thread, self._status_from_thread, hello=self._hello, reconnect=reconnect
        )
        self._link.start()

    def disconnect(self) -> None:
        link, self._link = self._link, None
        if link is not None:
            link.stop()
        self.status = LinkStatus(LinkState.DISCONNECTED)
        self._fail_pending("not_connected", "disconnected")
        self._publish(self.status)

    @property
    def connected(self) -> bool:
        return self._link is not None and self._link.connected

    @property
    def link(self) -> LinkManager | None:
        return self._link

    async def wait_connected(self, timeout: float) -> bool:
        deadline = asyncio.get_running_loop().time() + timeout
        while not self.connected:
            if asyncio.get_running_loop().time() > deadline:
                return False
            await asyncio.sleep(0.02)
        return True

    def subscribe(self, callback: Subscriber) -> Callable[[], None]:
        self._subscribers.append(callback)
        return lambda: self._subscribers.remove(callback) if callback in self._subscribers else None

    # ---- commands -----------------------------------------------------------------

    async def command(self, text: str, timeout: float = DONE_TIMEOUT_S) -> CommandResult:
        result = CommandResult(text.strip())
        if self._link is None or not self._link.connected:
            result.status, result.error = "not_connected", "device not connected"
            return result
        loop = asyncio.get_running_loop()
        seq = self._link.next_seq()
        result.seq = seq
        pending = _Pending(result, loop.create_future(), loop.create_future())
        self._pending[seq] = pending  # before sending: the reply can be quicker than this coroutine
        try:
            self._link.send(MsgType.COMMAND, result.text.encode(), seq=seq)
            await asyncio.wait_for(asyncio.shield(pending.acked), ACK_TIMEOUT_S)
            if result.status == "ok":
                await asyncio.wait_for(asyncio.shield(pending.done), timeout)
        except LinkError as exc:
            result.status, result.error = "not_connected", str(exc)
        except TimeoutError:
            result.status = "timeout"
            result.error = (
                "no acknowledgement from the device" if not pending.acked.done() else "command did not finish"
            )
        finally:
            self._pending.pop(seq, None)
        return result

    async def estop(self) -> bool:
        if self._link is None or not self._link.connected:
            return False
        self._link.estop()
        return True

    def set_telemetry(self, telemetry_divider: int, slow_divider: int) -> None:
        if self._link is not None and self._link.connected:
            self._link.send(MsgType.TELEM_CONFIG, TelemConfig(telemetry_divider, slow_divider).encode())

    async def dump(self) -> list[str]:
        result = await self.command("dump")
        dump = extract_dump((result.config or "").splitlines()) if result.ok else None
        if dump is None:
            raise RuntimeError(result.error or "no dump from the device")
        return dump

    async def diff(self, wanted: list[str]) -> list[Change]:
        return diff(await self.dump(), wanted)

    async def restore(self, wanted: list[str], *, save: bool = True, force: bool = False) -> RestoreResult:
        """Applies a dump, verifies it with a fresh dump, then saves to NVS.

        Stops at the first rejected command and sends `load`, so RAM falls back
        to what NVS holds.
        """
        current = await self.dump()
        if not force and joint_names(current) != joint_names(wanted):
            return RestoreResult(False, "the backup was made for another joint table (ids / names in config.h)")
        for line in commands_of(wanted):
            result = await self.command(line)
            if not result.ok:
                await self.command("load")
                reason = result.error or "; ".join(e.text for e in result.events) or result.status
                return RestoreResult(False, f"`{line}` rejected: {reason}; RAM reverted with `load`")
        mismatch = diff(await self.dump(), wanted)
        if mismatch:
            await self.command("load")
            return RestoreResult(False, "verification failed; RAM reverted with `load`", mismatch)
        if save:
            saved = await self.command("save")
            if not saved.ok:
                return RestoreResult(False, "applied and verified, but `save` failed")
        return RestoreResult(True, "applied, verified" + (" and saved to NVS" if save else " (not saved)"))

    # ---- incoming (link threads -> loop) ------------------------------------------------

    def _from_thread(self, message: DeviceMessage | UnframedText) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._dispatch, message)

    def _status_from_thread(self, status: LinkStatus) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._on_status, status)

    def _on_status(self, status: LinkStatus) -> None:
        self.status = status
        if status.state != LinkState.CONNECTED:
            self._fail_pending("not_connected", status.detail or status.state.value)
        self._publish(status)

    def _dispatch(self, m: DeviceMessage | UnframedText) -> None:
        match m:
            case Telemetry():
                self.telemetry = m
            case SlowStatus():
                self.slow = m
            case DeviceInfo():
                self.info = m
            case Event():
                self.events.append(m)
                if (p := self._pending.get(m.cmd_seq)) is not None:
                    p.result.events.append(m)
            case Log():
                self.logs.append(m.text)
                if self._log_target is not None and (p := self._pending.get(self._log_target)) is not None:
                    p.result.logs.append(m.text)
            case UnframedText():
                for line in m.text.replace("\x00", "").splitlines():
                    if line.strip():
                        self.logs.append(line)
            case Ack():
                self._on_ack(m)
            case Done():
                self._on_done(m)
            case Config():
                if (p := self._pending.get(m.ack_seq)) is not None:
                    p.result.config = m.text
        self._publish(m)

    def _on_ack(self, ack: Ack) -> None:
        p = self._pending.get(ack.ack_seq)
        if p is None:
            return
        if ack.status != AckStatus.ACCEPTED:
            p.result.status = {AckStatus.PARSE_ERROR: "parse_error", AckStatus.BUSY: "busy"}.get(
                AckStatus(ack.status), "bad_frame"
            )
            p.result.error = ack.text
        else:
            # Console-only verbs print between their ACK and DONE.
            self._log_target = ack.ack_seq
        if not p.acked.done():
            p.acked.set_result(None)

    def _on_done(self, done: Done) -> None:
        if self._log_target == done.ack_seq:
            self._log_target = None
        p = self._pending.get(done.ack_seq)
        if p is None:
            return
        if not done.ok:
            p.result.status = "failed"
            refusals = [e.text for e in p.result.events if e.severity >= 2]
            p.result.error = "; ".join(refusals)
        for future in (p.acked, p.done):
            if not future.done():
                future.set_result(None)

    def _fail_pending(self, status: str, error: str) -> None:
        for p in list(self._pending.values()):
            if p.result.status == "ok":
                p.result.status, p.result.error = status, error
            for future in (p.acked, p.done):
                if not future.done():
                    future.set_result(None)

    def _publish(self, message: DeviceMessage | UnframedText | LinkStatus) -> None:
        for callback in list(self._subscribers):
            with contextlib.suppress(Exception):  # one broken subscriber must not stop the others
                callback(message)


@dataclass(slots=True)
class RestoreResult:
    ok: bool
    message: str
    mismatch: list[Change] = field(default_factory=list)
