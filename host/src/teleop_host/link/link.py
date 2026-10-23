"""Host link session over a Transport (docs/host-protocol.md, "Session").

One supervisor thread opens the transport, switches the device to the host
link (`host on` -> HELLO -> HOST_HELLO), then reads frames until the session
ends; it reconnects with backoff unless stopped. A writer thread owns all
writes after the handshake: ESTOP jumps the queue, and when nothing else is
queued it sends the heartbeat the device's watchdog waits for.

Callbacks run on the link's threads; the asyncio side (teleop_host.device)
hands them over to its event loop.
"""

from __future__ import annotations

import itertools
import queue
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import StrEnum

from ..protocol.frames import FLAG_REPLY_EXPECTED, Frame, FrameDecoder, UnframedText, encode_frame
from ..protocol.messages import Bye, DeviceInfo, DeviceMessage, HostHello, MsgType, decode_device_message
from .transport import Transport

# Sent before `host on`: a device still in host mode (host restarted within the
# heartbeat timeout) leaves it first; in text mode the frame's 0x00 just clears
# the line buffer. seq 0 keeps the frame free of '\n' bytes.
TAKEOVER = encode_frame(MsgType.HOST_EXIT, 0) + b"host on\n"
HANDSHAKE_TIMEOUT_S = 1.5
HANDSHAKE_ATTEMPTS = 3
HEARTBEAT_S = 0.2
HEARTBEAT_ACK_EVERY = 5  # ask for an ACK once a second so a silent device is noticed
RX_SILENCE_S = 1.5  # nothing received for this long: the session is gone (device reset?)
BACKOFF_S = (0.5, 1.0, 2.0, 5.0)

PRIORITY_ESTOP = 0
PRIORITY_NORMAL = 1


class LinkState(StrEnum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class LinkStatus:
    state: LinkState
    port: str = ""
    detail: str = ""
    firmware: str = ""


@dataclass(slots=True)
class LinkStats:
    frames_rx: int = 0
    frames_tx: int = 0
    decode_errors: int = 0
    sessions: int = 0


class LinkError(Exception):
    pass


MessageCallback = Callable[[DeviceMessage | UnframedText], None]
StatusCallback = Callable[[LinkStatus], None]


@dataclass(order=True)
class _Outgoing:
    priority: int
    order: int
    wire: bytes = field(compare=False)


class LinkManager:
    def __init__(
        self,
        factory: Callable[[], Transport],
        port_label: str,
        on_message: MessageCallback,
        on_status: StatusCallback,
        *,
        hello: HostHello | None = None,
        reconnect: bool = True,
    ) -> None:
        self._factory = factory
        self._port = port_label
        self._on_message = on_message
        self._on_status = on_status
        self._hello = hello or HostHello()
        self._reconnect = reconnect
        self._status = LinkStatus(LinkState.DISCONNECTED, port_label)
        self._info: DeviceInfo | None = None
        self.stats = LinkStats()
        self.decoder_stats = FrameDecoder().stats
        self._seq = itertools.count(1)
        self._seq_lock = threading.Lock()
        self._order = itertools.count()
        self._outgoing: queue.PriorityQueue[_Outgoing] = queue.PriorityQueue()
        self._stop = threading.Event()
        self._session_over = threading.Event()
        self._connected = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- public API ---------------------------------------------------------------

    @property
    def status(self) -> LinkStatus:
        return self._status

    @property
    def info(self) -> DeviceInfo | None:
        return self._info

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._supervise, name="host-link", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        """Leaves the host link (HOST_EXIT: the device keeps its state) and closes the port."""
        if self._connected.is_set():
            self.send(MsgType.HOST_EXIT)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def wait_connected(self, timeout: float) -> bool:
        return self._connected.wait(timeout)

    def next_seq(self) -> int:
        with self._seq_lock:
            seq = next(self._seq) & 0xFFFF
            if seq == 0:  # 0 means "unsolicited" in EVENT.cmdSeq
                seq = next(self._seq) & 0xFFFF
            return seq

    def send(self, type_: MsgType, payload: bytes = b"", *, seq: int | None = None, flags: int = 0) -> int:
        """Queues a frame; returns its seq. Raises LinkError when not connected."""
        if not self._connected.is_set():
            raise LinkError("device not connected")
        seq = self.next_seq() if seq is None else seq
        priority = PRIORITY_ESTOP if type_ == MsgType.ESTOP else PRIORITY_NORMAL
        self._outgoing.put(_Outgoing(priority, next(self._order), encode_frame(type_, seq, payload, flags)))
        return seq

    def estop(self) -> int:
        return self.send(MsgType.ESTOP)

    # ---- threads --------------------------------------------------------------------

    def _set_status(self, state: LinkState, detail: str = "") -> None:
        firmware = self._info.firmware if self._info and state == LinkState.CONNECTED else ""
        self._status = LinkStatus(state, self._port, detail, firmware)
        self._on_status(self._status)

    def _supervise(self) -> None:
        failures = 0
        while not self._stop.is_set():
            self._set_status(LinkState.CONNECTING)
            try:
                transport = self._factory()
            except OSError as exc:
                self._set_status(LinkState.ERROR, str(exc))
            else:
                try:
                    reason = self._session(transport)
                    failures = 0
                    self._set_status(LinkState.DISCONNECTED, reason)
                except (OSError, LinkError) as exc:
                    self._set_status(LinkState.ERROR, str(exc))
                finally:
                    self._connected.clear()
                    transport.close()
            if self._stop.is_set() or not self._reconnect:
                break
            self._stop.wait(BACKOFF_S[min(failures, len(BACKOFF_S) - 1)])
            failures += 1
        self._set_status(LinkState.DISCONNECTED, "closed")

    def _session(self, transport: Transport) -> str:
        decoder = FrameDecoder()
        self.decoder_stats = decoder.stats
        self._drain_text(transport, decoder)
        self._handshake(transport, decoder)
        self.stats.sessions += 1
        self._session_over.clear()
        while not self._outgoing.empty():  # nothing from a previous session
            self._outgoing.get_nowait()
        self._connected.set()
        self._set_status(LinkState.CONNECTED)
        writer = threading.Thread(target=self._write_loop, args=(transport,), name="host-link-tx", daemon=True)
        writer.start()
        try:
            return self._read_loop(transport, decoder)
        finally:
            self._connected.clear()
            self._session_over.set()
            writer.join(1.0)

    def _drain_text(self, transport: Transport, decoder: FrameDecoder) -> None:
        """Shows whatever the text console printed before we took over (boot output)."""
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            data = transport.read(0.05)
            if data:
                self._on_message(UnframedText(data))

    def _handshake(self, transport: Transport, decoder: FrameDecoder) -> None:
        for _ in range(HANDSHAKE_ATTEMPTS):
            transport.write(TAKEOVER)
            deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
            while time.monotonic() < deadline and not self._stop.is_set():
                for item in decoder.feed(transport.read(0.05)):
                    if isinstance(item, UnframedText):
                        self._on_message(item)
                        continue
                    if item.type != MsgType.HELLO:
                        continue
                    info = DeviceInfo.decode(item.payload)
                    if info.protocol != self._hello.protocol:
                        raise LinkError(f"device speaks host link v{info.protocol}, this host v{self._hello.protocol}")
                    self._info = info
                    self._on_message(info)
                    transport.write(encode_frame(MsgType.HOST_HELLO, self.next_seq(), self._hello.encode()))
                    return
            if self._stop.is_set():
                raise LinkError("stopped")
        raise LinkError("no HELLO from the device: is the teleoperation firmware running on this port?")

    def _read_loop(self, transport: Transport, decoder: FrameDecoder) -> str:
        last_rx = time.monotonic()
        while not self._stop.is_set():
            data = transport.read(0.05)
            now = time.monotonic()
            if not data:
                if now - last_rx > RX_SILENCE_S:
                    return "device stopped answering"
                continue
            for item in decoder.feed(data):
                if isinstance(item, UnframedText):
                    self._on_message(item)
                    continue
                last_rx = now
                message = self._decode(item)
                if message is None:
                    continue
                if isinstance(message, DeviceInfo):
                    self._info = message
                self._on_message(message)
                if isinstance(message, Bye):
                    return "device left the host link" if message.reason == 0 else "device heartbeat timeout"
        # Stopping: give the HOST_EXIT a moment to go out and be answered.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            for item in decoder.feed(transport.read(0.05)):
                if isinstance(item, Frame) and item.type == MsgType.BYE:
                    return "closed"
        return "closed"

    def _decode(self, frame: Frame) -> DeviceMessage | None:
        self.stats.frames_rx += 1
        try:
            return decode_device_message(frame.type, frame.payload)
        except (KeyError, ValueError, IndexError, struct.error):  # one bad frame must not end the session
            self.stats.decode_errors += 1
            return None

    def _write_loop(self, transport: Transport) -> None:
        beats = 0
        while not self._session_over.is_set():
            try:
                item = self._outgoing.get(timeout=HEARTBEAT_S)
                wire = item.wire
            except queue.Empty:
                beats += 1
                flags = FLAG_REPLY_EXPECTED if beats % HEARTBEAT_ACK_EVERY == 0 else 0
                wire = encode_frame(MsgType.HEARTBEAT, self.next_seq(), b"", flags)
            try:
                transport.write(wire)
                self.stats.frames_tx += 1
            except OSError:
                self._session_over.set()
                return
