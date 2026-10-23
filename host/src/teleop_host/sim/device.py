"""Byte-level fake ESP32: a Transport whose other end runs the console's text
mode and host link session (src/console.cpp) around a HandModel.

Used by the tests and by `teleop-host --sim`. Like the firmware, it starts
in text mode, answers `host on` with HELLO, and falls back to text mode when
the heartbeat stops.
"""

from __future__ import annotations

import queue
import threading
import time

from ..protocol.frames import FLAG_REPLY_EXPECTED, Frame, FrameDecoder, encode_frame
from ..protocol.messages import (
    HELLO_ESTOP_ON_LINK_LOSS,
    Ack,
    AckStatus,
    Bye,
    ByeReason,
    Config,
    ConfigFormat,
    Done,
    Event,
    HostHello,
    Log,
    LogSource,
    MsgType,
    TelemConfig,
)
from .model import HandModel

CONTROL_HZ = 100


class FakeDevice:
    """Transport interface: write(), read(timeout), close()."""

    def __init__(self, model: HandModel | None = None, realtime: bool = True) -> None:
        self.model = model or HandModel()
        self.realtime = realtime
        self._out: queue.Queue[bytes] = queue.Queue()
        self._in: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.Lock()
        self._host = False
        self._config = HostHello(telemetry_divider=0, slow_divider=0)
        self._decoder = FrameDecoder()
        self._text = bytearray()
        self._seq = 0
        self._last_rx = 0.0
        self._closed = threading.Event()
        self.estops = 0
        self._send_text("[boot] encoders responding: 9 / 9 (simulated)\n")
        self._send_text("Teleoperation M2 ready. Type `help`.\n")
        self._thread: threading.Thread | None = None
        if realtime:
            self._thread = threading.Thread(target=self._run, name="fake-device", daemon=True)
            self._thread.start()

    # ---- Transport ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        if self._closed.is_set():
            raise OSError("fake device closed")
        self._in.put(bytes(data))
        if not self.realtime:
            self.poll()

    def read(self, timeout: float = 0.05) -> bytes:
        try:
            chunk = self._out.get(timeout=timeout)
        except queue.Empty:
            return b""
        parts = [chunk]
        while True:
            try:
                parts.append(self._out.get_nowait())
            except queue.Empty:
                return b"".join(parts)

    def close(self) -> None:
        self._closed.set()

    # ---- device side -----------------------------------------------------------------

    @property
    def host_mode(self) -> bool:
        return self._host

    def _run(self) -> None:
        period = 1.0 / CONTROL_HZ
        next_tick = time.monotonic()
        while not self._closed.is_set():
            self.poll()
            self.step()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def poll(self) -> None:
        """Handles everything the host has written so far."""
        while True:
            try:
                data = self._in.get_nowait()
            except queue.Empty:
                return
            with self._lock:
                if self._host:
                    self._feed_frames(data)
                else:
                    self._feed_text(data)

    def step(self, now: float | None = None) -> None:
        """One control cycle: motion, telemetry, watchdog."""
        now = time.monotonic() if now is None else now
        with self._lock:
            events = self.model.tick(1.0 / CONTROL_HZ)
            if not self._host:
                for e in events:
                    self._send_text(f"[ctl] {e.text}\n")
                return
            for e in events:
                self._send(MsgType.EVENT, e.encode())
            cycle = self.model.cycle
            if self._config.telemetry_divider and cycle % self._config.telemetry_divider == 0:
                self._send(MsgType.TELEMETRY, self.model.telemetry(int(now * 1e6)).encode())
            if self._config.slow_divider and cycle % self._config.slow_divider == 0:
                self._send(MsgType.SLOW_STATUS, self.model.slow_status().encode())
            if now - self._last_rx > self._config.heartbeat_timeout_ms / 1000:
                self._exit(ByeReason.HEARTBEAT_TIMEOUT)

    def _send_text(self, text: str) -> None:
        self._out.put(text.encode())

    def _send(self, type_: MsgType, payload: bytes) -> None:
        self._out.put(encode_frame(type_, self._seq, payload))
        self._seq = (self._seq + 1) & 0xFFFF

    def _feed_text(self, data: bytes) -> None:
        self._text += data
        while b"\n" in self._text:
            raw, _, rest = bytes(self._text).partition(b"\n")
            self._text = bytearray(rest)
            line = raw.decode(errors="replace").strip()
            if line == "host on":
                self._enter()
                # Bytes after the line are already frames.
                remaining = bytes(self._text)
                self._text.clear()
                if remaining:
                    self._feed_frames(remaining)
                return
            result = self.model.handle(line)
            if result.parse_error:
                self._send_text(f"error: {result.parse_error}\n")
            for log in result.logs:
                self._send_text(log + "\n")
            if result.config:
                self._send_text(result.config)
            for e in result.events:
                self._send_text(f"[ctl] {e.text}\n")

    def _enter(self) -> None:
        self._host = True
        self._config = HostHello(telemetry_divider=0, slow_divider=0)
        self._decoder = FrameDecoder()
        self._last_rx = time.monotonic()
        self._out.put(b"\x00")
        self._send(MsgType.HELLO, self.model.device_info().encode())

    def _exit(self, reason: ByeReason) -> None:
        self._send(MsgType.BYE, Bye(reason).encode())
        self._host = False
        if reason == ByeReason.HEARTBEAT_TIMEOUT:
            stop = bool(self._config.flags & HELLO_ESTOP_ON_LINK_LOSS) and self.model.armed_count() > 0
            if stop:
                self.model.estop("host link lost")
                self.estops += 1
            self._send_text(f"[host] link lost{', torque off' if stop else ''}, text console\n")
        else:
            self._send_text("[host] link closed, text console\n")

    def _feed_frames(self, data: bytes) -> None:
        for item in self._decoder.feed(data):
            if isinstance(item, Frame) and self._host:
                self._last_rx = time.monotonic()
                self._handle_frame(item)

    def _ack(self, seq: int, status: AckStatus = AckStatus.ACCEPTED, text: str = "") -> None:
        self._send(MsgType.ACK, Ack(seq, status, text).encode())

    def _handle_frame(self, f: Frame) -> None:
        match f.type:
            case MsgType.HEARTBEAT:
                if f.flags & FLAG_REPLY_EXPECTED:
                    self._ack(f.seq)
            case MsgType.ESTOP:
                self.estops += 1
                self._send(MsgType.EVENT, self.model.estop("E-stop (host)").encode())
                self._ack(f.seq)
            case MsgType.HOST_HELLO:
                hello = HostHello.decode(f.payload)
                if hello.protocol != 1:
                    self._ack(f.seq, AckStatus.BAD_FRAME, "unsupported protocol version")
                    return
                hello.heartbeat_timeout_ms = min(max(hello.heartbeat_timeout_ms, 200), 10000)
                self._config = hello
                self._ack(f.seq)
            case MsgType.TELEM_CONFIG:
                c = TelemConfig.decode(f.payload)
                self._config.telemetry_divider, self._config.slow_divider = c.telemetry_divider, c.slow_divider
                self._ack(f.seq)
            case MsgType.GET_INFO:
                self._send(MsgType.HELLO, self.model.device_info().encode())
            case MsgType.HOST_EXIT:
                self._ack(f.seq)
                self._exit(ByeReason.HOST_EXIT)
            case MsgType.COMMAND:
                self._command(f.seq, f.payload.decode(errors="replace"))
            case _:
                self._ack(f.seq, AckStatus.BAD_FRAME, "unknown frame type")

    def _command(self, seq: int, line: str) -> None:
        if len(line) > 127:
            self._ack(seq, AckStatus.PARSE_ERROR, "command longer than 127 characters")
            return
        if line.strip() in ("host on", "host off"):
            self._ack(seq)
            self._send(MsgType.DONE, Done(seq, True).encode())
            if line.strip() == "host off":
                self._exit(ByeReason.HOST_EXIT)
            return
        result = self.model.handle(line)
        if result.parse_error:
            self._ack(seq, AckStatus.PARSE_ERROR, result.parse_error)
            return
        self._ack(seq)
        for log in result.logs:
            self._send(MsgType.LOG, Log(LogSource.CONSOLE, log).encode())
        if result.config is not None:
            self._send(MsgType.CONFIG, Config(seq, ConfigFormat.DUMP_TEXT, result.config).encode())
        for e in result.events:
            tagged = Event(e.code, e.severity, e.joint, seq, e.args, e.text)
            self._send(MsgType.EVENT, tagged.encode())
        self._send(MsgType.DONE, Done(seq, result.ok).encode())
