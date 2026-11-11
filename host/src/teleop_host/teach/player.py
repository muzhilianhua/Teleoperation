"""Plays a recording back on one or both hands.

The board plays frames from its own ring buffer on its control clock; the
host only keeps that buffer about LEAD_S ahead (docs/host-protocol.md,
Replay), so USB and scheduling jitter never reach the motion. Per hand:

  replay begin -> prefill -> replay start (arm, approach the first frame) -> ready

Once every hand is ready, `replay go` goes to all of them together, which
lines the hands' starts up. Each hand plays its recorded `goal` of the joints
that followed the glove while recording, resampled to the board's rate at the
chosen speed; the limiter, grip limiter and supervisor still apply on the
board. At the end each hand holds its last frame until the operator disarms.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from ..hands import HandManager
from ..link.link import LinkState, LinkStatus
from ..protocol.messages import Event, EventCode, ReplayChunk, ReplayPhase, ReplayStatus
from .store import HandTrack, Recording, RecordingStore

LEAD_S = 1.0  # buffered ahead of playback (the board holds 2.56 s)
CHUNK_FRAMES = 20  # firmware ReplayChunkItem::kMaxFrames
START_FRAMES = 50  # firmware kReplayStartFrames
RESEND_S = 0.3  # frames sent but not buffered after this long were dropped: resend
TICK_S = 0.02
READY_TIMEOUT_S = 20.0
MIN_SPEED, MAX_SPEED = 0.25, 1.0


class PlayerState(StrEnum):
    IDLE = "idle"
    PREPARING = "preparing"  # loading and approaching the first frame
    READY = "ready"
    PLAYING = "playing"
    DONE = "done"  # every hand played to its end and holds
    STOPPED = "stopped"
    ERROR = "error"


ACTIVE = (PlayerState.PREPARING, PlayerState.READY, PlayerState.PLAYING)


class ReplayEnded(RuntimeError):
    """The board left the replay on its own: disarm, E-stop or a fault."""


@dataclass(slots=True)
class HandPlan:
    servo_ids: list[int]
    mask: int
    frames: list[list[int]]  # one value per joint in the mask, in joint order
    sent: int = 0
    last_send: float = 0.0
    phase: ReplayPhase = ReplayPhase.IDLE
    play_index: int = 0
    buffered_until: int = 0
    underruns: int = 0
    lost: bool = False
    ended: str = ""  # why this hand left the replay early; the other hands play on


@dataclass(slots=True)
class PlayerStatus:
    state: PlayerState = PlayerState.IDLE
    message: str = ""
    recording_id: str = ""
    name: str = ""
    speed: float = 1.0
    duration_s: float = 0.0  # recording time
    position_s: float = 0.0  # recording time played so far (slowest hand)
    hands: dict[str, dict[str, Any]] = field(default_factory=dict)


def resample(track: HandTrack, servo_ids: list[int], rate_hz: float, speed: float) -> list[list[int]]:
    """`goal` of `servo_ids` at rate_hz from recording time 0, `speed` x real time.

    Linear between samples, so dropped cycles are bridged; before the first
    sample the first value is held.
    """
    t = track.t
    if not t:
        return []
    columns = [track.column(s, "goal") for s in servo_ids]
    count = math.floor(t[-1] / speed * rate_hz + 1e-9) + 1
    frames: list[list[int]] = []
    i = 0
    for k in range(count):
        tau = k * speed / rate_hz
        while i + 1 < len(t) and t[i + 1] <= tau:
            i += 1
        if tau <= t[0] or i + 1 >= len(t):
            frames.append([_i16(c[0 if tau <= t[0] else i]) for c in columns])
        else:
            f = (tau - t[i]) / (t[i + 1] - t[i])
            frames.append([_i16(c[i] + f * (c[i + 1] - c[i])) for c in columns])
    return frames


def _i16(v: float) -> int:
    return max(-32768, min(32767, round(v)))


class Player:
    def __init__(self, hands: HandManager, store: RecordingStore, clock: Callable[[], float] = time.monotonic) -> None:
        self._hands = hands
        self._store = store
        self._clock = clock
        self._plans: dict[str, HandPlan] = {}
        self._recording: Recording | None = None
        self._rate = 100.0
        self._task: asyncio.Task[None] | None = None
        self._listeners: list[Callable[[str, dict[str, Any]], None]] = []
        self.state = PlayerState.IDLE
        self.message = ""
        self._going = False  # go() called, the boards not playing yet
        self.speed = 1.0

    def on_event(self, callback: Callable[[str, dict[str, Any]], None]) -> None:
        """callback(kind, data) on ready / playing / done / stopped / error."""
        self._listeners.append(callback)

    @property
    def active(self) -> bool:
        return self.state in ACTIVE

    # ---- control ----------------------------------------------------------------------

    def prepare(self, recording_id: str, hands: list[str], speed: float = 1.0) -> None:
        """Checks the recording against the boards, then loads and approaches in the background.

        Raises LookupError (no such recording), ValueError (does not fit) or
        RuntimeError (busy, hand not connected).
        """
        if self.active:
            raise RuntimeError(f"a replay is already {self.state}")
        if not MIN_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"speed must be {MIN_SPEED}..{MAX_SPEED}")
        if not hands:
            raise ValueError("no hand to replay")
        recording = self._store.load(recording_id)
        plans: dict[str, HandPlan] = {}
        rate = 100.0
        for hand in hands:
            meta = recording.meta.hands.get(hand)
            if meta is None:
                raise ValueError(f"the recording has no {hand} hand")
            device = self._hands.device(hand)
            info = device.info if device and device.connected else None
            if info is None:
                raise RuntimeError(f"{hand} hand not connected")
            board = [(j.servo_id, j.name, j.model) for j in info.joints]
            if [(j["servo_id"], j["name"], j["model"]) for j in meta.joints] != board:
                raise ValueError(f"{hand}: recorded with another joint table (ids / names / models in config.h)")
            ids = [s for s, _, _ in board if s in meta.active]
            if not ids:
                raise ValueError(f"{hand}: no joint followed the glove in this recording")
            rate = float(info.control_hz or 100)
            frames = resample(recording.tracks[hand], ids, rate, speed)
            if len(frames) < START_FRAMES:
                raise ValueError(f"{hand}: recording too short ({len(frames)} frames, {START_FRAMES} needed)")
            mask = sum(1 << i for i, (s, _, _) in enumerate(board) if s in ids)
            plans[hand] = HandPlan(ids, mask, frames)
        self._plans, self._recording, self._rate, self.speed = plans, recording, rate, speed
        self._set(PlayerState.PREPARING, f"loading {', '.join(hands)}")
        self._run(self._prepare())

    def go(self, at: float | None = None) -> None:
        """Plays on every prepared hand, now or at epoch time `at` (time.time())."""
        if self.state != PlayerState.READY:
            raise RuntimeError(f"not ready (replay is {self.state})")
        self._going = True
        self._run(self._play(at))

    async def stop(self, reason: str = "stopped by the operator") -> None:
        task, self._task = self._task, None
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        await self._stop_hands()
        if self.active:
            self._set(PlayerState.STOPPED, reason, "stopped")

    async def wait(self, timeout: float) -> PlayerState:
        """Until preparing, or a started replay, has ended; returns the state."""
        deadline = self._clock() + timeout
        while self._clock() < deadline and (
            self.state in (PlayerState.PREPARING, PlayerState.PLAYING)
            or (self._going and self.state == PlayerState.READY)
        ):
            await asyncio.sleep(TICK_S)
        return self.state

    def status(self) -> PlayerStatus:
        rec = self._recording
        hands = {
            hand: {
                "phase": p.phase.name.lower(),
                "play_index": p.play_index,
                "frames": len(p.frames),
                "sent": p.sent,
                "underruns": p.underruns,
                "servo_ids": p.servo_ids,
                "ended": p.ended,
            }
            for hand, p in self._plans.items()
        }
        playing = [p.play_index for p in self._plans.values() if not p.ended]
        played = min(playing or [p.play_index for p in self._plans.values()], default=0)
        return PlayerStatus(
            self.state,
            self.message,
            rec.meta.id if rec else "",
            rec.meta.name if rec else "",
            self.speed,
            rec.meta.duration_s if rec else 0.0,
            round(played / self._rate * self.speed, 3),
            hands,
        )

    # ---- board messages --------------------------------------------------------------

    def on_message(self, hand: str | None, message: Any) -> None:
        plan = self._plans.get(hand or "")
        if plan is None:
            return
        if isinstance(message, ReplayStatus):
            with contextlib.suppress(ValueError):
                plan.phase = ReplayPhase(message.phase)
            if plan.phase != ReplayPhase.IDLE:  # back at idle the board has cleared its counters; keep the last ones
                plan.play_index = message.play_index
                plan.buffered_until = message.buffered_until
                plan.underruns = message.underruns
        elif isinstance(message, Event) and message.code == EventCode.REPLAY:  # args: phase, play index, mask
            with contextlib.suppress(ValueError):
                plan.phase = ReplayPhase(message.args[0])
                if plan.phase != ReplayPhase.IDLE:
                    plan.play_index = max(plan.play_index, message.args[1])
        elif isinstance(message, LinkStatus) and message.state != LinkState.CONNECTED:
            plan.lost = True

    # ---- sequence ----------------------------------------------------------------------

    def _run(self, coroutine: Any) -> None:
        self._task = asyncio.create_task(self._guard(coroutine))

    async def _guard(self, coroutine: Any) -> None:
        try:
            await coroutine
        except asyncio.CancelledError:
            raise
        except ReplayEnded as exc:
            await self._stop_hands()
            self._set(PlayerState.STOPPED, str(exc), "stopped")
        except Exception as exc:  # noqa: BLE001  (reported, and the hands are told to hold)
            await self._stop_hands()
            self._set(PlayerState.ERROR, str(exc), "error")

    async def _command_all(self, text: str) -> None:
        results = await asyncio.gather(*(self._device(h).command(text) for h in self._plans))
        for hand, result in zip(self._plans, results, strict=True):
            if not result.ok:
                reason = result.error or "; ".join(e.text for e in result.events) or result.status
                raise RuntimeError(f"{hand}: `{text}` failed: {reason}")

    async def _prepare(self) -> None:
        for plan in self._plans.values():
            plan.phase = ReplayPhase.LOADING
        await self._command_all("replay begin")
        for hand, plan in self._plans.items():
            self._feed(hand, plan)
        await self._command_all("replay start")
        deadline = self._clock() + READY_TIMEOUT_S
        while not all(p.phase == ReplayPhase.READY for p in self._plans.values()):
            self._check_hands()
            if self._clock() > deadline:
                raise RuntimeError("the joints did not reach the first frame in time")
            await asyncio.sleep(TICK_S)
        self._set(PlayerState.READY, "at the first frame: ready to play", "ready")

    async def _play(self, at: float | None) -> None:
        if at is not None:
            await asyncio.sleep(max(0.0, at - time.time()))
        await self._command_all("replay go")
        for plan in self._plans.values():
            plan.phase = ReplayPhase.PLAYING
        self._set(PlayerState.PLAYING, "playing", "playing")
        # From here on the hands are independent: one that drops out (link lost,
        # disarmed, fault) is set aside and the others play to their end.
        while True:
            for hand, plan in self._plans.items():
                if not plan.ended:
                    plan.ended = self._ended(hand, plan)
                if not plan.ended:
                    self._feed(hand, plan)
            if all(p.ended or p.phase == ReplayPhase.HOLDING for p in self._plans.values()):
                break
            await asyncio.sleep(TICK_S)
        self._task = None
        problems = [
            f"{h}: {p.ended}" if p.ended else f"{h}: stopped at frame {p.play_index} of {len(p.frames)}"
            for h, p in self._plans.items()
            if p.ended or p.play_index < len(p.frames)
        ]
        if problems:
            self._set(PlayerState.STOPPED, "; ".join(problems), "stopped")
        else:
            self._set(PlayerState.DONE, "finished: holding the last pose until disarmed", "done")

    def _ended(self, hand: str, plan: HandPlan) -> str:
        device = self._hands.device(hand)
        if plan.lost or device is None or not device.connected:
            return "link lost"
        if plan.phase == ReplayPhase.IDLE:
            return "replay ended on the board (disarmed, E-stop or a fault)"
        return ""

    def _check_hands(self) -> None:
        """While preparing, any hand dropping out ends the whole replay."""
        for hand, plan in self._plans.items():
            reason = self._ended(hand, plan)
            if reason == "link lost":
                raise RuntimeError(f"{hand}: link lost")
            if reason:
                raise ReplayEnded(f"{hand}: {reason}")

    def _feed(self, hand: str, plan: HandPlan) -> None:
        now = self._clock()
        if plan.buffered_until < plan.sent and now - plan.last_send > RESEND_S:
            plan.sent = max(plan.buffered_until, plan.play_index)  # chunks were dropped on the way
        lead = round(LEAD_S * self._rate)
        device = self._hands.device(hand)
        if device is None:
            plan.ended = "link lost"
            return
        while plan.sent < len(plan.frames) and plan.sent - plan.play_index < lead:
            count = min(CHUNK_FRAMES, len(plan.frames) - plan.sent, lead - (plan.sent - plan.play_index))
            end = plan.sent + count == len(plan.frames)
            chunk = ReplayChunk(plan.sent, plan.mask, plan.frames[plan.sent : plan.sent + count], end)
            if not device.send_replay(chunk):
                plan.ended = "link lost"
                return
            plan.sent += count
            plan.last_send = now

    async def _stop_hands(self) -> None:
        hands = [
            h
            for h, p in self._plans.items()
            if p.phase not in (ReplayPhase.IDLE, ReplayPhase.HOLDING) and (d := self._hands.device(h)) and d.connected
        ]
        await asyncio.gather(*(self._device(h).command("replay stop") for h in hands))

    def _device(self, hand: str) -> Any:
        device = self._hands.device(hand)
        if device is None:
            raise RuntimeError(f"{hand}: link lost")
        return device

    def _set(self, state: PlayerState, message: str, event: str | None = None) -> None:
        self.state, self.message = state, message
        if state != PlayerState.READY:
            self._going = False
        if event is not None:
            data = {"recording_id": self._recording.meta.id if self._recording else "", "message": message}
            for callback in list(self._listeners):
                with contextlib.suppress(Exception):
                    callback(f"replay_{event}", data)
