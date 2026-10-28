"""The teach feature as one object for the server: store, recorder and player."""

from __future__ import annotations

from typing import Any

from ..hands import HandManager
from .player import Player
from .recorder import Recorder
from .store import RecordingStore


class Teach:
    def __init__(self, hands: HandManager, store: RecordingStore) -> None:
        self.store = store

        async def dump_of(hand: str) -> list[str]:
            device = hands.device(hand)
            if device is None:
                raise RuntimeError(f"{hand} hand not connected")
            return await device.dump()

        self.recorder = Recorder(store, lambda hand: d.info if (d := hands.device(hand)) else None, dump_of)
        self.player = Player(hands, store)
        hands.subscribe(self.recorder.on_message)
        hands.subscribe(self.player.on_message)

    @property
    def active(self) -> bool:
        return self.recorder.recording or self.player.active

    def status(self) -> dict[str, Any]:
        return {"record": self.recorder.status(), "replay": self.player.status()}
