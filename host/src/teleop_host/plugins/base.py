"""Extension point for features beyond the console commands (design doc M3/M4).

A plugin gets every message both hands send at full rate, tagged with the
hand (on_message runs on the event loop: keep it cheap, hand heavy work to a
thread), may add REST routes under /api/plugins/<id>/, and is listed in the
GUI's navigation, which loads the page with the same id
(web/src/plugins/registry.ts).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from fastapi import APIRouter

if TYPE_CHECKING:
    from ..hands import HandManager


class Plugin:
    id: str = ""
    title: str = ""
    description: str = ""
    status: str = "planned"  # planned | active

    def router(self) -> APIRouter | None:
        return None

    async def start(self, hands: HandManager) -> None:
        self.hands = hands  # hands.device("right" | "left") -> Device or None

    async def stop(self) -> None:
        pass

    def on_message(self, hand: str | None, message: Any) -> None:
        pass

    def describe(self) -> dict[str, str]:
        return {"id": self.id, "title": self.title, "description": self.description, "status": self.status}
