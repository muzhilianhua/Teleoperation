"""Extension point for features beyond the console commands (design doc M3/M4).

A plugin gets every message the device sends at full rate (on_message runs on
the event loop: keep it cheap, hand heavy work to a thread), may add REST
routes under /api/plugins/<id>/, and is listed in the GUI's navigation, which
loads the page with the same id (web/src/plugins/registry.ts).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from fastapi import APIRouter

if TYPE_CHECKING:
    from ..device import Device


class Plugin:
    id: str = ""
    title: str = ""
    description: str = ""
    status: str = "planned"  # planned | active

    def router(self) -> APIRouter | None:
        return None

    async def start(self, device: Device) -> None:
        self.device = device

    async def stop(self) -> None:
        pass

    def on_message(self, message: Any) -> None:
        pass

    def describe(self) -> dict[str, str]:
        return {"id": self.id, "title": self.title, "description": self.description, "status": self.status}
