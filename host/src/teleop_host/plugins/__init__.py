"""Plugin registry (see base.Plugin)."""

from .base import Plugin
from .placeholders import placeholders


def builtin_plugins() -> list[Plugin]:
    return placeholders()


__all__ = ["Plugin", "builtin_plugins"]
