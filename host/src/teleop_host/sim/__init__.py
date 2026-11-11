"""Simulated device for tests and `teleop-host --sim`."""

from .device import FakeDevice
from .model import HandModel

__all__ = ["FakeDevice", "HandModel"]
