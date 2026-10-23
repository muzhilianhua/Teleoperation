"""Planned features, registered so the GUI shows where they will live."""

from __future__ import annotations

from .base import Plugin


class Recording(Plugin):
    id = "recording"
    title = "Recording"
    description = (
        "M3: record full-rate telemetry (exo, target, present at 100 Hz with device timestamps) and camera "
        "frames into a dataset, with host/device clock alignment."
    )


class FilterTuning(Plugin):
    id = "filter-tuning"
    title = "Filter tuning"
    description = (
        "M4: tune the One-Euro filter (mincut, beta) and look-ahead against live exo traces, with before/after plots."
    )


class ParameterTuning(Plugin):
    id = "param-tuning"
    title = "Parameter tuning"
    description = "M4: per-joint gain curves, thumb mapping and grip parameters with guided test motions."


class Camera(Plugin):
    id = "camera"
    title = "Camera"
    description = "M3: camera preview and synchronised capture for demonstration data."


def placeholders() -> list[Plugin]:
    return [Recording(), FilterTuning(), ParameterTuning(), Camera()]
