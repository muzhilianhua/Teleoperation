"""Device clock -> host clock, per board.

Each board stamps its telemetry with micros() (32 bit, wraps every ~71 min).
Arrival times on the host carry USB and scheduling delay, which is never
negative, so the lower envelope of (arrival - device time) follows the true
offset; a line through the per-second minima also absorbs crystal drift:
t_host = a * t_device + b.
"""

from __future__ import annotations

from dataclasses import dataclass, field


class Unwrapper:
    """32-bit microsecond counter -> continuous seconds."""

    def __init__(self) -> None:
        self._last: int | None = None
        self._wraps = 0

    def seconds(self, t_us: int) -> float:
        if self._last is not None and t_us < self._last and self._last - t_us > 2**31:
            self._wraps += 1
        self._last = t_us
        return (self._wraps * 2**32 + t_us) / 1e6


@dataclass(slots=True)
class ClockFit:
    bin_s: float = 1.0
    _points: dict[int, tuple[float, float]] = field(default_factory=dict)  # bin -> (device s, host - device)

    def add(self, device_s: float, host_s: float) -> None:
        key = int(device_s // self.bin_s)
        delay = host_s - device_s
        best = self._points.get(key)
        if best is None or delay < best[1]:
            self._points[key] = (device_s, delay)

    def fit(self) -> tuple[float, float]:
        """(a, b) with host = a * device + b; identity slope until two bins exist."""
        points = list(self._points.values())
        if not points:
            return 1.0, 0.0
        if len(points) == 1:
            d, off = points[0]
            return 1.0, off
        n = len(points)
        xs = [p[0] for p in points]
        ys = [p[0] + p[1] for p in points]  # host time at the envelope
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx == 0:
            return 1.0, my - mx
        a = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True)) / sxx
        return a, my - a * mx
