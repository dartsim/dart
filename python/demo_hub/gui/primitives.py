from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Segment2D:
    start: tuple[float, float]
    end: tuple[float, float]
    color: tuple[float, float, float] = (1.0, 1.0, 1.0)

