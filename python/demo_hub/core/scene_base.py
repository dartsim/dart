from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class SceneMetadata:
    """Lightweight metadata for browsing scenes."""

    scene_id: str
    label: str
    summary: str
    tags: tuple[str, ...] = ()


class Scene(ABC):
    """Abstract base scene used by the demo hub."""

    metadata: SceneMetadata

    def __init__(self) -> None:
        self._last_dt = 0.0

    @abstractmethod
    def setup(self, dt: float) -> None:
        """Allocate resources and reset the scene."""

    @abstractmethod
    def update(self, dt: float) -> None:
        """Advance simulation by one step."""

    def reset(self) -> None:
        """Reset using the last timestep."""
        if self._last_dt <= 0:
            raise RuntimeError("reset() called before setup()")
        self.setup(self._last_dt)

    def export_state(self) -> dict | None:
        """Return a serializable state blob (optional)."""
        return None

    def import_state(self, state: dict) -> None:
        """Restore state from export_state (optional)."""

    def debug_draw_2d(self) -> list[tuple[tuple[float, float], tuple[float, float], tuple[float, float, float]]]:
        """Return line segments ((x1, z1), (x2, z2), (r, g, b)) for 2D top-down debug view."""
        return []

    def get_world(self):
        """Return the underlying dartpy.World if available (optional)."""
        return None

    def _set_last_dt(self, dt: float) -> None:
        self._last_dt = dt
