from __future__ import annotations

from typing import Callable, Dict, Iterable, List

from .scene_base import Scene, SceneMetadata


class SceneRegistry:
    """Registers scenes and creates instances on demand."""

    def __init__(self) -> None:
        self._entries: Dict[str, tuple[Callable[[], Scene], SceneMetadata]] = {}

    def register(self, metadata: SceneMetadata, factory: Callable[[], Scene]) -> None:
        if metadata.scene_id in self._entries:
            raise ValueError(f"Scene id already registered: {metadata.scene_id}")
        self._entries[metadata.scene_id] = (factory, metadata)

    @property
    def scene_ids(self) -> List[str]:
        return list(self._entries.keys())

    @property
    def scenes(self) -> Iterable[SceneMetadata]:
        return (entry[1] for entry in self._entries.values())

    def create(self, scene_id: str) -> Scene:
        try:
            factory, metadata = self._entries[scene_id]
        except KeyError as exc:
            raise KeyError(f"Unknown scene id: {scene_id}") from exc
        scene = factory()
        scene.metadata = metadata  # type: ignore[attr-defined]
        return scene


def build_default_registry() -> SceneRegistry:
    """Register built-in scenes shipped with the repository."""
    registry = SceneRegistry()

    # Import locally to avoid import cycles during module import.
    from demo_hub.scenes.hello_world.scene import HelloWorldScene

    registry.register(HelloWorldScene.metadata, HelloWorldScene)
    return registry

