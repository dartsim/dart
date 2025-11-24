from __future__ import annotations

import importlib
import inspect
import pkgutil
from typing import Callable, Dict, Iterable, List, Sequence, Type

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

  for scene_cls in _discover_scene_classes():
    registry.register(scene_cls.metadata, scene_cls)

  return registry


def _discover_scene_classes() -> Sequence[Type[Scene]]:
  """Import scene modules under demo_hub.scenes and return Scene subclasses."""
  import demo_hub.scenes as scenes_pkg

  scene_classes: list[Type[Scene]] = []
  for module_info in pkgutil.walk_packages(scenes_pkg.__path__, scenes_pkg.__name__ + "."):
    if module_info.ispkg:
      continue
    if not module_info.name.endswith(".scene"):
      continue
    module = importlib.import_module(module_info.name)
    for _, obj in inspect.getmembers(module, inspect.isclass):
      if issubclass(obj, Scene) and obj is not Scene and hasattr(obj, "metadata"):
        scene_classes.append(obj)
  if not scene_classes:
    raise RuntimeError("No scenes discovered under demo_hub.scenes")
  return scene_classes
