"""DART Python demos: a headless scene-registry runner.

See ``python/examples/demos/README.md`` for the scene contract and CLI shape;
see ``docs/plans/103-examples-strategy.md`` for the strategy.
"""

from .registry import make_demo_scenes
from .runner import PythonDemoScene, SceneSetup, run

__all__ = ["PythonDemoScene", "SceneSetup", "make_demo_scenes", "run"]
