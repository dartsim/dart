"""Fixed tutorial scene for agent-capture parity across dartpy binders."""

import importlib.util
from pathlib import Path


def dominoes():
    path = (
        Path(__file__).resolve().parents[2]
        / "python/tutorials/dominoes/main_finished.py"
    )
    spec = importlib.util.spec_from_file_location("parity_dominoes", path)
    tutorial = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tutorial)
    node = tutorial.build_scene()
    world = node.world
    assert world.getNumSkeletons() == 3
    return world
