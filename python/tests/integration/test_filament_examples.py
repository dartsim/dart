"""The neutral viewer uses the same demo and tutorial scene/controller code."""

import gc
import importlib.util
import sys
import weakref
from pathlib import Path

import dartpy as dart
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "python"))

from examples.demos.registry import make_demo_scenes
from examples.demos.runner import (
    Navigator,
    _install_scene,
    _run_headless_steps,
    _uninstall_scene,
    main,
)


def test_demo_catalog_does_not_require_a_renderer(capsys):
    assert main(["--list"]) == 0
    assert "atlas_puppet" in capsys.readouterr().out
    ids = {scene.id for scene in make_demo_scenes()}
    assert len(ids) == (11 if hasattr(dart.utils, "ModelResourceRetriever") else 9)
    if hasattr(dart.utils, "ModelResourceRetriever"):
        assert {"atlas_v5", "unitree_g1"} <= ids


def test_ssik_catalog_does_not_require_a_renderer_or_optional_solver(capsys):
    path = ROOT / "python" / "examples" / "ssik_analytical_ik" / "main.py"
    spec = importlib.util.spec_from_file_location("ssik_example", path)
    example = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(example)
    assert example.main(["--list"]) == 0
    assert "franka_panda_ik" in capsys.readouterr().out


def test_demo_navigator_does_not_keep_viewer_alive():
    class Handler(Navigator, dart.gui.GUIEventHandler):
        pass

    viewer = dart.gui.Viewer()
    navigator = Handler(viewer, [], dart.gui)
    viewer.addEventHandler(navigator)
    reference = weakref.ref(viewer)
    del viewer
    gc.collect()
    assert reference() is None
    with pytest.raises(RuntimeError, match="destroyed"):
        navigator.viewer


@pytest.mark.parametrize(
    "scene",
    [
        scene
        for scene in make_demo_scenes()
        if scene.id not in {"atlas_v5", "unitree_g1"}
    ],
    ids=lambda scene: scene.id,
)
def test_neutral_demo_callbacks_and_deterministic_steps(scene):
    handle = scene.build()
    world = handle.node.getWorld()
    assert isinstance(handle.node, dart.gui.WorldNode)
    _run_headless_steps(handle, world, 3)
    assert world.getSimFrames() == (3 if handle.allow_simulation else 0)
    viewer = dart.gui.Viewer()
    state = _install_scene(viewer, handle, dart.gui)
    assert viewer.getWorldNode(world) is handle.node
    _uninstall_scene(viewer, handle, state)
    assert viewer.getNumWorldNodes() == 0


@pytest.mark.parametrize("builder", ["build_atlas", "build_g1"])
def test_modern_humanoid_callbacks_without_external_model_cache(builder, monkeypatch):
    from examples.demos.scenes import modern_humanoids

    world = dart.simulation.World()
    robot = dart.dynamics.Skeleton("humanoid")
    robot.createRevoluteJointAndBodyNodePair()
    world.addSkeleton(robot)
    monkeypatch.setattr(modern_humanoids, "_world", lambda *_: world)
    handle = getattr(modern_humanoids, builder)()
    assert isinstance(handle.node, dart.gui.WorldNode)
    assert handle.node.getWorld() is world
    assert handle.allow_simulation is False
    for key in "[]=t":
        handle.key_actions[ord(key)][1]()
    assert robot.getPosition(0) == 0.0


@pytest.mark.parametrize("name", ("biped", "collisions", "dominoes", "multi_pendulum"))
@pytest.mark.parametrize("filename", ("main.py", "main_finished.py"))
def test_neutral_tutorial_callbacks(name, filename):
    path = ROOT / "python" / "tutorials" / name / filename
    spec = importlib.util.spec_from_file_location(f"neutral_{name}_{filename}", path)
    tutorial = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tutorial)
    if filename == "main.py" and name in {"collisions", "dominoes"}:
        with pytest.raises(NotImplementedError, match="Lesson"):
            tutorial.build_scene()
        return
    node = tutorial.build_scene()
    assert isinstance(node, dart.gui.RealTimeWorldNode)
    assert isinstance(node.handler, dart.gui.GUIEventHandler)
    world = node.getWorld()
    node.refresh()
    assert world.getSimFrames() == 0
    if filename == "main_finished.py":
        node.customPreRefresh()
        node.customPreStep()
        world.step()
        node.customPostStep()
        node.customPostRefresh()
        assert world.getSimFrames() == 1
