"""The native app host runs Python demo scenes without a graphics device."""

import importlib.util
import runpy
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "python"))

viewer = getattr(dart, "viewer", None)
HUMANOID_IDS = {"atlas_v5", "unitree_g1"}


requires_app_host = pytest.mark.skipif(
    viewer is None, reason="a vanilla system ImGui compiles out the demo app host"
)


def _demo_scenes():
    from examples.demos.registry import make_demo_scenes

    return make_demo_scenes()


CATALOG_IDS = set(
    "empty hello_world simple_frames boxes rigid_cubes rigid_chain "
    "add_delete_skels simulation_event_handler rigid_shapes sleeping "
    "hardcoded_design rigid_loop box_stacking dynamic_joint_constraints "
    "tinkertoy joint_constraints hybrid_dynamics "
    "biped_stand operational_space_control contact_inverse_dynamics "
    "ssik_ik_gui wam_analytical_ik atlas_puppet atlas_simbicon mixed_chain "
    "adaptive_soft_contact soft_worm soft_foot_simbicon soft_bodies "
    "soft_cubes soft_open_chain atlas_v5 unitree_g1 vehicle heightmap "
    "point_cloud drag_and_drop".split()
)


@requires_app_host
def test_demo_catalog_does_not_require_a_renderer(capfd):
    scenes = _demo_scenes()
    ids = [scene.id for scene in scenes]
    assert ids and len(ids) == len(set(ids))
    expected = CATALOG_IDS.copy()
    if not hasattr(dart.utils, "ModelResourceRetriever"):
        expected.difference_update(HUMANOID_IDS)
    if not hasattr(dart.dynamics, "VoxelGridShape"):
        expected.remove("point_cloud")
    assert set(ids) == expected
    assert all(isinstance(scene, viewer.Scene) for scene in scenes)
    if hasattr(dart.utils, "ModelResourceRetriever"):
        assert HUMANOID_IDS <= set(ids)
    else:
        assert HUMANOID_IDS.isdisjoint(ids)
    assert viewer.run(scenes, ["examples.demos", "--list-scenes"]) == 0
    listed = capfd.readouterr().out
    for scene in scenes:
        assert f"{scene.id}  -  {scene.title}: {scene.summary}" in listed


@requires_app_host
def test_demo_catalog_cycles_without_a_renderer(capfd):
    assert (
        viewer.run(_demo_scenes(), ["examples.demos", "--cycle-scenes", "--frames", "2"])
        == 0
    )
    assert "OK" in capfd.readouterr().out


@requires_app_host
@pytest.mark.parametrize("raises", [False, True], ids=["empty-path", "cache-error"])
def test_demo_catalog_skips_missing_humanoid_assets(raises, monkeypatch, capfd):
    class MissingModels:
        def __init__(self, cache, offline):
            assert cache == "" and offline

        def getFilePath(self, uri):
            if raises:
                raise RuntimeError("Verified model cache is unavailable.")
            return ""

    monkeypatch.setattr(dart.utils, "ModelResourceRetriever", MissingModels, raising=False)
    scenes = _demo_scenes()
    assert HUMANOID_IDS <= {scene.id for scene in scenes}
    for scene in scenes:
        if scene.id in HUMANOID_IDS:
            with pytest.raises(viewer.AssetsNotPrefetchedError, match="Prefetch with"):
                scene.factory()
    assert (
        viewer.run(scenes, ["examples.demos", "--cycle-scenes", "--frames", "2"]) == 0
    )
    output = capfd.readouterr().out
    for scene_id in HUMANOID_IDS:
        assert f"demo '{scene_id}' skipped:" in output
    assert "OK" in output


@requires_app_host
@pytest.mark.parametrize(
    ("scene_id", "title"),
    [("ssik_ik_gui", "ssik Analytical IK"), ("wam_analytical_ik", "WAM Analytical IK")],
)
def test_ssik_catalog_does_not_require_a_renderer_or_optional_solver(
    monkeypatch, capfd, scene_id, title
):
    monkeypatch.setitem(sys.modules, "ssik", None)
    scene = next(scene for scene in _demo_scenes() if scene.id == scene_id)
    assert viewer.run([scene], ["examples.demos", "--list-scenes"]) == 0
    assert title in capfd.readouterr().out
    with pytest.raises(viewer.AssetsNotPrefetchedError, match="ssik"):
        scene.factory()
    assert viewer.run([scene], ["examples.demos", "--cycle-scenes", "--frames", "2"]) == 0
    assert f"demo '{scene_id}' skipped:" in capfd.readouterr().out


@requires_app_host
def test_demo_catalog_failing_python_hook_fails_the_cycle(capfd):
    def fail():
        raise RuntimeError("demo hook failed")

    scene = viewer.Scene(
        "broken_python_hook",
        title="Broken Python hook",
        category="Tests",
        factory=lambda: viewer.SceneSetup(dart.simulation.World(), pre_step=fail),
    )
    assert (
        viewer.run(
            [*_demo_scenes(), scene],
            ["examples.demos", "--cycle-scenes", "--frames", "2"],
        )
        != 0
    )
    assert "RuntimeError: demo hook failed" in capfd.readouterr().err


def _world_state(world):
    values = []
    for index in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(index)
        values.extend((skeleton.getPositions().copy(), skeleton.getVelocities().copy()))
    for index in range(world.getNumSimpleFrames()):
        transform = world.getSimpleFrame(index).getWorldTransform()
        values.extend((transform.translation().copy(), transform.rotation().ravel().copy()))
    return np.concatenate(values) if values else np.array([])


@requires_app_host
@pytest.mark.parametrize(
    "scene",
    [
        scene
        for scene in _demo_scenes()
        if scene.id not in HUMANOID_IDS
    ]
    if viewer is not None
    else [],
    ids=lambda scene: scene.id,
)
def test_neutral_demo_callbacks_and_deterministic_steps(scene):
    runs = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        try:
            setup = scene.factory()
        except viewer.AssetsNotPrefetchedError as error:
            # Optional scenes, such as ssik_ik_gui without ssik, skip as in cycles.
            pytest.skip(str(error))
        assert isinstance(setup, viewer.SceneSetup)
        world = setup.world
        assert isinstance(world, dart.simulation.World)
        states = [_world_state(world)]
        for _ in range(3):
            # cycleScenes invokes these in this order, including for scenes
            # whose allow_simulation flag disables only interactive simulation.
            if setup.pre_refresh is not None:
                setup.pre_refresh()
            if setup.pre_step is not None:
                setup.pre_step()
            world.step()
            if setup.post_step is not None:
                setup.post_step()
            states.append(_world_state(world))
        assert world.getSimFrames() == 3
        runs.append(np.array(states))
    assert np.isfinite(runs[0]).all()
    np.testing.assert_array_equal(runs[0], runs[1])


@requires_app_host
@pytest.mark.parametrize(
    "factory, builder", [("atlas_world", "build_atlas"), ("g1_world", "build_g1")]
)
def test_modern_humanoid_callbacks_without_external_model_cache(
    factory, builder, monkeypatch
):
    from examples.demos.scenes import modern_humanoids

    world = dart.simulation.World()
    robot = dart.dynamics.Skeleton("humanoid")
    robot.createRevoluteJointAndBodyNodePair()
    world.addSkeleton(robot)
    monkeypatch.setattr(modern_humanoids, "_world", lambda *_: world)
    assert getattr(modern_humanoids, factory)() is world
    assert isinstance(getattr(modern_humanoids, factory)(), dart.simulation.World)
    setup = getattr(modern_humanoids, builder)()
    assert isinstance(setup, viewer.SceneSetup)
    assert setup.world is world
    assert setup.allow_simulation is False
    assert not setup.key_actions
    assert isinstance(setup.panel, viewer.Control)
    setup.pre_refresh()
    assert robot.getPosition(0) == 0.0


def test_demo_entrypoint_rebuild_hint(monkeypatch, capsys):
    monkeypatch.delattr(dart, "viewer", raising=False)
    monkeypatch.setattr(sys, "argv", ["examples.demos", "--list-scenes"])
    with pytest.raises(SystemExit) as error:
        runpy.run_module("examples.demos", run_name="__main__")
    assert error.value.code == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert len(output.err.splitlines()) == 1
    assert "demo app host" in output.err
    assert "bundled docking ImGui" in output.err
    assert "-DDART_USE_SYSTEM_IMGUI=OFF" in output.err
    assert "pixi run config" in output.err


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


def test_humanoid_world_factories_report_missing_models_without_the_host(monkeypatch):
    from examples.demos.scenes import modern_humanoids

    class MissingModels:
        def __init__(self, cache, offline):
            pass

        def getFilePath(self, uri):
            return ""

    monkeypatch.setattr(dart.utils, "ModelResourceRetriever", MissingModels, raising=False)
    monkeypatch.delattr(dart, "viewer", raising=False)
    with pytest.raises(modern_humanoids.MissingModelsError, match="Prefetch with"):
        modern_humanoids.atlas_world()


@requires_app_host
def test_atlas_puppet_zeroes_gravity_like_the_cpp_scene():
    from examples.demos.scenes import atlas_puppet

    # Step, cycles and captures force physics steps on this kinematic scene.
    assert not atlas_puppet.build().world.getGravity().any()


@requires_app_host
def test_biped_controller_uses_the_current_timestep():
    from examples.demos.scenes import biped_stand

    changed = biped_stand.build()
    changed.world.setTimeStep(0.01)
    changed.pre_step()
    fresh = biped_stand.build().world
    fresh.setTimeStep(0.01)
    skeleton = fresh.getSkeleton("fullbody1")
    biped_stand.BipedStandController(fresh, skeleton).pre_step()
    np.testing.assert_allclose(
        changed.world.getSkeleton("fullbody1").getForces(), skeleton.getForces()
    )
