"""Construction and deterministic frame hooks for the final demo batch."""

import gc
import sys
from pathlib import Path
from types import SimpleNamespace

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")


class _Context:
    def __init__(self):
        self.attachments = []
        self.handlers = []
        self.drag = []

    def add_attachment(self, attachment):
        self.attachments.append(attachment)

    def add_event_handler(self, handler):
        self.handlers.append(handler)

    def enable_drag_and_drop(self, *args, **kwargs):
        self.drag.append((args, kwargs))

    def log(self, message):
        pass


def _scenes():
    from examples.demos.registry import make_demo_scenes

    return make_demo_scenes()


def test_s6b_catalog_order_and_metadata():
    scenes = _scenes()
    ids = [scene.id for scene in scenes]
    assert ids.index("contact_inverse_dynamics") + 1 == ids.index("ssik_ik_gui")
    assert ids.index("ssik_ik_gui") + 1 == ids.index("wam_ikfast")
    assert ids.index("wam_ikfast") + 1 == ids.index("atlas_puppet")
    for scene_id, title, summary in (
        (
            "atlas_puppet",
            "Atlas Puppet",
            "Atlas whole-body IK puppet with balance and support overlays.",
        ),
        (
            "ssik_ik_gui",
            "ssik Analytical IK",
            "Analytical IK against 19 prebuilt robot arms (optional ssik package).",
        ),
    ):
        scene = next(scene for scene in scenes if scene.id == scene_id)
        assert (scene.title, scene.category, scene.summary) == (
            title,
            "Control & IK",
            summary,
        )


@pytest.mark.parametrize("scene_id", ["atlas_puppet", "ssik_ik_gui"])
def test_s6b_world_structure_and_deterministic_steps(scene_id):
    if scene_id == "ssik_ik_gui":
        pytest.importorskip("ssik")
    scene = next(scene for scene in _scenes() if scene.id == scene_id)
    states = []
    for _ in range(2):
        setup = scene.factory()
        context = _Context()
        setup.on_activate(context)
        world = setup.world
        skeletons = [world.getSkeleton(i) for i in range(world.getNumSkeletons())]
        expected = (
            [("drc_skeleton", 34, 33), ("ground", 1, 0)]
            if scene_id == "atlas_puppet"
            else [(f"ssik_arm_{i}", 7, 6) for i in range(16)]
        )
        assert [
            (s.getName(), s.getNumBodyNodes(), s.getNumDofs()) for s in skeletons
        ] == expected
        assert world.getConstraintSolver().getNumConstraints() == 0
        assert all(s.getNumSoftBodyNodes() == 0 for s in skeletons)
        assert world.getNumSimpleFrames() == (0 if scene_id == "atlas_puppet" else 2)
        assert world.getTimeStep() == pytest.approx(0.001)
        np.testing.assert_array_equal(world.getGravity(), [0, 0, 0])
        assert setup.shadows and not setup.allow_simulation
        for _ in range(4):
            if setup.pre_refresh:
                setup.pre_refresh()
            if setup.pre_step:
                setup.pre_step()
            world.step()
            if setup.post_step:
                setup.post_step()
        assert world.getSimFrames() == 4
        states.append(
            np.concatenate(
                [
                    value
                    for skeleton in skeletons
                    for value in (skeleton.getPositions(), skeleton.getVelocities())
                ]
            ).copy()
        )
    assert np.isfinite(states[0]).all()
    np.testing.assert_array_equal(*states)


@pytest.mark.parametrize("failure", ["empty", "solve", "fk", "missing_fk", "load"])
def test_ssik_solved_marker_requires_current_fk_pose(monkeypatch, failure):
    from examples.demos.scenes import ssik_ik_gui

    pose = np.eye(4)
    pose[:3, 3] = [0.3, -0.2, 0.5]
    module = SimpleNamespace(
        DOF=1,
        T_HOME=np.eye(4),
        solve=lambda target, **kwargs: [SimpleNamespace(q=np.array([0.4]))],
        fk=lambda q: pose,
    )
    monkeypatch.setattr(ssik_ik_gui.importlib, "import_module", lambda name: module)
    setup = ssik_ik_gui.build()
    state = setup.pre_refresh.__self__
    marker = setup.world.getSimpleFrame("ssik_fk_solution")
    visual = marker.getVisualAspect()
    solve = next(
        child
        for child in gc.get_referents(setup.panel)
        if isinstance(child, viewer.Control) and child.label == "Solve"
    )
    assert visual.isHidden()
    setup.on_activate(_Context())
    assert not visual.isHidden()
    np.testing.assert_array_equal(marker.getTransform().matrix(), pose)

    def fail(*args, **kwargs):
        raise RuntimeError("unavailable")

    with monkeypatch.context() as failing:
        if failure == "empty":
            failing.setattr(module, "solve", lambda target, **kwargs: [])
        elif failure in ("solve", "fk"):
            failing.setattr(module, failure, fail)
        elif failure == "missing_fk":
            failing.delattr(module, "fk")
        else:
            failing.setattr(ssik_ik_gui.importlib, "import_module", fail)
        if failure == "load":
            state.arm_choice.on_change()
        else:
            solve.on_change()
        assert visual.isHidden()
        if failure != "missing_fk":
            assert state.solutions == [] and state.fk_poses == []

    pose[:3, 3] = [-0.1, 0.4, 0.6]
    state.arm_choice.on_change()
    assert not visual.isHidden()
    np.testing.assert_array_equal(marker.getTransform().matrix(), pose)
