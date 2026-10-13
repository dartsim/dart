"""Construction and deterministic frame hooks for the final demo batch."""

import sys
from pathlib import Path

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
