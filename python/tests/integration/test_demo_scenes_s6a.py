"""Construction, frame updates, and controls for the S6a demo batch."""

import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

STRUCTURES = {
    "wam_ikfast": [("wam", 8, 7), ("ground", 1, 0)],
    "contact_inverse_dynamics": [("ground skeleton", 1, 6), ("fullbody1", 20, 37)],
}


def _scene(scene_id):
    from examples.demos.registry import make_demo_scenes

    return next(scene for scene in make_demo_scenes() if scene.id == scene_id)


def _state(world):
    return np.concatenate(
        [
            value
            for skeleton in _skeletons(world)
            for value in (
                skeleton.getPositions(),
                skeleton.getVelocities(),
                skeleton.getAccelerations(),
            )
        ]
    ).copy()


def _skeletons(world):
    return [world.getSkeleton(i) for i in range(world.getNumSkeletons())]


def _step(setup, count=1):
    for _ in range(count):
        if setup.pre_refresh:
            setup.pre_refresh()
        if setup.pre_step:
            setup.pre_step()
        setup.world.step()
        if setup.post_step:
            setup.post_step()


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


@pytest.fixture
def controls(monkeypatch):
    recorded = []
    for name in ("add_text", "add_slider", "add_checkbox", "add_button", "add_plot"):
        original = getattr(viewer.Control, name)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded.append(control)
            return control

        monkeypatch.setattr(viewer.Control, name, record)
    return recorded


def _control(controls, label):
    return next(control for control in controls if control.label == label)


@pytest.fixture
def contact_models(monkeypatch):
    from examples.demos.scenes import contact_inverse_dynamics

    models = []
    constructor = contact_inverse_dynamics.ContactInverseDynamicsModel

    def create(world):
        model = constructor(world)
        models.append(model)
        return model

    monkeypatch.setattr(contact_inverse_dynamics, "ContactInverseDynamicsModel", create)
    return models


def test_s6a_registry_order_and_metadata():
    from examples.demos.registry import make_demo_scenes

    scenes = make_demo_scenes()
    ids = [scene.id for scene in scenes]
    assert (
        ids.index("operational_space_control")
        < ids.index("contact_inverse_dynamics")
        < ids.index("wam_ikfast")
        < ids.index("atlas_puppet")
    )
    for scene_id, title, summary in (
        (
            "wam_ikfast",
            "WAM IkFast",
            "WAM arm posed by IkFast analytical whole-body IK, fully draggable.",
        ),
        (
            "contact_inverse_dynamics",
            "Contact Inverse Dynamics",
            "Kinematic squat replay solved for contact-consistent joint torques.",
        ),
    ):
        scene = next(scene for scene in scenes if scene.id == scene_id)
        assert (scene.title, scene.category, scene.summary) == (
            title,
            "Control & IK",
            summary,
        )


@pytest.mark.parametrize("scene_id", STRUCTURES)
def test_s6a_construction_and_deterministic_steps(scene_id):
    states = []
    for _ in range(2):
        setup = _scene(scene_id).factory()
        world = setup.world
        assert [
            (s.getName(), s.getNumBodyNodes(), s.getNumDofs())
            for s in _skeletons(world)
        ] == STRUCTURES[scene_id]
        assert world.getConstraintSolver().getNumConstraints() == 0
        assert all(s.getNumSoftBodyNodes() == 0 for s in _skeletons(world))
        assert setup.shadows
        assert setup.allow_simulation == (scene_id == "contact_inverse_dynamics")
        np.testing.assert_array_equal(
            world.getGravity(),
            [0, 0, 0] if scene_id == "wam_ikfast" else [0, 0, -9.81],
        )
        assert world.getNumSimpleFrames() == (0 if scene_id == "wam_ikfast" else 4)
        _step(setup, 4)
        assert world.getSimFrames() == 4
        states.append(_state(world))
    assert np.isfinite(states[0]).all()
    np.testing.assert_array_equal(*states)


class _Context:
    def __init__(self):
        self.drag = []
        self.logs = []

    def enable_drag_and_drop(self, *args, **kwargs):
        self.drag.append((args, kwargs))

    def log(self, message):
        self.logs.append(message)


def test_wam_keys_solver_endpoint_and_activation(controls):
    setup = _scene("wam_ikfast").factory()
    wam = setup.world.getSkeleton("wam")
    ee = wam.getEndEffector("ee")
    ik = ee.getIK()
    assert ee.getName() == "ee"
    assert isinstance(wam.getIK(True), dart.dynamics.WholeBodyIK)
    ground = setup.world.getSkeleton("ground").getBodyNode(0)
    np.testing.assert_array_equal(
        ground.getShapeNode(0).getVisualAspect().getRGBA(), [0.1, 0.1, 0.9, 0.2]
    )
    np.testing.assert_array_equal(
        ee.getRelativeTransform().translation(), [0, 0, -0.09]
    )
    np.testing.assert_array_equal(setup.camera_home.eye, [5.34, 3, 1.91])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 0.5])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 0, 0.98])
    context = _Context()
    setup.on_activate(context)
    assert len(context.drag) == wam.getNumBodyNodes() + 1
    for body, (args, kwargs) in zip(wam.getBodyNodes(), context.drag):
        assert args[0] is body
        assert kwargs == {"use_external_ik": False, "use_whole_body": False}
    assert context.drag[-1][0][0] is ik.getTarget()
    actions = {chr(action.key): action for action in setup.key_actions}
    assert {key: action.label for key, action in actions.items()} == {
        "p": "Print DOF values",
        "t": "Reset to rest pose",
        "1": "Toggle IK target",
    }
    rest = wam.getPositions().copy()
    wam.setPosition(0, 0.1)
    actions["p"].callback()
    assert context.logs == [
        f"{wam.getDof(i).getName()}: {wam.getPosition(i):.6g}"
        for i in range(wam.getNumDofs())
    ]
    actions["t"].callback()
    np.testing.assert_array_equal(wam.getPositions(), rest)
    actions["1"].callback()
    assert setup.world.getNumSimpleFrames() == 1
    setup.pre_refresh()
    assert _control(controls, "IK target: active")
    wam.setPositions([0.2, -0.3, 0.1, 0.8, -0.4, 0.3, -0.2])
    desired = ee.getWorldTransform()
    wam.setPositions(rest)
    ik.getTarget().setTransform(desired)
    for _ in range(5):
        setup.pre_refresh()
    # The source solver targets /wam7 directly, including the scene's 9 cm offset.
    np.testing.assert_allclose(
        wam.getBodyNode("/wam7").getWorldTransform().matrix(),
        desired.matrix(),
        atol=1e-5,
    )
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(),
        desired.multiply(ee.getRelativeTransform()).matrix(),
        atol=1e-5,
    )
    actions["1"].callback()
    assert setup.world.getNumSimpleFrames() == 0
    lower, upper = ik.getErrorMethod().getBounds()
    assert np.isneginf(lower).all() and np.isposinf(upper).all()
    np.testing.assert_array_equal(
        ik.getTarget().getRelativeTransform().matrix(), np.eye(4)
    )
    setup.pre_refresh()
    assert _control(controls, "IK target: inactive")


def test_contact_every_panel_control_and_frame_results(controls, contact_models):
    setup = _scene("contact_inverse_dynamics").factory()
    model = contact_models[0]
    assert not setup.key_actions and setup.on_activate is None
    np.testing.assert_array_equal(setup.camera_home.eye, [3, -3, 1.5])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 0])
    assert model.keyframe_count == 60
    setup.pre_refresh()
    assert model.time == pytest.approx(1 / 60)
    assert len(model.contact_forces) == 4
    assert np.isfinite(model.contact_forces).all()
    assert model.total_vertical_force > 0
    _edit(_control(controls, "Play"), False)
    time = model.time
    setup.pre_refresh()
    assert model.time == time
    np.testing.assert_array_equal(model.biped.getVelocities(), np.zeros(37))
    np.testing.assert_array_equal(model.biped.getAccelerations(), np.zeros(37))
    _edit(_control(controls, "Playback Speed"), 1.5)
    _edit(_control(controls, "Play"), True)
    setup.pre_refresh()
    assert model.time == pytest.approx(time + 1.5 / 60)
    _edit(_control(controls, "Friction Coefficient"), 0.3)
    assert model.friction_coeff == pytest.approx(float(np.float32(0.3)))
    _edit(_control(controls, "Force Arrow Scale"), 0.005)
    assert model.force_scale == pytest.approx(float(np.float32(0.005)))
    _edit(_control(controls, "Squat Depth (m)"), 0.05)
    assert model.regeneration_requested
    prior_pose = model.biped.getPositions().copy()
    setup.pre_refresh()
    assert not model.regeneration_requested
    _edit(_control(controls, "Regenerate Keyframes"))
    assert model.regeneration_requested
    setup.pre_refresh()
    assert model.squat_depth == pytest.approx(float(np.float32(0.05)))
    assert model.keyframe_count == 60
    assert not model.regeneration_requested
    assert np.linalg.norm(model.biped.getPositions() - prior_pose) > 0
    assert model.sample_index == 5
    assert any(c.label.startswith("Residual Norm: ") for c in controls)
    assert any(c.label.startswith("Total Vertical Force: ") for c in controls)
    assert any(c.label in ("FEASIBLE", "INFEASIBLE") for c in controls)
    plots = [c for c in controls if c.kind == viewer.Control.Kind.PLOT]
    assert [c.label for c in plots] == list(model.torque_names) + [
        "Total Vertical GRF (N)"
    ]
    assert all(len(c.samples) == 240 for c in plots)
    for control, samples in zip(plots, list(model.torque_plots) + [model.plot_samples]):
        np.testing.assert_array_equal(control.samples, samples)
    for index, force in enumerate(model.contact_forces):
        frame = setup.world.getSimpleFrame(index)
        arrow = frame.getShape()
        if np.linalg.norm(force) < 1e-8:
            assert frame.getVisualAspect().isHidden()
            continue
        np.testing.assert_allclose(
            arrow.getHead() - arrow.getTail(), model.force_scale * force, atol=1e-14
        )


def test_private_native_binding_validation():
    with pytest.raises((ValueError, TypeError)):
        dart._demos.set_wam_ikfast(None)
    body = dart.dynamics.Skeleton().createWeldJointAndBodyNodePair()[1]
    with pytest.raises(ValueError):
        dart._demos.set_wam_ikfast(body.getIK(True))
