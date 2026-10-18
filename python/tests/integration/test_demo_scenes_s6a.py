"""Construction, frame updates, and controls for the S6a demo batch."""

import faulthandler
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

STRUCTURES = {
    "wam_analytical_ik": [("wam", 8, 7), ("ground", 1, 0)],
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
        < ids.index("wam_analytical_ik")
        < ids.index("atlas_puppet")
    )
    for scene_id, title, summary in (
        (
            "wam_analytical_ik",
            "WAM Analytical IK",
            "WAM arm posed by ssik closed-form analytical IK (optional ssik "
            "package), fully draggable.",
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
    if scene_id == "wam_analytical_ik":
        pytest.importorskip("ssik")
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
            [0, 0, 0] if scene_id == "wam_analytical_ik" else [0, 0, -9.81],
        )
        frames = 0 if scene_id == "wam_analytical_ik" else 4
        assert world.getNumSimpleFrames() == frames
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
    pytest.importorskip("ssik")
    setup = _scene("wam_analytical_ik").factory()
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
    assert ik.getGradientMethod().getMethodName() == "ssik SRS"
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=1e-8
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


def _wam_with_ssik():
    ssik = pytest.importorskip("ssik")
    from examples.demos.scenes import wam_analytical_ik

    setup = _scene("wam_analytical_ik").factory()
    wam = setup.world.getSkeleton("wam")
    ee = wam.getEndEffector("ee")
    chain = wam_analytical_ik.ssik_chain(ssik, wam, ee)
    return setup, wam, ee, chain


def _random_in_limits(wam, rng):
    return rng.uniform(wam.getPositionLowerLimits(), wam.getPositionUpperLimits())


def _rotation_error(a, b):
    relative = a[:3, :3].T @ b[:3, :3]
    skew = relative - relative.T
    sin = np.linalg.norm([skew[2, 1], skew[0, 2], skew[1, 0]]) / 2.0
    return float(np.arctan2(sin, (np.trace(relative) - 1.0) / 2.0))


def test_wam_ssik_chain_matches_dart_fk():
    _, wam, ee, chain = _wam_with_ssik()
    rng = np.random.default_rng(7)
    for _ in range(100):
        q = _random_in_limits(wam, rng)
        wam.setPositions(q)
        np.testing.assert_allclose(
            chain.fk(q), ee.getWorldTransform().matrix(), rtol=0, atol=1e-12
        )


def test_wam_ssik_solutions_reach_target_within_limits():
    _, wam, ee, _ = _wam_with_ssik()
    analytical = ee.getIK().getAnalytical()
    lower, upper = wam.getPositionLowerLimits(), wam.getPositionUpperLimits()
    rng = np.random.default_rng(11)
    found = 0
    for _ in range(100):
        wam.setPositions(_random_in_limits(wam, rng))
        target = ee.getWorldTransform()
        wam.setPositions(_random_in_limits(wam, rng))
        valid = [
            solution.mConfig
            for solution in analytical.getSolutions(target)
            if solution.mValidity == dart.dynamics.InverseKinematicsAnalytical.VALID
        ]
        found += bool(valid)
        for q in valid:
            assert np.all((lower <= q) & (q <= upper))
            wam.setPositions(q)
            reached = ee.getWorldTransform().matrix()
            assert np.abs(reached[:3, 3] - target.translation()).max() <= 1e-8
            assert _rotation_error(reached, target.matrix()) <= 1e-8
    # A random in-limit target is always reachable; the fallback keeps it so.
    assert found == 100


def _activate_at(setup, wam, q):
    wam.setPositions(q)
    actions = {chr(action.key): action for action in setup.key_actions}
    actions["1"].callback()
    return wam.getEndEffector("ee").getIK().getTarget()


def test_wam_ssik_tracks_a_dragged_target_without_jumps():
    setup, wam, ee, _ = _wam_with_ssik()
    target = _activate_at(setup, wam, [0.3, 0.6, 0.2, 1.5, -0.3, 0.5, 0.1])
    start = target.getTransform().matrix()
    previous = wam.getPositions().copy()
    max_step = 0.0
    for k in range(1, 201):
        angle = 2.0 * np.pi * k / 200
        # A 3 cm circle with a +/-11 degree roll: ~1 mm and <0.4 degree per step.
        transform = dart.math.Isometry3()
        transform.set_rotation(
            dart.math.expMapRot([0.2 * np.sin(angle), 0.0, 0.0]) @ start[:3, :3]
        )
        transform.set_translation(
            start[:3, 3] + 0.03 * np.array([np.cos(angle) - 1.0, np.sin(angle), 0.0])
        )
        target.setTransform(transform)
        setup.pre_refresh()
        np.testing.assert_allclose(
            ee.getWorldTransform().matrix(), transform.matrix(), atol=1e-8
        )
        max_step = max(max_step, np.abs(wam.getPositions() - previous).max())
        previous = wam.getPositions().copy()
    assert max_step < 0.05


def test_wam_ssik_keeps_a_dragged_elbow_pose():
    setup, wam, ee, chain = _wam_with_ssik()
    target = _activate_at(setup, wam, [0.3, 0.6, 0.2, 1.5, -0.3, 0.5, 0.1])
    # Swivels are measured in the target's basis, as the solver holds them.
    motion = chain.self_motion(target.getTransform().matrix())
    held = motion.param_of(wam.getPositions())
    wam.getDof("/j3").setPosition(0.6)
    moved = motion.param_of(wam.getPositions())
    assert abs(moved - held) > 0.05
    setup.pre_refresh()
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), target.getTransform().matrix(), atol=1e-8
    )
    assert motion.param_of(wam.getPositions()) == pytest.approx(moved, abs=1e-9)


_NARROW = np.array([-2.6, -2.0, 2.8, 3.1, -4.76, -1.6, 3.0])
_INWARD = np.array([1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0])


_NEARBY_TARGETS = pytest.mark.parametrize(
    ("start", "goal"),
    [
        (
            [0.3, -0.036474599, 0.2, 0.5, -0.3, 0.5, 0.1],
            [0.3, -0.036274599, 0.2, 0.5, -0.3, 0.5, 0.1],
        ),
        (
            [0.3, 1e-5, 0.2, 1.5, -0.3, 1e-5, 0.1],
            [0.3, 1.1e-4, 0.2, 1.5, -0.3, 1e-5, 0.1],
        ),
        ([0.3, 0.6, 0.2, 3.0999, -0.3, 0.5, 0.1], [0.3, 0.6, 0.2, 3.1, -0.3, 0.5, 0.1]),
        (_NARROW + 1e-4 * _INWARD, _NARROW + 1e-6 * _INWARD),
        (
            [-1.499, -1.99999, 2.79999, 2.6, -1.1, 1.59999, 2.2],
            [-1.5, -1.99999, 2.79999, 2.6, -1.1, 1.59999, 2.2],
        ),
        (
            [0.3, 0.6, 0.2, 1.5, -0.3, 1e-4, 0.1],
            [0.3001, 0.6, 0.2, 1.5, -0.3, 1e-4, 0.1],
        ),
        (
            [2.5999, -0.2, 0.8, 1.85, -2.7, -1.5993, 1.89],
            [2.5999, -0.2001, 0.8, 1.85, -2.7, -1.5993, 1.89],
        ),
        ([0.0, 1.8, 0.0, 1.9, 0.3, -1e-6, 1.9], [1e-4, 1.8, 0.0, 1.9, 0.3, -1e-6, 1.9]),
        (
            [2.5, 1.15, 2.799999, 1.78, 1.239999, 0.67, 1.93],
            [2.5, 1.1499, 2.799999, 1.78, 1.239999, 0.67, 1.93],
        ),
        (
            [0.3, 1.9999, 0.2, 1.5, 1.2399, 0.5, 0.1],
            [0.3, 2.0, 0.2, 1.5, 1.24, 0.5, 0.1],
        ),
        (
            [0.0, 0.0, 0.0, 0.2315263, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.2305263, 0.0, 0.0, 0.0],
        ),
        (
            [-0.9, 0.0, 2.4, 2.4, 1.1, 1e-4, -2.6],
            [-0.8999, 1e-4, 2.3998, 2.4, 1.0998, 0.0, -2.5999],
        ),
        (
            [-2.6, -1.2, 1.0, 1.5, -0.9, -0.2, -2.999999],
            [-2.6, -1.1999, 1.0, 1.5, -0.9, -0.2, -2.999999],
        ),
        (
            [-2.6, 0.02, -2.8, 1.3, 0.4, 1.6, 2.3],
            [-2.6, 0.0201, -2.8, 1.3, 0.4, 1.6, 2.3],
        ),
        (
            [-2.377, 0.0, 2.8, 0.2315263, -4.76, 1.6, -0.5],
            [-2.377, 0.0, 2.8, 0.2315263, -4.76, 1.6, -0.5001],
        ),
        (
            [2.23, -0.1, -2.79999, 0.2315263, 1.24, 0.0, -2.49],
            [2.229995, -0.099935, -2.8, 0.2315056, 1.24, -0.0000918, -2.490026],
        ),
        (
            [-0.34, 0.04, -2.79996, 1.4, 1.24, 0.115, -2.99999],
            [-0.3399, 0.04, -2.79996, 1.4, 1.24, 0.115, -2.99999],
        ),
        (
            [0.0, -0.004, -2.79, -0.9, 1.24, 1e-4, -1.3],
            [0.007, -0.006, -2.79, -0.9, 1.24, -0.0149, -1.308],
        ),
        (
            [2.0, 0.0, 0.3, 0.2315263, 0.0, 0.0, 2.1],
            [1.999, 0.0014, 0.2998, 0.2318263, 0.0008, 0.0005, 2.1012],
        ),
        (
            [-2.6, 1e-5, 2.8, 0.0, 1e-5, 0.0, 0.0],
            [-2.6, 1e-5, 2.8, 0.00112, 1e-5, 0.0, 0.0],
        ),
        (
            [0.0, 1e-7, -2.8, 0.0, 1.24, 0.0, 0.0],
            [0.0, 1e-7, -2.8, -3.5e-4, 1.24, 0.0, 2.8e-4],
        ),
        (
            [-0.16, 1.5, 2.44, 0.2315263, -4.76, 1.6, -3.0],
            [-0.1601, 1.4999, 2.4401, 0.2314263, -4.76, 1.6, -2.9999],
        ),
        (
            [2.6, -1.49, -2.8, 0.23, 0.74, -1.59, -0.2],
            [2.6, -1.48992, -2.79997, 0.23004, 0.73993, -1.58994, -0.20009],
        ),
        (
            [0.0, -2.0, -2.8, 3.1, 0.0, 1.6, -3.0],
            [0.0, -2.0, -2.8, 3.1, 0.0, 1.5999, -3.0],
        ),
        (
            [-1.7, 0.0, 2.8, 0.2315, -4.76, 0.0, -3.0],
            [-1.69991, -7e-05, 2.79994, 0.23145, -4.76, 3e-05, -3.0],
        ),
        (
            [2.6, 2.0, -2.8, 0.2315, 1.12, -1.13, 3.0],
            [2.59996, 2.0, -2.79992, 0.23146, 1.11992, -1.13008, 3.0],
        ),
        (
            [2.6, 2.0, 0.1982, 0.2315, -4.76, 0.0, -3.0],
            [2.6, 2.0, 0.1935, 0.2289, -4.7531, 0.0004, -2.9978],
        ),
        (
            [-0.7, 2.0, -1.6, 3.1, -1.0, 1.6, 3.0],
            [-0.7, 2.0, -1.5999, 3.1, -1.0, 1.6, 3.0],
        ),
        (
            [0.0, 0.0, 0.0, 0.2315263, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.23052629, 0.0, 0.0, 0.0],
        ),
        (
            [2.23, -0.62, 0.0, 0.2315263, -4.76, 1.6, 0.0],
            [2.2625, -0.5725, 0.045, 0.2070263, -4.73, 1.6, 0.04],
        ),
        (
            [1.6263, -2.0, 0.6187, 0.23052629, 0.9754, 0.0, -2.9648],
            [1.6434, -2.0, 0.611, 0.23052629, 0.9477, 0.0187, -2.9456],
        ),
        (
            [0.926, -2.0, -0.086, 0.23052629, 1.24, -1.6, -1.073],
            [0.929, -1.9978, -0.0878, 0.22592629, 1.24, -1.5967, -1.0767],
        ),
        (
            [0.698, 0.0, 2.8, 0.23152629, -4.76, -1.162, -3.0],
            [0.705, 0.0, 2.8, 0.23152629, -4.76, -1.156, -2.999],
        ),
        (
            [-2.6, 0.0, -2.8, 0.2315263, 1.24, -1.6, 3.0],
            [-2.5956, -0.0031, -2.8, 0.2284, 1.2355, -1.6, 2.9982],
        ),
        (
            [0.388, 2.0, 1.0, 0.23152629, -0.393, 0.0, 3.0],
            [0.3644, 1.9945, 0.9522, 0.2473, -0.3966, -0.0328, 3.0],
        ),
        (
            [-1.8, 0.0, -2.8, 0.23052629, 1.24, -1.6, 1.0],
            [-1.82, 0.0, -2.8, 0.23052629, 1.24, -1.6, 1.0],
        ),
    ],
    # ssik switches the swivel basis between the first pair; the held swivel
    # would flip joints near the singular pose of the second; the third goal
    # lies on the elbow limit; the next two goals' in-limit arcs are narrower
    # than the coarse swivel samples; near an aligned wrist, the held swivel
    # turns the wrist, and the nearest solution lies in a narrow basin between
    # the coarse samples, also between two joint limits; Newton overshoots a
    # limit next to the last two-limit goals, and converges only linearly at
    # full extension and with aligned shoulder and wrist axes; for the last two
    # goals, Newton steps a few 1e-7 rad past a limit that another goal joint
    # sits on; the remaining starts sit on several limits or singularities at
    # once, where Newton crosses limits, converges only linearly, or stalls
    # under too much damping, and where a rank-deficient constrained solve
    # once raised, an active set went stale or kept a bound it should
    # release, or the last usable iterate was dropped; ssik finds no chart at
    # full extension for the next goal, and Newton alone lands 6 rad away from
    # the one after; from the last two starts at full extension on several
    # limits, weak damping or the elbow fold sent continuation to the wrong
    # side of the limits; continuation later wandered out of reach after a
    # usable iterate, and stalled at a stationary point of active limits; the
    # last two recover from an overshoot, or reduce the least-squares residual
    # while the largest pose error briefly grows, so a stall guard must judge
    # the residual and allow a recovery.
    ids=[
        "basis-switch",
        "near-singular",
        "on-limit",
        "narrow-arc",
        "narrow-arc-2",
        "wrist-aligned",
        "two-limits",
        "wrist-basin",
        "two-limits-2",
        "two-limits-3",
        "full-extension",
        "aligned-axes",
        "limit-crossing",
        "limit-crossing-2",
        "full-extension-on-limits",
        "full-extension-on-limits-2",
        "limit-release",
        "aligned-wrist-on-limits",
        "full-extension-aligned",
        "unconstrained-budget",
        "constrained-damping",
        "constrained-damping-2",
        "rank-deficient",
        "rank-deficient-2",
        "stale-active-set",
        "bound-release",
        "stale-active-set-2",
        "usable-iterate",
        "chartless-extension",
        "distant-root",
        "aligned-wrist-damping",
        "elbow-fold",
        "wandering-continuation",
        "stalled-fold",
        "overshoot-recovery",
        "residual-progress",
    ],
)


@_NEARBY_TARGETS
def test_wam_ssik_reaches_nearby_targets_without_jumps(start, goal):
    setup, wam, ee, _ = _wam_with_ssik()
    wam.setPositions(goal)
    desired = ee.getWorldTransform()
    target = _activate_at(setup, wam, start)
    target.setTransform(desired)
    setup.pre_refresh()
    reach = max(0.01, 2.0 * np.abs(np.subtract(goal, start)).max())
    assert np.abs(wam.getPositions() - start).max() < reach
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=1e-6
    )


@_NEARBY_TARGETS
def test_wam_ssik_solver_offers_nearby_solutions_for_whole_moves(start, goal):
    from examples.demos.scenes import wam_analytical_ik

    # DART clamps each IK iteration's error, so the scene sees intermediate
    # targets; the solver must also reach the whole move in one call.
    _, wam, _, chain = _wam_with_ssik()
    desired = chain.fk(np.asarray(goal, dtype=float))
    wam.setPositions(start)
    found = wam_analytical_ik.swivel_solver(chain, wam)(desired)
    reach = max(0.01, 2.0 * np.abs(np.subtract(goal, start)).max())
    assert any(
        np.abs(c - start).max() < reach
        and np.abs(chain.fk(c) - desired).max() < 1e-6
        for c in found
    )


def test_wam_ssik_rejects_inaccurate_charts_at_a_wrist_singularity():
    setup, wam, ee, _ = _wam_with_ssik()
    start = np.array([-0.77, 0.54, 0.82, -0.77, -1.77, 0.0001, 0.93])
    wam.setPositions(start - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0])
    desired = ee.getWorldTransform()
    target = _activate_at(setup, wam, start)
    target.setTransform(desired)
    for _ in range(3):
        setup.pre_refresh()
    # Near the aligned wrist, ssik's charts close FK to only about 0.05 rad.
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=1e-6
    )
    assert np.abs(wam.getPositions() - start).max() < 0.5


def test_wam_ssik_keeps_a_continuation_reached_on_its_last_step():
    from examples.demos.scenes import wam_analytical_ik

    _, wam, _, chain = _wam_with_ssik()
    # At full extension ssik finds no chart for this target, and the short
    # boundary continuation first reaches it on its last permitted step.
    start = np.array([-2.422, 0.0, -1.274, 0.23052629, -1.817, 0.0, 2.551])
    goal = np.array([-2.441, -0.047, -1.322, 0.23052629, -1.838, 0.044, 2.507])
    desired = chain.fk(start + 0.99 * (goal - start))
    wam.setPositions(start)
    found = wam_analytical_ik.swivel_solver(chain, wam)(desired)
    assert len(found) == 1
    np.testing.assert_allclose(chain.fk(found[0]), desired, atol=1e-6)
    assert np.abs(found[0] - start).max() <= wam_analytical_ik.TRACK_REACH


def test_wam_ssik_keeps_infeasible_drags_cheap(monkeypatch):
    from examples.demos.scenes import wam_analytical_ik

    setup, wam, ee, _ = _wam_with_ssik()
    rescue = pytest.importorskip("ssik.refinement.rescue")
    refinement = pytest.importorskip("ssik.refinement")
    budgets = []
    lm_refine = refinement.lm_refine

    def fail(*args, **kwargs):
        raise AssertionError("interactive solves must not run ssik's rescue")

    def record(*args, **kwargs):
        budgets.append(kwargs["max_iters"])
        return lm_refine(*args, **kwargs)

    monkeypatch.setattr(rescue, "rescue_via_T_perturbation", fail)
    monkeypatch.setattr(refinement, "lm_refine", record)

    def drag(start, offsets):
        target = _activate_at(setup, wam, start)
        pose = target.getTransform().matrix()
        for offset in offsets:
            transform = dart.math.Isometry3()
            transform.set_rotation(pose[:3, :3])
            transform.set_translation(pose[:3, 3] + offset)
            target.setTransform(transform)
            setup.pre_refresh()
        actions = {chr(action.key): action for action in setup.key_actions}
        actions["1"].callback()

    # From 6 mm straight up, the stretched rest pose's target is out of reach
    # but inside ssik's reach sphere, where its rescue would run for every new
    # target; only a short boundary continuation may run.
    drag(np.zeros(7), [[0.0, 0.0, 0.002 * k] for k in range(5, 10)])
    assert budgets and max(budgets) <= wam_analytical_ik.BOUNDARY_ITERATIONS
    # Straightening the elbow past its 3.1 rad limit needs no search at all.
    budgets.clear()
    wam.setPositions([0.3, 0.6, 0.2, 3.1001, -0.3, 0.5, 0.1])
    beyond = ee.getWorldTransform().translation()
    wam.setPositions([0.3, 0.6, 0.2, 3.1, -0.3, 0.5, 0.1])
    step = beyond - ee.getWorldTransform().translation()
    drag([0.3, 0.6, 0.2, 3.1, -0.3, 0.5, 0.1], [step * k for k in range(1, 4)])
    assert budgets == []
    # Pulling the shoulder past its limit stalls the bounded continuation,
    # which must stop instead of spending its whole budget on every frame.
    optimize = pytest.importorskip("scipy.optimize")
    steps = []
    lsq_linear = optimize.lsq_linear

    def count(*args, **kwargs):
        steps[-1] += 1
        return lsq_linear(*args, **kwargs)

    monkeypatch.setattr(optimize, "lsq_linear", count)
    # Pushing past an elbow limit on the near branch leaves only far elbows,
    # which continuation cannot reach; a near-repeat elbow retry is skipped.
    shoulder = np.array([0.4, -2.0, 2.8, 3.1, 1.09, -0.22, -3.0])
    for start, joint, offset in (
        (shoulder, 1, -0.001),
        (shoulder, 1, -0.003),
        (np.array([0.0, 0.0, 0.0, -0.9, 0.0, 0.0, 0.0]), 3, -1e-4),
        (np.array([2.6, -2.0, -2.8, -0.9, -0.3, 0.0, -3.0]), 1, -0.001),
        # Past the wrist limit, bounded steps cycle instead of stalling.
        (np.array([-1.092, 1.163, -2.604, 2.857, -0.06, -1.6, 0.94]), 5, -0.005),
    ):
        moved = start.copy()
        moved[joint] += offset
        wam.setPositions(moved)
        desired = ee.getWorldTransform()
        target = _activate_at(setup, wam, start)
        target.setTransform(desired)
        steps.append(0)
        setup.pre_refresh()
        actions = {chr(action.key): action for action in setup.key_actions}
        actions["1"].callback()
    assert max(steps) < 60


def test_wam_ssik_skips_continuation_the_held_swivel_already_beats(monkeypatch):
    setup, wam, ee, _ = _wam_with_ssik()
    refinement = pytest.importorskip("ssik.refinement")

    def fail(*args, **kwargs):
        raise AssertionError("continuation cannot win; skip it")

    monkeypatch.setattr(refinement, "lm_refine", fail)
    start = np.array([0.3, 0.6, 0.2, 0.2315263, -0.3, 0.5, 0.1])
    wam.setPositions(start + [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0])
    desired = ee.getWorldTransform()
    target = _activate_at(setup, wam, start)
    target.setTransform(desired)
    setup.pre_refresh()
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=1e-6
    )


def test_wam_ssik_reuses_a_failed_solve_while_dart_iterates():
    from examples.demos.scenes import wam_analytical_ik

    setup, wam, ee, chain = _wam_with_ssik()
    calls = []

    class CountingChain:
        def __getattr__(self, name):
            return getattr(chain, name)

        def self_motion(self, desired):
            calls.append(desired)
            return chain.self_motion(desired)

    ee.getIK().setPythonAnalytical(
        wam_analytical_ik.swivel_solver(CountingChain(), wam), methodName="ssik SRS"
    )
    # Only elbow angles past the 3.1 rad limit reach this target.
    wam.setPositions([0.3, 0.6, 0.2, 3.3, -0.3, 0.5, 0.1])
    desired = ee.getWorldTransform()
    target = _activate_at(setup, wam, [0.3, 0.6, 0.2, 3.0, -0.3, 0.5, 0.1])
    target.setTransform(desired)
    setup.pre_refresh()
    assert len(calls) == 1


@pytest.mark.parametrize(
    ("move", "turn"),
    [
        ([0, 1, 0], [0, 0, 0]),
        ([0, 0, -1], [0, 0, 0]),
        ([0, 0, 0], [1, 0, 0]),
        ([0, 0, 0], [0, 0, 1]),
    ],
    ids=["sideways", "down", "roll", "yaw"],
)
def test_wam_ssik_leaves_the_singular_rest_pose_without_flips(move, turn):
    setup, wam, ee, _ = _wam_with_ssik()
    # At rest the shoulder-wrist line is the base axis, so the swivel is undefined.
    target = _activate_at(setup, wam, np.zeros(7))
    previous = wam.getPositions().copy()
    max_step = 0.0
    # ssik's native limit arcs once stalled here; fail loudly instead of hanging.
    faulthandler.dump_traceback_later(120, exit=True)
    try:
        for _ in range(20):
            current = target.getTransform().matrix()
            transform = dart.math.Isometry3()
            transform.set_rotation(
                dart.math.expMapRot(0.005 * np.asarray(turn, float)) @ current[:3, :3]
            )
            transform.set_translation(current[:3, 3] + 0.001 * np.asarray(move, float))
            target.setTransform(transform)
            setup.pre_refresh()
            np.testing.assert_allclose(
                ee.getWorldTransform().matrix(), transform.matrix(), atol=1e-8
            )
            max_step = max(max_step, np.abs(wam.getPositions() - previous).max())
            previous = wam.getPositions().copy()
    finally:
        faulthandler.cancel_dump_traceback_later()
    assert max_step < 1.0


@pytest.mark.parametrize("ssik", [None, object()], ids=["missing", "too old"])
def test_wam_scene_skips_without_ssik(monkeypatch, ssik):
    monkeypatch.setitem(sys.modules, "ssik", ssik)
    with pytest.raises(viewer.AssetsNotPrefetchedError, match="ssik>=8.1.0"):
        _scene("wam_analytical_ik").factory()
