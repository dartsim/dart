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


@pytest.mark.parametrize(
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
    ],
    # ssik switches the swivel basis between the first pair; the held swivel
    # would flip joints near the singular pose of the second; the third goal
    # lies on the elbow limit; the next two goals' in-limit arcs are narrower
    # than the coarse swivel samples; near an aligned wrist, the held swivel
    # turns the wrist, and the nearest solution lies in a narrow basin between
    # the coarse samples, also between two joint limits.
    ids=[
        "basis-switch",
        "near-singular",
        "on-limit",
        "narrow-arc",
        "narrow-arc-2",
        "wrist-aligned",
        "two-limits",
        "wrist-basin",
    ],
)
def test_wam_ssik_reaches_nearby_targets_without_jumps(start, goal):
    setup, wam, ee, _ = _wam_with_ssik()
    wam.setPositions(goal)
    desired = ee.getWorldTransform()
    target = _activate_at(setup, wam, start)
    target.setTransform(desired)
    setup.pre_refresh()
    assert np.abs(wam.getPositions() - start).max() < 0.01
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=1e-6
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


def test_wam_ssik_never_runs_the_slow_rescue_while_dragging(monkeypatch):
    setup, wam, ee, _ = _wam_with_ssik()
    rescue = pytest.importorskip("ssik.refinement.rescue")

    def fail(*args, **kwargs):
        raise AssertionError("interactive solves must not run ssik's rescue")

    monkeypatch.setattr(rescue, "rescue_via_T_perturbation", fail)
    target = _activate_at(setup, wam, np.zeros(7))
    start = target.getTransform().matrix()
    # Straight up from the stretched rest pose is out of reach but inside
    # ssik's reach sphere, where its rescue would run for every new target.
    for k in range(1, 6):
        transform = dart.math.Isometry3()
        transform.set_rotation(start[:3, :3])
        transform.set_translation(start[:3, 3] + [0.0, 0.0, 0.002 * k])
        target.setTransform(transform)
        setup.pre_refresh()


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
