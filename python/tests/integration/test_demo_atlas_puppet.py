"""Atlas puppet native objective, scene controls, and held-key integration."""

import gc
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")


def _build():
    from examples.demos.scenes.atlas_puppet import build

    return build()


class _Context:
    def __init__(self):
        self.drag = []
        self.attachments = []
        self.handlers = []
        self.logs = []

    def enable_drag_and_drop(self, *args, **kwargs):
        self.drag.append((args, kwargs))

    def add_attachment(self, attachment):
        self.attachments.append(attachment)

    def add_event_handler(self, handler):
        self.handlers.append(handler)

    def log(self, message):
        self.logs.append(message)


def _event(key, down=True):
    event = dart.gui.GUIEventAdapter
    return event(event.KEYDOWN if down else event.KEYUP, ord(key))


def test_atlas_end_effectors_support_solver_and_activation():
    setup = _build()
    atlas = setup.world.getSkeleton(0)
    assert atlas.getNumEndEffectors() == 4
    assert [atlas.getEndEffector(i).getName() for i in range(4)] == [
        "l_hand",
        "r_hand",
        "l_foot",
        "r_foot",
    ]
    for name, translation in (
        ("l_hand", [0.0009, 0.1254, 0.012]),
        ("r_hand", [-0.0009, -0.1254, 0.012]),
        ("l_foot", [0.186, 0.0, -0.08]),
        ("r_foot", [0.186, 0.0, -0.08]),
    ):
        ee = atlas.getEndEffector(name)
        np.testing.assert_array_equal(
            ee.getRelativeTransform().translation(), translation
        )
        lower, upper = ee.getIK().getErrorMethod().getBounds()
        if name.endswith("hand"):
            assert not ee.hasSupport()
            assert np.isneginf(lower).all() and np.isposinf(upper).all()
        else:
            assert ee.getIK().getHierarchyLevel() == 1
            assert ee.getSupport().isActive()
            np.testing.assert_array_equal(
                upper, [1e-8, 1e-8, np.inf, np.inf, np.inf, 1e-8]
            )
            np.testing.assert_array_equal(lower, -upper)
            assert len(ee.getSupport().getGeometry()) == 4
            assert ee.getWorldTransform().translation()[2] == pytest.approx(
                0.0, abs=1e-15
            )
    whole_body = atlas.getIK(True)
    assert isinstance(whole_body, dart.dynamics.WholeBodyIK)
    assert isinstance(whole_body.getObjective(), dart._demos.RelaxedPosture)
    assert whole_body.getSolver().getNumMaxIterations() == 10
    assert whole_body.getProblem().getNumEqConstraints() == 2
    state = setup.pre_refresh.__self__
    assert (
        state.balance.getErrorMethod()
        == dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID
    )
    assert (
        state.balance.getBalanceMethod()
        == dart.constraint.BalanceConstraint.BalanceMethod.SHIFT_SUPPORT
    )
    context = _Context()
    setup.on_activate(context)
    assert len(context.drag) == atlas.getNumBodyNodes() + 4
    for body, (args, kwargs) in zip(atlas.getBodyNodes(), context.drag):
        assert args[0] is body
        assert kwargs == {"use_external_ik": False, "use_whole_body": False}
    assert len(context.handlers) == 1
    assert len(context.attachments) == 1
    visual = context.attachments[0]
    assert isinstance(visual, dart.gui.SupportPolygonVisual)
    assert visual.getSkeleton() is atlas
    assert visual.getDisplayElevation() == 0.05
    np.testing.assert_array_equal(
        setup.world.getSkeleton(1)
        .getBodyNode(0)
        .getShapeNode(0)
        .getVisualAspect()
        .getRGBA(),
        [0.1, 0.1, 0.9, 0.2],
    )


def test_atlas_every_key_action_and_panel_text():
    setup = _build()
    atlas = setup.world.getSkeleton(0)
    state = setup.pre_refresh.__self__
    context = _Context()
    setup.on_activate(context)
    actions = {chr(action.key): action for action in setup.key_actions}
    assert {key: action.label for key, action in actions.items()} == {
        "1": "Toggle l_hand target",
        "2": "Toggle r_hand target",
        "3": "Toggle l_foot target",
        "4": "Toggle r_foot target",
        "x": "Toggle l_foot support",
        "c": "Toggle r_foot support",
        "t": "Reset to rest pose",
        "p": "Print DOF values",
    }
    assert (
        state.status.label == "Targets: l_hand off  r_hand off  l_foot off  r_foot off"
    )
    for index, key in enumerate("1234"):
        ee = atlas.getEndEffector(index)
        actions[key].callback()
        assert setup.world.getNumSimpleFrames() == 1
        np.testing.assert_array_equal(
            ee.getIK().getTarget().getTransform().matrix(), ee.getTransform().matrix()
        )
        lower, upper = ee.getIK().getErrorMethod().getBounds()
        assert np.isfinite(lower).all() and np.isfinite(upper).all()
        setup.pre_refresh()
        assert f"{ee.getName()} on" in state.status.label
        actions[key].callback()
        assert setup.world.getNumSimpleFrames() == 0
        for actual, expected in zip(
            ee.getIK().getErrorMethod().getBounds(), state.default_bounds[index]
        ):
            np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(
            ee.getIK().getTarget().getRelativeTransform().matrix(),
            state.default_target_tf[index].matrix(),
        )
    for key, name in (("x", "l_foot"), ("c", "r_foot")):
        support = atlas.getEndEffector(name).getSupport()
        actions[key].callback()
        assert not support.isActive()
        actions[key].callback()
        assert support.isActive()
    atlas.setPosition(0, 0.1)
    atlas.setPosition(3, 0.7)
    actions["t"].callback()
    assert atlas.getPosition(0) == state.rest_config[0]
    assert atlas.getPosition(3) == 0.7
    actions["p"].callback()
    assert context.logs == [
        f"{atlas.getDof(i).getName()}: {atlas.getPosition(i):.6g}"
        for i in range(atlas.getNumDofs())
    ]
    setup.pre_refresh()
    assert (
        state.status.label == "Targets: l_hand off  r_hand off  l_foot off  r_foot off"
    )


@pytest.mark.parametrize("index", range(4))
def test_atlas_dragged_end_effector_target_reaches_pose(index):
    setup = _build()
    atlas = setup.world.getSkeleton(0)
    setup.key_actions[index].callback()
    ee = atlas.getEndEffector(index)
    target = ee.getIK().getTarget()
    desired = target.getTransform()
    desired.pretranslate([0.01, 0.0, 0.0])
    target.setTransform(desired)
    for _ in range(20):
        setup.pre_refresh()
    np.testing.assert_allclose(
        ee.getWorldTransform().matrix(), desired.matrix(), atol=3e-6
    )


@pytest.mark.parametrize("key", list("wasdqefz"))
def test_atlas_held_movement_keys_release_and_apply_motion(key, monkeypatch):
    setup = _build()
    atlas = setup.world.getSkeleton(0)
    state = setup.pre_refresh.__self__
    context = _Context()
    setup.on_activate(context)
    handler = context.handlers[0]
    # Isolate each frame's root delta from the native IK correction.
    monkeypatch.setattr(
        dart.dynamics.WholeBodyIK, "solveAndApply", lambda self, apply: True
    )
    initial = atlas.getBodyNode(0).getWorldTransform().matrix().copy()
    assert handler(_event(key))
    setup.pre_refresh()
    moved = atlas.getBodyNode(0).getWorldTransform().matrix().copy()
    assert state.movement[key]
    assert not np.array_equal(initial, moved)
    if key in "wasdfz":
        displacement = moved[:3, 3] - initial[:3, 3]
        assert np.linalg.norm(displacement) == pytest.approx(
            0.002 if key in "fz" else 0.01
        )
    else:
        angle = np.arccos(
            np.clip((np.trace(moved[:3, :3] @ initial[:3, :3].T) - 1) / 2, -1, 1)
        )
        assert angle == pytest.approx(np.deg2rad(2.0))
    assert handler(_event(key, down=False))
    setup.pre_refresh()
    assert not state.movement[key]
    np.testing.assert_array_equal(
        atlas.getBodyNode(0).getWorldTransform().matrix(), moved
    )
    assert not handler(_event("?"))
    assert not handler(dart.gui.GUIEventAdapter(dart.gui.GUIEventAdapter.MOVE))


def test_atlas_posture_hold_and_solver_failure_diagnostics(monkeypatch):
    setup = _build()
    state = setup.pre_refresh.__self__
    context = _Context()
    setup.on_activate(context)
    handler = context.handlers[0]
    assert handler(_event("r"))
    assert state.posture.enforce_ideal_posture
    assert (
        state.balance.getErrorMethod()
        == dart.constraint.BalanceConstraint.ErrorMethod.OPTIMIZE_BALANCE
    )
    setup.pre_refresh()
    assert handler(_event("r", down=False))
    assert not state.posture.enforce_ideal_posture
    assert (
        state.balance.getErrorMethod()
        == dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID
    )
    monkeypatch.setattr(
        dart.dynamics.WholeBodyIK, "solveAndApply", lambda self, apply: False
    )
    state.consecutive_failures = 999
    setup.pre_refresh()
    setup.pre_refresh()
    assert context.logs == [
        "atlas_puppet: IK solver has failed 1000 consecutive times."
    ]
    monkeypatch.setattr(
        dart.dynamics.WholeBodyIK, "solveAndApply", lambda self, apply: True
    )
    setup.pre_refresh()
    assert state.consecutive_failures == 0
    assert not state.reported_failing


def test_relaxed_posture_bound_values_and_objective_retention():
    posture = dart._demos.RelaxedPosture(
        [0.0, 0.5, -0.25], [-1.0, -1.0, -np.inf], [1.0, 1.0, np.inf], [2.0, 3.0, 4.0]
    )
    assert isinstance(posture, dart.optimizer.Function)
    assert posture.eval([-2.0, 2.0, 0.0]) == 6.5
    np.testing.assert_array_equal(
        posture.eval_gradient([-2.0, 2.0, 0.0]), [-2.0, 3.0, 0.0]
    )
    posture.enforce_ideal_posture = True
    assert posture.eval([-2.0, 2.0, 0.0]) == 18.625
    np.testing.assert_array_equal(
        posture.eval_gradient([-2.0, 2.0, 0.0]), [-4.0, 4.5, 1.0]
    )
    skeleton = dart.dynamics.Skeleton()
    skeleton.createFreeJointAndBodyNodePair()
    objective = dart._demos.RelaxedPosture([0.0] * 6, [-1.0] * 6, [1.0] * 6, [1.0] * 6)
    objective.enforce_ideal_posture = True
    ik = skeleton.getIK(True)
    ik.setObjective(objective)
    del objective
    gc.collect()
    assert isinstance(ik.getObjective(), dart._demos.RelaxedPosture)
    assert ik.getObjective().enforce_ideal_posture
    skeleton.setPosition(0, 0.25)
    assert ik.getObjective().eval(skeleton.getPositions()) == 0.03125
    ik.setObjective(None)
    gc.collect()
    assert ik.getObjective() is None


@pytest.mark.parametrize(
    "ideal,lower,upper,weights",
    [
        ([0.0], [], [1.0], [1.0]),
        ([np.nan], [-1.0], [1.0], [1.0]),
        ([0.0], [np.nan], [1.0], [1.0]),
        ([0.0], [-1.0], [np.nan], [1.0]),
        ([0.0], [2.0], [1.0], [1.0]),
        ([0.0], [-1.0], [1.0], [-1.0]),
        ([0.0], [-1.0], [1.0], [np.inf]),
    ],
)
def test_relaxed_posture_constructor_validation(ideal, lower, upper, weights):
    with pytest.raises(ValueError):
        dart._demos.RelaxedPosture(ideal, lower, upper, weights)


@pytest.mark.parametrize("positions", [[], [0.0, 0.0], [np.nan], [np.inf]])
def test_relaxed_posture_evaluation_validation(positions):
    posture = dart._demos.RelaxedPosture([0.0], [-np.inf], [np.inf], [0.01])
    for evaluate in (posture.eval, posture.eval_gradient):
        with pytest.raises(ValueError):
            evaluate(positions)
