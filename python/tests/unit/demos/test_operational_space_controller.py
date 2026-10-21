"""Task-space control, input validation and Python owner lifetime."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "python"))
from examples.demos.models.operational_space_control import OperationalSpaceController


def make_inputs(
    robot_class=dart.dynamics.Skeleton, frame_class=dart.dynamics.SimpleFrame
):
    robot = robot_class("arm")
    parent = None
    for axis in np.eye(3)[::-1]:
        joint, parent = robot.createRevoluteJointAndBodyNodePair(parent)
        joint.setAxis(axis)
        tf = dart.math.Isometry3()
        tf.set_translation([0.2, 0, 0.1])
        joint.setTransformFromParentBodyNode(tf)
        joint.setDampingCoefficient(0, 0.5)
    robot.setPositions([0.1, 0.2, -0.3])
    tf = parent.getWorldTransform()
    tf.pretranslate([0.1, 0.05, 0.05])
    target = frame_class(dart.dynamics.Frame.World(), "target", tf)
    return robot, target


def test_controller_properties_and_forces():
    robot, target = make_inputs()
    controller = OperationalSpaceController(robot, target)
    assert controller.robot is robot and controller.target is target
    np.testing.assert_array_equal(controller.offset, [0.05, 0, 0])
    np.testing.assert_array_equal(controller.kp, 50 * np.eye(3))
    np.testing.assert_array_equal(controller.kd, 5 * np.eye(3))
    controller.kp = np.zeros((3, 3))
    np.testing.assert_array_equal(
        controller.compute_forces(), robot.getCoriolisAndGravityForces()
    )
    controller.update()
    np.testing.assert_array_equal(robot.getForces(), controller.compute_forces())
    controller.kp = 50 * np.eye(3)
    first = controller.compute_forces()
    tf = target.getWorldTransform()
    tf.pretranslate([0.03, -0.04, 0.02])
    target.setTransform(tf)
    assert not np.allclose(controller.compute_forces(), first)
    controller.offset = [0, 0, 0.1]
    controller.kd = 3 * np.eye(3)
    np.testing.assert_array_equal(controller.offset, [0, 0, 0.1])
    np.testing.assert_array_equal(controller.kd, 3 * np.eye(3))
    second = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "second")
    controller.target = second
    assert controller.target is second
    assert np.isfinite(controller.compute_forces()).all()
    copy = controller.kp
    copy[:] = 0
    np.testing.assert_array_equal(controller.kp, 50 * np.eye(3))


@pytest.mark.parametrize("which", ["robot", "target"])
def test_controller_rejects_invalid_owners(which):
    robot, target = make_inputs()
    args = {"robot": robot, "target": target}
    for invalid in (None, object()):
        args[which] = invalid
        with pytest.raises((TypeError, ValueError)):
            OperationalSpaceController(**args)
    with pytest.raises(ValueError):
        OperationalSpaceController(dart.dynamics.Skeleton(), target)
    fixed = dart.dynamics.Skeleton()
    fixed.createWeldJointAndBodyNodePair()
    with pytest.raises(
        ValueError, match="robot must have bodies and degrees of freedom"
    ):
        OperationalSpaceController(fixed, target)


@pytest.mark.parametrize(
    "name,invalid",
    [
        ("target", None),
        ("target", object()),
        ("kp", np.ones((2, 2))),
        ("kp", np.full((3, 3), np.nan)),
        ("kd", np.ones((2, 2))),
        ("kd", np.full((3, 3), np.inf)),
        ("offset", [0, 0]),
        ("offset", [0, np.nan, 0]),
    ],
)
def test_controller_rejects_invalid_settings_without_mutation(name, invalid):
    controller = OperationalSpaceController(*make_inputs())
    previous = getattr(controller, name)
    with pytest.raises((TypeError, ValueError)):
        setattr(controller, name, invalid)
    if name == "target":
        assert controller.target is previous
    else:
        np.testing.assert_array_equal(getattr(controller, name), previous)
    assert np.isfinite(controller.compute_forces()).all()


@pytest.mark.parametrize("name", ["offset", "kp", "kd"])
def test_controller_settings_own_their_storage(name):
    controller = OperationalSpaceController(*make_inputs())
    value = getattr(controller, name)
    setattr(controller, name, value)
    saved = value.copy()
    value[:] = 0
    np.testing.assert_array_equal(getattr(controller, name), saved)
    returned = getattr(controller, name)
    returned[:] = 0
    np.testing.assert_array_equal(getattr(controller, name), saved)


def test_controller_retains_subclasses_and_collects_owner_cycles():
    class Robot(dart.dynamics.Skeleton):
        pass

    class Target(dart.dynamics.SimpleFrame):
        pass

    robot, target = make_inputs(Robot, Target)
    controller = OperationalSpaceController(robot, target)
    robot_ref, target_ref = weakref.ref(robot), weakref.ref(target)
    robot.owner = controller
    target.owner = controller
    del robot, target
    gc.collect()
    assert robot_ref() is controller.robot and target_ref() is controller.target
    assert np.isfinite(controller.compute_forces()).all()
    del controller
    gc.collect()
    assert robot_ref() is None and target_ref() is None


def test_controller_rejects_removed_end_effector():
    robot, target = make_inputs()
    controller = OperationalSpaceController(robot, target)
    detached = robot.getBodyNode(robot.getNumBodyNodes() - 1).remove()
    del detached
    gc.collect()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.compute_forces()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.update()


def test_controller_rejects_changed_dof_count():
    robot, target = make_inputs()
    controller = OperationalSpaceController(robot, target)
    robot.createRevoluteJointAndBodyNodePair()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.compute_forces()


def test_controller_rejects_branched_robot():
    robot, target = make_inputs()
    robot.createRevoluteJointAndBodyNodePair()
    with pytest.raises(ValueError, match="end effector must depend on all robot DOFs"):
        OperationalSpaceController(robot, target)


@pytest.mark.parametrize("configured", [False, True])
def test_controller_forces_match_native_baseline(configured):
    robot, target = make_inputs()
    controller = OperationalSpaceController(robot, target)
    if configured:
        robot.setVelocities([0.2, -0.3, 0.15])
        controller.offset = [0.1, -0.07, 0.04]
        controller.kp = [[40, 2, 3], [-1, 30, 0], [2, 1, 20]]
        controller.kd = [[3, 0.2, 0], [0, 4, -0.1], [0.3, 0, 2]]
        expected = [-3.923245539885375, 12.636438314716422, 5.521957714384366]
    else:
        expected = [21.140836648483113, -46.094791563371516, 0]
    # Recorded from the native controller before moving the demo law to Python.
    np.testing.assert_allclose(controller.compute_forces(), expected, rtol=1e-12)


def test_controller_repeats_controlled_steps_deterministically():
    states = []
    for _ in range(2):
        robot, target = make_inputs()
        world = dart.simulation.World()
        world.addSkeleton(robot)
        world.addSimpleFrame(target)
        controller = OperationalSpaceController(robot, target)
        for _ in range(40):
            controller.update()
            world.step()
            assert np.isfinite(robot.getPositions()).all()
        states.append((robot.getPositions(), robot.getVelocities()))
    np.testing.assert_array_equal(states[0], states[1])
