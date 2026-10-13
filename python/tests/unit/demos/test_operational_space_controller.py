"""The private native task-space controller validates and owns its inputs."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest
from dartpy import _demos


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


def test_native_controller_properties_and_forces():
    robot, target = make_inputs()
    controller = _demos.OperationalSpaceController(robot, target)
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
def test_native_controller_rejects_invalid_owners(which):
    robot, target = make_inputs()
    args = {"robot": robot, "target": target}
    for invalid in (None, object()):
        args[which] = invalid
        with pytest.raises((TypeError, ValueError)):
            _demos.OperationalSpaceController(**args)
    with pytest.raises(ValueError):
        _demos.OperationalSpaceController(dart.dynamics.Skeleton(), target)


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
def test_native_controller_rejects_invalid_settings_without_mutation(name, invalid):
    controller = _demos.OperationalSpaceController(*make_inputs())
    previous = getattr(controller, name)
    with pytest.raises((TypeError, ValueError)):
        setattr(controller, name, invalid)
    if name == "target":
        assert controller.target is previous
    else:
        np.testing.assert_array_equal(getattr(controller, name), previous)
    assert np.isfinite(controller.compute_forces()).all()


def test_native_controller_retains_subclasses_and_collects_owner_cycles():
    class Robot(dart.dynamics.Skeleton):
        pass

    class Target(dart.dynamics.SimpleFrame):
        pass

    robot, target = make_inputs(Robot, Target)
    controller = _demos.OperationalSpaceController(robot, target)
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


def test_native_controller_rejects_removed_end_effector():
    robot, target = make_inputs()
    controller = _demos.OperationalSpaceController(robot, target)
    detached = robot.getBodyNode(robot.getNumBodyNodes() - 1).remove()
    del detached
    gc.collect()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.compute_forces()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.update()


def test_native_controller_rejects_changed_dof_count():
    robot, target = make_inputs()
    controller = _demos.OperationalSpaceController(robot, target)
    robot.createRevoluteJointAndBodyNodePair()
    with pytest.raises(RuntimeError, match="robot structure changed"):
        controller.compute_forces()


def test_native_keyboard_capture_query_without_gui_context():
    assert _demos.keyboard_capture_active() is False
