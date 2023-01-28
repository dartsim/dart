import math

import dartpy as dart
import numpy as np
import pytest


def read_world(uri: dart.common.Uri):
    options = dart.io.SdfParser.Options()
    options.mDefaultRootJointType = dart.io.SdfParser.RootJointType.FIXED
    world = dart.io.SdfParser.readWorld(uri, options)
    for i in range(world.getNumSkeletons()):
        skel = world.getSkeleton(i)
        for j in range(skel.getNumJoints()):
            joint = skel.getJoint(j)
            joint.setLimitEnforcement(True)
    return world


def test_static():
    # Load world
    world = read_world("dart://sample/sdf/test/force_torque_test.world")
    assert world is not None
    assert world.getNumSkeletons() == 1

    # Check if the world is correctly loaded
    model_1 = world.getSkeleton(0)
    assert model_1 is not None
    assert model_1.getName() == "model_1"
    assert model_1.getNumBodyNodes() == 2
    assert model_1.getNumJoints() == 2

    world.setGravity([0, 0, -50])

    # Simulate 1 step
    world.step()
    t = world.getTime()

    # Get time step size
    dt = world.getTimeStep()
    assert dt > 0
    assert t == pytest.approx(dt)

    # Get joint and get force torque
    link_1 = model_1.getBodyNode("link_1")
    assert link_1 is not None
    link_2 = model_1.getBodyNode("link_2")
    assert link_2 is not None
    joint_01 = model_1.getJoint("joint_01")
    assert joint_01 is not None
    joint_12 = model_1.getJoint("joint_12")
    assert joint_12 is not None

    tf = dart.math.Isometry3()

    # Run 10 steps
    for _ in range(10):
        world.step()

        # ----------------------
        # Test joint_01 wrench
        # ----------------------

        # Reference adjustment for the difference of the joint frame conventions
        # between Gazebo and DART
        tf.set_identity()
        tf.set_translation(joint_01.getTransformFromParentBodyNode().translation())
        parent_frame01 = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "parent_frame01", tf
        )
        tf.set_identity()
        tf.set_translation(joint_01.getTransformFromChildBodyNode().translation())
        child_frame01 = dart.dynamics.SimpleFrame(link_1, "child_frame01", tf)

        parent_f01 = joint_01.getWrenchToParentBodyNode(parent_frame01)
        assert (parent_f01 == [0, 0, 0, 0, 0, 1000]).all()

        child_f01 = joint_01.getWrenchToChildBodyNode(child_frame01)
        assert (child_f01 == -parent_f01).all()

        # ----------------------
        # Test joint_12 wrench
        # ----------------------

        # Reference adjustment for the difference of the joint frame conventions
        # between Gazebo and DART
        tf.set_identity()
        tf.set_translation(joint_12.getTransformFromParentBodyNode().translation())
        parent_frame12 = dart.dynamics.SimpleFrame(link_1, "parent_frame12", tf)
        tf.set_identity()
        tf.set_translation(joint_12.getTransformFromChildBodyNode().translation())
        child_frame12 = dart.dynamics.SimpleFrame(link_2, "child_frame12", tf)

        parent_f12 = joint_12.getWrenchToParentBodyNode(parent_frame12)
        assert (parent_f12 == [0, 0, 0, 0, 0, 500]).all()

        child_f12 = joint_12.getWrenchToChildBodyNode(child_frame12)
        assert (child_f12 == -parent_f12).all()


def test_force_torque_at_joint_limits():
    # Load world
    world = read_world("dart://sample/sdf/test/force_torque_test.world")
    assert world is not None
    assert world.getNumSkeletons() == 1

    # Check if the world is correctly loaded
    model_1 = world.getSkeleton(0)
    assert model_1 is not None
    assert model_1.getName() == "model_1"
    assert model_1.getNumBodyNodes() == 2
    assert model_1.getNumJoints() == 2

    world.setGravity([0, 0, -50])

    # Simulate 1 step
    world.step()
    t = world.getTime()

    # Get time step size
    dt = world.getTimeStep()
    assert dt > 0
    assert t == pytest.approx(dt)

    # Get joint and get force torque
    link_1 = model_1.getBodyNode("link_1")
    assert link_1 is not None
    link_2 = model_1.getBodyNode("link_2")
    assert link_2 is not None
    joint_01 = model_1.getJoint("joint_01")
    assert joint_01 is not None
    joint_12 = model_1.getJoint("joint_12")
    assert joint_12 is not None

    # Change gravity so that the top link topples over, then remeasure
    world.setGravity([-30, 10, -50])

    # Wait for dynamics to be stabilized
    for i in range(2000):
        world.step()

    tf = dart.math.Isometry3()

    # Run 5 steps
    for _ in range(5):
        world.step()

        # ----------------------
        # Test joint_01 wrench
        # ----------------------

        # Reference adjustment for the difference of the joint frame conventions
        # between Gazebo and DART
        tf.set_identity()
        tf.set_translation(joint_01.getTransformFromParentBodyNode().translation())
        parent_frame01 = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "parent_frame01", tf
        )
        tf.set_identity()
        tf.set_translation(joint_01.getTransformFromChildBodyNode().translation())
        child_frame01 = dart.dynamics.SimpleFrame(link_1, "child_frame01", tf)

        parent_f01 = joint_01.getWrenchToParentBodyNode(parent_frame01)
        assert np.isclose(
            parent_f01, [750, 0, -450, 600, -200, 1000], rtol=0.1, atol=4.5
        ).all()

        child_f01 = joint_01.getWrenchToChildBodyNode(child_frame01)
        assert np.isclose(
            child_f01, [-750, -450, 0, -600, 1000, 200], rtol=0.1, atol=4.5
        ).all()

        # ----------------------
        # Test joint_12 wrench
        # ----------------------

        # Reference adjustment for the difference of the joint frame conventions
        # between Gazebo and DART
        tf.set_identity()
        tf.set_translation(joint_12.getTransformFromParentBodyNode().translation())
        parent_frame12 = dart.dynamics.SimpleFrame(link_1, "parent_frame12", tf)
        tf.set_identity()
        tf.set_translation(joint_12.getTransformFromChildBodyNode().translation())
        child_frame12 = dart.dynamics.SimpleFrame(link_2, "child_frame12", tf)

        parent_f12 = joint_12.getWrenchToParentBodyNode(parent_frame12)
        assert np.isclose(
            parent_f12, [250, 150, 0, 300, -500, -100], rtol=0.1, atol=0.1
        ).all()

        child_f12 = joint_12.getWrenchToChildBodyNode(child_frame12)
        assert np.isclose(
            child_f12, [-250, -150, 0, -300, 500, 100], rtol=0.1, atol=0.1
        ).all()


def test_force_torque_at_joint_limits_with_external_forces():
    # Load world
    world = read_world("dart://sample/sdf/test/force_torque_test2.world")
    assert world is not None
    assert world.getNumSkeletons() == 1

    # Check if the world is correctly loaded
    model_1 = world.getSkeleton(0)
    assert model_1 is not None
    assert model_1.getName() == "boxes"
    assert model_1.getNumBodyNodes() == 3
    assert model_1.getNumJoints() == 3
    assert model_1.getNumDofs() == 2

    # The first joint is fixed joint
    assert model_1.getRootJoint().getType() == dart.dynamics.WeldJoint.getStaticType()

    world.setGravity([0, 0, -50])

    # Simulate 1 step
    world.step()
    t = world.getTime()

    # Get time step size
    dt = world.getTimeStep()
    assert dt > 0
    assert t == pytest.approx(dt)

    # Get joint and get force torque
    link_1 = model_1.getBodyNode("link1")
    assert link_1 is not None
    link_2 = model_1.getBodyNode("link2")
    assert link_2 is not None
    link_3 = model_1.getBodyNode("link3")
    assert link_3 is not None
    joint_12 = model_1.getJoint("joint1")
    assert joint_12 is not None
    joint_23 = model_1.getJoint("joint2")
    assert joint_23 is not None

    tf = dart.math.Isometry3()

    # Run 45005 steps
    kp1 = 5e4
    kp2 = 1e4
    target1 = 0
    target2 = -0.25 * math.pi
    steps = 4500
    for _ in range(steps):
        # PD control
        j1_state = joint_12.getPosition(0)
        j2_state = joint_23.getPosition(0)
        p1_error = target1 - j1_state
        p2_error = target2 - j2_state
        effort1 = kp1 * p1_error
        effort2 = kp2 * p2_error
        joint_12.setForce(0, effort1)
        joint_23.setForce(0, effort2)

        world.step()

    assert joint_12.getPosition(0) == pytest.approx(target1, abs=1e-1)
    assert joint_23.getPosition(0) == pytest.approx(target2, abs=1e-1)

    tol = 2

    # ----------------------
    # Test joint_12 wrench
    # ----------------------

    # Reference adjustment for the difference of the joint frame conventions
    # between Gazebo and DART
    tf.set_identity()
    tf.set_translation(joint_12.getTransformFromParentBodyNode().translation())
    parent_frame01 = dart.dynamics.SimpleFrame(link_1, "parent_frame01", tf)
    tf.set_identity()
    tf.set_translation(joint_12.getTransformFromChildBodyNode().translation())
    child_frame01 = dart.dynamics.SimpleFrame(link_2, "child_frame01", tf)

    parent_f01 = joint_12.getWrenchToParentBodyNode(parent_frame01)
    assert np.isclose(parent_f01, [25, -175, 0, 0, 0, 300], rtol=0.01, atol=tol).all()

    child_f01 = joint_12.getWrenchToChildBodyNode(child_frame01)
    assert np.isclose(child_f01, [-25, 175, 0, 0, 0, -300], rtol=0.01, atol=tol).all()

    # ----------------------
    # Test joint_23 wrench
    # ----------------------

    # Reference adjustment for the difference of the joint frame conventions
    # between Gazebo and DART
    tf.set_identity()
    tf.set_translation(joint_23.getTransformFromParentBodyNode().translation())
    parent_frame12 = dart.dynamics.SimpleFrame(link_2, "parent_frame12", tf)
    tf.set_identity()
    tf.set_translation(joint_23.getTransformFromChildBodyNode().translation())
    child_frame12 = dart.dynamics.SimpleFrame(link_3, "child_frame12", tf)

    parent_f12 = joint_23.getWrenchToParentBodyNode(parent_frame12)
    assert np.isclose(parent_f12, [25, 0, 0, 0, 0, 50], rtol=0.01, atol=tol).all()

    child_f12 = joint_23.getWrenchToChildBodyNode(child_frame12)
    assert np.isclose(
        child_f12, [-17.678, 0, 17.679, -35.355, 0, -35.355], rtol=0.01, atol=tol
    ).all()


if __name__ == "__main__":
    pytest.main()
