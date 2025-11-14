import dartpy_nb as dart
import numpy as np
import pytest


def make_skeleton_with_dof():
    skeleton = dart.dynamics.Skeleton()
    skeleton.setName("dof_skeleton")

    joint_props = dart.dynamics.RevoluteJointProperties()
    joint_props.mName = "hinge"
    joint_props.mAxis = np.array([0.0, 0.0, 1.0])
    skeleton.createRevoluteJointAndBodyNodePair(None, joint_props)
    return skeleton


def test_degree_of_freedom_limits_and_states():
    skeleton = make_skeleton_with_dof()
    dof = skeleton.getDof(0)

    assert dof.getIndexInSkeleton() == 0
    assert dof.getIndexInJoint() == 0
    assert dof.getTreeIndex() == 0

    dof.setName("yaw")
    assert dof.getName() == "yaw"
    dof.preserveName(True)
    assert dof.isNamePreserved()

    dof.setPositionLimits(-1.0, 1.0)
    assert dof.getPositionLimits() == (-1.0, 1.0)
    dof.setPosition(0.2)
    assert dof.getPosition() == pytest.approx(0.2)
    dof.resetPosition()
    assert dof.getPosition() == pytest.approx(0.0)

    dof.setVelocityLimits((-2.0, 2.0))
    assert dof.getVelocityLimits() == (-2.0, 2.0)
    dof.setVelocity(0.5)
    assert dof.getVelocity() == pytest.approx(0.5)
    dof.resetVelocity()
    assert dof.getVelocity() == pytest.approx(0.0)

    dof.setAccelerationLimits(-3.0, 3.0)
    assert dof.getAccelerationLimits() == (-3.0, 3.0)
    dof.setAcceleration(1.2)
    assert dof.getAcceleration() == pytest.approx(1.2)
    dof.resetAcceleration()
    assert dof.getAcceleration() == pytest.approx(0.0)

    dof.setForceLimits(-5.0, 5.0)
    assert dof.getForceLimits() == (-5.0, 5.0)
    dof.setForce(4.0)
    assert dof.getForce() == pytest.approx(4.0)
    dof.resetForce()
    assert dof.getForce() == pytest.approx(0.0)

    dof.setSpringStiffness(10.0)
    assert dof.getSpringStiffness() == pytest.approx(10.0)
    dof.setRestPosition(0.1)
    assert dof.getRestPosition() == pytest.approx(0.1)
    dof.setDampingCoefficient(0.3)
    assert dof.getDampingCoefficient() == pytest.approx(0.3)
    dof.setCoulombFriction(0.05)
    assert dof.getCoulombFriction() == pytest.approx(0.05)

    assert dof.getSkeleton().getName() == skeleton.getName()


if __name__ == "__main__":
    pytest.main()
