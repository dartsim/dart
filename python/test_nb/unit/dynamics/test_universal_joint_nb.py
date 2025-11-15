import dartpy_nb as dart
import numpy as np
import pytest


def test_universal_joint_axes():
    skeleton = dart.dynamics.Skeleton()
    joint, _ = skeleton.createUniversalJointAndBodyNodePair()

    assert joint.getType() == dart.dynamics.UniversalJoint.getStaticType()
    assert joint.isCyclic(0)
    assert joint.isCyclic(1)

    joint.setAxis1(np.array([1.0, 0.0, 0.0]))
    joint.setAxis2(np.array([0.0, 1.0, 0.0]))

    np.testing.assert_allclose(joint.getAxis1(), np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(joint.getAxis2(), np.array([0.0, 1.0, 0.0]))

    jacobian = joint.getRelativeJacobianStatic(np.array([0.1, 0.2]))
    assert jacobian.shape == (6, 2)


if __name__ == "__main__":
    pytest.main()
