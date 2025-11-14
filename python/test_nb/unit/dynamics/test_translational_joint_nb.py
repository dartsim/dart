import dartpy_nb as dart
import numpy as np
import pytest


def test_translational_joint_properties():
    skeleton = dart.dynamics.Skeleton()
    joint, _ = skeleton.createTranslationalJointAndBodyNodePair()

    assert joint.getType() == dart.dynamics.TranslationalJoint.getStaticType()
    assert joint.getNumDofs() == 3

    # Translational joints are cyclic for each axis
    for i in range(3):
        assert joint.isCyclic(i)

    positions = np.array([0.1, -0.2, 0.3])
    joint.setPositions(positions)
    np.testing.assert_allclose(joint.getPositions(), positions)

    jacobian = joint.getRelativeJacobianStatic(positions)
    assert jacobian.shape == (6, 3)


if __name__ == "__main__":
    pytest.main()
