import dartpy_nb as dart
import numpy as np
import pytest


def test_translational_joint_2d_planes():
    skeleton = dart.dynamics.Skeleton()
    joint, _ = skeleton.createTranslationalJoint2DAndBodyNodePair()

    assert joint.getType() == dart.dynamics.TranslationalJoint2D.getStaticType()

    joint.setXYPlane(renameDofs=True)
    joint.setPositions(np.array([0.1, 0.2]))
    np.testing.assert_allclose(joint.getPositions(), np.array([0.1, 0.2]))

    joint.setYZPlane(renameDofs=False)
    joint.setZXPlane(renameDofs=False)

    joint.setArbitraryPlane(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), True)

    assert joint.isCyclic(0)
    assert joint.isCyclic(1)


if __name__ == "__main__":
    pytest.main()
