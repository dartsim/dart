import dartpy_nb as dart
import numpy as np
import pytest


def test_axis_and_pitch_configuration():
    skeleton = dart.dynamics.Skeleton()
    joint, _ = skeleton.createScrewJointAndBodyNodePair()

    axis = np.array([0.0, 0.0, 1.0])
    joint.setAxis(axis)
    np.testing.assert_allclose(joint.getAxis(), axis)

    joint.setPitch(0.5)
    assert joint.getPitch() == pytest.approx(0.5)
    assert joint.isCyclic(0)

    assert joint.getType() == dart.dynamics.ScrewJoint.getStaticType()


if __name__ == "__main__":
    pytest.main()
