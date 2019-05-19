import platform
import pytest
import numpy as np
import dartpy as dart


def test_basic():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    for i in range(kr5.getNumBodyNodes()):
        body = kr5.getBodyNode(i)
        assert np.array_equal(np.array(body.getSpatialVelocity()), np.zeros(6)) is True


if __name__ == "__main__":
    pytest.main()
