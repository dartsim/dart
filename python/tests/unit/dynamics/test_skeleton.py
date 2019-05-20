import platform
import pytest
import numpy as np
import dartpy as dart


def test_basic():
    skel = dart.dynamics.Skeleton()
    [joint1, body1] = skel.createFreeJointAndBodyNode()
    assert joint1.getType() == 'FreeJoint'


if __name__ == "__main__":
    pytest.main()
