import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    skel = dart.dynamics.Skeleton()

    joint_prop = dart.dynamics.FreeJointProperties()
    joint_prop.mName = "joint0"
    assert joint_prop.mName == "joint0"

    [joint1, body1] = skel.createFreeJointAndBodyNodePair(None, joint_prop)
    assert joint1.getType() == "FreeJoint"
    assert joint1.getName() == "joint0"

    assert skel.getBodyNodes()[0].getName() == body1.getName()


if __name__ == "__main__":
    pytest.main()
