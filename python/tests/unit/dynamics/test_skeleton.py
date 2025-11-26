import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    skel = dart.Skeleton()

    joint_prop = dart.FreeJointProperties()
    joint_prop.m_name = "joint0"
    assert joint_prop.m_name == "joint0"

    [joint1, body1] = skel.create_free_joint_and_body_node_pair(None, joint_prop)
    assert joint1.get_type() == "FreeJoint"
    assert joint1.get_name() == "joint0"

    assert skel.get_body_node(0).get_name() == body1.get_name()


if __name__ == "__main__":
    pytest.main()
