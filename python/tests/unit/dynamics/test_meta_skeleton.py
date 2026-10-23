import platform

import dartpy as dart
import pytest

# TODO(JS): Move this to integration category once created


def test_basic():
    urdfParser = dart.utils.DartLoader()
    arm = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    assert arm is not None

    shoulder = arm.getBodyNode("shoulder")
    assert shoulder is not None

    elbow = arm.getBodyNode("elbow")
    assert elbow is not None

    chain1 = dart.dynamics.Chain(shoulder, elbow, False, "midchain")
    assert chain1 is not None
    assert chain1.getNumBodyNodes() == 2

    chain2 = dart.dynamics.Chain(shoulder, elbow, True, "midchain")
    assert chain2 is not None
    assert chain2.getNumBodyNodes() == 3

    assert len(arm.getPositions()) != 0
    assert arm.getNumJoints() != 0
    assert arm.getRootJoint() is not None
    assert len(arm.getRootJoint().getPositions()) == 0

    rootBody = arm.getBodyNode(0)
    assert rootBody is not None

    rootJoint = arm.getJoint(0)
    assert rootJoint is not None


if __name__ == "__main__":
    pytest.main()
