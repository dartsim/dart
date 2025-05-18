import platform

import dartpy as dart
import pytest

# TODO(JS): Move this to integration category once created


def test_basic():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    shoulder = kr5.getBodyNode("shoulder")
    assert shoulder is not None

    elbow = kr5.getBodyNode("elbow")
    assert elbow is not None

    chain1 = dart.dynamics.Chain(shoulder, elbow, False, "midchain")
    assert chain1 is not None
    assert chain1.getNumBodyNodes() == 2

    chain2 = dart.dynamics.Chain(shoulder, elbow, True, "midchain")
    assert chain2 is not None
    assert chain2.getNumBodyNodes() == 3

    assert len(kr5.getPositions()) != 0
    assert kr5.getNumJoints() != 0
    assert kr5.getRootJoint() is not None
    assert len(kr5.getRootJoint().getPositions()) == 0

    rootBody = kr5.getBodyNode(0)
    assert rootBody is not None

    rootJoint = kr5.getJoint(0)
    assert rootJoint is not None


if __name__ == "__main__":
    pytest.main()
