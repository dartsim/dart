import dartpy as dart
import pytest

def test_basic():
    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    shoulder = kr5.get_body_node("shoulder")
    assert shoulder is not None

    elbow = kr5.get_body_node("elbow")
    assert elbow is not None

    chain1 = dart.Chain(shoulder, elbow, False, "midchain")
    assert chain1 is not None
    assert chain1.get_num_body_nodes() == 2

    chain2 = dart.Chain(shoulder, elbow, True, "midchain")
    assert chain2 is not None
    assert chain2.get_num_body_nodes() == 3

    assert len(kr5.get_positions()) != 0
    assert kr5.get_num_joints() != 0
    assert kr5.get_root_joint() is not None
    assert len(kr5.get_root_joint().get_positions()) == 0

    rootBody = kr5.get_body_node(0)
    assert rootBody is not None

    rootJoint = kr5.get_joint(0)
    assert rootJoint is not None


if __name__ == "__main__":
    pytest.main()
