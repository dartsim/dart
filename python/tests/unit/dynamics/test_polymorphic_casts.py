import dartpy as dart

from dartpy._dartpy import dynamics as _dyn


def _make_body():
    skel = dart.Skeleton("polymorphic_cast_skel")
    _, body = skel.create_revolute_joint_and_body_node_pair()
    return skel, body


def test_body_node_casts_across_frame_and_jacobian_bases():
    skel, body = _make_body()

    assert isinstance(body, dart.BodyNode)
    assert isinstance(body, _dyn.JacobianNode)
    assert isinstance(body, _dyn.Frame)

    child_frame = dart.SimpleFrame(body, "body_child_frame")
    parent = child_frame.get_parent_frame()

    assert isinstance(parent, dart.BodyNode)
    assert parent.get_name() == body.get_name()

    ik = body.get_or_create_ik()
    assert ik.is_active()


def test_end_effector_casts_across_frame_and_jacobian_bases():
    skel, body = _make_body()
    end_effector = body.create_end_effector("tool")

    assert isinstance(end_effector, dart.EndEffector)
    assert isinstance(end_effector, _dyn.JacobianNode)
    assert isinstance(end_effector, _dyn.Frame)

    child_frame = dart.SimpleFrame(end_effector, "tool_child_frame")
    parent = child_frame.get_parent_frame()

    assert isinstance(parent, dart.EndEffector)
    assert parent.get_name() == end_effector.get_name()

    ik = end_effector.get_or_create_ik()
    assert ik.is_active()
