import gc

import dartpy as dart


def _make_single_body_skeleton(name: str = "skel"):
    skel = dart.Skeleton(name)
    _, body = skel.create_revolute_joint_and_body_node_pair()
    return skel, body


def _make_two_body_skeleton(name: str = "skel"):
    skel = dart.Skeleton(name)
    _, first = skel.create_revolute_joint_and_body_node_pair()
    _, second = skel.create_revolute_joint_and_body_node_pair(first)
    return skel, first, second


def test_simple_frame_survives_parent_deletion():
    skel, body = _make_single_body_skeleton("lifetime_skel")
    frame = dart.SimpleFrame(body, "child_frame")

    del skel
    del body
    gc.collect()

    parent = frame.get_parent_frame()
    assert parent is not None
    assert parent.is_world()


def test_joint_constraint_keeps_joint_alive():
    skel, body = _make_single_body_skeleton("constraint_skel")
    joint = body.get_parent_joint()
    constraint = dart.JointConstraint(joint)

    del skel
    del body
    del joint
    gc.collect()

    constraint.update()


def test_chain_criteria_weak_fields_are_python_body_nodes():
    skel, first, second = _make_two_body_skeleton("chain_criteria_skel")
    criteria = dart.ChainCriteria(first, second)

    assert criteria.m_start.get_name() == first.get_name()
    assert criteria.m_target.get_name() == second.get_name()

    criteria.m_start = None
    assert criteria.m_start is None
    criteria.m_start = second
    assert criteria.m_start.get_name() == second.get_name()


def test_linkage_criteria_target_and_terminal_fields_are_python_body_nodes():
    skel, first, second = _make_two_body_skeleton("linkage_criteria_skel")

    target = dart.LinkageCriteria.Target()
    assert target.m_node is None
    target.m_node = first
    assert target.m_node.get_name() == first.get_name()

    terminal = dart.LinkageCriteria.Terminal()
    assert terminal.m_terminal is None
    terminal.m_terminal = second
    assert terminal.m_terminal.get_name() == second.get_name()


def test_chain_criteria_body_nodes_expire_when_skeleton_is_deleted():
    def _make_criteria():
        skel, first, second = _make_two_body_skeleton("chain_criteria_expire_skel")
        return dart.ChainCriteria(first, second)

    criteria = _make_criteria()
    gc.collect()

    assert criteria.m_start is None
    assert criteria.m_target is None
