import dartpy as dart


def _make_two_link_skeleton(name: str = "skel"):
    skel = dart.Skeleton(name)
    _, root = skel.create_revolute_joint_and_body_node_pair()
    _, child = skel.create_revolute_joint_and_body_node_pair(root)
    return skel, root, child


def test_world_repr():
    world = dart.World("repr_world")
    text = repr(world)
    assert "World(" in text
    assert "repr_world" in text
    assert "time_step" in text


def test_skeleton_joint_and_dof_repr():
    skel, root, child = _make_two_link_skeleton("repr_skel")
    joint = child.get_parent_joint()
    joint.set_name("child_joint")
    dof = joint.get_dof(0)

    skel_repr = repr(skel)
    assert "Skeleton(" in skel_repr
    assert "repr_skel" in skel_repr

    body_repr = repr(root)
    assert "BodyNode(" in body_repr
    assert "repr_skel" in body_repr

    joint_repr = repr(joint)
    assert "Joint(" in joint_repr
    assert "child_joint" in joint_repr

    dof_repr = repr(dof)
    assert "DegreeOfFreedom(" in dof_repr
    assert "child_joint" in dof_repr


def test_shapes_and_frames_repr():
    skel, body, _ = _make_two_link_skeleton("shape_skel")
    sphere = dart.SphereShape(0.25)
    box = dart.BoxShape([0.1, 0.2, 0.3])

    assert "SphereShape" in repr(sphere)
    assert "radius" in repr(sphere)

    box_repr = repr(box)
    assert "BoxShape" in box_repr
    assert "size" in box_repr

    shape_node = body.create_shape_node(sphere)
    shape_node_repr = repr(shape_node)
    assert "ShapeNode" in shape_node_repr
    assert "SphereShape" in shape_node_repr

    simple_frame = dart.SimpleFrame(body, "simple_frame")
    simple_frame.set_shape(box)
    simple_frame_repr = repr(simple_frame)
    assert "SimpleFrame" in simple_frame_repr
    assert "shape" in simple_frame_repr


def test_chain_and_linkage_repr():
    skel, root, child = _make_two_link_skeleton("chain_skel")
    criteria = dart.ChainCriteria(root, child)
    chain = dart.Chain(criteria, "chain_repr")
    linkage = dart.Linkage(criteria.convert(), "linkage_repr")

    chain_repr = repr(chain)
    assert "Chain(" in chain_repr
    assert "chain_repr" in chain_repr

    linkage_repr = repr(linkage)
    assert "Linkage(" in linkage_repr
    assert "linkage_repr" in linkage_repr


def test_inverse_kinematics_repr():
    skel = dart.Skeleton("ik_skel")
    _, body = skel.create_free_joint_and_body_node_pair()
    ik = body.get_or_create_ik()
    target = dart.SimpleFrame(body, "target_frame")
    ik.set_target(target)

    ik_repr = repr(ik)
    assert "InverseKinematics(" in ik_repr
    assert "target_frame" in ik_repr


def test_collision_repr():
    option = dart.CollisionOption()
    result = dart.CollisionResult()

    assert "CollisionOption(" in repr(option)
    assert "max_contacts" in repr(option)

    result_repr = repr(result)
    assert "CollisionResult(" in result_repr
    assert "contacts" in result_repr
