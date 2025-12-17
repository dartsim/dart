import dartpy as dart


def test_clone_meta_skeleton_preserves_dynamic_type():
    skel = dart.Skeleton()

    root_joint_props = dart.FreeJointProperties()
    [root_joint, root_body] = skel.create_free_joint_and_body_node_pair(
        None, root_joint_props
    )

    child_joint_props = dart.RevoluteJointProperties()
    [child_joint, child_body] = skel.create_revolute_joint_and_body_node_pair(
        root_body, child_joint_props
    )

    chain = dart.Chain(root_body, child_body)
    clone = chain.clone_meta_skeleton("clone")

    assert isinstance(clone, dart.Chain)
    assert clone.is_still_chain()
