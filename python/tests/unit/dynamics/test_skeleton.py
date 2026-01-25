import dartpy as dart
import numpy as np
import pytest


class TestSkeletonCreation:
    def test_create_empty_skeleton(self):
        skel = dart.Skeleton()
        assert skel is not None
        assert skel.get_num_body_nodes() == 0
        assert skel.get_num_joints() == 0
        assert skel.get_num_dofs() == 0

    def test_create_named_skeleton(self):
        skel = dart.Skeleton("test_robot")
        assert skel.get_name() == "test_robot"

    def test_set_name(self):
        skel = dart.Skeleton()
        skel.set_name("renamed_robot")
        assert skel.get_name() == "renamed_robot"


class TestSkeletonJointCreation:
    def test_create_free_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        assert skel.get_num_body_nodes() == 1
        assert skel.get_num_joints() == 1
        assert skel.get_num_dofs() == 6
        assert joint.get_type() == "FreeJoint"

    def test_create_revolute_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_revolute_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 1
        assert joint.get_type() == "RevoluteJoint"

    def test_create_prismatic_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_prismatic_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 1
        assert joint.get_type() == "PrismaticJoint"

    def test_create_weld_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_weld_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 0
        assert joint.get_type() == "WeldJoint"

    def test_create_ball_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_ball_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 3
        assert joint.get_type() == "BallJoint"

    def test_create_euler_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_euler_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 3
        assert joint.get_type() == "EulerJoint"

    def test_create_planar_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_planar_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 3
        assert joint.get_type() == "PlanarJoint"

    def test_create_screw_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_screw_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 1
        assert joint.get_type() == "ScrewJoint"

    def test_create_universal_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_universal_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 2
        assert joint.get_type() == "UniversalJoint"

    def test_create_translational_joint(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_translational_joint_and_body_node_pair()

        assert skel.get_num_dofs() == 3
        assert joint.get_type() == "TranslationalJoint"

    def test_create_translational_joint_2d(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_translational_joint2_d_and_body_node_pair()

        assert skel.get_num_dofs() == 2
        assert joint.get_type() == "TranslationalJoint2D"

    def test_create_joint_with_properties(self):
        skel = dart.Skeleton()
        props = dart.FreeJointProperties()
        props.mName = "custom_joint"

        [joint, body] = skel.create_free_joint_and_body_node_pair(None, props)

        assert joint.get_name() == "custom_joint"

    def test_create_chain_of_joints(self):
        skel = dart.Skeleton()

        [j1, b1] = skel.create_free_joint_and_body_node_pair()
        [j2, b2] = skel.create_revolute_joint_and_body_node_pair(b1)
        [j3, b3] = skel.create_revolute_joint_and_body_node_pair(b2)

        assert skel.get_num_body_nodes() == 3
        assert skel.get_num_joints() == 3
        assert skel.get_num_dofs() == 8


class TestSkeletonAccessors:
    @pytest.fixture
    def three_link_skeleton(self):
        skel = dart.Skeleton("three_link")

        props1 = dart.FreeJointProperties()
        props1.mName = "joint1"
        [j1, b1] = skel.create_free_joint_and_body_node_pair(None, props1)

        [j2, b2] = skel.create_revolute_joint_and_body_node_pair(b1)
        j2.set_name("joint2")

        [j3, b3] = skel.create_revolute_joint_and_body_node_pair(b2)
        j3.set_name("joint3")

        return skel

    def test_get_body_node_by_index(self, three_link_skeleton):
        body0 = three_link_skeleton.get_body_node(0)
        assert body0 is not None

        body1 = three_link_skeleton.get_body_node(1)
        assert body1 is not None
        assert body0 != body1

    def test_get_body_node_by_name(self, three_link_skeleton):
        body = three_link_skeleton.get_body_node(0)
        name = body.get_name()
        body_by_name = three_link_skeleton.get_body_node(name)
        assert body_by_name is not None
        assert body_by_name.get_name() == name

    def test_get_root_body_node(self, three_link_skeleton):
        root = three_link_skeleton.get_root_body_node()
        assert root is not None
        assert root == three_link_skeleton.get_body_node(0)

    def test_get_joint_by_index(self, three_link_skeleton):
        joint = three_link_skeleton.get_joint(0)
        assert joint is not None
        assert joint.get_name() == "joint1"

    def test_get_joint_by_name(self, three_link_skeleton):
        joint = three_link_skeleton.get_joint("joint2")
        assert joint is not None
        assert joint.get_name() == "joint2"

    def test_get_root_joint(self, three_link_skeleton):
        root_joint = three_link_skeleton.get_root_joint()
        assert root_joint is not None
        assert root_joint.get_name() == "joint1"

    def test_get_dof_by_index(self, three_link_skeleton):
        dof = three_link_skeleton.get_dof(0)
        assert dof is not None

    def test_get_num_end_effectors(self, three_link_skeleton):
        assert three_link_skeleton.get_num_end_effectors() == 0


class TestSkeletonPositionsVelocities:
    @pytest.fixture
    def revolute_chain(self):
        skel = dart.Skeleton()
        [j1, b1] = skel.create_revolute_joint_and_body_node_pair()
        [j2, b2] = skel.create_revolute_joint_and_body_node_pair(b1)
        [j3, b3] = skel.create_revolute_joint_and_body_node_pair(b2)
        return skel

    def test_get_positions_initial(self, revolute_chain):
        positions = revolute_chain.get_positions()
        assert len(positions) == 3
        assert np.allclose(positions, np.zeros(3))

    def test_set_positions(self, revolute_chain):
        new_pos = np.array([0.1, 0.2, 0.3])
        revolute_chain.set_positions(new_pos)

        positions = revolute_chain.get_positions()
        assert np.allclose(positions, new_pos)

    def test_reset_positions(self, revolute_chain):
        revolute_chain.set_positions(np.array([1.0, 2.0, 3.0]))
        revolute_chain.reset_positions()

        positions = revolute_chain.get_positions()
        assert np.allclose(positions, np.zeros(3))

    def test_joint_velocities(self, revolute_chain):
        joint = revolute_chain.get_joint(0)
        velocities = joint.get_velocities()
        assert len(velocities) == 1
        assert np.allclose(velocities, np.zeros(1))

    def test_set_joint_velocities(self, revolute_chain):
        joint = revolute_chain.get_joint(0)
        new_vel = np.array([0.5])
        joint.set_velocities(new_vel)

        velocities = joint.get_velocities()
        assert np.allclose(velocities, new_vel)


class TestSkeletonCollisionSettings:
    def test_self_collision_check_default_disabled(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        assert skel.is_enabled_self_collision_check() is False

    def test_enable_self_collision_check(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        skel.enable_self_collision_check()
        assert skel.is_enabled_self_collision_check() is True

    def test_disable_self_collision_check(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        skel.enable_self_collision_check()
        skel.disable_self_collision_check()
        assert skel.is_enabled_self_collision_check() is False

    def test_adjacent_body_check(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        skel.enable_adjacent_body_check()
        assert skel.is_enabled_adjacent_body_check() is True

        skel.disable_adjacent_body_check()
        assert skel.is_enabled_adjacent_body_check() is False


class TestSkeletonRepr:
    def test_repr_contains_name(self):
        skel = dart.Skeleton("my_robot")
        [j, b] = skel.create_free_joint_and_body_node_pair()

        repr_str = repr(skel)
        assert "Skeleton" in repr_str
        assert "my_robot" in repr_str

    def test_repr_contains_counts(self):
        skel = dart.Skeleton()
        [j1, b1] = skel.create_free_joint_and_body_node_pair()
        [j2, b2] = skel.create_revolute_joint_and_body_node_pair(b1)

        repr_str = repr(skel)
        assert "body_nodes" in repr_str
        assert "joints" in repr_str
        assert "dofs" in repr_str


class TestSkeletonIK:
    def test_get_ik_without_creation(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        ik = skel.get_ik(False)
        assert ik is None

    def test_get_ik_with_creation(self):
        skel = dart.Skeleton()
        [j, b] = skel.create_free_joint_and_body_node_pair()

        ik = skel.get_ik(True)
        assert ik is not None


if __name__ == "__main__":
    pytest.main()
