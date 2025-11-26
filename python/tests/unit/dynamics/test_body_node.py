import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    for i in range(kr5.get_num_body_nodes()):
        body = kr5.get_body_node(i)
        body_force = body.get_body_force()
        assert body_force.size == 6
        bodyPtr = body.get_body_node_ptr()
        assert body == bodyPtr
        assert body.get_name() == bodyPtr.get_name()
        assert np.array_equal(np.array(body.get_spatial_velocity()), np.zeros(6)) is True
        shape_nodes = body.get_shape_nodes()
        for shape_node in shape_nodes:
            print(shape_node)
            if shape_node.has_visual_aspect():
                visual = shape_node.get_visual_aspect()
                visual.get_rgba()
            if shape_node.has_collision_aspect():
                collision = shape_node.get_collision_aspect()
            if shape_node.has_dynamics_aspect():
                dynamics = shape_node.get_dynamics_aspect()


def test_get_child_methods():
    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    currentBodyNode = kr5.get_root_body_node()
    assert currentBodyNode is not None

    for i in range(1, kr5.get_num_body_nodes()):
        childBodyNode = currentBodyNode.get_child_body_node(0)
        childJoint = currentBodyNode.get_child_joint(0)

        assert childBodyNode is not None
        assert childJoint is not None
        assert childBodyNode.get_name() == kr5.get_body_node(i).get_name()
        assert childJoint.get_name() != ""

        currentBodyNode = childBodyNode


def test_get_inertia():
    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    inertias = [
        kr5.get_body_node(i).get_inertia() for i in range(1, kr5.get_num_body_nodes())
    ]
    assert all([inertia is not None for inertia in inertias])


def test_free_joint_world_jacobian_translational_dofs():
    skel = dart.Skeleton("test_free_joint_world_jacobian")
    joint, body = skel.createFreeJointAndBodyNodePair()

    world = dart.Frame.World()

    positions = np.zeros(6)
    positions[:3] = np.array([0.7, -0.4, 0.2])
    positions[3:] = np.array([0.2, -0.1, 0.05])
    joint.setPositions(positions)

    world_jac = np.array(body.getWorldJacobian())
    translational_block = world_jac[3:6, 3:6]
    assert np.allclose(translational_block, np.eye(3), atol=1e-12)

    velocities = np.zeros(6)
    velocities[:3] = np.array([0.1, 0.15, -0.2])
    velocities[3:] = np.array([0.05, -0.08, 0.12])
    joint.setVelocities(velocities)

    spatial_from_jac = world_jac.dot(velocities)
    assert np.allclose(spatial_from_jac[3:], velocities[3:], atol=1e-12)

    linear_velocity = np.array(body.getLinearVelocity(world, world)).reshape(3)
    assert np.allclose(linear_velocity, velocities[3:], atol=1e-12)

    num_tests = 10
    for _ in range(num_tests):
        positions = np.random.uniform(-0.5, 0.5, 6)
        joint.setPositions(positions)

        world_jac = np.array(body.getWorldJacobian())
        translational_block = world_jac[3:6, 3:6]
        assert np.allclose(translational_block, np.eye(3), atol=1e-12)

        velocities = np.random.uniform(-1.0, 1.0, 6)
        joint.setVelocities(velocities)

        spatial_from_jac = world_jac.dot(velocities)
        assert np.allclose(spatial_from_jac[3:], velocities[3:], atol=1e-12)

        linear_velocity = np.array(body.getLinearVelocity(world, world)).reshape(3)
        assert np.allclose(linear_velocity, velocities[3:], atol=1e-12)


if __name__ == "__main__":
    pytest.main()
