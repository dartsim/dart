import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    for i in range(kr5.getNumBodyNodes()):
        body = kr5.getBodyNode(i)
        body_force = body.getBodyForce()
        assert body_force.size == 6
        bodyPtr = body.getBodyNodePtr()
        assert body == bodyPtr
        assert body.getName() == bodyPtr.getName()
        assert np.array_equal(np.array(body.getSpatialVelocity()), np.zeros(6)) is True
        shape_nodes = body.getShapeNodes()
        for shape_node in shape_nodes:
            print(shape_node)
            if shape_node.hasVisualAspect():
                visual = shape_node.getVisualAspect()
                visual.getRGBA()
            if shape_node.hasCollisionAspect():
                collision = shape_node.getCollisionAspect()
            if shape_node.hasDynamicsAspect():
                dynamics = shape_node.getDynamicsAspect()


def test_get_child_methods():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    currentBodyNode = kr5.getRootBodyNode()
    assert currentBodyNode is not None

    for i in range(1, kr5.getNumBodyNodes()):
        childBodyNode = currentBodyNode.getChildBodyNode(0)
        childJoint = currentBodyNode.getChildJoint(0)

        assert childBodyNode is not None
        assert childJoint is not None
        assert childBodyNode.getName() == kr5.getBodyNode(i).getName()
        assert childJoint.getName() == kr5.getJoint(i).getName()

        currentBodyNode = childBodyNode


def test_get_inertia():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    inertias = [
        kr5.getBodyNode(i).getInertia() for i in range(1, kr5.getNumBodyNodes())
    ]
    assert all([inertia is not None for inertia in inertias])


def test_free_joint_world_jacobian_translational_dofs():
    skel = dart.dynamics.Skeleton("test_free_joint_world_jacobian")
    joint, body = skel.createFreeJointAndBodyNodePair()

    world = dart.dynamics.Frame.World()

    positions = np.zeros(6)
    positions[:3] = np.array([0.7, -0.4, 0.2])
    positions[3:] = np.array([0.2, -0.1, 0.05])
    joint.setPositions(positions)

    world_jac = np.array(body.getWorldJacobian())
    translational_block = world_jac[3:6, 3:6]
    assert np.allclose(translational_block, np.eye(3), atol=1e-12)

    velocities = np.zeros(6)
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
