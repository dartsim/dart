import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    urdfParser = dart.utils.DartLoader()
    arm = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    assert arm is not None

    for i in range(arm.getNumBodyNodes()):
        body = arm.getBodyNode(i)
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
    arm = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    assert arm is not None

    currentBodyNode = arm.getRootBodyNode()
    assert currentBodyNode is not None

    for i in range(1, arm.getNumBodyNodes()):
        childBodyNode = currentBodyNode.getChildBodyNode(0)
        childJoint = currentBodyNode.getChildJoint(0)

        assert childBodyNode is not None
        assert childJoint is not None
        assert childBodyNode.getName() == arm.getBodyNode(i).getName()
        assert childJoint.getName() == arm.getJoint(i).getName()

        currentBodyNode = childBodyNode


def test_get_inertia():
    urdfParser = dart.utils.DartLoader()
    arm = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    assert arm is not None

    # Solid-cylinder inertias, with the long axis in each body's local frame.
    cylinders = [
        (2.0, 0.08, 0.12, 2, [0, 0, 0.06]),
        (1.0, 0.045, 0.06, 2, [0, 0, 0.03]),
        (0.8, 0.0325, 0.28, 2, [0, 0, 0.14]),
        (0.6, 0.0275, 0.23, 0, [0.115, 0, 0]),
        (0.25, 0.025, 0.06, 0, [0.03, 0, 0]),
        (0.18, 0.0225, 0.05, 0, [0.025, 0, 0]),
        (0.12, 0.0175, 0.04, 0, [0.02, 0, 0]),
    ]
    assert arm.getNumBodyNodes() == len(cylinders)
    for body, (mass, radius, length, axis, center) in zip(
        arm.getBodyNodes(), cylinders
    ):
        inertia = body.getInertia()
        assert inertia.getMass() == pytest.approx(mass)
        np.testing.assert_allclose(inertia.getLocalCOM(), center, atol=1e-12)
        moment = np.full(3, mass * (3 * radius**2 + length**2) / 12)
        moment[axis] = mass * radius**2 / 2
        np.testing.assert_allclose(inertia.getMoment(), np.diag(moment), atol=1e-12)


def test_six_axis_arm_upright_meshes():
    arm = dart.utils.DartLoader().parseSkeleton(
        "dart://sample/urdf/six_axis_arm/six_axis_arm.urdf"
    )
    np.testing.assert_allclose(
        arm.getBodyNode("tool").getWorldTransform().translation(), [0.34, 0, 0.46]
    )
    np.testing.assert_allclose(
        arm.getBodyNode("tool").getWorldTransform().rotation(), np.eye(3)
    )
    for body in arm.getBodyNodes():
        mesh = next(
            node.getShape() for node in body.getShapeNodes() if node.hasVisualAspect()
        )
        vertices = np.concatenate(mesh.mesh_vertex_groups)
        axis = 2 if body.getName() in ("base", "shoulder", "upper_arm") else 0
        levels = np.unique(np.round(vertices[:, axis], 7))
        # Raw mesh vertices are the point-cloud demo's sampling surface.
        assert np.diff(levels).max() <= 0.040001
        for level in levels:
            ring = vertices[np.isclose(vertices[:, axis], level)]
            assert len(np.unique(np.round(ring, 7), axis=0)) >= 20


if __name__ == "__main__":
    pytest.main()
