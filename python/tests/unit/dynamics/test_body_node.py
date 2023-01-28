import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    urdfParser = dart.io.DartLoader()
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
    urdfParser = dart.io.DartLoader()
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
    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    inertias = [
        kr5.getBodyNode(i).getInertia() for i in range(1, kr5.getNumBodyNodes())
    ]
    assert all([inertia is not None for inertia in inertias])


if __name__ == "__main__":
    pytest.main()
