import platform
import pytest
import numpy as np
import dartpy as dart


def test_basic():
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert kr5 is not None

    for i in range(kr5.getNumBodyNodes()):
        body = kr5.getBodyNode(i)
        assert np.array_equal(
            np.array(body.getSpatialVelocity()), np.zeros(6)) is True
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


def testGetChildPMethods():
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


if __name__ == "__main__":
    pytest.main()
