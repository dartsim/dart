"""One parity oracle for the pybind11 module and the nanobind binder."""

import gc
import math
import weakref

import dartpy as dart
import numpy as np
import pytest

from ._support import IS_NANOBIND

D = dart.dynamics


def scene():
    skeleton = D.Skeleton(name="inheritance")
    joint, body = skeleton.createFreeJointAndBodyNodePair(parent=None)
    joint.setName(name="root_joint")
    body.setName(name="root")
    shape = body.createShapeNode(D.BoxShape(np.array([1.0, 2.0, 3.0])))
    return skeleton, joint, body, shape


@pytest.mark.parametrize(
    ("kind", "dofs"), [("Free", 6), ("Revolute", 1), ("Ball", 3), ("Weld", 0)]
)
def test_joint_primary_and_adjusted_base_methods(kind, dofs):
    skeleton = D.Skeleton(name=kind)
    joint, body = getattr(skeleton, f"create{kind}JointAndBodyNodePair")(parent=None)
    joint.setName(name=f"{kind}_joint")
    assert type(joint) is getattr(D, f"{kind}Joint")
    assert joint.getName() == D.Joint.getName(joint) == f"{kind}_joint"
    assert joint.getNumDofs() == D.Joint.getNumDofs(joint) == dofs
    assert joint.getType() == f"{kind}Joint"
    assert joint.getChildBodyNode() is body
    assert joint.getParentBodyNode() is None
    assert body.getParentJoint() is joint
    assert skeleton.getJoint(0) is joint
    assert skeleton.getJoint(idx=0) is joint
    assert D.MetaSkeleton.getJoint(skeleton, index=0) is joint
    assert D.MetaSkeleton.getNumDofs(skeleton) == dofs
    assert D.MetaSkeleton.getName(skeleton) == kind


def test_body_virtual_bases_and_jacobian_offsets():
    skeleton, joint, body, _ = scene()
    assert body.getName() == D.Entity.getName(body) == "root"
    assert not body.isWorld()
    assert not D.Frame.isWorld(body)
    assert D.Entity.isFrame(body)
    assert not D.Entity.isQuiet(body)
    assert not D.Frame.isShapeFrame(body)
    np.testing.assert_allclose(body.getWorldTransform().matrix(), np.eye(4))
    np.testing.assert_allclose(D.Frame.getWorldTransform(body).matrix(), np.eye(4))
    for cls in (D.JacobianNode, D.TemplatedJacobianBodyNode):
        np.testing.assert_allclose(cls.getJacobian(body, D.Frame.World()), np.eye(6))
        np.testing.assert_allclose(cls.getWorldJacobian(body, np.zeros(3)), np.eye(6))
    assert D.Node.getSkeleton(body) is skeleton
    assert D.Node.getBodyNodePtr(body) is body
    assert skeleton.getBodyNode(0) is skeleton.getRootBodyNode()
    assert skeleton.getBodyNode("root") is body
    assert skeleton.getBodyNode(treeIndex="root") is body
    assert skeleton.getBodyNode(index=0) is body
    assert skeleton.getRootBodyNode(treeIndex=0) is body
    assert body.getIndexInSkeleton() == 0
    _, child = skeleton.createRevoluteJointAndBodyNodePair(parent=body)
    assert body.getChildBodyNode(index=0) is child
    assert child.getParentJoint().getParentBodyNode() is body


def test_shape_secondary_bases_and_base_arguments():
    skeleton, _, body, shape = scene()
    assert shape.getName() == D.Entity.getName(shape) == "root_ShapeNode_0"
    assert shape.isShapeFrame()
    assert D.Frame.isShapeFrame(shape)
    assert shape.getSkeleton() is D.Node.getSkeleton(shape) is skeleton
    assert shape.getBodyNodePtr() is D.Node.getBodyNodePtr(shape) is body
    assert shape.getShape().getVolume() == pytest.approx(6.0)
    np.testing.assert_allclose(
        shape.getJacobian(inCoordinatesOf=D.Frame.World()), np.eye(6)
    )
    np.testing.assert_allclose(
        D.JacobianNode.getWorldJacobian(shape, np.zeros(3)), np.eye(6)
    )
    replacement = D.EllipsoidShape(np.array([2.0, 4.0, 6.0]))
    shape.setShape(shape=replacement)
    assert shape.getShape() is D.ShapeFrame.getShape(shape) is replacement
    assert shape.getShape().getVolume() == pytest.approx(8.0 * math.pi)
    simple = D.SimpleFrame(
        refFrame=shape, name="child", relativeTransform=dart.math.Isometry3()
    )
    assert simple.getParentFrame() is shape
    assert simple.descendsFrom(someFrame=body)
    assert simple.descendsFrom(someFrame=shape)
    assert D.Entity.descendsFrom(simple, body)
    assert not body.descendsFrom(simple)
    simple.setShape(shape=replacement)
    assert D.ShapeFrame.getShape(simple) is replacement
    simple.setParentFrame(newParentFrame=body)
    assert simple.getParentFrame() is body
    D.Detachable.setParentFrame(simple, shape)
    assert simple.getParentFrame() is shape


def test_frame_overloads_transforms_and_python_subclass():
    _, _, body, shape = scene()

    class PythonFrame(D.SimpleFrame):
        pass

    transform = dart.math.Isometry3()
    transform.set_translation(np.array([1.0, 2.0, 3.0]))
    parent = PythonFrame(
        refFrame=body, name="python_parent", relativeTransform=transform
    )
    parent.setShape(shape=D.SphereShape(0.5))
    assert D.ShapeFrame.getShape(parent).getVolume() == pytest.approx(math.pi / 6.0)
    child = D.SimpleFrame(refFrame=parent, name="python_child")
    child.setRelativeTranslation(newTranslation=np.array([4.0, 5.0, 6.0]))
    np.testing.assert_allclose(parent.getWorldTransform().translation(), [1, 2, 3])
    np.testing.assert_allclose(child.getWorldTransform().translation(), [5, 7, 9])
    np.testing.assert_allclose(child.getTransform().translation(), [5, 7, 9])
    np.testing.assert_allclose(
        child.getTransform(withRespectTo=parent).translation(), [4, 5, 6]
    )
    np.testing.assert_allclose(
        child.getTransform(withRespectTo=parent, inCoordinatesOf=body).translation(),
        [4, 5, 6],
    )
    assert child.descendsFrom(someFrame=parent)
    assert D.Entity.getParentFrame(child) is parent
    child.setParentFrame(newParentFrame=shape)
    assert child.getParentFrame() is shape
    world = dart.simulation.World()
    assert world.addSimpleFrame(frame=parent) == "python_parent"


def test_base_returns_dynamic_types_and_identity():
    skeleton, joint, body, shape = scene()
    frame = D.SimpleFrame(refFrame=shape, name="simple")
    world = D.Frame.World()
    assert type(world) is D.Frame
    assert world.isWorld()
    assert D.Frame.World() is world
    assert type(body.getParentFrame()) is D.Frame
    assert body.getParentFrame().isWorld()
    assert body.getParentFrame().getName() == "World"
    assert shape.getParentFrame() is body
    assert D.Entity.getParentFrame(shape) is body
    assert frame.getParentFrame() is shape
    assert type(frame.getParentFrame()) is D.ShapeNode
    assert type(shape.getParentFrame()) is D.BodyNode
    for get in (body.getChildFrames, body.getChildEntities):
        children = list(get())
        assert len(children) == 1
        assert children[0] is shape
        assert type(children[0]) is D.ShapeNode
    for get in (shape.getChildFrames, shape.getChildEntities):
        children = list(get())
        assert children == [frame]
    assert skeleton.getBodyNode(0) is body
    assert skeleton.getJoint(0) is joint
    assert skeleton.getJoint(idx=0) is joint
    assert D.MetaSkeleton.getJoint(skeleton, index=0) is joint
    assert body.getShapeNode(0) is shape
    assert body.getNumShapeNodes() == 1
    assert skeleton.getNumBodyNodes() == 1


def test_python_primary_base_membership():
    _, _, body, shape = scene()
    simple = D.SimpleFrame()
    for value, bases in (
        (body, (D.Frame, D.Entity, D.JacobianNode, D.TemplatedJacobianBodyNode)),
        (shape, (D.Frame, D.Entity, D.JacobianNode)),
        (simple, (D.Frame, D.Entity, D.ShapeFrame)),
    ):
        for base in bases:
            assert isinstance(value, base)
            assert issubclass(type(value), base)
            assert base in type(value).__mro__


def test_accepted_secondary_base_method_parity():
    """The accepted ancestry difference still requires adjusted method parity."""
    skeleton, _, body, shape = scene()
    assert body.getSkeleton() is D.Node.getSkeleton(body) is skeleton
    assert body.getBodyNodePtr() is D.Node.getBodyNodePtr(body) is body
    assert shape.getSkeleton() is D.Node.getSkeleton(shape) is skeleton
    assert shape.getBodyNodePtr() is D.Node.getBodyNodePtr(shape) is body
    assert shape.getShape() is D.ShapeFrame.getShape(shape)
    replacement = D.SphereShape(0.5)
    D.ShapeFrame.setShape(shape, replacement)
    assert shape.getShape() is replacement
    simple = D.SimpleFrame(refFrame=body, name="secondary")
    D.Detachable.setParentFrame(simple, shape)
    assert simple.getParentFrame() is shape
    simple.setParentFrame(body)
    assert D.Entity.getParentFrame(simple) is body


def test_accepted_difference_secondary_python_membership():
    """The oracle records the accepted isinstance/issubclass/MRO difference."""
    _, _, body, shape = scene()
    for value, bases in (
        (body, (D.Node,)),
        (shape, (D.Node, D.ShapeFrame)),
        (D.SimpleFrame(), (D.Detachable,)),
    ):
        for base in bases:
            assert isinstance(value, base) is (not IS_NANOBIND)
            assert issubclass(type(value), base) is (not IS_NANOBIND)
            assert (base in type(value).__mro__) is (not IS_NANOBIND)


def test_weak_references_and_dof_roundtrip():
    skeleton, joint, body, shape = scene()
    simple = D.SimpleFrame()
    for value in (
        skeleton,
        joint,
        body,
        shape,
        simple,
        shape.getShape(),
        D.Frame.World(),
    ):
        assert weakref.ref(value)() is value
    dofs = skeleton.getDofs()
    assert len(dofs) == 6
    for index, dof in enumerate(dofs):
        assert type(dof) is D.DegreeOfFreedom
        assert dof.getSkeleton() is skeleton
        assert dof.getName()
        dof.setPosition(position=index / 10.0)
        assert dof.getPosition() == pytest.approx(index / 10.0)
    np.testing.assert_allclose(
        D.MetaSkeleton.getPositions(skeleton), np.arange(6) / 10.0
    )
    D.MetaSkeleton.setPositions(skeleton, np.zeros(6))
    np.testing.assert_allclose(skeleton.getPositions(), np.zeros(6))


@pytest.mark.parametrize("bad", [object(), 1, "frame", dart.math.Isometry3()])
def test_non_instances_do_not_convert_to_bases(bad):
    simple = D.SimpleFrame()
    with pytest.raises(TypeError):
        simple.setParentFrame(bad)
    with pytest.raises(TypeError):
        simple.descendsFrom(bad)
    with pytest.raises(TypeError):
        D.Frame.getWorldTransform(bad)


@pytest.mark.parametrize(
    "operation",
    [
        "box",
        "ellipsoid",
        "isometry_translation",
        "frame_translation",
        "positions",
        "jacobian",
        "world_jacobian",
    ],
)
@pytest.mark.parametrize("sequence_type", [list, tuple])
def test_python_sequence_arguments_match_baseline(operation, sequence_type):
    """Stock Eigen conversion must retain pybind11's ordinary list inputs."""
    skeleton, _, body, _ = scene()
    if operation == "box":
        assert D.BoxShape(sequence_type([1, 2, 3])).getVolume() == pytest.approx(6.0)
    elif operation == "ellipsoid":
        assert D.EllipsoidShape(sequence_type([2, 4, 6])).getVolume() == pytest.approx(
            8 * math.pi
        )
    elif operation == "isometry_translation":
        transform = dart.math.Isometry3()
        transform.set_translation(sequence_type([1, 2, 3]))
        np.testing.assert_allclose(transform.translation(), [1, 2, 3])
    elif operation == "frame_translation":
        frame = D.SimpleFrame()
        frame.setRelativeTranslation(sequence_type([1, 2, 3]))
        np.testing.assert_allclose(frame.getWorldTransform().translation(), [1, 2, 3])
    elif operation == "positions":
        skeleton.setPositions(sequence_type([0] * 6))
        np.testing.assert_allclose(skeleton.getPositions(), np.zeros(6))
    elif operation == "jacobian":
        np.testing.assert_allclose(
            body.getJacobian(sequence_type([0, 0, 0])), np.eye(6)
        )
    else:
        np.testing.assert_allclose(
            body.getWorldJacobian(sequence_type([0, 0, 0])), np.eye(6)
        )


@pytest.mark.parametrize("bad", [[1.0, 2.0], ["one", "two", "three"], object()])
def test_invalid_eigen_arguments_preserve_conversion_errors(bad):
    with pytest.raises(TypeError):
        D.EllipsoidShape(bad)


def test_python_subclass_can_translate_constructor_arguments():
    _, _, body, _ = scene()

    class NamedFrame(D.SimpleFrame):
        def __init__(self, parent, label):
            super().__init__(refFrame=parent, name=f"python_{label}")

    frame = NamedFrame(parent=body, label="translated")
    assert D.Entity.getName(frame) == "python_translated"
    assert frame.getParentFrame() is body
    assert frame.descendsFrom(body)
    frame.setShape(D.SphereShape(1.0))
    assert D.ShapeFrame.getShape(frame).getVolume() == pytest.approx(4 * math.pi / 3)
