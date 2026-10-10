"""pybind11 extensions exchange DART objects with dartpy through its C API."""

import gc

import dartpy as dart
import numpy as np
import pytest

from ._support import IS_NANOBIND, run_isolated

pytestmark = pytest.mark.skipif(
    not IS_NANOBIND, reason="the nanobind binder exports dartpy._C_API"
)


@pytest.fixture(scope="module")
def interop():
    return pytest.importorskip("dartpy_pybind11_interop_test")


def make_skeleton(name="robot"):
    skeleton = dart.dynamics.Skeleton(name)
    skeleton.createFreeJointAndBodyNodePair()
    return skeleton


def test_c_api_capsule_is_exported():
    assert type(dart._C_API).__name__ == "PyCapsule"


def test_shared_skeleton_round_trip_keeps_identity(interop):
    skeleton = make_skeleton()
    assert interop.skeleton_name(skeleton) == "robot"
    assert interop.same_skeleton(skeleton) is skeleton
    assert interop.const_skeleton(skeleton) is skeleton


def test_native_skeleton_becomes_dartpy_skeleton(interop):
    skeleton = interop.make_skeleton("native")
    assert isinstance(skeleton, dart.dynamics.Skeleton)
    assert skeleton.getName() == "native"
    assert skeleton.getNumBodyNodes() == 1


def test_borrowed_graph_objects_keep_identity(interop):
    skeleton = make_skeleton()
    assert interop.body(skeleton, 0) is skeleton.getBodyNode(0)
    assert interop.joint(skeleton, 0) is skeleton.getJoint(0)
    assert interop.dof(skeleton, 0) is skeleton.getDof(0)


def test_borrowed_body_keeps_its_skeleton_alive(interop):
    body = interop.body(interop.make_skeleton("temporary"), 0)
    gc.collect()
    assert body.getSkeleton().getName() == "temporary"


def test_base_class_arguments_are_adjusted(interop):
    skeleton = make_skeleton()
    body = skeleton.getBodyNode(0)
    assert interop.frame_name(body) == body.getName()
    assert interop.entity_name(body) == body.getName()
    assert interop.jacobian_node_name(body) == body.getName()
    assert interop.meta_skeleton_dofs(skeleton) == skeleton.getNumDofs()
    assert interop.frame_name(dart.dynamics.Frame.World()) == "World"
    assert interop.frame_name(None) == ""


def test_callbacks_receive_existing_wrappers(interop):
    skeleton = make_skeleton()
    seen = []
    interop.call_with_body(seen.append, skeleton, 0)
    assert len(seen) == 1
    assert seen[0] is skeleton.getBodyNode(0)


def test_shared_world_round_trip(interop):
    world = dart.simulation.World()
    skeleton = make_skeleton()
    interop.add_skeleton(world, skeleton)
    assert world.getNumSkeletons() == 1
    assert world.getSkeleton(0) is skeleton
    assert interop.same_world(world) is world


def test_isometry_values_are_copied(interop):
    transform = dart.math.Isometry3()
    transform.set_translation([1.0, 2.0, 3.0])
    result = interop.translate(transform, [0.0, 0.0, 1.0])
    assert isinstance(result, dart.math.Isometry3)
    np.testing.assert_allclose(result.translation(), [1.0, 2.0, 4.0])
    np.testing.assert_allclose(transform.translation(), [1.0, 2.0, 3.0])

    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 0.0, 0.0]
    result = interop.translate(matrix, [0.0, 1.0, 0.0])
    np.testing.assert_allclose(result.translation(), [1.0, 1.0, 0.0])


def test_wrong_types_raise_type_error(interop):
    skeleton = make_skeleton()
    with pytest.raises(TypeError):
        interop.frame_name("not a frame")
    with pytest.raises(TypeError):
        interop.skeleton_name(skeleton.getBodyNode(0))
    with pytest.raises(TypeError):
        interop.translate("not a transform", [0.0, 0.0, 0.0])


def test_shape_nodes_cross_both_ways(interop):
    body = make_skeleton().getBodyNode(0)
    shape_node = interop.add_shape_node(body)
    assert isinstance(shape_node, dart.dynamics.ShapeNode)
    assert shape_node is body.getShapeNode(0)
    assert interop.shape_node_name(shape_node) == shape_node.getName()


@pytest.mark.parametrize("name", ["owned_body", "automatic_body", "copied_body"])
def test_owning_and_copying_return_policies_are_rejected(interop, name):
    with pytest.raises(RuntimeError, match="return_value_policy::reference"):
        getattr(interop, name)(make_skeleton())


def test_released_last_owner_keeps_borrowed_wrapper_alive(interop):
    holder = interop.FrameHolder()
    borrowed = holder.borrow()
    released = holder.release()
    assert released is borrowed
    del holder
    gc.collect()
    assert interop.released_frame_alive()
    assert borrowed.getName() == "held"
    del borrowed, released
    gc.collect()
    assert not interop.released_frame_alive()


def test_pointer_members_keep_identity(interop):
    body = make_skeleton().getBodyNode(0)
    attachment = interop.Attachment()
    assert attachment.body is None
    attachment.body = body
    assert attachment.body is body


def test_interop_objects_survive_interpreter_shutdown(interop):
    run_isolated(
        """
        import dartpy_pybind11_interop_test as interop
        skeleton = interop.make_skeleton("exit")
        body = interop.body(skeleton, 0)
        world = dart.simulation.World()
        interop.add_skeleton(world, skeleton)
        del skeleton
        gc.collect()
        assert body.getSkeleton().getName() == "exit"
        assert world.getSkeleton(0) is body.getSkeleton()
        """
    )
