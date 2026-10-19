"""Engine snapshots own complete state independently of the originating graph."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest


FIELDS = ("Positions", "Velocities", "Accelerations", "Forces", "Commands")


def _rigid():
    skeleton = dart.dynamics.Skeleton()
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    joint.setActuatorType(dart.dynamics.ActuatorType.SERVO)
    return skeleton, body


def _soft(slices=3, stacks=2):
    skeleton = dart.dynamics.Skeleton()
    properties = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
        [0.2, 0.2, 0.2], slices, stacks, 0.5
    )
    _, body = skeleton.createFreeJointAndSoftBodyNodePair(
        None,
        dart.dynamics.FreeJointProperties(),
        dart.dynamics.SoftBodyNodeProperties(
            dart.dynamics.BodyNodeProperties(), properties
        ),
    )
    return skeleton, body


def _configuration(skeleton):
    return [getattr(skeleton, "get" + field)().copy() for field in FIELDS]


def test_configuration_owns_all_five_vectors_after_original_graph_releases():
    source, body = _rigid()
    for index, field in enumerate(FIELDS):
        getattr(source, "set" + field)(np.arange(6) * 0.01 + index * 0.1)
    expected = _configuration(source)
    snapshot = source.getConfiguration()
    reference = weakref.ref(source)
    for field in FIELDS:
        getattr(source, "set" + field)(np.zeros(6))
    del source, body
    gc.collect()
    assert reference() is None
    destination, _ = _rigid()
    destination.setConfiguration(snapshot)
    for actual, saved in zip(_configuration(destination), expected):
        np.testing.assert_array_equal(actual, saved)


def _points(body):
    return np.array(
        [
            np.concatenate(
                [
                    getattr(body.get_point_mass(index), field)
                    for field in ("positions", "velocities", "accelerations", "forces")
                ]
            )
            for index in range(body.getNumPointMasses())
        ]
    )


def test_body_snapshot_owns_soft_positions_velocities_accelerations_and_forces():
    source, body = _soft()
    point = body.get_point_mass(0)
    point.positions = [0.01, 0.02, 0.03]
    point.velocities = [0.1, 0.2, 0.3]
    source.computeInverseDynamics()
    world = dart.simulation.World()
    world.addSkeleton(source)
    world.step()
    source.computeInverseDynamics()
    expected = _points(body)
    assert np.any(expected[:, 6:9] != 0)
    assert np.any(expected[:, 9:12] != 0)
    snapshot = body.get_composite_state()
    reference = weakref.ref(source)
    world.step()
    del source, body, point, world
    gc.collect()
    assert reference() is None
    _, destination = _soft()
    destination.set_composite_state(snapshot)
    np.testing.assert_array_equal(_points(destination), expected)


@pytest.mark.parametrize("route", ["body", "skeleton"])
@pytest.mark.parametrize("large_source", [False, True])
def test_soft_snapshot_rejects_wrong_point_count_without_changing_state(
    route, large_source
):
    source_skeleton, source = _soft(*((12, 8) if large_source else (3, 2)))
    destination_skeleton, destination = _soft(*((3, 2) if large_source else (12, 8)))
    source_skeleton.setPositions(np.ones(6) * 0.1)
    expected = _points(destination)
    configuration = _configuration(destination_skeleton)
    with pytest.raises(ValueError, match="point count"):
        if route == "body":
            destination.set_composite_state(source.get_composite_state())
        else:
            destination_skeleton.setState(source_skeleton.getState())
    np.testing.assert_array_equal(_points(destination), expected)
    for actual, saved in zip(_configuration(destination_skeleton), configuration):
        np.testing.assert_array_equal(actual, saved)
    destination.set_composite_state(destination.get_composite_state())
    assert np.isfinite(destination.get_point_mass(0).accelerations).all()


def test_skeleton_snapshot_preserves_joint_and_soft_state_after_origin_releases():
    source, body = _soft()
    world = dart.simulation.World()
    world.addSkeleton(source)
    body.get_point_mass(0).velocities = [0.1, 0.2, 0.3]
    world.step()
    source.computeInverseDynamics()
    configuration = _configuration(source)
    points = _points(body)
    snapshot = source.getState()
    reference = weakref.ref(source)
    del source, body, world
    gc.collect()
    assert reference() is None
    destination, body = _soft()
    destination.setState(snapshot)
    for actual, saved in zip(_configuration(destination)[:4], configuration[:4]):
        np.testing.assert_array_equal(actual, saved)
    # C++ state restores forces; a FORCE joint derives commands from them.
    np.testing.assert_array_equal(destination.getCommands(), configuration[3])
    np.testing.assert_array_equal(_points(body), points)


def test_common_composite_state_is_an_owned_value():
    composite = dart.common.Composite()
    state = composite.getCompositeState()
    reference = weakref.ref(composite)
    del composite
    gc.collect()
    assert reference() is None
    replacement = dart.common.Composite()
    replacement.setCompositeState(state)
    replacement.copyCompositeStateTo(state)
