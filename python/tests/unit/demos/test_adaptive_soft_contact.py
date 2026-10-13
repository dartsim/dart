"""Adaptive model state and the C++ scene's point-marker contract."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest
from dartpy import _demos

adaptive = _demos.adaptive_soft_contact

CHECKSUM_FIELDS = (
    "skeleton_position_l1",
    "skeleton_position_squared",
    "point_world_position_l1",
    "point_world_position_squared",
    "active",
    "total",
    "contacts",
    "finite",
)


def _checksum_values(model):
    checksum = adaptive.compute_checksum(model)
    return tuple(getattr(checksum, field) for field in CHECKSUM_FIELDS)


@pytest.mark.parametrize("adaptive_enabled", [False, True])
def test_adaptive_construction_steps_and_determinism(adaptive_enabled):
    lhs, rhs = [
        adaptive.create_model(
            adaptive_enabled=adaptive_enabled,
            ring_count=adaptive.default_ring_count,
            linger_steps=adaptive.default_linger_steps,
        )
        for _ in range(2)
    ]
    assert lhs.world.getNumSkeletons() == 3
    assert lhs.world.getTimeStep() == adaptive.time_step
    assert lhs.soft_body.getNumPointMasses() > 0
    assert lhs.pusher_joint.getNumDofs() == 6
    assert lhs.soft_skeleton.getNumSoftBodyNodes() == 1
    assert lhs.pusher_skeleton.getNumBodyNodes() == 1
    assert adaptive.soft_body_radius > 0
    assert adaptive.compare_tolerance > 0

    for _ in range(8):
        assert adaptive.step(lhs)
        adaptive.prepare_step(rhs)
        rhs.world.step()
    assert adaptive.is_finite(lhs)
    assert adaptive.is_finite(rhs)
    assert _checksum_values(lhs) == _checksum_values(rhs)
    assert adaptive.position_checksum(lhs) == adaptive.position_checksum(rhs)
    assert adaptive.surface_pose_delta(lhs, rhs) == 0


def test_configure_and_pusher_pose():
    model = adaptive.create_model()
    initial_pose = model.pusher_joint.getPositions().copy()
    adaptive.set_pusher_pose(model, 0.7)
    assert not np.array_equal(initial_pose, model.pusher_joint.getPositions())
    adaptive.configure(model, adaptive_enabled=False, ring_count=2, linger_steps=4)
    assert adaptive.step(model)
    checksum = adaptive.compute_checksum(model)
    assert checksum.active == checksum.total


@pytest.mark.parametrize("with_contacts", [False, True])
def test_point_marker_arrays_match_scene_ranking(with_contacts):
    model = adaptive.create_model()
    if with_contacts:
        # Start against the floor so this also checks the nearest-contact branch.
        model.soft_skeleton.setPosition(5, 0.25)
        assert adaptive.step(model)
        assert model.world.getLastCollisionResult().getNumContacts() > 0

    positions = adaptive.point_positions(model)
    flags = adaptive.active_flags(model)
    checksum = adaptive.compute_checksum(model)
    assert positions.shape == (checksum.total, 3)
    assert positions.dtype == np.dtype(np.float64)
    assert flags.shape == (checksum.total,)
    assert flags.dtype == np.dtype(np.bool_)
    assert np.isfinite(positions).all()
    assert np.count_nonzero(flags) == min(checksum.active, checksum.total)
    assert np.abs(positions).sum() == pytest.approx(checksum.point_world_position_l1)
    assert np.square(positions).sum() == pytest.approx(
        checksum.point_world_position_squared
    )

    contacts = [
        contact.point
        for contact in model.world.getLastCollisionResult().getContacts()
        if np.isfinite(contact.point).all()
    ]
    distances = positions[:, 2].copy()
    if contacts:
        distances = np.full(checksum.total, np.inf)
        for point in contacts:
            distances = np.minimum(distances, np.square(positions - point).sum(axis=1))
    expected = np.zeros(checksum.total, dtype=np.bool_)
    expected[np.argsort(distances, kind="stable")[: checksum.active]] = True
    np.testing.assert_array_equal(flags, expected)

    snapshot = positions.copy()
    del model
    gc.collect()
    np.testing.assert_array_equal(positions, snapshot)


@pytest.mark.parametrize("property_name", ["soft_body", "pusher_joint"])
def test_adaptive_borrowed_child_keeps_model_alive(property_name):
    model = adaptive.create_model()
    existing = (
        model.soft_skeleton.getBodyNode(0)
        if property_name == "soft_body"
        else model.pusher_skeleton.getJoint(0)
    )
    child = getattr(model, property_name)
    assert child is existing
    del existing
    reference = weakref.ref(model)
    del model
    gc.collect()
    assert reference() is not None
    assert child.getName()
    assert adaptive.step(reference())
    del child
    gc.collect()
    assert reference() is None


def test_adaptive_native_graph_preserves_python_shape_until_explicit_cleanup():
    class Shape(dart.dynamics.BoxShape):
        pass

    model = adaptive.create_model()
    shape = Shape([0.1, 0.1, 0.1])
    shape.label = "retained_demo_shape"
    shape.owner = model
    model.soft_body.createShapeNode(shape)
    model_ref, shape_ref = weakref.ref(model), weakref.ref(shape)
    del shape, model
    gc.collect()
    assert model_ref() is not None
    assert shape_ref() is not None
    assert shape_ref().label == "retained_demo_shape"
    assert adaptive.is_finite(model_ref())
    # The World and Model share Skeleton aliases, so GC stays conservative.
    shape_ref().owner = None
    gc.collect()
    assert model_ref() is None
    assert shape_ref() is None
