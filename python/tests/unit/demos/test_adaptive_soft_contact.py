"""Adaptive state, activation estimates and the paired simulation contract."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models import adaptive_soft_contact as adaptive

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

    positions[0] = 123
    flags[0] = not flags[0]
    np.testing.assert_array_equal(adaptive.point_positions(model)[1:], positions[1:])
    assert not np.array_equal(adaptive.point_positions(model)[0], positions[0])
    np.testing.assert_array_equal(adaptive.active_flags(model), expected)
    snapshot = positions.copy()
    flag_snapshot = flags.copy()
    del model
    gc.collect()
    np.testing.assert_array_equal(positions, snapshot)
    np.testing.assert_array_equal(flags, flag_snapshot)


@pytest.mark.parametrize("property_name", ["soft_body", "pusher_joint"])
def test_adaptive_borrowed_child_keeps_native_skeleton_alive(property_name):
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
    # Core graph ownership retains resources without pinning the Python helper.
    assert reference() is None
    assert child.getName()
    skeleton = child.getSkeleton()
    skeleton_reference = weakref.ref(skeleton)
    assert skeleton.getName() == (
        "adaptive_ellipsoid" if property_name == "soft_body" else "periodic_pusher"
    )
    state = (
        child.getWorldTransform().matrix()
        if property_name == "soft_body"
        else child.getPositions()
    )
    assert np.isfinite(state).all()
    assert skeleton.getNumBodyNodes() == 1
    del child, skeleton
    gc.collect()
    assert skeleton_reference() is None


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


def _comparison():
    models = [
        adaptive.create_model(adaptive_enabled=enabled) for enabled in (True, False)
    ]
    for model in models:
        model.world.setNumSimulationThreads(1)
    total = models[0].soft_body.getNumPointMasses()
    reduced = adaptive_contacts = all_active_contacts = 0
    maximum_delta = 0.0
    checkpoints = []
    for index in range(1, 2001):
        assert adaptive.step(models[0]), index
        assert adaptive.step(models[1]), index
        active = models[0].soft_body.num_active_point_masses
        reduced += active < total
        adaptive_contacts += (
            models[0].world.getLastCollisionResult().getNumContacts() > 0
        )
        all_active_contacts += (
            models[1].world.getLastCollisionResult().getNumContacts() > 0
        )
        delta = adaptive.surface_pose_delta(*models)
        maximum_delta = max(maximum_delta, delta)
        if index % 250 == 0:
            left, right = (adaptive.compute_checksum(model) for model in models)
            checkpoints.append(
                (
                    index,
                    left.active,
                    left.contacts,
                    right.active,
                    right.contacts,
                    adaptive.position_checksum(models[0]),
                    adaptive.position_checksum(models[1]),
                    delta,
                )
            )
    assert total == 86
    assert reduced > 1000
    assert adaptive_contacts > 0 and all_active_contacts > 0
    final_active = models[0].soft_body.num_active_point_masses
    assert final_active * 4 <= total * 3
    assert maximum_delta <= adaptive.compare_tolerance
    assert all(checkpoint[3] == total for checkpoint in checkpoints)
    return (
        total,
        reduced,
        adaptive_contacts,
        all_active_contacts,
        final_active,
        maximum_delta,
        adaptive.position_checksum(models[0]),
        adaptive.position_checksum(models[1]),
        tuple(checkpoints),
    )


def test_preserves_full_2000_step_adaptive_comparison_and_exact_repeat():
    assert _comparison() == _comparison()


def test_partial_activation_estimate_matches_public_count():
    model = adaptive.create_model()
    total = model.soft_body.getNumPointMasses()
    for _ in range(5000):
        active = model.soft_body.num_active_point_masses
        if 0 < active < total:
            break
        assert adaptive.step(model)
    assert 0 < active < total
    flags = adaptive.active_flags(model)
    assert len(flags) == total
    assert np.count_nonzero(flags) == active


def test_marker_estimate_orders_ties_and_nonfinite_points_last(monkeypatch):
    model = adaptive.create_model()
    adaptive.configure(model, adaptive_enabled=False, ring_count=1, linger_steps=12)
    positions = np.tile([0.0, 0.0, 1.0], (86, 1))
    positions[0] = np.nan
    positions[1] = [0.0, 0.0, -1.0]
    positions[2] = np.inf
    monkeypatch.setattr(adaptive, "point_positions", lambda _model: positions.copy())
    # Use a smaller public active count while retaining deterministic positions.
    monkeypatch.setattr(
        type(model.soft_body), "num_active_point_masses", property(lambda self: 3)
    )
    flags = adaptive.active_flags(model)
    np.testing.assert_array_equal(np.flatnonzero(flags), [1, 3, 4])


@pytest.mark.parametrize("name", ("ring_count", "linger_steps"))
@pytest.mark.parametrize("value", (-1, 0.5, "1", None, 10**400))
def test_activation_counts_require_unsigned_integers(name, value):
    with pytest.raises(TypeError):
        adaptive.create_model(**{name: value})
    model = adaptive.create_model()
    settings = {"adaptive_enabled": True, "ring_count": 1, "linger_steps": 12}
    settings[name] = value
    with pytest.raises(TypeError):
        adaptive.configure(model, **settings)


@pytest.mark.parametrize("value", (1, None, np.bool_(True), "true"))
def test_adaptive_enabled_requires_a_python_bool(value):
    with pytest.raises(TypeError):
        adaptive.create_model(adaptive_enabled=value)


@pytest.mark.parametrize("time", (np.nan, np.inf, -np.inf))
def test_nonfinite_pusher_time_preserves_native_nonfinite_pose(time):
    model = adaptive.create_model()
    adaptive.set_pusher_pose(model, time)
    assert not np.isfinite(model.pusher_joint.getPositions()).all()


class _StoredFloat(float):
    def __float__(self):
        return 0.7


class _RaisingFloat:
    def __float__(self):
        raise RuntimeError("conversion failed")


class _NumericFloat:
    def __float__(self):
        return 0.125


class _NumericIndex:
    def __index__(self):
        return 1


@pytest.mark.parametrize(
    "time",
    [
        None,
        "0.1",
        b"0.1",
        bytearray(b"0.1"),
        memoryview(b"0.1"),
        object(),
        10**500,
        _RaisingFloat(),
    ],
)
def test_pusher_rejects_text_buffers_and_failed_numeric_conversion(time):
    model = adaptive.create_model()
    before = model.pusher_joint.getPositions().copy()
    with pytest.raises(TypeError, match="time must be"):
        adaptive.set_pusher_pose(model, time)
    np.testing.assert_array_equal(model.pusher_joint.getPositions(), before)


@pytest.mark.parametrize(
    "time, expected",
    [
        (_StoredFloat(0.125), 0.125),
        (_NumericFloat(), 0.125),
        (_NumericIndex(), 1.0),
        (True, 1.0),
        (np.float32(0.125), 0.125),
        (np.uint64(1), 1.0),
    ],
)
def test_pusher_numeric_protocol_and_stored_float_value(time, expected):
    model = adaptive.create_model()
    adaptive.set_pusher_pose(model, expected)
    pose = model.pusher_joint.getPositions().copy()
    adaptive.set_pusher_pose(model, time)
    np.testing.assert_array_equal(model.pusher_joint.getPositions(), pose)


@pytest.mark.parametrize("field", ("positions", "velocities"))
def test_nonfinite_soft_point_is_detected_independently_of_skeleton(field):
    model = adaptive.create_model()
    point = model.soft_body.get_point_mass(0)
    setattr(point, field, [np.nan, 0, 0])
    assert np.isfinite(model.soft_skeleton.getPositions()).all()
    assert np.isfinite(model.soft_skeleton.getVelocities()).all()
    assert not adaptive.is_finite(model)


@pytest.mark.parametrize(
    "function",
    (
        adaptive.prepare_step,
        adaptive.step,
        adaptive.compute_checksum,
        adaptive.position_checksum,
        adaptive.is_finite,
        adaptive.point_positions,
        adaptive.active_flags,
    ),
)
def test_model_functions_reject_missing_owner(function):
    with pytest.raises(TypeError, match="model must be a Model"):
        function(None)


@pytest.mark.parametrize(
    "error", [ValueError, OverflowError, KeyboardInterrupt, SystemExit, GeneratorExit]
)
@pytest.mark.parametrize("field", ["time", "ring_count", "linger_steps"])
def test_numeric_protocol_errors_are_cleared_before_model_mutation(error, field):
    class Number:
        def __float__(self):
            raise error

        def __index__(self):
            raise error

    model = adaptive.create_model()
    body = model.soft_body
    before = model.pusher_joint.getPositions().copy()
    settings = (
        body.adaptive_contact_activation_ring_count,
        body.adaptive_contact_activation_linger_steps,
        body.adaptive_contact_activation_enabled,
    )
    with pytest.raises(TypeError):
        if field == "time":
            adaptive.set_pusher_pose(model, Number())
        else:
            kwargs = dict(adaptive_enabled=False, ring_count=2, linger_steps=3)
            kwargs[field] = Number()
            adaptive.configure(model, **kwargs)
    np.testing.assert_array_equal(model.pusher_joint.getPositions(), before)
    assert (
        body.adaptive_contact_activation_ring_count,
        body.adaptive_contact_activation_linger_steps,
        body.adaptive_contact_activation_enabled,
    ) == settings


def test_adaptive_checks_actual_types_instead_of_spoofed_class_attributes():
    class Spoof:
        def __init__(self, kind):
            self.kind = kind

        @property
        def __class__(self):
            return self.kind

        def __float__(self):
            return 0.125

        def __bool__(self):
            return True

    model = adaptive.create_model()
    with pytest.raises(TypeError):
        adaptive.set_pusher_pose(Spoof(adaptive.Model), 0.0)
    body = model.soft_body
    settings = (
        body.adaptive_contact_activation_ring_count,
        body.adaptive_contact_activation_linger_steps,
        body.adaptive_contact_activation_enabled,
    )
    with pytest.raises(TypeError):
        adaptive.configure(
            model, adaptive_enabled=Spoof(bool), ring_count=2, linger_steps=3
        )
    assert (
        body.adaptive_contact_activation_ring_count,
        body.adaptive_contact_activation_linger_steps,
        body.adaptive_contact_activation_enabled,
    ) == settings
    adaptive.set_pusher_pose(model, 0.125)
    expected = model.pusher_joint.getPositions().copy()
    adaptive.set_pusher_pose(model, Spoof(float))
    np.testing.assert_array_equal(model.pusher_joint.getPositions(), expected)


@pytest.mark.parametrize("ring_count", [-1, int(np.iinfo(np.uintp).max) + 1])
@pytest.mark.parametrize("create", [False, True])
def test_invalid_ring_count_rejects_before_converting_linger(ring_count, create):
    calls = []

    class Linger:
        def __index__(self):
            calls.append("linger")
            return 3

    kwargs = dict(adaptive_enabled=True, ring_count=ring_count, linger_steps=Linger())
    with pytest.raises(TypeError):
        if create:
            adaptive.create_model(**kwargs)
        else:
            adaptive.configure(adaptive.create_model(), **kwargs)
    assert calls == []
