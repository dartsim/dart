import gc
import weakref

import dartpy as dart
import numpy as np
import pytest


def make_soft_body(properties):
    skeleton = dart.dynamics.Skeleton("soft")
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties("skin")
    )
    joint, body = skeleton.createFreeJointAndSoftBodyNodePair(
        None,
        dart.dynamics.FreeJointProperties(),
        dart.dynamics.SoftBodyNodeProperties(body_properties, properties),
    )
    return skeleton, joint, body


@pytest.fixture
def soft_body():
    return make_soft_body(
        dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
            [0.2, 0.2, 0.2], 3, 2, 0.5
        )
    )


@pytest.mark.parametrize(
    "factory, count",
    [
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
                [0.2, 0.2, 0.1], dart.math.Isometry3(), [4, 4, 4], 0.5
            ),
            56,
        ),
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
                [0.2, 0.2, 0.2], 6, 6, 0.5
            ),
            32,
        ),
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeCylinderProperties(
                0.1, 0.1, 8, 3, 2, 0.5
            ),
            50,
        ),
    ],
)
def test_soft_factories_construct_native_bodies(factory, count):
    properties = factory()
    properties.mKv = 400
    properties.mKe = 100
    properties.mDampCoeff = 3
    skeleton, joint, body = make_soft_body(properties)
    assert isinstance(body, dart.dynamics.SoftBodyNode)
    assert skeleton.getNumSoftBodyNodes() == 1
    assert body.getNumPointMasses() == count
    assert body.getMass() == pytest.approx(0.5 + body.getInertia().getMass())
    assert body.getVertexSpringStiffness() == 400
    assert body.getEdgeSpringStiffness() == 100
    assert body.getDampingCoefficient() == 3
    shape = body.getShapeNode(0)
    assert shape.getShape().getType() == "SoftMeshShape"
    assert shape.hasVisualAspect()
    assert shape.hasCollisionAspect()
    assert shape.hasDynamicsAspect()
    world = dart.simulation.World()
    world.addSkeleton(skeleton)
    world.step()
    assert np.isfinite(skeleton.getPositions()).all()


def test_box_preserves_native_fragment_clamping():
    properties = dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
        [1, 1, 1], dart.math.Isometry3(), [1, 2, 3], 1
    )
    assert make_soft_body(properties)[2].getNumPointMasses() == 26


def test_lowest_safe_ellipsoid_and_cylinder_subdivisions():
    helper = dart.dynamics.SoftBodyNodeHelper
    ellipsoid = helper.makeEllipsoidProperties([1, 1, 1], 3, 2, 1)
    cylinder = helper.makeCylinderProperties(1, 1, 3, 2, 1, 1)
    assert make_soft_body(ellipsoid)[2].getNumPointMasses() == 5
    assert make_soft_body(cylinder)[2].getNumPointMasses() == 11


def test_box_rejects_nonfinite_transform():
    transform = dart.math.Isometry3()
    transform.set_translation([np.nan, 0, 0])
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
            [1, 1, 1], transform, [4, 4, 4], 1
        )


@pytest.mark.parametrize("slices, stacks", [(0, 6), (2, 6), (6, 0), (6, 1), (-1, 6)])
def test_ellipsoid_rejects_unsafe_subdivisions(slices, stacks):
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
            [1, 1, 1], slices, stacks, 1
        )


@pytest.mark.parametrize(
    "slices, stacks, rings",
    [(0, 3, 2), (2, 3, 2), (8, 0, 2), (8, 1, 2), (8, 3, 0), (8, 3, -1)],
)
def test_cylinder_rejects_unsafe_subdivisions(slices, stacks, rings):
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeCylinderProperties(
            1, 1, slices, stacks, rings, 1
        )


@pytest.mark.parametrize("bad", [0, -1, np.nan, np.inf])
def test_soft_factories_reject_invalid_dimensions_and_mass(bad):
    helper = dart.dynamics.SoftBodyNodeHelper
    for factory in (
        lambda: helper.makeBoxProperties(
            [bad, 1, 1], dart.math.Isometry3(), [4, 4, 4], 1
        ),
        lambda: helper.makeEllipsoidProperties([bad, 1, 1], 6, 6, 1),
        lambda: helper.makeCylinderProperties(bad, 1, 8, 3, 2, 1),
        lambda: helper.makeCylinderProperties(1, bad, 8, 3, 2, 1),
        lambda: helper.makeBoxProperties(
            [1, 1, 1], dart.math.Isometry3(), [4, 4, 4], bad
        ),
        lambda: helper.makeEllipsoidProperties([1, 1, 1], 6, 6, bad),
        lambda: helper.makeCylinderProperties(1, 1, 8, 3, 2, bad),
    ):
        with pytest.raises(ValueError):
            factory()


def test_point_mass_lookup_checks_indexes_and_preserves_identity(soft_body):
    body = soft_body[2]
    point = body.get_point_mass(0)
    assert isinstance(point, dart.dynamics.PointMass)
    assert body.get_point_mass(0) is point
    assert body.get_point_mass(body.getNumPointMasses() - 1) is not point
    for index in (-1, body.getNumPointMasses(), body.getNumPointMasses() + 1):
        with pytest.raises(IndexError, match="point mass index"):
            body.get_point_mass(index)


def test_point_mass_state_copies_and_world_position_update(soft_body):
    point = soft_body[2].get_point_mass(0)
    old_position = point.positions
    old_world_position = point.world_position
    displacement = np.array([0.01, -0.02, 0.03])
    velocity = np.array([-0.04, 0.05, 0.06])
    point.positions = displacement
    point.velocities = velocity
    np.testing.assert_array_equal(point.positions, displacement)
    np.testing.assert_array_equal(point.velocities, velocity)
    np.testing.assert_array_equal(old_position, [0, 0, 0])
    np.testing.assert_allclose(point.world_position, old_world_position + displacement)
    for name in (
        "positions",
        "velocities",
        "world_position",
        "world_velocity",
        "rest_position",
        "forces",
    ):
        array = getattr(point, name)
        assert array.shape == (3,)
        assert not array.flags.writeable
        assert not np.shares_memory(array, getattr(point, name))
        with pytest.raises(ValueError):
            array[0] = 1


def test_soft_generator_metadata_matches_native_point_and_face_order():
    properties = dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
        [0.123456789012345, 0.25, 0.5], dart.math.Isometry3(), [4, 4, 4], 0.5
    )
    skeleton, joint, body = make_soft_body(properties)
    positions = properties.point_rest_positions
    masses = properties.point_masses
    faces = properties.faces
    assert positions.dtype == np.float64
    assert positions.shape == (body.getNumPointMasses(), 3)
    assert faces.shape == (body.num_faces, 3)
    for index, (position, mass) in enumerate(zip(positions, masses)):
        point = body.get_point_mass(index)
        np.testing.assert_array_equal(point.rest_position, position)
        assert point.mass == mass
    for index, face in enumerate(faces):
        np.testing.assert_array_equal(body.get_face(index), face)
    for index in (-1, body.num_faces, body.num_faces + 1):
        with pytest.raises(IndexError, match="face index"):
            body.get_face(index)
    saved = (positions.copy(), masses.copy(), faces.copy())
    positions[:] = 99
    masses[:] = 99
    faces[:] = 0
    body.get_face(0)[:] = 0
    for actual, expected in zip(
        (properties.point_rest_positions, properties.point_masses, properties.faces),
        saved,
    ):
        np.testing.assert_array_equal(actual, expected)
    copies = (
        properties.point_rest_positions,
        properties.point_masses,
        properties.faces,
    )
    body_reference = weakref.ref(body)
    del properties, skeleton, joint, body, point
    gc.collect()
    assert body_reference() is None
    for actual, expected in zip(copies, saved):
        np.testing.assert_array_equal(actual, expected)


def test_point_mass_mass_updates_soft_total_and_rejects_invalid_mass(soft_body):
    _, _, body = soft_body
    point = body.get_point_mass(0)
    original_point_mass = point.mass
    original_body_mass = body.getMass()
    point.mass = 2 * original_point_mass
    assert body.getMass() == pytest.approx(original_body_mass + original_point_mass)
    for value in (0, -1, np.nan, np.inf):
        with pytest.raises(ValueError, match="mass must be finite and positive"):
            point.mass = value
        assert point.mass == 2 * original_point_mass


def test_point_mass_world_velocity_and_resets_preserve_other_state(soft_body):
    skeleton, _, body = soft_body
    point = body.get_point_mass(0)
    rest_position = point.rest_position
    point.positions = [0.01, 0.02, 0.03]
    point.velocities = [0.04, 0.05, 0.06]
    np.testing.assert_array_equal(point.world_velocity, point.velocities)
    point.reset_positions()
    np.testing.assert_array_equal(point.positions, [0, 0, 0])
    np.testing.assert_array_equal(point.rest_position, rest_position)
    np.testing.assert_array_equal(point.velocities, [0.04, 0.05, 0.06])
    world = dart.simulation.World()
    world.addSkeleton(skeleton)
    world.step()
    for field in ("velocities", "accelerations", "forces"):
        before = {
            name: getattr(point, name)
            for name in ("positions", "velocities", "accelerations", "forces")
        }
        getattr(point, "reset_" + field)()
        np.testing.assert_array_equal(getattr(point, field), [0, 0, 0])
        for name, saved in before.items():
            if name != field:
                np.testing.assert_array_equal(getattr(point, name), saved)


def test_skeleton_clears_native_point_collision_flags(soft_body):
    skeleton, _, body = soft_body
    point = body.get_point_mass(0)
    assert not body.is_colliding
    assert not point.is_colliding
    point.is_colliding = True
    assert point.is_colliding
    with pytest.raises(TypeError):
        point.is_colliding = 1
    assert point.is_colliding
    skeleton.clear_colliding_bodies()
    assert not point.is_colliding
    assert not body.is_colliding


@pytest.mark.parametrize("name", ["positions", "velocities"])
def test_point_mass_state_setters_validate_shape_and_preserve_nonfinite_state(
    soft_body, name
):
    point = soft_body[2].get_point_mass(0)
    with pytest.raises(TypeError):
        setattr(point, name, [1, 2])
    value = np.array([np.nan, np.inf, -np.inf])
    setattr(point, name, value)
    np.testing.assert_array_equal(getattr(point, name), value)


def test_revolute_soft_body_factory_creates_attached_native_link(soft_body):
    skeleton, _, parent = soft_body
    properties = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
        [0.2, 0.2, 0.2], 3, 2, 0.5
    )
    joint, body = skeleton.create_revolute_joint_and_soft_body_node_pair(
        parent=parent,
        joint_properties=dart.dynamics.RevoluteJointProperties(),
        body_properties=dart.dynamics.SoftBodyNodeProperties(
            dart.dynamics.BodyNodeProperties(), properties
        ),
    )
    assert isinstance(joint, dart.dynamics.RevoluteJoint)
    assert isinstance(body, dart.dynamics.SoftBodyNode)
    assert body.getParentBodyNode() is parent
    assert skeleton.getNumSoftBodyNodes() == 2
    assert body.getNumPointMasses() == 5
    joint.setPosition(0, 0.2)
    assert skeleton.getPosition(6) == pytest.approx(0.2)


def test_adaptive_properties_freeze_and_reactivate_points(soft_body):
    skeleton, _, body = soft_body
    world = dart.simulation.World()
    world.setGravity([0, 0, 0])
    world.addSkeleton(skeleton)
    assert not body.adaptive_contact_activation_enabled
    assert body.num_active_point_masses == body.getNumPointMasses()
    body.adaptive_contact_activation_ring_count = 0
    body.adaptive_contact_activation_linger_steps = 0
    body.adaptive_contact_activation_velocity_tolerance = np.inf
    body.adaptive_contact_activation_position_tolerance = np.inf
    body.adaptive_contact_activation_enabled = True
    assert body.adaptive_contact_activation_enabled
    assert body.adaptive_contact_activation_ring_count == 0
    assert body.adaptive_contact_activation_linger_steps == 0
    assert body.adaptive_contact_activation_velocity_tolerance == np.inf
    assert body.adaptive_contact_activation_position_tolerance == np.inf
    for _ in range(3):
        world.step()
    assert body.num_active_point_masses == 0
    body.adaptive_contact_activation_enabled = False
    assert body.num_active_point_masses == body.getNumPointMasses()
    body.adaptive_contact_activation_enabled = True
    assert body.num_active_point_masses == body.getNumPointMasses()


@pytest.mark.parametrize(
    "name",
    [
        "adaptive_contact_activation_velocity_tolerance",
        "adaptive_contact_activation_position_tolerance",
    ],
)
@pytest.mark.parametrize("bad", [-1, np.nan, -np.inf])
def test_adaptive_tolerance_rejects_invalid_value_without_mutating_state(
    soft_body, name, bad
):
    body = soft_body[2]
    setattr(body, name, 0.1)
    with pytest.raises(ValueError, match="tolerance must be non-negative"):
        setattr(body, name, bad)
    assert getattr(body, name) == 0.1


@pytest.mark.parametrize(
    "name",
    [
        "adaptive_contact_activation_ring_count",
        "adaptive_contact_activation_linger_steps",
    ],
)
def test_adaptive_count_rejects_negative_value_without_unsigned_wrap(soft_body, name):
    body = soft_body[2]
    setattr(body, name, 2)
    with pytest.raises(TypeError):
        setattr(body, name, -1)
    assert getattr(body, name) == 2
