"""Python point sampling, sensor state and retained-owner boundary checks."""

import gc
import math
import weakref
from decimal import Decimal
from fractions import Fraction

import dartpy as dart
import numpy as np
import pytest

from examples.demos.models.point_cloud import PointCloudSensor as Sensor

pytestmark = pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="OctoMap is unavailable"
)


def _owners():
    return (
        dart.dynamics.Skeleton(),
        dart.dynamics.SimpleFrame(dart.dynamics.Frame.World()),
        dart.dynamics.PointCloudShape(),
        dart.dynamics.VoxelGridShape(0.05),
    )


def test_box_sampling_repeats_dart_random_seed_and_updates_voxel_grid():
    clouds = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        owners = _owners()
        sensor = Sensor(*owners)
        assert sensor.updating and sensor.sampling_mode == 0
        sensor.sampling_mode = 1
        sensor.update(0.02)
        _, frame, points, voxels = owners
        assert points.getNumPoints() == 500
        cloud = np.asarray(points.getPoints())
        assert np.isfinite(cloud).all() and np.max(np.abs(cloud)) <= 0.5
        colors = np.asarray(points.getColors())
        assert colors.shape == (500, 4)
        assert colors[:, :3].min() >= 0.1 - 1e-7
        assert colors[:, :3].max() <= 0.9 + 1e-7
        np.testing.assert_array_equal(colors[:, 3], 0.75)
        assert any(voxels.getOccupancy(point) > 0.5 for point in cloud)
        np.testing.assert_array_equal(
            frame.getWorldTransform().translation(), [0, 1, 0.5]
        )
        clouds.append(cloud.copy())
        sensor.updating = False
        sensor.update(0.4)
        np.testing.assert_array_equal(points.getPoints(), cloud)
        sensor.updating = True
        sensor.update(0.02)
        np.testing.assert_allclose(
            frame.getWorldTransform().translation(),
            [np.sin(0.02), np.cos(0.02), 0.5 + 0.25 * np.sin(0.04)],
        )
    np.testing.assert_array_equal(*clouds)


def test_point_shape_property_covers_every_cpp_panel_mode():
    owners = _owners()
    sensor = Sensor(*owners)
    for index in range(4):
        sensor.point_shape_type = index
        assert sensor.point_shape_type == index
        assert int(owners[2].getPointShapeType()) == index
    for property_name, invalid in (
        ("sampling_mode", -1),
        ("sampling_mode", 2),
        ("point_shape_type", -1),
        ("point_shape_type", 4),
    ):
        before = getattr(sensor, property_name)
        with pytest.raises(ValueError):
            setattr(sensor, property_name, invalid)
        assert getattr(sensor, property_name) == before
    with pytest.raises(TypeError):
        sensor.updating = 1


@pytest.mark.parametrize("index", range(4))
def test_sensor_rejects_null_and_wrong_owner_types(index):
    owners = list(_owners())
    owners[index] = None
    with pytest.raises(ValueError):
        Sensor(*owners)
    owners[index] = object()
    with pytest.raises(TypeError):
        Sensor(*owners)


@pytest.mark.parametrize("index", range(4))
def test_sensor_rejects_spoofed_owner_class(index):
    owners = list(_owners())
    expected = type(owners[index])

    class Masquerade:
        @property
        def __class__(self):
            return expected

    owners[index] = Masquerade()
    with pytest.raises(TypeError):
        Sensor(*owners)


@pytest.mark.parametrize("time_step", [0, -0.1, np.inf, np.nan])
@pytest.mark.parametrize("updating", [True, False])
def test_sensor_rejects_invalid_timestep_without_mutation(time_step, updating):
    owners = _owners()
    sensor = Sensor(*owners)
    sensor.updating = updating
    with pytest.raises(ValueError):
        sensor.update(time_step)
    assert owners[2].getNumPoints() == 0
    np.testing.assert_array_equal(
        owners[1].getWorldTransform().translation(), [0, 0, 0]
    )


def test_sensor_retains_wrappers_and_releases_them_after_deletion():
    owners = _owners()
    references = tuple(weakref.ref(owner) for owner in owners)
    sensor = Sensor(*owners)
    del owners
    gc.collect()
    assert all(reference() is not None for reference in references)
    sensor.sampling_mode = 1
    sensor.update(0.001)
    del sensor
    gc.collect()
    assert all(reference() is None for reference in references)


@pytest.mark.parametrize("index", range(4))
def test_sensor_owner_back_reference_cycles_collect(index):
    base_types = (
        dart.dynamics.Skeleton,
        dart.dynamics.SimpleFrame,
        dart.dynamics.PointCloudShape,
        dart.dynamics.VoxelGridShape,
    )

    class Owner(base_types[index]):
        pass

    owners = list(_owners())
    owners[index] = Owner()
    sensor = Sensor(*owners)
    owners[index].sensor = sensor
    references = weakref.ref(sensor), weakref.ref(owners[index])
    del sensor, owners
    gc.collect()
    collected = all(reference() is None for reference in references)
    if references[1]() is not None:
        references[1]().sensor = None
    gc.collect()
    assert collected, "Sensor's retained owner edge was invisible to GC"


def test_sensor_rejects_unaddressable_origin_before_point_or_voxel_update():
    owners = list(_owners())
    owners[3] = dart.dynamics.VoxelGridShape(1e-6)
    sensor = Sensor(*owners)
    sensor.sampling_mode = 1
    with pytest.raises(ValueError, match="outside the voxel grid"):
        sensor.update(0.001)
    assert owners[2].getNumPoints() == 0
    np.testing.assert_array_equal(
        owners[1].getWorldTransform().translation(), [0, 0, 0]
    )
    assert owners[3].getOccupancy([0, 0, 0]) == 0


def _mesh_robot(tmp_path, *, negative=False, flat=False, multiple=False):
    vertices = np.array(
        [[0.117, 0.123, 0.147], [0.237, 0.231, 0.253], [0.347, 0.343, 0.359]]
    )
    if negative:
        vertices[:, 2] *= -1
    if flat:
        vertices[:, 2] = 0.147
    obj = "o first\n" + "".join(f"v {x} {y} {z}\n" for x, y, z in vertices)
    if multiple:
        extra = np.array(
            [
                [-0.119, 0.217, -0.137],
                [-0.219, 0.317, -0.237],
                [-0.319, 0.417, -0.337],
                [-0.419, 0.517, -0.437],
            ]
        )
        (tmp_path / "vertices.mtl").write_text(
            "newmtl first\nKd 0.2 0.3 0.4\nnewmtl second\nKd 0.7 0.6 0.5\n"
        )
        obj = "mtllib vertices.mtl\n" + obj + "usemtl first\nf 1 2 3\no second\n"
        obj += "".join(f"v {x} {y} {z}\n" for x, y, z in extra)
        obj += "usemtl second\nf 4 5 6\nf 4 6 7\n"
        vertices = np.concatenate((vertices, extra))
    else:
        obj += "f 1 2 3\n"
    (tmp_path / "vertices.obj").write_text(obj)
    urdf = tmp_path / "fixture.urdf"
    urdf.write_text(
        '<robot name="fixture"><link name="base"><visual><geometry><mesh '
        f'filename="{tmp_path / "vertices.obj"}" scale="2 3 4"/>'
        "</geometry></visual></link></robot>"
    )
    loaded = dart.utils.DartLoader().parseSkeleton(str(urdf))
    mesh = loaded.getBodyNode(0).getShapeNode(0).getShape()
    owners = list(_owners())
    _, body = owners[0].createRevoluteJointAndBodyNodePair()
    nodes = []
    translations = ([0, 0, 0], [0.1, 0.1, 0])
    if not (negative or flat):
        translations = ([0.414, 0.028, 0.731], [-0.31, 0.17, 0.12])
    for translation in translations:
        node = body.createShapeNode(mesh)
        node.createVisualAspect()
        node.setRelativeTranslation(translation)
        nodes.append(node)
    body.createShapeNode(dart.dynamics.BoxShape([0.2] * 3)).createVisualAspect()
    _, invisible = owners[0].createWeldJointAndBodyNodePair()
    invisible.createShapeNode(mesh)
    return owners, nodes, vertices.astype(np.float32).astype(float)


def _state(owners):
    robot, frame, points, grid = owners
    return (
        robot.getPositions().copy(),
        frame.getWorldTransform().matrix().copy(),
        np.array(points.getPoints()),
        np.array(points.getColors()),
        grid.getOccupancy([0, 0, 0]),
    )


def test_mesh_sampling_uses_every_raw_unscaled_submesh_vertex(tmp_path):
    owners, nodes, vertices = _mesh_robot(tmp_path, multiple=True)
    dart.math.Random.setSeed(42)
    Sensor(*owners).update(0.02)
    cloud = np.array(owners[2].getPoints())
    candidates = np.concatenate(
        [
            vertices @ node.getWorldTransform().rotation().T
            + node.getWorldTransform().translation()
            for node in nodes
        ]
    )
    assert cloud.shape == (500, 3)
    assert len(np.unique(cloud, axis=0)) == 14
    distances = np.linalg.norm(cloud[:, None, :] - candidates[None, :, :], axis=2)
    assert np.max(distances.min(axis=1)) < 1e-7
    assert all(np.any(distances[:, index] < 1e-7) for index in range(len(candidates)))


@pytest.mark.parametrize("flat", [False, True])
def test_height_colors_preserve_negative_and_flat_native_quirks(tmp_path, flat):
    owners, _, _ = _mesh_robot(tmp_path, negative=not flat, flat=flat)
    dart.math.Random.setSeed(42)
    Sensor(*owners).update(0.02)
    colors = np.array(owners[2].getColors())
    if flat:
        np.testing.assert_array_equal(
            colors, np.tile([float(np.float32(0.1))] * 3 + [0.75], (500, 1))
        )
    else:
        assert colors[:, 0].max() < 0.6
        assert colors[:, 2].min() > 0.4
        np.testing.assert_array_equal(colors[:, 3], 0.75)


@pytest.mark.parametrize("visual", [False, True])
def test_non_mesh_robot_falls_back_after_bounded_attempts(monkeypatch, visual):
    owners = _owners()
    _, body = owners[0].createRevoluteJointAndBodyNodePair()
    node = body.createShapeNode(dart.dynamics.BoxShape([1] * 3))
    if visual:
        node.createVisualAspect()
    calls = 0
    uniform_int = dart.math.Random.uniform_int

    def count_draws(lower, upper):
        nonlocal calls
        calls += 1
        return uniform_int(lower, upper)

    monkeypatch.setattr(dart.math.Random, "uniform_int", count_draws)
    dart.math.Random.setSeed(42)
    Sensor(*owners).update(0.02)
    assert calls == 200000 * (2 if visual else 1)
    cloud = np.array(owners[2].getPoints())
    assert cloud.shape == (500, 3) and np.max(np.abs(cloud)) <= 0.5


def test_box_shape_spoofing_mesh_class_uses_box_fallback(monkeypatch):
    from examples.demos.models import point_cloud as model

    class MeshSpoofingBox(dart.dynamics.BoxShape):
        @property
        def __class__(self):
            return dart.dynamics.MeshShape

    owners = _owners()
    _, body = owners[0].createRevoluteJointAndBodyNodePair()
    body.createShapeNode(MeshSpoofingBox([1] * 3)).createVisualAspect()
    # The separate cap regression covers 200000 attempts; one suffices here.
    monkeypatch.setattr(model, "_MAX_ATTEMPTS", 1)
    Sensor(*owners).update(0.02)
    cloud = np.array(owners[2].getPoints())
    assert cloud.shape == (500, 3) and np.max(np.abs(cloud)) <= 0.5


@pytest.mark.parametrize(
    "resolution,translation,message",
    [
        (1e-6, [0.414, 0.028, 0.731], "outside the voxel grid"),
        (0.05, [1e6, 1e6, 1e6], "outside the voxel grid"),
        (3.1e-5, [-1.1, -1.1, -1.1], "too many voxels"),
        (0.05, [np.nan, 0, 0.5], "finite"),
    ],
)
def test_preflight_preserves_cloud_colors_frame_voxels_after_joint_motion(
    tmp_path, resolution, translation, message
):
    owners, nodes, _ = _mesh_robot(tmp_path)
    owners[3] = dart.dynamics.VoxelGridShape(resolution)
    for node in nodes:
        node.setRelativeTranslation(translation)
    owners[2].setPoint([[0, 0, 0]])
    owners[2].setColors([[0.2, 0.3, 0.4, 0.75]])
    owners[3].updateOccupancy([0, 0, 0], True)
    sensor = Sensor(*owners)
    before = _state(owners)
    dart.math.Random.setSeed(42)
    with pytest.raises(ValueError, match=message):
        sensor.update(0.02)
    after = _state(owners)
    assert not np.array_equal(before[0], after[0])
    for old, new in zip(before[1:], after[1:]):
        np.testing.assert_array_equal(old, new)
    if resolution != 1e-6:
        for node in nodes:
            node.setRelativeTranslation([0, 0.45, 0.5])
        sensor.update(0.02)
        np.testing.assert_array_equal(
            owners[1].getWorldTransform().translation(), [0, 1, 0.5]
        )


class _Number:
    def __float__(self):
        return 0.123


class _Index:
    def __index__(self):
        return 1


class _FloatSubclass(float):
    def __float__(self):
        return 0.999


class _RaisingNumber:
    def __float__(self):
        raise RuntimeError("conversion failed")


def test_spoofed_bool_class_is_rejected_by_strict_bool_property():
    class FakeBool:
        @property
        def __class__(self):
            return bool

        def __bool__(self):
            return True

    sensor = Sensor(*_owners())
    with pytest.raises(TypeError):
        sensor.updating = FakeBool()
    assert sensor.updating is True


def test_spoofed_float_class_still_uses_its_numeric_protocol():
    class FakeFloat:
        @property
        def __class__(self):
            return float

        def __float__(self):
            return 0.123

    owners = _owners()
    sensor = Sensor(*owners)
    sensor.update(FakeFloat())
    sensor.update(0.02)
    np.testing.assert_array_equal(
        owners[1].getWorldTransform().translation(),
        [math.sin(0.123), math.cos(0.123), 0.5 + 0.25 * math.sin(0.246)],
    )


def test_spoofed_numpy_bool_class_still_uses_its_index_protocol():
    class FakeNumpyBool:
        @property
        def __class__(self):
            return np.bool_

        def __index__(self):
            return 1

    sensor = Sensor(*_owners())
    sensor.sampling_mode = FakeNumpyBool()
    sensor.point_shape_type = FakeNumpyBool()
    assert sensor.sampling_mode == 1 and sensor.point_shape_type == 1


@pytest.mark.parametrize("exception", [KeyboardInterrupt, SystemExit])
@pytest.mark.parametrize("protocol", ["__float__", "__index__"])
@pytest.mark.parametrize("operation", ["update", "sampling_mode", "point_shape_type"])
def test_numeric_protocol_interruptions_become_type_errors(
    exception, protocol, operation
):
    def fail(self):
        raise exception("conversion interrupted")

    number = type("InterruptedNumber", (), {protocol: fail})()
    owners = _owners()
    sensor = Sensor(*owners)
    before = _state(owners)
    with pytest.raises(TypeError):
        if operation == "update":
            sensor.update(number)
        else:
            setattr(sensor, operation, number)
    assert sensor.sampling_mode == 0 and sensor.point_shape_type == 0
    for old, new in zip(before, _state(owners)):
        np.testing.assert_array_equal(old, new)


@pytest.mark.parametrize(
    "value,expected",
    [
        (0.123, 0.123),
        (np.float32(0.123), float(np.float32(0.123))),
        (Decimal("0.123"), 0.123),
        (Fraction(1, 8), 0.125),
        (_Number(), 0.123),
        (_Index(), 1),
        (_FloatSubclass(0.123), 0.123),
    ],
)
def test_time_numeric_protocol_and_float_subclass_stored_value(value, expected):
    owners = _owners()
    sensor = Sensor(*owners)
    sensor.update(value)
    sensor.update(0.02)
    np.testing.assert_array_equal(
        owners[1].getWorldTransform().translation(),
        [math.sin(expected), math.cos(expected), 0.5 + 0.25 * math.sin(expected * 2)],
    )


@pytest.mark.parametrize(
    "value",
    [
        "0.123",
        b"0.123",
        bytearray(b"0.123"),
        memoryview(b"0.123"),
        None,
        object(),
        10**500,
        _RaisingNumber(),
    ],
)
@pytest.mark.parametrize("updating", [True, False])
def test_time_protocol_failures_are_type_errors_without_mutation(value, updating):
    owners = _owners()
    sensor = Sensor(*owners)
    sensor.updating = updating
    before = _state(owners)
    with pytest.raises(TypeError):
        sensor.update(value)
    for old, new in zip(before, _state(owners)):
        np.testing.assert_array_equal(old, new)


@pytest.mark.parametrize("property_name", ["sampling_mode", "point_shape_type"])
def test_integer_properties_accept_index_protocol_and_preserve_invalid_state(
    property_name,
):
    sensor = Sensor(*_owners())
    for value in (True, np.int64(1), np.uint64(1), np.array(1), _Index()):
        setattr(sensor, property_name, value)
        assert getattr(sensor, property_name) == 1
    for value in (1.0, np.bool_(True), "1", None, 2**31, -(2**31) - 1, 10**500):
        with pytest.raises(TypeError):
            setattr(sensor, property_name, value)
        assert getattr(sensor, property_name) == 1


def test_finite_timestep_with_overflowed_orbit_phase_rejects_next_update():
    owners = _owners()
    sensor = Sensor(*owners)
    sensor.update(1e308)
    before = _state(owners)
    with pytest.raises(ValueError, match="finite"):
        sensor.update(0.02)
    for old, new in zip(before, _state(owners)):
        np.testing.assert_array_equal(old, new)
