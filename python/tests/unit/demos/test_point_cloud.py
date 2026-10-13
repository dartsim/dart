"""Native point sampling, sensor state and retained-owner boundary checks."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest

Sensor = getattr(dart._demos, "PointCloudSensor", None)
pytestmark = pytest.mark.skipif(Sensor is None, reason="OctoMap is unavailable")


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
        if index < 3:
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


@pytest.mark.parametrize("time_step", [0, -0.1, np.inf, np.nan])
def test_sensor_rejects_invalid_timestep_without_mutation(time_step):
    owners = _owners()
    sensor = Sensor(*owners)
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
